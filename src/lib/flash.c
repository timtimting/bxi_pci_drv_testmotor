/*
 * Firmware flashing commands and flash-plan parsing for main.c.
 *
 * Included by main.c; do not add as a standalone CMake source.
 */

static int console_resolve_firmware_path(const flash_state *state,
                                         const char *input,
                                         char *path,
                                         size_t path_len);
static int console_configured_firmware_path(const flash_state *state,
                                            const char *type,
                                            char *path,
                                            size_t path_len);
static int console_prefetch_firmware(flash_state *state);

static const char *console_detected_firmware_type_for_slot(const flash_state *state,
                                                           size_t slot)
{
    const motor_runtime *runtime;

    if (state == NULL || slot >= state->config.entry_count) {
        return NULL;
    }
    runtime = &state->motors[slot];
    if (!runtime->detected_type_valid) {
        return NULL;
    }
    if (find_type_config(&state->config, runtime->detected_type) == NULL) {
        return NULL;
    }
    return runtime->detected_type;
}

static const char *console_flash_type_for_motor(const flash_state *state,
                                                size_t slot,
                                                const char *fallback_type,
                                                bool *detected)
{
    const char *type = console_detected_firmware_type_for_slot(state, slot);

    if (detected != NULL) {
        *detected = type != NULL;
    }
    return type != NULL ? type : fallback_type;
}

static bool console_firmware_major_for_slot(const flash_state *state,
                                            size_t slot,
                                            unsigned int *major)
{
    const motor_runtime *runtime;

    if (state == NULL || slot >= state->config.entry_count || major == NULL) {
        return false;
    }
    runtime = &state->motors[slot];
    if (runtime->fw_version_valid) {
        *major = runtime->fw_version_major;
        return true;
    }
    if (runtime->config_version_valid) {
        *major = (runtime->config_version >> 8) & 0xffu;
        return true;
    }
    return false;
}

static int console_firmware_name_for_slot(const flash_state *state,
                                          const char *type,
                                          size_t slot,
                                          char *filename,
                                          size_t filename_len,
                                          char *source,
                                          size_t source_len)
{
    unsigned int major = 0u;
    int ret;

    if (console_firmware_major_for_slot(state, slot, &major)) {
        ret = configured_firmware_name_for_major(&state->config, type, major,
                                                 filename, filename_len);
        if (ret == 0 && source != NULL && source_len != 0u) {
            snprintf(source, source_len, "v%u", major);
        }
        return ret;
    }
    ret = configured_firmware_name(&state->config, type, filename, filename_len);
    if (ret == 0 && source != NULL && source_len != 0u) {
        snprintf(source, source_len, "default");
    }
    return ret;
}

static bool flash_debug_arg_is_one_digit_range(const char *text,
                                               unsigned int min_value,
                                               unsigned int max_value,
                                               unsigned int *value)
{
    unsigned int parsed;

    if (text == NULL || strlen(text) != 1u ||
        !isdigit((unsigned char)text[0])) {
        return false;
    }
    parsed = (unsigned int)(text[0] - '0');
    if (parsed < min_value || parsed > max_value) {
        return false;
    }
    if (value != NULL) {
        *value = parsed;
    }
    return true;
}

static int console_flash(flash_state *state, int argc, char **argv, bool all)
{
    bool cycle = false;
    const char *explicit_firmware = NULL;
    bool old_monitor;
    bool old_input;
    bool old_suppress_boot_text;
    bool old_brief_flash_output;
    char paths[MOTOR_MAP_MAX][PATH_LEN];
    char selected_types[MOTOR_MAP_MAX][MOTOR_TYPE_LEN];
    char selected_fw_sources[MOTOR_MAP_MAX][16];
    bool selected_type_detected[MOTOR_MAP_MAX];
    size_t slots[MOTOR_MAP_MAX];
    size_t selected = 0u;
    size_t succeeded = 0u;
    size_t failed = 0u;
    size_t flashed_slot = 0u;
    size_t i;

    if (console_require_power(state) != 0) {
        return -1;
    }
    if (console_any_enabled(state)) {
        printf("%s\n", console_text(state,
               "烧录被拒绝：进入 Bootloader 前必须失能全部电机",
               "flash refused: disable all motors before entering bootloader"));
        return -1;
    }
    if ((!all && (argc < 2 || argc > 4)) || (all && argc > 2)) {
        printf("%s: %s%s\n", console_text(state, "用法", "usage"),
               all ? "flash_all" : "flash_single <index00> [version|firmware.bin]", " [cycle]");
        return -1;
    }
    if (!all) {
        int argi;

        for (argi = 2; argi < argc; argi++) {
            if (strcmp(argv[argi], "cycle") == 0) {
                cycle = true;
            } else if (explicit_firmware == NULL) {
                explicit_firmware = argv[argi];
            } else {
                printf("%s: flash_single <index00> [version|firmware.bin] [cycle]\n",
                       console_text(state, "用法", "usage"));
                return -1;
            }
        }
    } else if (argc == 2 && strcmp(argv[1], "cycle") == 0) {
        cycle = true;
    } else if (argc == 2) {
        printf("%s\n", console_text(state,
               "可选参数只能是 cycle", "optional argument must be: cycle"));
        return -1;
    }
    if (!all) {
        unsigned int index;
        const motor_map_entry *motor;

        if (console_parse_index_arg(argv[1], &index) != 0 ||
            (motor = console_motor_by_index(state, index, &flashed_slot)) == NULL) {
            printf("%s: %s\n", console_text(state,
                   "未知的电机序号", "unknown motor index"), argv[1]);
            return -1;
        }
        if (explicit_firmware != NULL) {
            snprintf(selected_types[0], sizeof(selected_types[0]), "%s", explicit_firmware);
            snprintf(selected_fw_sources[0], sizeof(selected_fw_sources[0]), "%s", "manual");
            selected_type_detected[0] = false;
            if (console_resolve_firmware_path(state, explicit_firmware,
                                              paths[0], sizeof(paths[0])) != 0) {
                printf("%s: %s  %s: %s/%s\n",
                       console_text(state,
                                    "找不到固件文件",
                                    "cannot read firmware file"),
                       explicit_firmware,
                       console_text(state, "也尝试过固件目录", "also tried firmware_dir"),
                       state->config.firmware_dir,
                       explicit_firmware);
                return -1;
            }
        } else {
            bool use_detected = false;
            const char *flash_type = console_flash_type_for_motor(state,
                                                                  flashed_slot,
                                                                  motor->type,
                                                                  &use_detected);
            char filename[FIRMWARE_NAME_MAX];

            snprintf(selected_types[0], sizeof(selected_types[0]), "%s", flash_type);
            selected_type_detected[0] = use_detected;
            if (console_firmware_name_for_slot(state, flash_type, flashed_slot,
                                               filename, sizeof(filename),
                                               selected_fw_sources[0],
                                               sizeof(selected_fw_sources[0])) != 0 ||
                console_resolve_firmware_path(state, filename,
                                              paths[0], sizeof(paths[0])) != 0) {
                fprintf(stderr,
                        "cannot read firmware for index=%02u bus=%u id=%u version=%s: %s\n",
                        motor->index, motor->bus, motor->id, flash_type, paths[0]);
                return -1;
            }
        }
        slots[0] = flashed_slot;
        selected = 1u;
    } else {
        for (i = 0u; i < state->config.entry_count; i++) {
            const motor_map_entry *motor = &state->config.entries[i];
            bool use_detected = false;
            const char *flash_type = console_flash_type_for_motor(state,
                                                                  i,
                                                                  motor->type,
                                                                  &use_detected);
            char filename[FIRMWARE_NAME_MAX];

            snprintf(selected_types[selected], sizeof(selected_types[selected]), "%s", flash_type);
            selected_type_detected[selected] = use_detected;
            if (console_firmware_name_for_slot(state, flash_type, i,
                                               filename, sizeof(filename),
                                               selected_fw_sources[selected],
                                               sizeof(selected_fw_sources[selected])) != 0 ||
                console_resolve_firmware_path(state, filename,
                                              paths[selected], sizeof(paths[selected])) != 0) {
                fprintf(stderr,
                        "cannot read firmware for index=%02u bus=%u id=%u version=%s: %s\n",
                        motor->index, motor->bus, motor->id, flash_type, paths[selected]);
                failed++;
                continue;
            }
            slots[selected] = i;
            selected++;
        }
        if (failed != 0u) {
            fprintf(stderr,
                    "preflight failed: %zu/%zu selected firmware files unavailable; nothing flashed\n",
                    failed, state->config.entry_count);
            return -1;
        }
    }
    if (selected == 0u) {
        fprintf(stderr, "no motors selected for flash\n");
        return -1;
    }

    old_monitor = state->show_can_output;
    old_input = state->show_motor_input;
    old_suppress_boot_text = state->suppress_boot_text;
    old_brief_flash_output = state->brief_flash_output;
    state->show_can_output = false;
    if (all) {
        state->show_motor_input = false;
        state->suppress_boot_text = true;
        state->brief_flash_output = true;
        printf("flash_all: start total=%zu%s\n", selected, cycle ? " cycle" : "");
    } else {
        printf("flash_single: start total=1\n");
    }
    failed = 0u;
    for (i = 0u; i < selected && !stop_requested; i++) {
        const motor_map_entry *motor = &state->config.entries[slots[i]];
        size_t ordinal = i + 1u;
        int ret;

        console_use_motor(state, motor);
        state->brief_flash_index = motor->index;
        state->brief_flash_ordinal = ordinal;
        state->brief_flash_total = selected;
        if (all) {
            printf("[motor%02u]: start ordinal=%zu/%zu bus=%u id=%u version=%s%s firmware_version=%s\n",
                   motor->index, ordinal, selected, motor->bus, motor->id,
                   selected_types[i],
                   selected_type_detected[i] ? " source=detected" : "",
                   selected_fw_sources[i]);
        } else {
            printf("[motor%02u]: start bus=%u id=%u version=%s%s firmware_version=%s file=%s%s\n",
                   motor->index, motor->bus, motor->id,
                   selected_types[i],
                   selected_type_detected[i] ? " source=detected" : "",
                   selected_fw_sources[i],
                   paths[i], cycle ? " cycle" : "");
        }
        ret = boot_flash_file(state, paths[i], cycle);
        if (ret == 0) {
            succeeded++;
            memset(&state->motors[slots[i]], 0, sizeof(state->motors[slots[i]]));
            if (all) {
                printf("[motor%02u]: done result=success\n", motor->index);
            } else {
                printf("[motor%02u]: done result=success\n", motor->index);
            }
        } else {
            failed++;
            if (all) {
                fprintf(stderr, "[motor%02u]: done result=failed\n", motor->index);
            } else {
                fprintf(stderr, "[motor%02u]: done result=failed\n", motor->index);
            }
        }
    }

    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    state->suppress_boot_text = old_suppress_boot_text;
    state->brief_flash_output = old_brief_flash_output;
    if (all) {
        printf("flash_all: done total=%zu success=%zu failed=%zu%s\n",
               selected, succeeded, failed, stop_requested ? " interrupted" : "");
    } else {
        printf("flash_single: done total=1 success=%zu failed=%zu%s\n",
               succeeded, failed, stop_requested ? " interrupted" : "");
    }
    return failed == 0u && succeeded == selected ? 0 : -1;
}

static int console_resolve_firmware_path_in_dir(const char *firmware_dir,
                                                const char *input,
                                                char *path,
                                                size_t path_len)
{
    int written;
    bool has_bin_suffix;

    if (firmware_dir == NULL || firmware_dir[0] == '\0') {
        firmware_dir = DEFAULT_FIRMWARE_DIR;
    }
    if (!firmware_dir_is_safe(firmware_dir) ||
        !firmware_type_is_safe(input) ||
        strlen(input) >= FIRMWARE_NAME_MAX) {
        return -1;
    }
    has_bin_suffix = has_suffix(input, ".bin");

    written = snprintf(path, path_len, "%s/%s", firmware_dir, input);
    if (written < 0 || (size_t)written >= path_len) {
        return -1;
    }
    if (access(path, R_OK) == 0) {
        return 0;
    }
    if (!has_bin_suffix) {
        written = snprintf(path, path_len, "%s/%s.bin", firmware_dir, input);
        if (written < 0 || (size_t)written >= path_len) {
            return -1;
        }
        if (access(path, R_OK) == 0) {
            return 0;
        }
    }

    written = snprintf(path, path_len, "%s/bxi_motor_%s", firmware_dir, input);
    if (written < 0 || (size_t)written >= path_len) {
        return -1;
    }
    if (access(path, R_OK) == 0) {
        return 0;
    }
    if (!has_bin_suffix) {
        written = snprintf(path, path_len, "%s/bxi_motor_%s.bin", firmware_dir, input);
        if (written < 0 || (size_t)written >= path_len) {
            return -1;
        }
        if (access(path, R_OK) == 0) {
            return 0;
        }
    }
    return -1;
}

static int ensure_relative_dir(const char *dir)
{
    char buffer[PATH_LEN];
    char *p;

    if (!firmware_dir_is_safe(dir) || strlen(dir) >= sizeof(buffer)) {
        return -1;
    }
    snprintf(buffer, sizeof(buffer), "%s", dir);
    for (p = buffer + 1; *p != '\0'; p++) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(buffer, 0755) != 0 && errno != EEXIST) {
                return -1;
            }
            *p = '/';
        }
    }
    if (mkdir(buffer, 0755) != 0 && errno != EEXIST) {
        return -1;
    }
    return 0;
}

static int build_firmware_url(const char *base_url,
                              const char *filename,
                              char *url,
                              size_t url_len)
{
    int written;
    const char *slash;

    if (!firmware_url_is_safe(base_url) ||
        !firmware_filename_is_safe(filename) ||
        base_url == NULL || base_url[0] == '\0') {
        return -1;
    }
    slash = base_url[strlen(base_url) - 1u] == '/' ? "" : "/";
    written = snprintf(url, url_len, "%s%s%s", base_url, slash, filename);
    return written < 0 || (size_t)written >= url_len ? -1 : 0;
}

static int run_download_tool(const char *url, const char *tmp_path)
{
    pid_t pid;
    int status;

    pid = fork();
    if (pid == 0) {
        execlp("curl", "curl", "-sS", "-fL", "--retry", "2", "--connect-timeout", "10",
               "-o", tmp_path, url, (char *)NULL);
        _exit(127);
    }
    if (pid < 0 || waitpid(pid, &status, 0) < 0) {
        return -1;
    }
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        return 0;
    }

    pid = fork();
    if (pid == 0) {
        execlp("wget", "wget", "-q", "-O", tmp_path, url, (char *)NULL);
        _exit(127);
    }
    if (pid < 0 || waitpid(pid, &status, 0) < 0) {
        return -1;
    }
    return WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : -1;
}

static int download_firmware_to_cache(const char *base_url,
                                      const char *cache_dir,
                                      const char *filename,
                                      bool force,
                                      char *path,
                                      size_t path_len)
{
    char url[PATH_LEN * 2u];
    char tmp_path[PATH_LEN];
    int written;

    if (base_url == NULL || base_url[0] == '\0') {
        return -1;
    }
    if (cache_dir == NULL || cache_dir[0] == '\0') {
        cache_dir = DEFAULT_FIRMWARE_DIR;
    }
    if (ensure_relative_dir(cache_dir) != 0 ||
        build_firmware_url(base_url, filename, url, sizeof(url)) != 0) {
        return -1;
    }
    written = snprintf(path, path_len, "%s/%s", cache_dir, filename);
    if (written < 0 || (size_t)written >= path_len) {
        return -1;
    }
    if (!force && access(path, R_OK) == 0) {
        return 0;
    }
    written = snprintf(tmp_path, sizeof(tmp_path), "%s/.%s.%ld.download",
                       cache_dir, filename, (long)getpid());
    if (written < 0 || (size_t)written >= sizeof(tmp_path)) {
        return -1;
    }
    printf("固件本地不存在，正在下载：%s -> %s\n", url, path);
    unlink(tmp_path);
    if (run_download_tool(url, tmp_path) != 0) {
        unlink(tmp_path);
        return -1;
    }
    if (rename(tmp_path, path) != 0) {
        unlink(tmp_path);
        return -1;
    }
    return access(path, R_OK) == 0 ? 0 : -1;
}

static int flash_plan_version_to_filename(const motor_map_config *config,
                                          const char *version,
                                          char *filename,
                                          size_t filename_len)
{
    if (!firmware_type_is_safe(version) || strlen(version) >= FIRMWARE_NAME_MAX) {
        return -1;
    }
    if (has_suffix(version, ".bin")) {
        if (!firmware_filename_is_safe(version) || strlen(version) >= filename_len) {
            return -1;
        }
        snprintf(filename, filename_len, "%s", version);
        return 0;
    }
    return configured_firmware_name(config, version, filename, filename_len);
}

static size_t firmware_candidate_names(const char *input,
                                       char names[][FIRMWARE_NAME_MAX],
                                       size_t max_names)
{
    bool has_bin_suffix = has_suffix(input, ".bin");
    size_t count = 0u;
    int written;

    if (!firmware_type_is_safe(input) || strlen(input) >= FIRMWARE_NAME_MAX) {
        return 0u;
    }
    written = snprintf(names[count], FIRMWARE_NAME_MAX, "%s", input);
    if (written >= 0 && (size_t)written < FIRMWARE_NAME_MAX) {
        count++;
    }
    if (!has_bin_suffix && count < max_names) {
        written = snprintf(names[count], FIRMWARE_NAME_MAX, "%s.bin", input);
        if (written >= 0 && (size_t)written < FIRMWARE_NAME_MAX) {
            count++;
        }
    }
    if (count < max_names) {
        written = snprintf(names[count], FIRMWARE_NAME_MAX, "bxi_motor_%s", input);
        if (written >= 0 && (size_t)written < FIRMWARE_NAME_MAX) {
            count++;
        }
    }
    if (!has_bin_suffix && count < max_names) {
        written = snprintf(names[count], FIRMWARE_NAME_MAX, "bxi_motor_%s.bin", input);
        if (written >= 0 && (size_t)written < FIRMWARE_NAME_MAX) {
            count++;
        }
    }
    return count;
}

static int console_resolve_firmware_path_local(const char *firmware_dir,
                                               const char *fallback_dir,
                                               const char *input,
                                               char *path,
                                               size_t path_len)
{
    if (console_resolve_firmware_path_in_dir(firmware_dir, input, path, path_len) == 0) {
        return 0;
    }
    if (fallback_dir != NULL && fallback_dir[0] != '\0' &&
        strcmp(fallback_dir, firmware_dir) != 0 &&
        console_resolve_firmware_path_in_dir(fallback_dir, input, path, path_len) == 0) {
        return 0;
    }
    return -1;
}

static int console_resolve_firmware_path(const flash_state *state,
                                         const char *input,
                                         char *path,
                                         size_t path_len)
{
    const char *primary = state->firmware_prefetch_ready ?
                          state->active_firmware_dir : state->config.firmware_dir;
    const char *fallback = state->firmware_prefetch_ready ?
                           state->config.firmware_dir : NULL;
    char filename[FIRMWARE_NAME_MAX];

    if (!has_suffix(input, ".bin") &&
        configured_firmware_name(&state->config, input, filename, sizeof(filename)) == 0) {
        return console_resolve_firmware_path_local(primary, fallback, filename, path, path_len);
    }
    return console_resolve_firmware_path_local(primary, fallback, input, path, path_len);
}

static int console_configured_firmware_path(const flash_state *state,
                                            const char *type,
                                            char *path,
                                            size_t path_len)
{
    motor_map_config lookup_config = state->config;

    if (state->firmware_prefetch_ready) {
        snprintf(lookup_config.firmware_dir, sizeof(lookup_config.firmware_dir),
                 "%s", state->active_firmware_dir);
    }
    if (configured_firmware_path(&lookup_config, type, path, path_len) != 0) {
        return -1;
    }
    if (access(path, R_OK) == 0) {
        return 0;
    }
    if (state->firmware_prefetch_ready &&
        configured_firmware_path(&state->config, type, path, path_len) == 0 &&
        access(path, R_OK) == 0) {
        return 0;
    }
    return -1;
}

static int prefetch_one_firmware(const char *base_url,
                                 const char *cache_dir,
                                 const char *filename,
                                 char *path,
                                 size_t path_len)
{
    return download_firmware_to_cache(base_url, cache_dir, filename, true,
                                      path, path_len);
}

static bool flash_plan_filename_seen(const char filenames[][FIRMWARE_NAME_MAX],
                                    size_t count,
                                    const char *filename)
{
    size_t i;

    for (i = 0u; i < count; i++) {
        if (strcmp(filenames[i], filename) == 0) {
            return true;
        }
    }
    return false;
}

static int console_prefetch_firmware(flash_state *state)
{
    flash_plan_config plan;
    const char *base_url;
    const char *cache_dir;
    char filenames[FLASH_PLAN_MAX_TARGETS][FIRMWARE_NAME_MAX];
    size_t filename_count = 0u;
    size_t i;
    size_t ok = 0u;
    size_t name_failed = 0u;
    char path[PATH_LEN];

    state->firmware_prefetch_ready = false;
    state->active_firmware_dir[0] = '\0';

    if (load_flash_plan_config(DEFAULT_FLASH_PLAN, &plan) != 0) {
        printf("%s\n", console_text(state,
               "固件预拉取跳过：默认烧录配置不可用，将使用项目自带固件",
               "firmware prefetch skipped: default flash plan unavailable; using bundled firmware"));
        return -1;
    }
    if (!state->config.firmware_sync_on_start) {
        snprintf(state->active_firmware_dir, sizeof(state->active_firmware_dir),
                 "%s", state->config.firmware_dir);
        printf("%s\n", console_text(state,
               "启动固件同步已关闭，使用项目自带固件",
               "startup firmware sync is disabled; using bundled firmware"));
        return 0;
    }

    base_url = state->config.firmware_base_url[0] != '\0' ?
               state->config.firmware_base_url : plan.firmware_base_url;
    cache_dir = state->config.firmware_cache_dir[0] != '\0' ?
                state->config.firmware_cache_dir : plan.firmware_cache_dir;
    if (base_url == NULL || base_url[0] == '\0') {
        snprintf(state->active_firmware_dir, sizeof(state->active_firmware_dir),
                 "%s", state->config.firmware_dir);
        printf("%s\n", console_text(state,
               "未配置固件下载站，使用项目自带固件",
               "firmware download URL is not configured; using bundled firmware"));
        return 0;
    }

    for (i = 0u; i < plan.target_count; i++) {
        const char *version = plan.targets[i].version;
        char filename[FIRMWARE_NAME_MAX];

        if (flash_plan_version_to_filename(&state->config, version,
                                           filename, sizeof(filename)) != 0) {
            printf("  FAIL version=%s config-name\n", version);
            name_failed++;
            continue;
        }
        if (!flash_plan_filename_seen(filenames, filename_count, filename)) {
            if (filename_count >= FLASH_PLAN_MAX_TARGETS) {
                printf("  FAIL version=%s too-many-firmware-files\n", version);
                name_failed++;
                continue;
            }
            snprintf(filenames[filename_count], sizeof(filenames[filename_count]), "%s", filename);
            filename_count++;
        }
        {
            const motor_type_config *type_config = find_type_config(&state->config, version);
            const char *versioned_files[] = {
                type_config != NULL ? type_config->firmware_v0 : "",
                type_config != NULL ? type_config->firmware_v1 : "",
            };
            size_t versioned_i;

            for (versioned_i = 0u;
                 versioned_i < sizeof(versioned_files) / sizeof(versioned_files[0]);
                 versioned_i++) {
                if (versioned_files[versioned_i][0] == '\0') {
                    continue;
                }
                if (!flash_plan_filename_seen(filenames, filename_count,
                                              versioned_files[versioned_i])) {
                    if (filename_count >= FLASH_PLAN_MAX_TARGETS) {
                        printf("  FAIL version=%s too-many-firmware-files\n", version);
                        name_failed++;
                        break;
                    }
                    snprintf(filenames[filename_count],
                             sizeof(filenames[filename_count]),
                             "%s", versioned_files[versioned_i]);
                    filename_count++;
                }
            }
        }
    }
    printf("%s %s -> %s (%zu %s)\n",
           console_text(state, "启动时拉取固件：", "prefetching firmware at startup:"),
           base_url,
           cache_dir,
           filename_count,
           console_text(state, "个文件", "file(s)"));
    for (i = 0u; i < filename_count; i++) {
        if (prefetch_one_firmware(base_url, cache_dir, filenames[i],
                                  path, sizeof(path)) == 0) {
            ok++;
            printf("  OK   file=%s\n", path);
        } else {
            printf("  FAIL file=%s\n", filenames[i]);
        }
    }
    if (name_failed == 0u && filename_count != 0u && ok == filename_count) {
        state->firmware_prefetch_ready = true;
        snprintf(state->active_firmware_dir, sizeof(state->active_firmware_dir),
                 "%s", cache_dir);
        printf("%s: %s\n", console_text(state,
               "固件预拉取成功，本次运行使用下载固件",
               "firmware prefetch succeeded; using downloaded firmware for this run"),
               state->active_firmware_dir);
        return 0;
    }

    snprintf(state->active_firmware_dir, sizeof(state->active_firmware_dir),
             "%s", state->config.firmware_dir);
    printf("%s: %s\n", console_text(state,
           "固件预拉取失败，本次运行回退项目自带固件",
           "firmware prefetch failed; falling back to bundled firmware for this run"),
           state->active_firmware_dir);
    return -1;
}

static int console_flash_debug(flash_state *state, int argc, char **argv)
{
    char path[PATH_LEN];
    unsigned int bus;
    unsigned int id;
    unsigned int old_bus;
    unsigned int old_id;
    bool cycle = false;
    bool old_monitor;
    const char *firmware_arg = NULL;
    const motor_map_entry *target_motor = NULL;
    int ret;
    int argi;

    if (console_require_power(state) != 0) {
        return -1;
    }
    if (console_any_enabled(state)) {
        printf("%s\n", console_text(state,
               "调试烧录被拒绝：进入 Bootloader 前必须失能全部电机",
               "debug flash refused: disable all motors before entering bootloader"));
        return -1;
    }
    if (argc == 3 || (argc == 4 && strcmp(argv[3], "cycle") == 0)) {
        unsigned int index;
        size_t slot;
        const motor_map_entry *motor;

        if (console_parse_index_arg(argv[1], &index) != 0 ||
            (motor = console_motor_by_index(state, index, &slot)) == NULL) {
            printf("%s: flash_debug <index00> <version|firmware.bin> [cycle]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
        (void)slot;
        bus = motor->bus;
        id = motor->id;
        target_motor = motor;
        firmware_arg = argv[2];
        if (argc == 4) {
            cycle = true;
        }
    } else if (argc >= 4 && argc <= 5) {
        if (!flash_debug_arg_is_one_digit_range(argv[1], 0u, CANFD_DEVICE_NUM - 1u, &bus) ||
            !flash_debug_arg_is_one_digit_range(argv[2], 0u, 7u, &id)) {
            printf("%s: flash_debug <bus0-4> <id0-7> <version|firmware.bin> [cycle]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
        firmware_arg = argv[3];
        for (argi = 4; argi < argc; argi++) {
            if (strcmp(argv[argi], "cycle") == 0) {
                cycle = true;
                continue;
            }
            printf("%s: flash_debug <bus0-4> <id0-7> <version|firmware.bin> [cycle]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
    } else {
        printf("%s: flash_debug <index00> <version|firmware.bin> [cycle]\n"
               "%s: flash_debug <bus0-4> <id0-7> <version|firmware.bin> [cycle]\n",
               console_text(state, "用法", "usage"),
               console_text(state, "用法", "usage"));
        return -1;
    }
    if (console_resolve_firmware_path(state, firmware_arg, path, sizeof(path)) != 0) {
        printf("%s: %s  %s: %s/%s\n",
               console_text(state,
                            "找不到调试固件文件",
                            "cannot read debug firmware file"),
               firmware_arg,
               console_text(state, "也尝试过固件目录", "also tried firmware_dir"),
               state->config.firmware_dir,
               firmware_arg);
        return -1;
    }

    old_bus = state->bus;
    old_id = state->boot_id;
    state->bus = bus;
    set_target_id(state, id);
    if (target_motor == NULL) {
        target_motor = console_motor_by_bus_id(state, bus, id, NULL);
    }

    printf("flash_debug: start total=1 bus=%u id=%u file=%s%s\n",
           bus, id, path, cycle ? " cycle" : "");
    if (target_motor != NULL) {
        printf("[motor%02u]: start bus=%u id=%u\n",
               target_motor->index, bus, id);
    }
    old_monitor = state->show_can_output;
    state->show_can_output = false;
    ret = boot_flash_file(state, path, cycle);
    state->show_can_output = old_monitor;

    state->bus = old_bus;
    set_target_id(state, old_id);

    if (ret == 0) {
        if (target_motor != NULL) {
            printf("[motor%02u]: done result=success\n", target_motor->index);
        }
        printf("flash_debug: done total=1 success=1 failed=0\n");
        return 0;
    }
    if (target_motor != NULL) {
        fprintf(stderr, "[motor%02u]: done result=failed\n", target_motor->index);
    }
    fprintf(stderr, "flash_debug: done total=1 success=0 failed=1\n");
    return -1;
}

static int flash_plan_finalize_target(const char *path,
                                      flash_plan_config *plan,
                                      const flash_plan_target *target)
{
    if (!target->has_index && !target->has_bus && !target->has_id &&
        target->version[0] == '\0') {
        return 0;
    }
    if (!target->has_index || !target->has_bus || !target->has_id ||
        target->version[0] == '\0') {
        fprintf(stderr, "%s:%u: flash target requires index, bus, id, and version\n",
                path, target->line_no);
        return -1;
    }
    if (target->has_index && target->index > 99u) {
        fprintf(stderr, "%s:%u: index must be 00..99\n", path, target->line_no);
        return -1;
    }
    if (target->has_bus && target->bus >= CANFD_DEVICE_NUM) {
        fprintf(stderr, "%s:%u: bus must be 0..%u\n",
                path, target->line_no, CANFD_DEVICE_NUM - 1u);
        return -1;
    }
    if (target->has_id && (target->id == 0u || target->id > 8u)) {
        fprintf(stderr, "%s:%u: id must be 1..8\n", path, target->line_no);
        return -1;
    }
    if (plan->target_count >= FLASH_PLAN_MAX_TARGETS) {
        fprintf(stderr, "%s:%u: too many flash targets, max is %u\n",
                path, target->line_no, (unsigned int)FLASH_PLAN_MAX_TARGETS);
        return -1;
    }
    plan->targets[plan->target_count++] = *target;
    return 0;
}

static int load_flash_plan_config(const char *path, flash_plan_config *plan)
{
    FILE *fp;
    char line[LINE_LEN];
    unsigned int line_no = 0u;
    bool in_targets = false;
    flash_plan_target target;

    memset(plan, 0, sizeof(*plan));
    memset(&target, 0, sizeof(target));
    snprintf(plan->firmware_dir, sizeof(plan->firmware_dir), "%s", DEFAULT_FIRMWARE_DIR);
    snprintf(plan->firmware_cache_dir, sizeof(plan->firmware_cache_dir), "%s", "firmware_cache");

    fp = fopen(path, "r");
    if (fp == NULL) {
        perror(path);
        return -1;
    }

    while (fgets(line, sizeof(line), fp) != NULL) {
        char *comment;
        char *p;
        char *key;
        char *value;

        line_no++;
        comment = strchr(line, '#');
        if (comment != NULL) {
            *comment = '\0';
        }
        p = trim_space(line);
        if (p[0] == '\0') {
            continue;
        }
        if (strncmp(p, "- ", 2u) == 0) {
            if (!in_targets) {
                fprintf(stderr, "%s:%u: list entries are only allowed under targets\n",
                        path, line_no);
                fclose(fp);
                return -1;
            }
            if (flash_plan_finalize_target(path, plan, &target) != 0) {
                fclose(fp);
                return -1;
            }
            memset(&target, 0, sizeof(target));
            target.line_no = line_no;
        }
        if (parse_yaml_key_value(p, &key, &value) != 0) {
            fprintf(stderr, "%s:%u: expected key: value\n", path, line_no);
            fclose(fp);
            return -1;
        }
        if (strcmp(key, "targets") == 0) {
            in_targets = true;
        } else if (!in_targets && strcmp(key, "firmware_dir") == 0) {
            if (!firmware_dir_is_safe(value) ||
                strlen(value) >= sizeof(plan->firmware_dir)) {
                fprintf(stderr,
                        "%s:%u: invalid firmware_dir: %s "
                        "(use a relative directory without absolute paths or ..)\n",
                        path, line_no, value);
                fclose(fp);
                return -1;
            }
            snprintf(plan->firmware_dir, sizeof(plan->firmware_dir), "%s", value);
        } else if (!in_targets && strcmp(key, "firmware_base_url") == 0) {
            if (!firmware_url_is_safe(value) ||
                strlen(value) >= sizeof(plan->firmware_base_url)) {
                fprintf(stderr,
                        "%s:%u: invalid firmware_base_url: %s "
                        "(use http:// or https:// without spaces)\n",
                        path, line_no, value);
                fclose(fp);
                return -1;
            }
            snprintf(plan->firmware_base_url, sizeof(plan->firmware_base_url), "%s", value);
        } else if (!in_targets && strcmp(key, "firmware_cache_dir") == 0) {
            if (!firmware_dir_is_safe(value) ||
                strlen(value) >= sizeof(plan->firmware_cache_dir)) {
                fprintf(stderr,
                        "%s:%u: invalid firmware_cache_dir: %s "
                        "(use a relative directory without absolute paths or ..)\n",
                        path, line_no, value);
                fclose(fp);
                return -1;
            }
            snprintf(plan->firmware_cache_dir, sizeof(plan->firmware_cache_dir), "%s", value);
        } else if (in_targets && strcmp(key, "index") == 0) {
            if (parse_uint_arg(value, &target.index) != 0) {
                fprintf(stderr, "%s:%u: invalid target index\n", path, line_no);
                fclose(fp);
                return -1;
            }
            if (target.line_no == 0u) {
                target.line_no = line_no;
            }
            target.has_index = true;
        } else if (in_targets && strcmp(key, "bus") == 0) {
            if (parse_uint_arg(value, &target.bus) != 0) {
                fprintf(stderr, "%s:%u: invalid target bus\n", path, line_no);
                fclose(fp);
                return -1;
            }
            if (target.line_no == 0u) {
                target.line_no = line_no;
            }
            target.has_bus = true;
        } else if (in_targets && strcmp(key, "id") == 0) {
            if (parse_uint_arg(value, &target.id) != 0) {
                fprintf(stderr, "%s:%u: invalid target id\n", path, line_no);
                fclose(fp);
                return -1;
            }
            if (target.line_no == 0u) {
                target.line_no = line_no;
            }
            target.has_id = true;
        } else if (in_targets && strcmp(key, "version") == 0) {
            if (!firmware_type_is_safe(value) ||
                strlen(value) >= sizeof(target.version)) {
                fprintf(stderr, "%s:%u: invalid firmware version\n", path, line_no);
                fclose(fp);
                return -1;
            }
            snprintf(target.version, sizeof(target.version), "%s", value);
            if (target.line_no == 0u) {
                target.line_no = line_no;
            }
        } else if (in_targets && strcmp(key, "name") == 0) {
            if (strlen(value) >= sizeof(target.name)) {
                fprintf(stderr, "%s:%u: target name is too long\n", path, line_no);
                fclose(fp);
                return -1;
            }
            snprintf(target.name, sizeof(target.name), "%s", value);
            if (target.line_no == 0u) {
                target.line_no = line_no;
            }
        } else {
            fprintf(stderr, "%s:%u: unknown flash plan key: %s\n", path, line_no, key);
            fclose(fp);
            return -1;
        }
    }

    if (flash_plan_finalize_target(path, plan, &target) != 0) {
        fclose(fp);
        return -1;
    }
    fclose(fp);

    if (plan->target_count == 0u) {
        fprintf(stderr, "%s: no flash targets configured\n", path);
        return -1;
    }
    return 0;
}

static const motor_map_entry *console_motor_by_bus_id(flash_state *state,
                                                       unsigned int bus,
                                                       unsigned int id,
                                                       size_t *slot)
{
    size_t i;

    for (i = 0u; i < state->config.entry_count; i++) {
        if (state->config.entries[i].bus == bus && state->config.entries[i].id == id) {
            if (slot != NULL) {
                *slot = i;
            }
            return &state->config.entries[i];
        }
    }
    return NULL;
}

static int console_flash_plan_file(flash_state *state, const char *plan_path, bool cycle)
{
    flash_plan_config plan;
    char resolved_paths[FLASH_PLAN_MAX_TARGETS][PATH_LEN];
    char resolved_versions[FLASH_PLAN_MAX_TARGETS][MOTOR_TYPE_LEN];
    char resolved_fw_sources[FLASH_PLAN_MAX_TARGETS][16];
    bool resolved_version_detected[FLASH_PLAN_MAX_TARGETS];
    size_t slots[FLASH_PLAN_MAX_TARGETS];
    unsigned int old_bus;
    unsigned int old_id;
    bool old_monitor;
    bool old_input;
    bool old_suppress_boot_text;
    bool old_brief_flash_output;
    size_t i;
    size_t succeeded = 0u;
    size_t failed = 0u;

    if (console_require_power(state) != 0) {
        return -1;
    }
    if (console_any_enabled(state)) {
        printf("%s\n", console_text(state,
               "策略烧录被拒绝：进入 Bootloader 前必须失能全部电机",
               "plan flash refused: disable all motors before entering bootloader"));
        return -1;
    }
    if (load_flash_plan_config(plan_path, &plan) != 0) {
        return -1;
    }

    for (i = 0u; i < plan.target_count; i++) {
        const flash_plan_target *target = &plan.targets[i];
        size_t slot = 0u;
        const motor_map_entry *configured_motor;

        configured_motor = console_motor_by_bus_id(state, target->bus, target->id, &slot);
        if (configured_motor != NULL && configured_motor->index != target->index) {
            fprintf(stderr,
                    "%s:%u: warning: plan index=%02u but terminal config has index=%02u for bus=%u id=%u\n",
                    plan_path, target->line_no, target->index, configured_motor->index,
                    target->bus, target->id);
        }
        slots[i] = configured_motor != NULL ? slot : (size_t)-1;
        {
            char filename[FIRMWARE_NAME_MAX];
            bool use_detected = false;
            const char *flash_version = target->version;

            if (configured_motor != NULL) {
                flash_version = console_flash_type_for_motor(state,
                                                            slot,
                                                            target->version,
                                                            &use_detected);
            }
            snprintf(resolved_versions[i], sizeof(resolved_versions[i]), "%s", flash_version);
            resolved_version_detected[i] = use_detected;
            if ((configured_motor != NULL ?
                    console_firmware_name_for_slot(state, flash_version, slot,
                                                   filename, sizeof(filename),
                                                   resolved_fw_sources[i],
                                                   sizeof(resolved_fw_sources[i])) :
                    flash_plan_version_to_filename(&state->config, flash_version,
                                                  filename, sizeof(filename))) != 0 ||
                console_resolve_firmware_path_local(state->firmware_prefetch_ready ?
                                                    state->active_firmware_dir : plan.firmware_dir,
                                                    state->firmware_prefetch_ready ?
                                                    plan.firmware_dir : NULL,
                                                    filename,
                                                    resolved_paths[i],
                                                    sizeof(resolved_paths[i])) != 0) {
                fprintf(stderr, "%s:%u: cannot read firmware version: %s (firmware_dir=%s)\n",
                        plan_path, target->line_no, flash_version, plan.firmware_dir);
                failed++;
                continue;
            }
            if (configured_motor == NULL) {
                snprintf(resolved_fw_sources[i], sizeof(resolved_fw_sources[i]), "%s", "plan");
            }
        }
    }
    if (failed != 0u) {
        fprintf(stderr, "flash_all preflight failed: %zu/%zu targets unavailable; nothing flashed\n",
                failed, plan.target_count);
        return -1;
    }

    old_bus = state->bus;
    old_id = state->boot_id;
    old_monitor = state->show_can_output;
    old_input = state->show_motor_input;
    old_suppress_boot_text = state->suppress_boot_text;
    old_brief_flash_output = state->brief_flash_output;
    state->show_can_output = false;
    state->show_motor_input = false;
    state->suppress_boot_text = true;
    state->brief_flash_output = true;

    printf("flash_all: start plan=%s total=%zu%s\n",
           plan_path, plan.target_count, cycle ? " cycle" : "");
    for (i = 0u; i < plan.target_count && !stop_requested; i++) {
        const flash_plan_target *target = &plan.targets[i];
        int ret;

        state->bus = target->bus;
        set_target_id(state, target->id);
        state->brief_flash_index = target->index;
        state->brief_flash_ordinal = i + 1u;
        state->brief_flash_total = plan.target_count;
        printf("[motor%02u]: start ordinal=%zu/%zu bus=%u id=%u version=%s%s firmware_version=%s file=%s%s\n",
               target->index, i + 1u, plan.target_count, target->bus, target->id,
               resolved_versions[i],
               resolved_version_detected[i] ? " source=detected" : "",
               resolved_fw_sources[i],
               resolved_paths[i], cycle ? " cycle" : "");
        ret = boot_flash_file(state, resolved_paths[i], cycle);
        if (ret == 0) {
            succeeded++;
            if (slots[i] != (size_t)-1) {
                memset(&state->motors[slots[i]], 0, sizeof(state->motors[slots[i]]));
            }
            printf("[motor%02u]: done result=success\n", target->index);
        } else {
            failed++;
            fprintf(stderr, "[motor%02u]: done result=failed\n", target->index);
        }
    }
    collect_text(state, 800u, 3000u);
    rx_ring_clear(&state->rx);
    frame_ring_clear(&state->frames);
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    state->suppress_boot_text = old_suppress_boot_text;
    state->brief_flash_output = old_brief_flash_output;
    state->bus = old_bus;
    set_target_id(state, old_id);

    printf("flash_all: done total=%zu success=%zu failed=%zu%s\n",
           plan.target_count, succeeded, failed, stop_requested ? " interrupted" : "");
    return failed == 0u && succeeded == plan.target_count ? 0 : -1;
}

static int console_flash_all(flash_state *state, int argc, char **argv)
{
    const char *plan_path = DEFAULT_FLASH_PLAN;
    bool has_plan = false;
    bool cycle = false;
    int i;

    if (argc > 3) {
        printf("%s: flash_all [plan.yaml] [cycle]\n", console_text(state, "用法", "usage"));
        return -1;
    }
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "cycle") == 0) {
            cycle = true;
        } else if (!has_plan) {
            plan_path = argv[i];
            has_plan = true;
        } else {
            printf("%s: flash_all [plan.yaml] [cycle]\n", console_text(state, "用法", "usage"));
            return -1;
        }
    }
    return console_flash_plan_file(state, plan_path, cycle);
}

static int console_load_default_topology(motor_map_config *config)
{
    flash_plan_config plan;
    size_t i;

    if (config->entry_count != 0u) {
        return 0;
    }
    if (load_flash_plan_config(DEFAULT_FLASH_PLAN, &plan) != 0) {
        return -1;
    }
    for (i = 0u; i < plan.target_count; i++) {
        const flash_plan_target *target = &plan.targets[i];

        if (config->type_config_count != 0u &&
            find_type_config(config, target->version) == NULL) {
            fprintf(stderr,
                    "%s:%u: default flash version %s has no motor_types entry\n",
                    DEFAULT_FLASH_PLAN, target->line_no, target->version);
            return -1;
        }
        if (motor_map_add_entry(config, target->index, target->bus, target->id,
                                target->version, target->name, target->line_no) != 0) {
            return -1;
        }
    }
    return 0;
}
