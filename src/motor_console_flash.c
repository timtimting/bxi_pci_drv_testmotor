/*
 * Firmware flashing commands and flash-plan parsing for motor_console.c.
 *
 * Included by motor_console.c; do not add as a standalone CMake source.
 */

static int console_resolve_firmware_path(const flash_state *state,
                                         const char *input,
                                         char *path,
                                         size_t path_len);

static int console_flash(flash_state *state, int argc, char **argv, bool all)
{
    bool cycle = false;
    const char *explicit_firmware = NULL;
    bool old_monitor;
    bool old_input;
    bool old_suppress_boot_text;
    bool old_brief_flash_output;
    char paths[MOTOR_MAP_MAX][PATH_LEN];
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
        } else if (configured_firmware_path(&state->config, motor->type,
                                            paths[0], sizeof(paths[0])) != 0 ||
                   access(paths[0], R_OK) != 0) {
            fprintf(stderr, "cannot read firmware for index=%02u bus=%u id=%u version=%s: %s\n",
                    motor->index, motor->bus, motor->id, motor->type, paths[0]);
            return -1;
        }
        slots[0] = flashed_slot;
        selected = 1u;
    } else {
        for (i = 0u; i < state->config.entry_count; i++) {
            const motor_map_entry *motor = &state->config.entries[i];

            if (configured_firmware_path(&state->config, motor->type,
                                         paths[selected], sizeof(paths[selected])) != 0 ||
                access(paths[selected], R_OK) != 0) {
                fprintf(stderr,
                        "cannot read firmware for index=%02u bus=%u id=%u version=%s: %s\n",
                        motor->index, motor->bus, motor->id, motor->type, paths[selected]);
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
        printf("FLASH SUMMARY total=%zu start\n", selected);
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
            printf("FLASH START %02zu/%02zu index=%02u bus=%u id=%u version=%s\n",
                   ordinal, selected, motor->index, motor->bus, motor->id, motor->type);
        } else {
            printf("\n######## FLASH SINGLE: index=%02u bus=%u id=%u file=%s%s ########\n",
                   motor->index, motor->bus, motor->id, paths[i], cycle ? " cycle" : "");
        }
        ret = boot_flash_file(state, paths[i], cycle);
        if (ret == 0) {
            succeeded++;
            memset(&state->motors[slots[i]], 0, sizeof(state->motors[slots[i]]));
            if (all) {
                printf("FLASH DONE  %02zu/%02zu index=%02u result=OK\n",
                       ordinal, selected, motor->index);
            } else {
                printf("######## FLASH SINGLE SUCCESS: index=%02u ########\n", motor->index);
            }
        } else {
            failed++;
            if (all) {
                fprintf(stderr, "FLASH DONE  %02zu/%02zu index=%02u result=FAILED\n",
                        ordinal, selected, motor->index);
            } else {
                fprintf(stderr, "######## FLASH SINGLE FAILED: index=%02u ########\n", motor->index);
            }
        }
    }

    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    state->suppress_boot_text = old_suppress_boot_text;
    state->brief_flash_output = old_brief_flash_output;
    if (all) {
        printf("FLASH SUMMARY total=%zu success=%zu failed=%zu%s\n",
               selected, succeeded, failed, stop_requested ? " interrupted" : "");
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

static int console_resolve_firmware_path(const flash_state *state,
                                         const char *input,
                                         char *path,
                                         size_t path_len)
{
    return console_resolve_firmware_path_in_dir(state->config.firmware_dir,
                                                input, path, path_len);
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
    if (argc < 4 || argc > 5 ||
        parse_uint_arg(argv[1], &bus) != 0 ||
        parse_uint_arg(argv[2], &id) != 0 ||
        bus >= CANFD_DEVICE_NUM ||
        id == 0u || id > 8u) {
        printf("%s: flash_debug <bus> <id> <version|firmware.bin> [cycle]\n",
               console_text(state, "用法", "usage"));
        return -1;
    }
    for (argi = 4; argi < argc; argi++) {
        if (strcmp(argv[argi], "cycle") == 0) {
            cycle = true;
            continue;
        }
        printf("%s: flash_debug <bus> <id> <version|firmware.bin> [cycle]\n",
               console_text(state, "用法", "usage"));
        return -1;
    }
    if (console_resolve_firmware_path(state, argv[3], path, sizeof(path)) != 0) {
        printf("%s: %s  %s: %s/%s\n",
               console_text(state,
                            "找不到调试固件文件",
                            "cannot read debug firmware file"),
               argv[3],
               console_text(state, "也尝试过固件目录", "also tried firmware_dir"),
               state->config.firmware_dir,
               argv[3]);
        return -1;
    }

    old_bus = state->bus;
    old_id = state->boot_id;
    state->bus = bus;
    set_target_id(state, id);

    printf("\n######## DEBUG FLASH: bus=%u id=%u file=%s%s ########\n",
           bus, id, path, cycle ? " cycle" : "");
    old_monitor = state->show_can_output;
    state->show_can_output = false;
    ret = boot_flash_file(state, path, cycle);
    state->show_can_output = old_monitor;

    state->bus = old_bus;
    set_target_id(state, old_id);

    if (ret == 0) {
        printf("######## DEBUG FLASH SUCCESS: bus=%u id=%u ########\n", bus, id);
        return 0;
    }
    fprintf(stderr, "######## DEBUG FLASH FAILED: bus=%u id=%u ########\n", bus, id);
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
    char line[256];
    unsigned int line_no = 0u;
    bool in_targets = false;
    flash_plan_target target;

    memset(plan, 0, sizeof(*plan));
    memset(&target, 0, sizeof(target));
    snprintf(plan->firmware_dir, sizeof(plan->firmware_dir), "%s", DEFAULT_FIRMWARE_DIR);

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
        if (console_resolve_firmware_path_in_dir(plan.firmware_dir, target->version,
                                                 resolved_paths[i],
                                                 sizeof(resolved_paths[i])) != 0) {
            fprintf(stderr, "%s:%u: cannot read firmware version: %s (firmware_dir=%s)\n",
                    plan_path, target->line_no, target->version, plan.firmware_dir);
            failed++;
            continue;
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

    printf("FLASH ALL plan=%s total=%zu start%s\n",
           plan_path, plan.target_count, cycle ? " cycle" : "");
    for (i = 0u; i < plan.target_count && !stop_requested; i++) {
        const flash_plan_target *target = &plan.targets[i];
        int ret;

        state->bus = target->bus;
        set_target_id(state, target->id);
        state->brief_flash_index = target->index;
        state->brief_flash_ordinal = i + 1u;
        state->brief_flash_total = plan.target_count;
        printf("FLASH START %02zu/%02zu index=%02u bus=%u id=%u version=%s file=%s%s\n",
               i + 1u, plan.target_count, target->index, target->bus, target->id,
               target->version, resolved_paths[i], cycle ? " cycle" : "");
        ret = boot_flash_file(state, resolved_paths[i], cycle);
        if (ret == 0) {
            succeeded++;
            if (slots[i] != (size_t)-1) {
                memset(&state->motors[slots[i]], 0, sizeof(state->motors[slots[i]]));
            }
            printf("FLASH DONE  %02zu/%02zu index=%02u result=OK\n",
                   i + 1u, plan.target_count, target->index);
        } else {
            failed++;
            fprintf(stderr, "FLASH DONE  %02zu/%02zu index=%02u result=FAILED\n",
                    i + 1u, plan.target_count, target->index);
        }
    }
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    state->suppress_boot_text = old_suppress_boot_text;
    state->brief_flash_output = old_brief_flash_output;
    state->bus = old_bus;
    set_target_id(state, old_id);

    printf("FLASH ALL SUMMARY total=%zu success=%zu failed=%zu%s\n",
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
                                target->version, target->line_no) != 0) {
            return -1;
        }
    }
    return 0;
}
