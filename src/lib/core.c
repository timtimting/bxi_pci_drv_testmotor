/*
 * Core helpers for main.c.
 *
 * This module is intentionally included by main.c instead of being
 * added to CMake as a standalone source file, because main.c reuses
 * static helpers from runtime.c in this directory.
 */

static const char *console_text(const flash_state *state,
                                const char *chinese,
                                const char *english)
{
    return state->config.chinese_ui ? chinese : english;
}

static void console_trim_line(char *line)
{
    char *start = line;
    char *end;

    while (*start != '\0' && isspace((unsigned char)*start)) {
        start++;
    }
    if (start != line) {
        memmove(line, start, strlen(start) + 1u);
    }
    end = line + strlen(line);
    while (end > line && isspace((unsigned char)end[-1])) {
        end--;
    }
    *end = '\0';
}

static int console_parse_index_arg(const char *text, unsigned int *index)
{
    if (text == NULL || index == NULL ||
        strlen(text) != 2u ||
        !isdigit((unsigned char)text[0]) ||
        !isdigit((unsigned char)text[1])) {
        return -1;
    }

    *index = (unsigned int)(text[0] - '0') * 10u +
             (unsigned int)(text[1] - '0');
    return 0;
}

static void console_restore_process_output(int saved_stdout, int saved_stderr)
{
    fflush(stdout);
    fflush(stderr);
    if (saved_stdout >= 0) {
        dup2(saved_stdout, STDOUT_FILENO);
        close(saved_stdout);
    }
    if (saved_stderr >= 0) {
        dup2(saved_stderr, STDERR_FILENO);
        close(saved_stderr);
    }
}

static int console_silence_process_output(int *saved_stdout, int *saved_stderr)
{
    int null_fd;

    *saved_stdout = -1;
    *saved_stderr = -1;
    null_fd = open("/dev/null", O_WRONLY);
    if (null_fd < 0) {
        return -1;
    }

    fflush(stdout);
    fflush(stderr);
    *saved_stdout = dup(STDOUT_FILENO);
    *saved_stderr = dup(STDERR_FILENO);
    if (*saved_stdout < 0 || *saved_stderr < 0 ||
        dup2(null_fd, STDOUT_FILENO) < 0 ||
        dup2(null_fd, STDERR_FILENO) < 0) {
        close(null_fd);
        console_restore_process_output(*saved_stdout, *saved_stderr);
        *saved_stdout = -1;
        *saved_stderr = -1;
        return -1;
    }

    close(null_fd);
    return 0;
}

static void console_quiet_sleep_ms(unsigned int ms)
{
    int saved_stdout = -1;
    int saved_stderr = -1;
    bool silenced;

    silenced = console_silence_process_output(&saved_stdout, &saved_stderr) == 0;
    sleep_ms(ms);
    if (silenced) {
        console_restore_process_output(saved_stdout, saved_stderr);
    }
}

static const char *console_default_config_path(const char *argv0,
                                               char *resolved,
                                               size_t resolved_size)
{
    char exe_path[PATH_LEN];
    char *slash;
    ssize_t len;

    if (access(DEFAULT_FIRMWARE_MAP, R_OK) == 0) {
        return DEFAULT_FIRMWARE_MAP;
    }

    len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1u);
    if (len > 0) {
        exe_path[len] = '\0';
        slash = strrchr(exe_path, '/');
        if (slash != NULL) {
            *slash = '\0';
            snprintf(resolved, resolved_size, "%s/../%s",
                     exe_path, DEFAULT_FIRMWARE_MAP);
            if (access(resolved, R_OK) == 0) {
                return resolved;
            }
        }
    }

    if (argv0 != NULL && strchr(argv0, '/') != NULL) {
        snprintf(exe_path, sizeof(exe_path), "%s", argv0);
        slash = strrchr(exe_path, '/');
        if (slash != NULL) {
            *slash = '\0';
            snprintf(resolved, resolved_size, "%s/../%s",
                     exe_path, DEFAULT_FIRMWARE_MAP);
            if (access(resolved, R_OK) == 0) {
                return resolved;
            }
        }
    }

    return DEFAULT_FIRMWARE_MAP;
}

static const motor_map_entry *console_motor_by_index(flash_state *state,
                                                      unsigned int index,
                                                      size_t *slot)
{
    size_t i;

    for (i = 0u; i < state->config.entry_count; i++) {
        if (state->config.entries[i].index == index) {
            if (slot != NULL) {
                *slot = i;
            }
            return &state->config.entries[i];
        }
    }
    return NULL;
}

static void console_use_motor(flash_state *state, const motor_map_entry *motor)
{
    state->bus = motor->bus;
    state->boot_id = motor->id;
    state->limits = *limits_for_entry(state, motor);
    update_boot_ids(state);
}

static int console_require_power(const flash_state *state)
{
    if (!state->motor_power_on) {
        printf("%s\n", console_text(state,
               "命令被拒绝：电机电源未开启，请先执行 `power_on`",
               "command refused: motor power is OFF; run `power_on` first"));
        return -1;
    }
    return 0;
}

static void console_expect_reply(flash_state *state, unsigned int bus)
{
    state->can_stats[bus].expected_replies++;
    state->can_stats[bus].outstanding_replies++;
}
