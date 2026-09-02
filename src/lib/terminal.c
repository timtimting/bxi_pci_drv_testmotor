/*
 * Configuration reload, command dispatch and interactive terminal loop.
 *
 * Included by main.c; do not add as a standalone CMake source.
 */

enum {
    CONSOLE_COMMAND_EXIT = 100,
};

static int console_reload_config(flash_state *state, const char *path)
{
    motor_map_config new_config;

    if (state->motor_power_on || console_any_enabled(state)) {
        printf("%s\n", console_text(state,
               "配置重载被拒绝：电机仍处于上电或使能状态",
               "config reload refused while motor power is on or a motor is enabled"));
        return -1;
    }
    if (load_motor_map_config(path, &new_config) != 0) {
        return -1;
    }
    if (console_load_default_topology(&new_config) != 0) {
        return -1;
    }
    if (state->language_override_active) {
        new_config.chinese_ui = state->chinese_override;
    }
    state->config = new_config;
    state->limits = new_config.mit_limits;
    state->debug_use_canfd = new_config.mit_canfd;
    state->show_can_output = new_config.live_output;
    state->show_motor_input = new_config.live_output;
    state->firmware_prefetch_ready = false;
    state->active_firmware_dir[0] = '\0';
    memset(state->motors, 0, sizeof(state->motors));
    snprintf(state->config_path, sizeof(state->config_path), "%s", path);
    state->config_loaded = true;
    if (state->config.chinese_ui) {
        printf("已从 %s 加载 %zu 台电机\n", path, state->config.entry_count);
    } else {
        printf("loaded %zu motors from %s\n", state->config.entry_count, path);
    }
    return 0;
}

static int console_run_command(flash_state *state, int argc, char **argv)
{
    const char *cmd;
    unsigned int timeout_ms;

    if (argc == 0) {
        return 0;
    }
    cmd = argv[0];
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "-h") == 0 || strcmp(cmd, "?") == 0) {
        if (argc > 2 || (argc == 2 && strcmp(argv[1], "all") != 0)) {
            printf("%s: %s [all]\n", console_text(state, "用法", "usage"), cmd);
            return -1;
        }
        console_print_help(state->config.chinese_ui, argc == 2);
    } else if (strcmp(cmd, "language") == 0 || strcmp(cmd, "lang") == 0) {
        if (argc != 2 ||
            (strcmp(argv[1], "zh") != 0 && strcmp(argv[1], "en") != 0)) {
            printf("%s\n", console_text(state,
                   "用法：language zh|en",
                   "usage: language zh|en"));
            return -1;
        }
        state->config.chinese_ui = strcmp(argv[1], "zh") == 0;
        state->language_override_active = true;
        state->chinese_override = state->config.chinese_ui;
        printf("%s\n", state->config.chinese_ui ?
               "界面语言已切换为中文" :
               "Console language switched to English");
    } else if (strcmp(cmd, "power_on") == 0) {
        return console_maita_power_on(state, argc, argv);
    } else if (strcmp(cmd, "power_off") == 0) {
        return console_maita_power_off(state, argc, argv);
    } else if (strcmp(cmd, "maita_power_on") == 0) {
        return console_maita_power_on(state, argc, argv);
    } else if (strcmp(cmd, "maita_power_off") == 0) {
        return console_maita_power_off(state, argc, argv);
    } else if (strcmp(cmd, "maita_info") == 0) {
        return console_maita_info(state, argc, argv);
    } else if (strcmp(cmd, "maita_enable") == 0) {
        return console_maita_enable(state, argc, argv);
    } else if (strcmp(cmd, "maita_disable") == 0) {
        return console_maita_disable(state, argc, argv);
    } else if (strcmp(cmd, "maita_torque") == 0) {
        return console_maita_torque(state, argc, argv);
    } else if (strcmp(cmd, "maita_zero") == 0) {
        return console_maita_zero(state, argc, argv);
    } else if (strcmp(cmd, "maita_reset") == 0) {
        return console_maita_reset(state, argc, argv);
    } else if (strcmp(cmd, "maita_version") == 0) {
        return console_maita_version(state, argc, argv);
    } else if (strcmp(cmd, "maita_pid") == 0) {
        return console_maita_pid(state, argc, argv);
    } else if (strcmp(cmd, "maita_accel") == 0) {
        return console_maita_accel(state, argc, argv);
    } else if (strcmp(cmd, "maita_temp") == 0) {
        return console_maita_temp(state, argc, argv);
    } else if (strcmp(cmd, "maita_broadcast") == 0) {
        return console_maita_broadcast(state, argc, argv);
    } else if (strcmp(cmd, "maita_pos") == 0) {
        return console_maita_pos(state, argc, argv);
    } else if (strcmp(cmd, "motor_probe") == 0) {
        if (argc != 1) {
            printf("%s: motor_probe\n", console_text(state, "用法", "usage"));
            return -1;
        }
        return console_motor_probe(state);
    } else if (strcmp(cmd, "motor_scan") == 0) {
        timeout_ms = state->config.scan_timeout_ms;
        if (argc > 2 || (argc == 2 && parse_uint_arg(argv[1], &timeout_ms) != 0)) {
            printf("%s: motor_scan [timeout_ms]\n", console_text(state, "用法", "usage"));
            return -1;
        }
        return console_probe_motors(state, timeout_ms);
    } else if (strcmp(cmd, "motor_list") == 0) {
        if (argc != 1) {
            printf("%s: motor_list\n", console_text(state, "用法", "usage"));
            return -1;
        }
        console_print_motors(state);
    } else if (strcmp(cmd, "mit_set") == 0 ||
               strcmp(cmd, "maita_mit") == 0 ||
               strcmp(cmd, "motor_set") == 0) {
        return console_maita_mit_set(state, argc, argv);
    } else if (strcmp(cmd, "reg_read") == 0) {
        printf("reg_read disabled on maita_motor branch\n");
        return -1;
    } else if (strcmp(cmd, "reg_write") == 0) {
        printf("reg_write disabled on maita_motor branch\n");
        return -1;
    } else if (strcmp(cmd, "reg_save") == 0) {
        printf("reg_save disabled on maita_motor branch\n");
        return -1;
    } else if (strcmp(cmd, "reg_info") == 0) {
        printf("reg_info disabled on maita_motor branch\n");
        return -1;
    } else if (strcmp(cmd, "flash_single") == 0) {
        printf("flash disabled on maita_motor branch\n");
        return -1;
    } else if (strcmp(cmd, "flash_all") == 0) {
        printf("flash disabled on maita_motor branch\n");
        return -1;
    } else if (strcmp(cmd, "flash_debug") == 0) {
        printf("flash disabled on maita_motor branch\n");
        return -1;
    } else if (strcmp(cmd, "motor_dbg") == 0) {
        return console_motor_dbg(state, argc, argv);
    } else if (strcmp(cmd, "can_dbg") == 0 ||
               strcmp(cmd, "motor_reply") == 0 ||
               strcmp(cmd, "motor_can_dbg") == 0) {
        return console_motor_reply(state, argc, argv);
    } else if (strcmp(cmd, "can_status") == 0) {
        if (argc > 2 || (argc == 2 && strcmp(argv[1], "reset") != 0)) {
            printf("%s: can_status [reset]\n", console_text(state, "用法", "usage"));
            return -1;
        }
        console_can_status(state, argc == 2);
    } else if (strcmp(cmd, "q") == 0 ||
               strcmp(cmd, "qq") == 0 ||
               strcmp(cmd, "quit") == 0 ||
               strcmp(cmd, "exit") == 0) {
        if (state->motor_power_on) {
            if (console_power_off(state) != 0) {
                printf("%s\n", console_text(state,
                       "退出失败：自动下电未完成，请检查后重试",
                       "exit failed: automatic power_off did not complete; check and retry"));
                return -1;
            }
        }
        return CONSOLE_COMMAND_EXIT;
    } else {
        if (state->config.chinese_ui) {
            printf("未知命令：%s（使用 `help` 或 `-h` 查看帮助）\n", cmd);
        } else {
            printf("unknown command: %s (use `help` or `-h`)\n", cmd);
        }
        return -1;
    }
    return 0;
}

static int console_terminal(flash_state *state)
{
    char line[LINE_LEN];
    char *argv[16];

    if (state->config.chinese_ui) {
        printf("统一电机终端已就绪，尚未执行上电、控制或烧录操作。\n");
        printf("配置文件：%s（%zu 台电机）。使用 `help` 或 `-h` 查看帮助。\n",
               state->config_path, state->config.entry_count);
    } else {
        printf("Unified motor console ready. No power/control/flash action has been performed.\n");
        printf("Configuration: %s (%zu motors). Use `help` or `-h`.\n",
               state->config_path, state->config.entry_count);
    }
    while (!stop_requested) {
        int argc;
        int ret;
        const char *prompt;

        if (state->config.chinese_ui) {
            prompt = state->motor_power_on ? "电机[已上电]> " : "电机[已下电]> ";
        } else {
            prompt = state->motor_power_on ? "motor[POWER-ON]> " : "motor[POWER-OFF]> ";
        }

        printf("%s", prompt);
        fflush(stdout);
        if (read_line_with_completion(prompt, line, sizeof(line)) != 0) {
            printf("\n");
            break;
        }
        argc = split_line(line, argv, (int)(sizeof(argv) / sizeof(argv[0])));
        ret = console_run_command(state, argc, argv);
        if (ret == CONSOLE_COMMAND_EXIT) {
            return 0;
        }
    }
    return state->motor_power_on ? -1 : 0;
}
