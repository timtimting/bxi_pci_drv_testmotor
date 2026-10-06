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
    unsigned int index;
    unsigned int timeout_ms;
    size_t slot;

    if (argc == 0) {
        return 0;
    }
    cmd = argv[0];
    if (console_ethercat_background_is_running()) {
        if (strcmp(cmd, "ethercat_enable") == 0) {
            printf("%s\n", console_text(state,
                   "EtherCAT 后台主站正在运行；请先执行 ethercat_disable，避免同时启动主站。",
                   "EtherCAT background master is running; use ethercat_disable before starting another master."));
            return -1;
        }
        if (strncmp(cmd, "ethercat_", 9u) == 0 &&
            strcmp(cmd, "ethercat_disable") != 0 &&
            strcmp(cmd, "ethercat_info") != 0) {
            if (console_ethercat_background_stop() > 0) {
                printf("%s\n", console_text(state,
                       "为交接 EtherCAT 主站，已先失能后台控制；正在执行本条命令。",
                       "Disabled the background drive before handing off the EtherCAT master to this command."));
            }
        } else if (strcmp(cmd, "power_off") == 0 || strcmp(cmd, "exit") == 0 ||
                   strcmp(cmd, "quit") == 0 || strcmp(cmd, "q") == 0 ||
                   strcmp(cmd, "qq") == 0) {
            console_ethercat_background_stop();
        }
    }
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
        if (argc != 1) {
            printf("%s: power_on\n", console_text(state, "用法", "usage"));
            return -1;
        }
        return console_power_on(state);
    } else if (strcmp(cmd, "power_off") == 0) {
        if (argc != 1) {
            printf("%s: power_off\n", console_text(state, "用法", "usage"));
            return -1;
        }
        return console_power_off(state);
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
    } else if (strcmp(cmd, "ethercat_scan") == 0) {
        if (argc > 2) {
            printf("%s: ethercat_scan [network_interface]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
        return console_ethercat_scan(state->config.chinese_ui,
                                     argc == 2 ? argv[1] : ETHERCAT_DEFAULT_INTERFACE);
    } else if (strcmp(cmd, "ethercat_enable") == 0) {
        unsigned int hold_ms = 0u;
        const char *sync0_shift_ns = NULL;
        const char *sync0_cycle_ms = NULL;
        const char *interface = ETHERCAT_DEFAULT_INTERFACE;
        unsigned int slave_id;
        bool all_slaves;
        int first_argument = 1;
        int last_argument = argc;
        unsigned int positional_count = 0u;
        char *shift_end;
        bool new_order;
        bool interface_set = false;

        if (argc < 2 || argc > 6) {
            printf("%s: ethercat_enable <slave_id|all> [hold_ms] [sync0_shift_ns] [sync0_cycle_ms=<0.5|1|2|4|5|8>] [network_interface]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
        new_order = console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) == 0;
        if (!new_order) {
            if (argc < 3 || argc > 6) {
                printf("%s: ethercat_enable <slave_id|all> [hold_ms] [sync0_shift_ns] [sync0_cycle_ms=<0.5|1|2|4|5|8>] [network_interface]\n",
                       console_text(state, "用法", "usage"));
                return -1;
            }
            interface = argv[1];
            interface_set = true;
            first_argument = 2;
            last_argument = argc;
        }
        for (int argument = first_argument + 1; argument < last_argument; argument++) {
            uint32_t parsed_cycle_ns;

            if (strncmp(argv[argument], "sync0_cycle_ms=", 15u) == 0) {
                if (sync0_cycle_ms != NULL) {
                    printf("%s: sync0_cycle_ms 只能指定一次\n", console_text(state, "错误", "error"));
                    return -1;
                }
                sync0_cycle_ms = argv[argument];
                continue;
            }
            errno = 0;
            (void)strtol(argv[argument], &shift_end, 10);
            if (errno != 0 || shift_end == argv[argument] || *shift_end != '\0') {
                if (interface_set || argument != last_argument - 1) {
                    printf("%s: ethercat_enable <slave_id|all> [hold_ms] [sync0_shift_ns] [sync0_cycle_ms=<0.5|1|2|4|5|8>] [network_interface]\n",
                           console_text(state, "用法", "usage"));
                    return -1;
                }
                interface = argv[argument];
                interface_set = true;
                continue;
            }
            if (positional_count == 0u) {
                if (parse_uint_arg(argv[argument], &hold_ms) != 0) {
                    return -1;
                }
            } else if (positional_count == 1u) {
                sync0_shift_ns = argv[argument];
            } else if (positional_count == 2u && sync0_cycle_ms == NULL &&
                       console_ethercat_parse_cycle_ms(argv[argument], &parsed_cycle_ns) == 0) {
                sync0_cycle_ms = argv[argument];
            } else {
                printf("%s: ethercat_enable <slave_id|all> [hold_ms] [sync0_shift_ns] [sync0_cycle_ms=<0.5|1|2|4|5|8>] [network_interface]\n",
                       console_text(state, "用法", "usage"));
                return -1;
            }
            positional_count++;
        }
        if (positional_count > 3u) {
            printf("%s: ethercat_enable <slave_id|all> [hold_ms] [sync0_shift_ns] [sync0_cycle_ms=<0.5|1|2|4|5|8>] [network_interface]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
        return console_ethercat_background_start(state->config.chinese_ui,
                                                 interface,
                                                 argv[first_argument],
                                                 hold_ms,
                                                 sync0_shift_ns,
                                                 sync0_cycle_ms);
    } else if (strcmp(cmd, "ethercat_disable") == 0) {
        const char *interface = ETHERCAT_DEFAULT_INTERFACE;
        const char *selection;
        unsigned int slave_id;
        bool all_slaves;

        if (argc == 2 && console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) == 0) {
            selection = argv[1];
        } else if (argc == 3) {
            interface = argv[1];
            selection = argv[2];
        } else {
            printf("%s: ethercat_disable <slave_id|all> [network_interface]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
        if (argc == 3 && console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) == 0) {
            interface = argv[2];
            selection = argv[1];
        }
        {
            int background_disable = console_ethercat_background_disable(interface, selection);

            if (background_disable < 0) {
                printf("%s\n", console_text(state,
                       "失能目标与后台 EtherCAT 主站的网卡或从站不匹配；后台循环保持运行。",
                       "Disable target does not match the background master's interface or slave; the cycle remains active."));
                return -1;
            }
            if (background_disable > 0) {
                printf("%s\n", console_text(state,
                       "后台 EtherCAT 循环已停止并发送失能。",
                       "Background EtherCAT cycle stopped and disable sent."));
                return 0;
            }
        }
        return console_ethercat_background_disable_start(state->config.chinese_ui,
                                                         interface,
                                                         selection);
    } else if (strcmp(cmd, "ethercat_position") == 0) {
        double position_rad;
        unsigned int hold_ms = 0u;
        const char *interface = ETHERCAT_DEFAULT_INTERFACE;
        const char *selection;
        unsigned int slave_id;
        bool all_slaves;
        int last_argument = argc;
        char *target_end;
        bool new_order = argc >= 3 &&
            console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) == 0;

        if (!new_order && (argc != 4 && argc != 5)) {
            printf("%s: ethercat_position <slave_id|all> <target_rad> [hold_ms] [network_interface]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
        if (new_order) {
            selection = argv[1];
            if (last_argument > 3) {
                errno = 0;
                (void)strtol(argv[last_argument - 1], &target_end, 10);
                if (errno != 0 || target_end == argv[last_argument - 1] || *target_end != '\0') {
                    interface = argv[--last_argument];
                }
            }
            if (console_ethercat_parse_position_rad(argv[2], &position_rad) != 0 ||
                (last_argument > 3 && parse_uint_arg(argv[3], &hold_ms) != 0) ||
                last_argument > 4) {
                printf("%s: ethercat_position <slave_id|all> <target_rad> [hold_ms] [network_interface]\n",
                       console_text(state, "用法", "usage"));
                return -1;
            }
        } else {
            interface = argv[1];
            selection = argv[2];
            if (console_ethercat_parse_position_rad(argv[3], &position_rad) != 0 ||
                (argc == 5 && parse_uint_arg(argv[4], &hold_ms) != 0)) {
                printf("%s: ethercat_position <slave_id|all> <target_rad> [hold_ms] [network_interface]\n",
                       console_text(state, "用法", "usage"));
                return -1;
            }
        }
        return console_ethercat_position(state->config.chinese_ui, interface, selection,
                                         position_rad, hold_ms);
    } else if (strcmp(cmd, "ethercat_zero") == 0) {
        const char *interface = ETHERCAT_DEFAULT_INTERFACE;
        const char *selection;
        unsigned int slave_id;
        bool all_slaves;

        if (argc == 2 && console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) == 0) {
            selection = argv[1];
        } else if (argc == 3 &&
                   console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) == 0) {
            selection = argv[1];
            interface = argv[2];
        } else if (argc == 3) {
            interface = argv[1];
            selection = argv[2];
        } else {
            printf("%s: ethercat_zero <slave_id|all> [network_interface]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
        return console_ethercat_zero(state->config.chinese_ui, interface, selection);
    } else if (strcmp(cmd, "ethercat_info") == 0) {
        const char *interface = ETHERCAT_DEFAULT_INTERFACE;
        const char *selection;
        unsigned int slave_id;
        bool all_slaves;

        if (argc == 2 && console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) == 0) {
            selection = argv[1];
        } else if (argc == 3 &&
                   console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) == 0) {
            selection = argv[1];
            interface = argv[2];
        } else if (argc == 3) {
            interface = argv[1];
            selection = argv[2];
        } else {
            printf("%s: ethercat_info <slave_id|all> [network_interface]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
        return console_ethercat_info(state->config.chinese_ui, interface, selection);
    } else if (strcmp(cmd, "ethercat_pnread") == 0) {
        const char *interface = ETHERCAT_DEFAULT_INTERFACE;
        const char *selection;
        uint16_t pn_number;
        unsigned int slave_id;
        bool all_slaves;

        if (argc == 3 &&
            console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) == 0) {
            selection = argv[1];
            if (console_ethercat_parse_pn_number(argv[2], &pn_number) != 0) {
                printf("%s: ethercat_pnread <slave_id|all> <Pn编号> [network_interface]\n",
                       console_text(state, "用法", "usage"));
                return -1;
            }
        } else if (argc == 4 &&
                   console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) == 0) {
            selection = argv[1];
            if (console_ethercat_parse_pn_number(argv[2], &pn_number) != 0) {
                printf("%s: ethercat_pnread <slave_id|all> <Pn编号> [network_interface]\n",
                       console_text(state, "用法", "usage"));
                return -1;
            }
            interface = argv[3];
        } else if (argc == 4 &&
                   console_ethercat_parse_slave_selection(argv[2], &slave_id, &all_slaves) == 0) {
            interface = argv[1];
            selection = argv[2];
            if (console_ethercat_parse_pn_number(argv[3], &pn_number) != 0) {
                printf("%s: ethercat_pnread <slave_id|all> <Pn编号> [network_interface]\n",
                       console_text(state, "用法", "usage"));
                return -1;
            }
        } else {
            printf("%s: ethercat_pnread <slave_id|all> <Pn编号> [network_interface]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
        return console_ethercat_pnread(state->config.chinese_ui,
                                       interface, selection, pn_number);
    } else if (strcmp(cmd, "ethercat_pn077") == 0) {
        unsigned int value;
        const char *interface = ETHERCAT_DEFAULT_INTERFACE;
        const char *selection;
        unsigned int slave_id;
        bool all_slaves;

        if ((argc != 3 && argc != 4) ||
            (argc == 4 && console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) != 0)) {
            if (argc != 4 || console_ethercat_parse_slave_selection(argv[2], &slave_id, &all_slaves) != 0) {
                printf("%s: ethercat_pn077 <slave_id|all> <0|1> [network_interface]\n",
                       console_text(state, "用法", "usage"));
                return -1;
            }
            interface = argv[1];
            selection = argv[2];
            if (parse_uint_arg(argv[3], &value) != 0 || value > 1u) {
                printf("%s: ethercat_pn077 <slave_id|all> <0|1> [network_interface]\n",
                       console_text(state, "用法", "usage"));
                return -1;
            }
        } else {
            selection = argv[1];
            if (parse_uint_arg(argv[2], &value) != 0 || value > 1u) {
                printf("%s: ethercat_pn077 <slave_id|all> <0|1> [network_interface]\n",
                       console_text(state, "用法", "usage"));
                return -1;
            }
            if (argc == 4) {
                interface = argv[3];
            }
        }
        return console_ethercat_pn077(state->config.chinese_ui,
                                      interface, selection, value);
    } else if (strcmp(cmd, "ethercat_save") == 0) {
        const char *interface = ETHERCAT_DEFAULT_INTERFACE;
        const char *selection;
        unsigned int slave_id;
        bool all_slaves;

        if (argc == 2 && console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) == 0) {
            selection = argv[1];
        } else if (argc == 3 &&
                   console_ethercat_parse_slave_selection(argv[1], &slave_id, &all_slaves) == 0) {
            selection = argv[1];
            interface = argv[2];
        } else if (argc == 3) {
            interface = argv[1];
            selection = argv[2];
        } else {
            printf("%s: ethercat_save <slave_id|all> [network_interface]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
        return console_ethercat_save(state->config.chinese_ui, interface, selection);
    } else if (strcmp(cmd, "mit_zero_set_all") == 0 || strcmp(cmd, "mit_zero_set") == 0) {
        if (argc != 1) {
            printf("%s: mit_zero_set_all\n", console_text(state, "用法", "usage"));
            return -1;
        }
        return console_send_special(state, -1, BXI_MOTOR_CMD_ZERO, "mit_zero_set_all");
    } else if (strcmp(cmd, "mit_zero_set_single") == 0 ||
               strcmp(cmd, "mit_enable_single") == 0 ||
               strcmp(cmd, "mit_disable_single") == 0) {
        uint8_t special = strcmp(cmd, "mit_zero_set_single") == 0 ? BXI_MOTOR_CMD_ZERO :
                          (strcmp(cmd, "mit_enable_single") == 0 ? BXI_MOTOR_CMD_ENABLE :
                           BXI_MOTOR_CMD_DISABLE);
        if (argc != 2 || console_parse_index_arg(argv[1], &index) != 0 ||
            console_motor_by_index(state, index, &slot) == NULL) {
            printf("%s: %s <index00>\n", console_text(state, "用法", "usage"), cmd);
            return -1;
        }
        return console_send_special(state, (int)slot, special, cmd);
    } else if (strcmp(cmd, "enable") == 0 || strcmp(cmd, "disable") == 0) {
        uint8_t special = strcmp(cmd, "enable") == 0 ? BXI_MOTOR_CMD_ENABLE :
                          BXI_MOTOR_CMD_DISABLE;

        if (argc == 1 || (argc == 2 && strcmp(argv[1], "all") == 0)) {
            return console_send_special(state, -1, special, cmd);
        }
        if (argc != 2 || console_parse_index_arg(argv[1], &index) != 0 ||
            console_motor_by_index(state, index, &slot) == NULL) {
            printf("%s: %s [<index00>|all]\n", console_text(state, "用法", "usage"), cmd);
            return -1;
        }
        return console_send_special(state, (int)slot, special, cmd);
    } else if (strcmp(cmd, "mit_enable_all") == 0) {
        if (argc != 1) {
            printf("%s: mit_enable_all\n", console_text(state, "用法", "usage"));
            return -1;
        }
        return console_send_special(state, -1, BXI_MOTOR_CMD_ENABLE, cmd);
    } else if (strcmp(cmd, "mit_disable_all") == 0) {
        if (argc != 1) {
            printf("%s: mit_disable_all\n", console_text(state, "用法", "usage"));
            return -1;
        }
        return console_send_special(state, -1, BXI_MOTOR_CMD_DISABLE, cmd);
    } else if (strcmp(cmd, "mit_set") == 0 || strcmp(cmd, "motor_set") == 0 ||
               strcmp(cmd, "mit") == 0) {
        return console_motor_set(state, argc, argv);
    } else if (strcmp(cmd, "stand_up") == 0 || strcmp(cmd, "mit_move_zero") == 0) {
        if (argc != 1) {
            printf("%s: stand_up\n", console_text(state, "用法", "usage"));
            return -1;
        }
        return console_move_zero(state);
    } else if (strcmp(cmd, "reg_read") == 0) {
        return console_reg_command(state, argc, argv, CAN_CMD_REG_READ);
    } else if (strcmp(cmd, "reg_write") == 0) {
        return console_reg_command(state, argc, argv, CAN_CMD_REG_WRITE);
    } else if (strcmp(cmd, "reg_save") == 0) {
        return console_reg_command(state, argc, argv, CAN_CMD_REG_SAVE);
    } else if (strcmp(cmd, "reg_info") == 0) {
        return console_reg_command(state, argc, argv, CAN_CMD_REG_INFO);
    } else if (strcmp(cmd, "flash_single") == 0) {
        return console_flash(state, argc, argv, false);
    } else if (strcmp(cmd, "flash_all") == 0) {
        return console_flash_all(state, argc, argv);
    } else if (strcmp(cmd, "flash_debug") == 0) {
        return console_flash_debug(state, argc, argv);
    } else if (strcmp(cmd, "motor_dbg") == 0 || strcmp(cmd, "debug") == 0) {
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
            console_ethercat_background_stop();
            return 0;
        }
    }
    console_ethercat_background_stop();
    return state->motor_power_on ? -1 : 0;
}
