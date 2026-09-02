/*
 * User-facing display and status printers for main.c.
 *
 * Included by main.c; do not add as a standalone CMake source.
 */

static void console_print_help(bool chinese, bool verbose)
{
    if (!verbose) {
        if (chinese) {
            printf("统一电机终端命令（详细说明：help all 或 -h all）：\n");
            printf("  帮助/语言： help | -h | ?    language zh|en\n\n");
            printf("  电源/状态： power_on [wait_ms]    power_off    can_status [reset]\n\n");
            printf("  脉塔调试：  maita_info <bus> <id> [timeout_ms] [torque_max_nm]\n");
            printf("             maita_enable <bus> <id>    maita_disable <bus> <id>\n");
            printf("             maita_torque <bus> <id> <iq_A>    maita_zero <bus> <id>    maita_reset <bus> <id>\n");
            printf("             maita_version <bus> <id>\n");
            printf("             maita_pos <bus> <id> <pos_rad> [kp] [kd]\n\n");
            printf("  MIT控制：   mit_set|maita_mit <bus> <id> <pos_rad> <torque_Nm> <vel_rad_s> <kp> <kd>\n\n");
            printf("  退出：      quit | exit | q | qq\n");
        } else {
            printf("Unified motor console commands (details: help all or -h all):\n");
            printf("  Help/lang:  help | -h | ?    language zh|en\n\n");
            printf("  Power/status: power_on [wait_ms]    power_off    can_status [reset]\n\n");
            printf("  Maita debug:   maita_info <bus> <id> [timeout_ms] [torque_max_nm]\n");
            printf("                 maita_enable <bus> <id>    maita_disable <bus> <id>\n");
            printf("                 maita_torque <bus> <id> <iq_A>    maita_zero <bus> <id>    maita_reset <bus> <id>\n");
            printf("                 maita_version <bus> <id>\n");
            printf("                 maita_pos <bus> <id> <pos_rad> [kp] [kd]\n\n");
            printf("  MIT control:  mit_set|maita_mit <bus> <id> <pos_rad> <torque_Nm> <vel_rad_s> <kp> <kd>\n\n");
            printf("  Exit:         quit | exit | q | qq\n");
        }
        return;
    }

    if (chinese) {
        printf("统一电机终端详细命令：\n");
        printf("  help | -h | ? [all]\n");
        printf("      默认显示精简分组命令；带 all 显示本详细说明。\n");
        printf("  language zh|en  （别名：lang）\n");
        printf("      在中文和英文界面之间即时切换。\n");
        printf("  power_on [wait_ms]\n");
        printf("      脉塔调试分支中只打开电机总电源并等待软启动，不扫描配置电机。\n");
        printf("  power_off\n");
        printf("      关闭电机总电源，不发送 BXI 失能帧。\n");
        printf("  maita_info <bus> <id> [timeout_ms] [torque_max_nm]\n");
        printf("      通过 0x400+ID 发送一帧脉塔运动模式零命令，并解析 0x500+ID 回复。\n");
        printf("  maita_enable <bus> <id>\n");
        printf("      发送 0x81 Stop/Hold 指令，测试电机进入保持状态。\n");
        printf("  maita_disable <bus> <id>\n");
        printf("      发送 0x80 Shutdown 指令，关闭电机输出。\n");
        printf("  maita_torque <bus> <id> <iq_A> [timeout_ms]\n");
        printf("      发送 0xA1 转矩电流闭环指令，iq_A 单位 A，协议单位 0.01A/LSB。\n");
        printf("  maita_zero <bus> <id> [timeout_ms]\n");
        printf("      发送 0x64，将当前多圈位置写入 ROM 作为零点；手册要求失能状态执行，复位后生效。\n");
        printf("  maita_reset <bus> <id> [wait_ms]\n");
        printf("      发送 0x76 系统复位；手册说明该命令无回复，默认等待 1000ms。\n");
        printf("  maita_version <bus> <id> [timeout_ms]\n");
        printf("      发送 0xB2 读取系统软件版本日期，回复 DATA[4..7] 为小端 uint32。\n");
        printf("  maita_pos <bus> <id> <pos_rad> [kp] [kd] [timeout_ms] [torque_max_nm]\n");
        printf("      使用 MIT 运动模式做位置闭环，默认 vel=0、torque=0、kp=20、kd=1。\n");
        printf("  mit_set|maita_mit <bus> <id> <pos_rad> <torque_Nm> <vel_rad_s> <kp> <kd> [timeout_ms] [torque_max_nm]\n");
        printf("      发送一帧 0x400+ID 脉塔运动模式控制帧，并解析 0x500+ID 回复。\n");
        printf("  can_status [reset]\n");
        printf("      显示各 Bus 的收发、发送失败、回复匹配、超时和估算丢包率；\n");
        printf("      reset 清零软件统计。公开驱动接口不提供硬件 TEC/REC。\n");
        printf("  quit | exit | q | qq\n");
        printf("      退出终端；如果电机仍上电，会先自动执行 power_off。\n");
        return;
    }

    printf("Detailed unified motor console commands:\n");
    printf("  help | -h | ? [all]\n");
    printf("      Show concise grouped commands by default; all shows this detailed help.\n");
    printf("  language zh|en  (alias: lang)\n");
    printf("      Switch the console interface between Chinese and English.\n");
    printf("  power_on [wait_ms]\n");
    printf("      In the Maita branch, only turn motor power on and wait; no configured motor scan.\n");
    printf("  power_off\n");
    printf("      Turn motor power off without sending BXI disable frames.\n");
    printf("  maita_info <bus> <id> [timeout_ms] [torque_max_nm]\n");
    printf("      Send one 0x400+ID motion-mode zero frame and decode the 0x500+ID reply.\n");
    printf("  maita_enable <bus> <id>\n");
    printf("      Send 0x81 Stop/Hold to test holding state.\n");
    printf("  maita_disable <bus> <id>\n");
    printf("      Send 0x80 Shutdown to turn off motor output.\n");
    printf("  maita_torque <bus> <id> <iq_A> [timeout_ms]\n");
    printf("      Send 0xA1 torque-current command. iq_A is in A; protocol scale is 0.01A/LSB.\n");
    printf("  maita_zero <bus> <id> [timeout_ms]\n");
    printf("      Send 0x64 to write current multi-turn position as zero; run disabled, reset to apply.\n");
    printf("  maita_reset <bus> <id> [wait_ms]\n");
    printf("      Send 0x76 system reset; no reply is expected, default wait is 1000ms.\n");
    printf("  maita_version <bus> <id> [timeout_ms]\n");
    printf("      Send 0xB2 to read system software version date; DATA[4..7] is little-endian uint32.\n");
    printf("  maita_pos <bus> <id> <pos_rad> [kp] [kd] [timeout_ms] [torque_max_nm]\n");
    printf("      Run MIT motion-mode position control; defaults: vel=0, torque=0, kp=20, kd=1.\n");
    printf("  mit_set|maita_mit <bus> <id> <pos_rad> <torque_Nm> <vel_rad_s> <kp> <kd> [timeout_ms] [torque_max_nm]\n");
    printf("      Send one 0x400+ID Maita motion-mode frame and decode the 0x500+ID reply.\n");
    printf("  can_status [reset]\n");
    printf("      Show per-bus TX/RX, TX failures, expected replies and reply timeout rate.\n");
    printf("      These are software statistics; the public driver API exposes no TEC/REC.\n");
    printf("  quit | exit | q | qq\n");
    printf("      Exit. If motor power is still on, power_off is run automatically first.\n");
}

static void console_print_config(const flash_state *state)
{
    size_t i;

    printf("%s=%s  %s=%s  firmware_sync_on_start=%s firmware_cache_dir=%s firmware_base_url=%s  %s=%zu  home_kp=%g home_kd=%g "
           "soft_start_ms=%u scan_timeout_ms=%u power_on_wait_ms=%u "
           "can_warmup_count=%u can_warmup_period_ms=%u "
           "mit_canfd=%s live_output=%s language=%s\n",
           console_text(state, "配置文件", "config"),
           state->config_path,
           console_text(state, "固件目录", "firmware_dir"),
           state->config.firmware_dir,
           state->config.firmware_sync_on_start ? "on" : "off",
           state->config.firmware_cache_dir,
           state->config.firmware_base_url[0] != '\0' ?
           state->config.firmware_base_url : "--",
           console_text(state, "电机数量", "motors"),
           state->config.entry_count,
           state->config.home_kp,
           state->config.home_kd,
           state->config.home_soft_start_ms,
           state->config.scan_timeout_ms,
           state->config.power_on_wait_ms,
           state->config.can_warmup_count,
           state->config.can_warmup_period_ms,
           state->config.mit_canfd ? "on" : "off",
           state->config.live_output ? "on" : "off",
           state->config.chinese_ui ? "zh" : "en");
    printf("  %s: p[%g,%g] v[%g,%g] torque[%g,%g] kp[0,%g] kd[0,%g]\n",
           console_text(state, "MIT 参数范围", "MIT limits"),
           state->config.mit_limits.p_min, state->config.mit_limits.p_max,
           state->config.mit_limits.v_min, state->config.mit_limits.v_max,
           state->config.mit_limits.t_min, state->config.mit_limits.t_max,
           state->config.mit_limits.kp_max, state->config.mit_limits.kd_max);
    for (i = 0u; i < state->config.firmware_mapping_count; i++) {
        printf("  %s type=%s file=%s\n",
               console_text(state, "固件", "firmware"),
               state->config.firmware_mappings[i].type,
               state->config.firmware_mappings[i].filename);
    }
    for (i = 0u; i < state->config.type_config_count; i++) {
        const motor_type_config *type_config = &state->config.type_configs[i];

        printf("  %s type=%s%s%s%s%s%s%s p[%g,%g] v[%g,%g] torque[%g,%g] kp[%g,%g] kd[%g,%g]\n",
               console_text(state, "型号", "type"),
               type_config->type,
               type_config->firmware[0] != '\0' ? " file=" : "",
               type_config->firmware[0] != '\0' ? type_config->firmware : "",
               type_config->firmware_v0[0] != '\0' ? " v0=" : "",
               type_config->firmware_v0[0] != '\0' ? type_config->firmware_v0 : "",
               type_config->firmware_v1[0] != '\0' ? " v1=" : "",
               type_config->firmware_v1[0] != '\0' ? type_config->firmware_v1 : "",
               type_config->limits.p_min, type_config->limits.p_max,
               type_config->limits.v_min, type_config->limits.v_max,
               type_config->limits.t_min, type_config->limits.t_max,
               type_config->limits.kp_min, type_config->limits.kp_max,
               type_config->limits.kd_min, type_config->limits.kd_max);
    }
}

static void console_print_motors(const flash_state *state)
{
    uint64_t now = time_us();
    unsigned int last_bus = CANFD_DEVICE_NUM;
    size_t i;

    printf("%s=%s  %s=%zu\n",
           console_text(state, "电机电源", "POWER"),
           state->motor_power_on ?
               console_text(state, "已上电", "ON") :
               console_text(state, "已下电", "OFF"),
           console_text(state, "配置电机数", "configured motors"),
           state->config.entry_count);
    if (state->config.chinese_ui) {
        printf("idx bus id type state      on en fw    cfg   age(ms) pos(rad)   vel(rad/s) torque     ntc1(C) ntc2(C) vbus(V) hb\n");
    } else {
        printf("idx bus id type state      on en fw    cfg   age(ms) pos(rad)   vel(rad/s) torque     ntc1(C) ntc2(C) vbus(V) hb\n");
    }
    for (i = 0u; i < state->config.entry_count; i++) {
        const motor_map_entry *m = &state->config.entries[i];
        const motor_runtime *r = &state->motors[i];
        const char *state_label = strcmp(motor_runtime_state(r), "unknown") == 0 ?
                                  "--" : motor_runtime_state(r);
        const char *online = r->online ? "Y" : "N";
        const char *enabled = r->enabled ? "Y" : "N";
        char fw_version[16];
        char config_version[16];

        if (r->fw_version_valid) {
            snprintf(fw_version, sizeof(fw_version), "%u.%u",
                     r->fw_version_major, r->fw_version_minor);
        } else {
            snprintf(fw_version, sizeof(fw_version), "--");
        }
        if (r->config_version_valid) {
            snprintf(config_version, sizeof(config_version), "%u.%u",
                     (unsigned int)((r->config_version >> 8) & 0xffu),
                     (unsigned int)(r->config_version & 0xffu));
        } else {
            snprintf(config_version, sizeof(config_version), "--");
        }

        if (i != 0u && m->bus != last_bus) {
            printf("\n");
        }
        last_bus = m->bus;

        if (r->last_reply_us == 0u) {
            printf("%02u  %u   %-2u %-4s %-10s %-2s %-2s %-5s %-5s --      --         --         --         --      --      --      --\n",
                   m->index, m->bus, m->id, m->type,
                   state_label,
                   online,
                   enabled,
                   fw_version,
                   config_version);
            continue;
        }
        printf("%02u  %u   %-2u %-4s %-10s %-2s %-2s %-5s %-5s %-7llu % .5f  % .5f  % .5f",
               m->index, m->bus, m->id, m->type,
               state_label,
               online,
               enabled,
               fw_version,
               config_version,
               (unsigned long long)((now - r->last_reply_us) / 1000ULL),
               r->last_reply.position,
               r->last_reply.velocity,
               r->last_reply.torque);
        if (!r->mit_aux_enabled) {
            printf("  % .1f", r->last_reply.mos_temperature);
        } else if (r->aux_valid[0]) {
            printf("  % .1f", r->aux_value[0]);
        } else {
            printf("  --    ");
        }
        if (!r->mit_aux_enabled) {
            printf("  % .1f", r->last_reply.motor_temperature);
        } else if (r->aux_valid[1]) {
            printf("  % .1f", r->aux_value[1]);
        } else {
            printf("  --    ");
        }
        if (r->aux_valid[3]) {
            printf("  % .2f", r->aux_value[3]);
        } else {
            printf("  --    ");
        }
        if (r->aux_valid[15]) {
            printf("  %.0f", r->aux_value[15]);
        } else {
            printf("  --");
        }
        printf("\n");
    }
}
