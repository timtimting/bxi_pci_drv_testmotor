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
            printf("  电源/状态： power_on    power_off    motor_probe\n");
            printf("             motor_scan [timeout_ms]    motor_list    can_status [reset]\n\n");
            printf("  MIT控制：   mit_zero_set_all    mit_zero_set_single <index00>\n");
            printf("             mit_enable_all    mit_disable_all\n");
            printf("             mit_enable_single <index00>    mit_disable_single <index00>\n");
            printf("             mit_set <index00> <pos> <torque> <vel> <kp> <kd>    stand_up\n\n");
            printf("  寄存器：    reg_read <index00|all> <reg_index> [wait_ms]\n");
            printf("             reg_write <index00|all> <reg_index> <type> <value> [wait_ms]\n");
            printf("             reg_save <index00|all> [wait_ms]    reg_info <index00|all> [wait_ms]\n\n");
            printf("  烧录：      flash_single <index00> [version|firmware.bin] [cycle]\n");
            printf("             flash_all [plan.yaml] [cycle]\n");
            printf("             flash_debug <index00>|<bus0-4 id0-7> <version|firmware.bin> [cycle]\n\n");
            printf("  调试/退出： motor_dbg <index00>    can_dbg <index00> [wait_ms] [passive]\n");
            printf("             quit | exit | q | qq\n");
        } else {
            printf("Unified motor console commands (details: help all or -h all):\n");
            printf("  Help/lang:  help | -h | ?    language zh|en\n\n");
            printf("  Power/status: power_on    power_off    motor_probe\n");
            printf("                motor_scan [timeout_ms]    motor_list    can_status [reset]\n\n");
            printf("  MIT control:  mit_zero_set_all    mit_zero_set_single <index00>\n");
            printf("                mit_enable_all    mit_disable_all\n");
            printf("                mit_enable_single <index00>    mit_disable_single <index00>\n");
            printf("                mit_set <index00> <pos> <torque> <vel> <kp> <kd>    stand_up\n\n");
            printf("  Registers:    reg_read <index00|all> <reg_index> [wait_ms]\n");
            printf("                reg_write <index00|all> <reg_index> <type> <value> [wait_ms]\n");
            printf("                reg_save <index00|all> [wait_ms]    reg_info <index00|all> [wait_ms]\n\n");
            printf("  Flash:        flash_single <index00> [version|firmware.bin] [cycle]\n");
            printf("                flash_all [plan.yaml] [cycle]\n");
            printf("                flash_debug <index00>|<bus0-4 id0-7> <version|firmware.bin> [cycle]\n\n");
            printf("  Debug/exit:   motor_dbg <index00>    can_dbg <index00> [wait_ms] [passive]\n");
            printf("                quit | exit | q | qq\n");
        }
        return;
    }

    if (chinese) {
        printf("统一电机终端详细命令：\n");
        printf("  help | -h | ? [all]\n");
        printf("      默认显示精简分组命令；带 all 显示本详细说明。\n");
        printf("  language zh|en  （别名：lang）\n");
        printf("      在中文和英文界面之间即时切换。\n");
        printf("  power_on\n");
        printf("      电机总电源上电，等待软启动，扫描全部配置电机并输出回复序号。\n");
        printf("  power_off\n");
        printf("      先失能已知处于使能状态的电机，再关闭电机总电源。\n");
        printf("  motor_probe\n");
        printf("      上电、接收并显示配置电机回复，最后无论结果如何都会自动下电。\n");
        printf("  motor_scan [timeout_ms]\n");
        printf("      扫描所有 CAN 总线上的配置电机，不改变使能状态。\n");
        printf("  motor_list\n");
        printf("      按 MIT/index 顺序显示在线/使能状态和最后一次位置、速度、力矩、温度反馈。\n");
        printf("  mit_zero_set_all\n");
        printf("      给全部电机发送 MIT 零位校准帧；要求电机已上电并处于失能状态。\n");
        printf("  mit_zero_set_single <index00>\n");
        printf("      给指定逻辑序号的电机发送 MIT 零位校准帧。\n");
        printf("  mit_enable_all | mit_disable_all\n");
        printf("      使能或失能全部配置电机。\n");
        printf("  mit_enable_single <index00> | mit_disable_single <index00>\n");
        printf("      使能或失能指定电机。\n");
        printf("  mit_set <index00> <pos> <torque> <vel> <kp> <kd>\n");
        printf("      发送单次 MIT 控制帧；指定电机必须处于已使能状态。\n");
        printf("  stand_up\n");
        printf("      所有在线且已使能的电机运动到位置 0；KP 按配置缓慢增加。\n");
        printf("  reg_read <index00|all> <reg_index> [wait_ms]\n");
        printf("      读取寄存器；all 按默认烧录配置逐台发送。\n");
        printf("  reg_write <index00|all> <reg_index> <type> <value> [wait_ms]\n");
        printf("      写入寄存器；type 由用户指定，支持 int/bool/float/uint/version 或 0..4；\n");
        printf("      电机端负责校验地址和类型是否匹配；all 按默认烧录配置逐台发送。\n");
        printf("  reg_save <index00|all> [wait_ms]\n");
        printf("      保存寄存器配置；all 按默认烧录配置逐台发送；要求没有已知使能电机。\n");
        printf("  reg_info <index00|all> [wait_ms]\n");
        printf("      读取电机支持的寄存器总数和可写寄存器数量。\n");
        printf("  flash_single <index00> [version|firmware.bin] [cycle]\n");
        printf("      烧录单台电机；不写固件时使用默认烧录配置里的 version。\n");
        printf("  flash_all [plan.yaml] [cycle]\n");
        printf("      按烧录配置依次烧录多个电机；不写 plan 时使用 config/flash_plan_default.yaml。\n");
        printf("  flash_debug <index00>|<bus0-4 id0-7> <version|firmware.bin> [cycle]\n");
        printf("      两位数按 index，一位 bus+一位 id 按 Bus/ID；必须显式指定固件。\n");
        printf("  motor_dbg <index00>\n");
        printf("      按配置序号进入单电机直通调试；所有按键立即发送给电机，` 退出。\n");
        printf("  motor_reply <index00> [wait_ms] [passive]  （别名：can_dbg）\n");
        printf("      打印指定电机相关的原始 CAN 回复；默认发送一帧零 MIT 触发回复，passive 只监听。\n");
        printf("  can_status [reset]\n");
        printf("      显示各 Bus 的收发、发送失败、回复匹配、超时和估算丢包率；\n");
        printf("      reset 清零软件统计。公开驱动接口不提供硬件 TEC/REC。\n");
        printf("  quit | exit | q | qq\n");
        printf("      退出终端；如果电机仍上电，会先自动执行 power_off。\n");
        printf("安全限制：烧录固件名和 version 只能在 firmware_dir 内解析，不接受绝对路径、/ 或 ..。\n");
        return;
    }

    printf("Detailed unified motor console commands:\n");
    printf("  help | -h | ? [all]\n");
    printf("      Show concise grouped commands by default; all shows this detailed help.\n");
    printf("  language zh|en  (alias: lang)\n");
    printf("      Switch the console interface between Chinese and English.\n");
    printf("  power_on\n");
    printf("      Turn motor power on, wait for soft start, probe every configured motor,\n");
    printf("      and print the responding motor indexes.\n");
    printf("  power_off\n");
    printf("      Disable known enabled motors first, then turn motor power off.\n");
    printf("  motor_probe\n");
    printf("      Power on, receive and print configured motor replies, then always power off.\n");
    printf("  motor_scan [timeout_ms]\n");
    printf("      Probe every configured motor on all CAN buses without changing enable state.\n");
    printf("  motor_list\n");
    printf("      Show online/enabled state and latest position, velocity, torque and temperatures in index order.\n");
    printf("  mit_zero_set_all\n");
    printf("      Send the MIT zero-calibration frame to every configured motor. Motors must\n");
    printf("      be powered and disabled.\n");
    printf("  mit_zero_set_single <index00>\n");
    printf("      Send the MIT zero-calibration frame to one configured motor.\n");
    printf("  mit_enable_all | mit_disable_all\n");
    printf("      Send MIT enable/disable to every configured motor.\n");
    printf("  mit_enable_single <index00> | mit_disable_single <index00>\n");
    printf("      Send MIT enable/disable to one motor.\n");
    printf("  mit_set <index00> <pos> <torque> <vel> <kp> <kd>\n");
    printf("      Send one MIT control frame. The selected motor must be enabled.\n");
    printf("  stand_up\n");
    printf("      Command all online/enabled motors to position 0. KP ramps from zero to\n");
    printf("      home_kp over home_soft_start_ms; KP/KD come from the YAML config.\n");
    printf("  reg_read <index00|all> <reg_index> [wait_ms]\n");
    printf("      Read registers. all sends to every target in the default flash plan.\n");
    printf("  reg_write <index00|all> <reg_index> <type> <value> [wait_ms]\n");
    printf("      Write registers. type is provided by the user: int/bool/float/uint/version or 0..4.\n");
    printf("      The motor validates address/type matching. all uses the default flash plan.\n");
    printf("  reg_save <index00|all> [wait_ms]\n");
    printf("      Save register configs. all uses the default flash plan. Refused while any motor is enabled.\n");
    printf("  reg_info <index00|all> [wait_ms]\n");
    printf("      Read supported register count and writable register count.\n");
    printf("  flash_single <index00> [version|firmware.bin] [cycle]\n");
    printf("      Flash one motor. Without firmware, use the version from the default plan.\n");
    printf("  flash_all [plan.yaml] [cycle]\n");
    printf("      Flash multiple motors from a flash plan. Without plan, use config/flash_plan_default.yaml.\n");
    printf("  flash_debug <index00>|<bus0-4 id0-7> <version|firmware.bin> [cycle]\n");
    printf("      Two digits select index; one-digit bus + one-digit id select Bus/ID. Firmware is required.\n");
    printf("  motor_dbg <index00>\n");
    printf("      Enter one configured motor's pass-through debug mode by index. Every key\n");
    printf("      is sent immediately; ` exits debug mode.\n");
    printf("  motor_reply <index00> [wait_ms] [passive]  (alias: can_dbg)\n");
    printf("      Print raw CAN replies related to one motor. By default sends one zero MIT\n");
    printf("      frame first; passive only listens.\n");
    printf("  can_status [reset]\n");
    printf("      Show per-bus TX/RX, TX failures, expected replies and reply timeout rate.\n");
    printf("      These are software statistics; the public driver API exposes no TEC/REC.\n");
    printf("  quit | exit | q | qq\n");
    printf("      Exit. If motor power is still on, power_off is run automatically first.\n");
    printf("Safety: firmware names and versions are resolved only inside firmware_dir; absolute paths, / and .. are rejected.\n");
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
