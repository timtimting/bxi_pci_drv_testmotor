/*
 * Unified interactive motor service console.
 *
 * Reuse the proven terminal editor, motor protocol, boot-menu and YMODEM
 * implementation from motor_flash without changing the legacy executable.
 */
#define main motor_flash_embedded_main
#include "motor_flash.c"
#undef main

static const char *const console_command_words[] = {
    "help", "-h", "?", "power_on", "power_off", "motor_scan", "motor_list",
    "mit_zero_set", "mit_zero_set_single", "mit_enable_all", "mit_disable_all",
    "mit_enable_single", "mit_disable_single", "motor_set", "stand_up",
    "flash_single", "flash_all", "can_status", "can_monitor", "config_show",
    "config_reload", "language", "lang", "quit", "exit", "q",
};

static const char *console_text(const flash_state *state,
                                const char *chinese,
                                const char *english)
{
    return state->config.chinese_ui ? chinese : english;
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

static void console_print_help(bool chinese)
{
    if (chinese) {
        printf("统一电机终端命令：\n");
        printf("  help | -h | ?\n");
        printf("      显示全部命令、参数和安全说明。\n");
        printf("  language zh|en  （别名：lang）\n");
        printf("      在中文和英文界面之间即时切换。\n");
        printf("  power_on\n");
        printf("      电机总电源上电，等待软启动，扫描全部配置电机并输出回复序号。\n");
        printf("  power_off\n");
        printf("      先失能已知处于使能状态的电机，再关闭电机总电源。\n");
        printf("  motor_scan [timeout_ms]\n");
        printf("      扫描所有 CAN 总线上的配置电机，不改变使能状态。\n");
        printf("  motor_list\n");
        printf("      显示序号、Bus、CAN ID、型号、在线/使能状态和最后一次反馈。\n");
        printf("  mit_zero_set\n");
        printf("      给全部电机发送 MIT 零位校准帧；要求电机已上电并处于失能状态。\n");
        printf("  mit_zero_set_single <index>\n");
        printf("      给指定逻辑序号的电机发送 MIT 零位校准帧。\n");
        printf("  mit_enable_all | mit_disable_all\n");
        printf("      使能或失能全部配置电机。\n");
        printf("  mit_enable_single <index> | mit_disable_single <index>\n");
        printf("      使能或失能指定电机。\n");
        printf("  motor_set <index> <pos> <torque> <vel> <kp> <kd>\n");
        printf("      发送单次 MIT 控制帧；指定电机必须处于已使能状态。\n");
        printf("  stand_up\n");
        printf("      所有在线且已使能的电机运动到位置 0；KP 按配置缓慢增加。\n");
        printf("  flash_single <index> [cycle]\n");
        printf("      按电机型号选择固件并烧录单台电机；cycle 表示断电重启进入 Boot。\n");
        printf("  flash_all [cycle]\n");
        printf("      预检并烧录配置中的全部电机，最后输出成功/失败汇总。\n");
        printf("  can_status [reset]\n");
        printf("      显示各 Bus 的收发、发送失败、回复匹配、超时和估算丢包率；\n");
        printf("      reset 清零软件统计。公开驱动接口不提供硬件 TEC/REC。\n");
        printf("  can_monitor on|off\n");
        printf("      开启或关闭实时 CAN 输出和电机反馈解析。\n");
        printf("  config_show | config_reload [path]\n");
        printf("      显示或重载 YAML 配置；电机上电或使能时禁止重载。\n");
        printf("  quit | exit | q\n");
        printf("      退出终端；电机电源开启时必须先执行 `power_off`。\n");
        return;
    }

    printf("Unified motor console commands:\n");
    printf("  help | -h | ?\n");
    printf("      Show every command, parameter and safety note.\n");
    printf("  language zh|en  (alias: lang)\n");
    printf("      Switch the console interface between Chinese and English.\n");
    printf("  power_on\n");
    printf("      Turn motor power on, wait for soft start, probe every configured motor,\n");
    printf("      and print the responding motor indexes.\n");
    printf("  power_off\n");
    printf("      Disable known enabled motors first, then turn motor power off.\n");
    printf("  motor_scan [timeout_ms]\n");
    printf("      Probe every configured motor on all CAN buses without changing enable state.\n");
    printf("  motor_list\n");
    printf("      Show index, bus, CAN id, type, online/enabled state and last feedback.\n");
    printf("  mit_zero_set\n");
    printf("      Send the MIT zero-calibration frame to every configured motor. Motors must\n");
    printf("      be powered and disabled.\n");
    printf("  mit_zero_set_single <index>\n");
    printf("      Send the MIT zero-calibration frame to one configured motor.\n");
    printf("  mit_enable_all | mit_disable_all\n");
    printf("      Send MIT enable/disable to every configured motor.\n");
    printf("  mit_enable_single <index> | mit_disable_single <index>\n");
    printf("      Send MIT enable/disable to one motor.\n");
    printf("  motor_set <index> <pos> <torque> <vel> <kp> <kd>\n");
    printf("      Send one MIT control frame. The selected motor must be enabled.\n");
    printf("  stand_up\n");
    printf("      Command all online/enabled motors to position 0. KP ramps from zero to\n");
    printf("      home_kp over home_soft_start_ms; KP/KD come from the YAML config.\n");
    printf("  flash_single <index> [cycle]\n");
    printf("      Select firmware by motor type and flash one disabled motor. `cycle` uses\n");
    printf("      a power cycle instead of the normal application reset path.\n");
    printf("  flash_all [cycle]\n");
    printf("      Preflight and flash every configured motor; failures are summarized.\n");
    printf("  can_status [reset]\n");
    printf("      Show per-bus TX/RX, TX failures, expected replies and reply timeout rate.\n");
    printf("      These are software statistics; the public driver API exposes no TEC/REC.\n");
    printf("  can_monitor on|off\n");
    printf("      Enable/disable immediate decoded motor/CAN output while at the prompt.\n");
    printf("  config_show | config_reload [path]\n");
    printf("      Show or reload the unified YAML configuration. Reload is refused while\n");
    printf("      motor power is on or motors are enabled.\n");
    printf("  quit | exit | q\n");
    printf("      Exit. `power_off` is required first while motor power is on.\n");
}

static void console_print_config(const flash_state *state)
{
    size_t i;

    printf("%s=%s  %s=%s  %s=%zu  home_kp=%g home_kd=%g "
           "soft_start_ms=%u scan_timeout_ms=%u power_on_wait_ms=%u "
           "mit_canfd=%s live_output=%s language=%s\n",
           console_text(state, "配置文件", "config"),
           state->config_path,
           console_text(state, "固件目录", "firmware_dir"),
           state->config.firmware_dir,
           console_text(state, "电机数量", "motors"),
           state->config.entry_count,
           state->config.home_kp,
           state->config.home_kd,
           state->config.home_soft_start_ms,
           state->config.scan_timeout_ms,
           state->config.power_on_wait_ms,
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
}

static void console_print_motors(const flash_state *state)
{
    size_t i;

    printf("%s=%s  %s=%zu\n",
           console_text(state, "电机电源", "POWER"),
           state->motor_power_on ?
               console_text(state, "已上电", "ON") :
               console_text(state, "已下电", "OFF"),
           console_text(state, "配置电机数", "configured motors"),
           state->config.entry_count);
    for (i = 0u; i < state->config.entry_count; i++) {
        const motor_map_entry *m = &state->config.entries[i];
        const motor_runtime *r = &state->motors[i];

        printf("  index=%-3u bus=%u id=%-2u type=%-4s %s=%-3s %s=%-3s rx=%u",
               m->index, m->bus, m->id, m->type,
               console_text(state, "在线", "online"),
               r->online ? console_text(state, "是", "yes") : console_text(state, "否", "no"),
               console_text(state, "使能", "enabled"),
               r->enabled ? console_text(state, "是", "yes") : console_text(state, "否", "no"),
               r->rx_count);
        if (r->last_reply_us != 0u) {
            printf(" pos=% .4f vel=% .4f torque=% .4f temp=% .1f/% .1fC",
                   r->last_reply.position,
                   r->last_reply.velocity,
                   r->last_reply.torque,
                   r->last_reply.mos_temperature,
                   r->last_reply.motor_temperature);
        }
        printf("\n");
    }
}

static void console_record_timeouts(flash_state *state,
                                    const unsigned int *before,
                                    int selected_slot)
{
    size_t i;

    for (i = 0u; i < state->config.entry_count; i++) {
        const motor_map_entry *m = &state->config.entries[i];

        if (selected_slot >= 0 && i != (size_t)selected_slot) {
            continue;
        }
        if (state->motors[i].rx_count == before[i]) {
            state->motors[i].timeout_count++;
            state->can_stats[m->bus].reply_timeouts++;
            if (state->can_stats[m->bus].outstanding_replies > 0u) {
                state->can_stats[m->bus].outstanding_replies--;
            }
        }
    }
}

static int console_probe_motors(flash_state *state, unsigned int timeout_ms)
{
    unsigned int before[MOTOR_MAP_MAX];
    unsigned int old_bus = state->bus;
    unsigned int old_id = state->boot_id;
    uint64_t deadline;
    size_t i;
    size_t found = 0u;
    uint64_t timeout_before[CANFD_DEVICE_NUM];
    unsigned int bus;

    if (console_require_power(state) != 0) {
        return -1;
    }
    frame_ring_clear(&state->frames);
    for (bus = 0u; bus < CANFD_DEVICE_NUM; bus++) {
        timeout_before[bus] = state->can_stats[bus].reply_timeouts;
    }
    for (i = 0u; i < state->config.entry_count; i++) {
        before[i] = state->motors[i].rx_count;
        state->motors[i].online = false;
    }
    if (state->config.chinese_ui) {
        printf("正在通过 %u 路 CAN 总线探测 %zu 台配置电机……\n",
               CANFD_DEVICE_NUM, state->config.entry_count);
    } else {
        printf("probing %zu configured motors across %u CAN buses...\n",
               state->config.entry_count, CANFD_DEVICE_NUM);
    }
    for (i = 0u; i < state->config.entry_count; i++) {
        const motor_map_entry *m = &state->config.entries[i];

        console_use_motor(state, m);
        console_expect_reply(state, m->bus);
        send_debug_mit(state, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    state->bus = old_bus;
    state->boot_id = old_id;
    update_boot_ids(state);

    deadline = time_us() + (uint64_t)timeout_ms * 1000ULL;
    while (!stop_requested && time_us() < deadline) {
        sleep_ms(10u);
    }
    console_record_timeouts(state, before, -1);
    printf("%s:", console_text(state, "收到回复的电机序号", "responding motor indexes"));
    for (i = 0u; i < state->config.entry_count; i++) {
        if (state->motors[i].rx_count != before[i]) {
            state->motors[i].online = true;
            printf(" %u", state->config.entries[i].index);
            found++;
        }
    }
    if (state->config.chinese_ui) {
        printf("\n扫描结果：%zu/%zu 台电机在线\n", found, state->config.entry_count);
    } else {
        printf("\nscan result: %zu/%zu motors online\n", found, state->config.entry_count);
    }
    for (bus = 0u; bus < CANFD_DEVICE_NUM; bus++) {
        uint64_t new_timeouts = state->can_stats[bus].reply_timeouts - timeout_before[bus];

        if (new_timeouts != 0u || state->can_stats[bus].tx_failed != 0u) {
            printf("%s bus=%u %s=%llu %s=%llu\n",
                   console_text(state, "[CAN 警告]", "[CAN WARNING]"),
                   bus,
                   console_text(state, "本次回复超时", "new_reply_timeouts"),
                   (unsigned long long)new_timeouts,
                   console_text(state, "累计发送失败", "total_tx_failed"),
                   (unsigned long long)state->can_stats[bus].tx_failed);
        }
    }
    return found == 0u ? -1 : 0;
}

static int console_send_special(flash_state *state,
                                int selected_slot,
                                uint8_t command,
                                const char *name)
{
    unsigned int before[MOTOR_MAP_MAX];
    unsigned int old_bus = state->bus;
    unsigned int old_id = state->boot_id;
    size_t i;
    size_t sent = 0u;

    if (console_require_power(state) != 0) {
        return -1;
    }
    for (i = 0u; i < state->config.entry_count; i++) {
        before[i] = state->motors[i].rx_count;
        if (selected_slot >= 0 && i != (size_t)selected_slot) {
            continue;
        }
        if (command == BXI_MOTOR_CMD_ZERO && state->motors[i].enabled) {
            if (state->config.chinese_ui) {
                printf("%s 被拒绝：电机 index %u 仍处于使能状态，请先失能\n",
                       name, state->config.entries[i].index);
            } else {
                printf("%s refused: motor index %u is enabled; disable it first\n",
                       name, state->config.entries[i].index);
            }
            return -1;
        }
    }
    frame_ring_clear(&state->frames);
    for (i = 0u; i < state->config.entry_count; i++) {
        const motor_map_entry *m = &state->config.entries[i];

        if (selected_slot >= 0 && i != (size_t)selected_slot) {
            continue;
        }
        console_use_motor(state, m);
        console_expect_reply(state, m->bus);
        if (send_debug_special(state, command) == 0) {
            sent++;
        }
    }
    state->bus = old_bus;
    state->boot_id = old_id;
    update_boot_ids(state);
    sleep_ms(state->config.scan_timeout_ms);
    console_record_timeouts(state, before, selected_slot);

    for (i = 0u; i < state->config.entry_count; i++) {
        if (selected_slot >= 0 && i != (size_t)selected_slot) {
            continue;
        }
        if (command == BXI_MOTOR_CMD_ENABLE) {
            state->motors[i].enabled = state->motors[i].rx_count != before[i];
        } else if (command == BXI_MOTOR_CMD_DISABLE) {
            if (state->motors[i].rx_count != before[i]) {
                state->motors[i].enabled = false;
            }
        }
    }
    if (state->config.chinese_ui) {
        printf("已向 %zu 台电机发送 %s\n", sent, name);
    } else {
        printf("%s sent to %zu motor(s)\n", name, sent);
    }
    return sent == 0u ? -1 : 0;
}

static int console_power_on(flash_state *state)
{
    if (state->motor_power_on) {
        printf("%s\n", console_text(state,
               "电机电源已经开启，重新扫描电机",
               "motor power is already ON; rescanning motors"));
        return console_probe_motors(state, state->config.scan_timeout_ms);
    }
    if (motor_pwr_set(1u) < 0) {
        fprintf(stderr, "%s\n", console_text(state,
                "电机上电失败：motor_pwr_set(1)",
                "motor_pwr_set(1) failed"));
        return -1;
    }
    state->motor_power_on = true;
    if (state->config.chinese_ui) {
        printf("电机电源已开启，等待 %u ms 完成软启动\n",
               state->config.power_on_wait_ms);
    } else {
        printf("motor power ON; waiting %u ms for soft start\n",
               state->config.power_on_wait_ms);
    }
    sleep_ms(state->config.power_on_wait_ms);
    return console_probe_motors(state, state->config.scan_timeout_ms);
}

static int console_power_off(flash_state *state)
{
    size_t i;
    bool any_enabled = false;

    if (!state->motor_power_on) {
        printf("%s\n", console_text(state,
               "电机电源已经关闭", "motor power is already OFF"));
        return 0;
    }
    for (i = 0u; i < state->config.entry_count; i++) {
        any_enabled |= state->motors[i].enabled;
    }
    if (any_enabled) {
        printf("%s\n", console_text(state,
               "下电前先失能已使能电机",
               "disabling motors before power off"));
        console_send_special(state, -1, BXI_MOTOR_CMD_DISABLE, "mit_disable_all");
    }
    if (motor_pwr_set(0u) < 0) {
        fprintf(stderr, "%s\n", console_text(state,
                "电机下电失败：motor_pwr_set(0)",
                "motor_pwr_set(0) failed"));
        return -1;
    }
    state->motor_power_on = false;
    for (i = 0u; i < state->config.entry_count; i++) {
        state->motors[i].online = false;
        state->motors[i].enabled = false;
    }
    printf("%s\n", console_text(state, "电机电源已关闭", "motor power OFF"));
    return 0;
}

static int console_motor_set(flash_state *state, int argc, char **argv)
{
    unsigned int index;
    float pos, torque, vel, kp, kd;
    size_t slot;
    const motor_map_entry *motor;

    if (argc != 7 || parse_uint_arg(argv[1], &index) != 0 ||
        parse_float_arg(argv[2], &pos) != 0 ||
        parse_float_arg(argv[3], &torque) != 0 ||
        parse_float_arg(argv[4], &vel) != 0 ||
        parse_float_arg(argv[5], &kp) != 0 ||
        parse_float_arg(argv[6], &kd) != 0) {
        printf("%s: motor_set <index> <pos> <torque> <vel> <kp> <kd>\n",
               console_text(state, "用法", "usage"));
        return -1;
    }
    if (console_require_power(state) != 0) {
        return -1;
    }
    motor = console_motor_by_index(state, index, &slot);
    if (motor == NULL) {
        printf("%s: %u\n", console_text(state,
               "未知的电机序号", "unknown motor index"), index);
        return -1;
    }
    if (!state->motors[slot].enabled) {
        if (state->config.chinese_ui) {
            printf("motor_set 被拒绝：电机 index %u 尚未确认使能\n", index);
        } else {
            printf("motor_set refused: motor index %u is not known to be enabled\n", index);
        }
        return -1;
    }
    if (!isfinite(pos) || !isfinite(torque) || !isfinite(vel) ||
        !isfinite(kp) || !isfinite(kd) ||
        pos < state->limits.p_min || pos > state->limits.p_max ||
        vel < state->limits.v_min || vel > state->limits.v_max ||
        torque < state->limits.t_min || torque > state->limits.t_max ||
        kp < state->limits.kp_min || kp > state->limits.kp_max ||
        kd < state->limits.kd_min || kd > state->limits.kd_max) {
        printf("%s: "
               "p[%g,%g] torque[%g,%g] vel[%g,%g] kp[%g,%g] kd[%g,%g]\n",
               console_text(state,
                   "motor_set 被拒绝，参数超出 MIT 范围",
                   "motor_set refused: values exceed MIT limits"),
               state->limits.p_min, state->limits.p_max,
               state->limits.t_min, state->limits.t_max,
               state->limits.v_min, state->limits.v_max,
               state->limits.kp_min, state->limits.kp_max,
               state->limits.kd_min, state->limits.kd_max);
        return -1;
    }
    console_use_motor(state, motor);
    {
        unsigned int before[MOTOR_MAP_MAX] = {0u};
        uint64_t deadline;
        int send_ret;

        before[slot] = state->motors[slot].rx_count;
        console_expect_reply(state, motor->bus);
        send_ret = send_debug_mit(state, pos, vel, kp, kd, torque);
        deadline = time_us() + (uint64_t)state->config.scan_timeout_ms * 1000ULL;
        while (send_ret == 0 && !stop_requested && time_us() < deadline &&
               state->motors[slot].rx_count == before[slot]) {
            sleep_ms(5u);
        }
        console_record_timeouts(state, before, (int)slot);
        return send_ret;
    }
}

static int console_move_zero(flash_state *state)
{
    unsigned int old_bus = state->bus;
    unsigned int old_id = state->boot_id;
    const unsigned int period_ms = 10u;
    unsigned int steps;
    unsigned int step;
    unsigned int bus;
    size_t i;
    bool old_monitor = state->show_can_output;
    uint64_t pending_before[CANFD_DEVICE_NUM];

    if (console_require_power(state) != 0) {
        return -1;
    }
    for (i = 0u; i < state->config.entry_count; i++) {
        if (!state->motors[i].online || !state->motors[i].enabled) {
            if (state->config.chinese_ui) {
                printf("stand_up 被拒绝：index %u 未同时满足在线和使能状态\n",
                       state->config.entries[i].index);
            } else {
                printf("stand_up refused: index %u is not online and enabled\n",
                       state->config.entries[i].index);
            }
            return -1;
        }
    }
    steps = state->config.home_soft_start_ms / period_ms;
    if (steps == 0u) {
        steps = 1u;
    }
    if (state->config.chinese_ui) {
        printf("全部电机软启动回零：kp 0 -> %g，kd=%g，持续 %u ms\n",
               state->config.home_kp, state->config.home_kd,
               state->config.home_soft_start_ms);
    } else {
        printf("moving all motors to zero: kp 0 -> %g, kd=%g, duration=%u ms\n",
               state->config.home_kp, state->config.home_kd,
               state->config.home_soft_start_ms);
    }
    if (old_monitor) {
        printf("%s\n", console_text(state,
               "高频软启动期间暂时关闭实时回复打印",
               "live reply printing is temporarily suppressed during the high-rate ramp"));
        state->show_can_output = false;
    }
    for (bus = 0u; bus < CANFD_DEVICE_NUM; bus++) {
        pending_before[bus] = state->can_stats[bus].outstanding_replies;
    }
    for (step = 1u; step <= steps && !stop_requested; step++) {
        float kp = state->config.home_kp * (float)step / (float)steps;

        for (i = 0u; i < state->config.entry_count; i++) {
            const motor_map_entry *m = &state->config.entries[i];

            console_use_motor(state, m);
            console_expect_reply(state, m->bus);
            if (send_debug_mit(state, 0.0f, 0.0f, kp,
                               state->config.home_kd, 0.0f) != 0) {
                printf("%s index %u\n", console_text(state,
                       "stand_up 发送失败：", "stand_up send failed at"), m->index);
            }
        }
        sleep_ms(period_ms);
    }
    sleep_ms(100u);
    for (bus = 0u; bus < CANFD_DEVICE_NUM; bus++) {
        uint64_t pending = state->can_stats[bus].outstanding_replies;

        if (pending > pending_before[bus]) {
            uint64_t lost = pending - pending_before[bus];

            state->can_stats[bus].reply_timeouts += lost;
            state->can_stats[bus].outstanding_replies -= lost;
            if (state->config.chinese_ui) {
                printf("[CAN 警告] bus=%u 在 stand_up 期间丢失 %llu 个回复\n",
                       bus, (unsigned long long)lost);
            } else {
                printf("[CAN WARNING] bus=%u lost %llu reply/replies during stand_up\n",
                       bus, (unsigned long long)lost);
            }
        }
    }
    state->bus = old_bus;
    state->boot_id = old_id;
    update_boot_ids(state);
    state->show_can_output = old_monitor;
    printf("%s\n", console_text(state,
           "stand_up 软启动序列完成",
           "stand_up soft-start sequence complete"));
    return stop_requested ? -1 : 0;
}

static void console_can_status(flash_state *state, bool reset)
{
    unsigned int bus;

    if (reset) {
        memset(state->can_stats, 0, sizeof(state->can_stats));
        printf("%s\n", console_text(state,
               "CAN 软件统计已清零", "CAN software statistics reset"));
        return;
    }
    printf("%s\n", console_text(state,
           "CAN 软件状态（公开驱动接口不提供硬件 TEC/REC/bus-off）：",
           "CAN software status (hardware TEC/REC/bus-off unavailable in public API):"));
    for (bus = 0u; bus < CANFD_DEVICE_NUM; bus++) {
        const can_bus_stats *s = &state->can_stats[bus];
        double tx_fail_rate = s->tx_packets == 0u ? 0.0 :
                              100.0 * (double)s->tx_failed / (double)s->tx_packets;
        uint64_t missing = s->expected_replies > s->received_replies ?
                           s->expected_replies - s->received_replies : 0u;
        double loss_rate = s->expected_replies == 0u ? 0.0 :
                           100.0 * (double)missing / (double)s->expected_replies;

        if (state->config.chinese_ui) {
            printf("  bus=%u 发送=%llu 发送失败=%llu (%.2f%%) 接收=%llu "
                   "期望回复=%llu 匹配回复=%llu 电机帧=%llu 等待中=%llu "
                   "命令超时=%llu 估算丢包率=%.2f%%\n",
                   bus,
                   (unsigned long long)s->tx_packets,
                   (unsigned long long)s->tx_failed,
                   tx_fail_rate,
                   (unsigned long long)s->rx_packets,
                   (unsigned long long)s->expected_replies,
                   (unsigned long long)s->received_replies,
                   (unsigned long long)s->decoded_motor_replies,
                   (unsigned long long)s->outstanding_replies,
                   (unsigned long long)s->reply_timeouts,
                   loss_rate);
        } else {
            printf("  bus=%u tx=%llu tx_failed=%llu (%.2f%%) rx=%llu "
                   "expected=%llu matched=%llu decoded_motor=%llu pending=%llu "
                   "command_timeouts=%llu estimated_loss=%.2f%%\n",
                   bus,
                   (unsigned long long)s->tx_packets,
                   (unsigned long long)s->tx_failed,
                   tx_fail_rate,
                   (unsigned long long)s->rx_packets,
                   (unsigned long long)s->expected_replies,
                   (unsigned long long)s->received_replies,
                   (unsigned long long)s->decoded_motor_replies,
                   (unsigned long long)s->outstanding_replies,
                   (unsigned long long)s->reply_timeouts,
                   loss_rate);
        }
    }
}

static bool console_any_enabled(const flash_state *state)
{
    size_t i;

    for (i = 0u; i < state->config.entry_count; i++) {
        if (state->motors[i].enabled) {
            return true;
        }
    }
    return false;
}

static int console_flash(flash_state *state, int argc, char **argv, bool all)
{
    char *flash_argv[3];
    int flash_argc = 0;
    bool cycle = false;
    bool old_monitor;
    int ret;
    size_t flashed_slot = 0u;

    if (console_require_power(state) != 0) {
        return -1;
    }
    if (console_any_enabled(state)) {
        printf("%s\n", console_text(state,
               "烧录被拒绝：进入 Bootloader 前必须失能全部电机",
               "flash refused: disable all motors before entering bootloader"));
        return -1;
    }
    if ((!all && (argc < 2 || argc > 3)) || (all && argc > 2)) {
        printf("%s: %s%s\n", console_text(state, "用法", "usage"),
               all ? "flash_all" : "flash_single <index>", " [cycle]");
        return -1;
    }
    if ((!all && argc == 3 && strcmp(argv[2], "cycle") == 0) ||
        (all && argc == 2 && strcmp(argv[1], "cycle") == 0)) {
        cycle = true;
    } else if ((!all && argc == 3) || (all && argc == 2)) {
        printf("%s\n", console_text(state,
               "可选参数只能是 cycle", "optional argument must be: cycle"));
        return -1;
    }
    if (!all) {
        unsigned int index;
        const motor_map_entry *motor;

        if (parse_uint_arg(argv[1], &index) != 0 ||
            (motor = console_motor_by_index(state, index, &flashed_slot)) == NULL) {
            printf("%s: %s\n", console_text(state,
                   "未知的电机序号", "unknown motor index"), argv[1]);
            return -1;
        }
        console_use_motor(state, motor);
        flash_argv[flash_argc++] = "one";
    } else {
        flash_argv[flash_argc++] = "all";
    }
    flash_argv[flash_argc++] = state->config_path;
    if (cycle) {
        flash_argv[flash_argc++] = "cycle";
    }
    old_monitor = state->show_can_output;
    state->show_can_output = false;
    ret = run_flash_auto(state, flash_argc, flash_argv);
    state->show_can_output = old_monitor;
    if (ret != 0) {
        return -1;
    }
    if (all) {
        memset(state->motors, 0, sizeof(state->motors));
    } else {
        memset(&state->motors[flashed_slot], 0, sizeof(state->motors[flashed_slot]));
    }
    return 0;
}

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
    if (state->language_override_active) {
        new_config.chinese_ui = state->chinese_override;
    }
    state->config = new_config;
    state->limits = new_config.mit_limits;
    state->debug_use_canfd = new_config.mit_canfd;
    state->show_can_output = new_config.live_output;
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
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "-h") == 0 || strcmp(cmd, "?") == 0) {
        console_print_help(state->config.chinese_ui);
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
    } else if (strcmp(cmd, "mit_zero_set") == 0) {
        if (argc != 1) {
            printf("%s: mit_zero_set\n", console_text(state, "用法", "usage"));
            return -1;
        }
        return console_send_special(state, -1, BXI_MOTOR_CMD_ZERO, cmd);
    } else if (strcmp(cmd, "mit_zero_set_single") == 0 ||
               strcmp(cmd, "mit_enable_single") == 0 ||
               strcmp(cmd, "mit_disable_single") == 0) {
        uint8_t special = strcmp(cmd, "mit_zero_set_single") == 0 ? BXI_MOTOR_CMD_ZERO :
                          (strcmp(cmd, "mit_enable_single") == 0 ? BXI_MOTOR_CMD_ENABLE :
                           BXI_MOTOR_CMD_DISABLE);
        if (argc != 2 || parse_uint_arg(argv[1], &index) != 0 ||
            console_motor_by_index(state, index, &slot) == NULL) {
            printf("%s: %s <index>\n", console_text(state, "用法", "usage"), cmd);
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
    } else if (strcmp(cmd, "motor_set") == 0) {
        return console_motor_set(state, argc, argv);
    } else if (strcmp(cmd, "stand_up") == 0 || strcmp(cmd, "mit_move_zero") == 0) {
        if (argc != 1) {
            printf("%s: stand_up\n", console_text(state, "用法", "usage"));
            return -1;
        }
        return console_move_zero(state);
    } else if (strcmp(cmd, "flash_single") == 0) {
        return console_flash(state, argc, argv, false);
    } else if (strcmp(cmd, "flash_all") == 0) {
        return console_flash(state, argc, argv, true);
    } else if (strcmp(cmd, "can_status") == 0) {
        if (argc > 2 || (argc == 2 && strcmp(argv[1], "reset") != 0)) {
            printf("%s: can_status [reset]\n", console_text(state, "用法", "usage"));
            return -1;
        }
        console_can_status(state, argc == 2);
    } else if (strcmp(cmd, "can_monitor") == 0) {
        bool enabled;
        if (argc != 2 || parse_on_off(argv[1], &enabled) != 0) {
            printf("%s: can_monitor on|off\n", console_text(state, "用法", "usage"));
            return -1;
        }
        state->show_can_output = enabled;
        printf("%s %s\n", console_text(state, "CAN 实时输出", "CAN live output"),
               enabled ? "ON" : "OFF");
    } else if (strcmp(cmd, "config_show") == 0) {
        if (argc != 1) {
            printf("%s: config_show\n", console_text(state, "用法", "usage"));
            return -1;
        }
        console_print_config(state);
    } else if (strcmp(cmd, "config_reload") == 0) {
        if (argc > 2) {
            printf("%s: config_reload [path]\n", console_text(state, "用法", "usage"));
            return -1;
        }
        return console_reload_config(state, argc == 2 ? argv[1] : state->config_path);
    } else if (strcmp(cmd, "q") == 0 || strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) {
        if (state->motor_power_on) {
            printf("%s\n", console_text(state,
                   "退出被拒绝：电机仍处于上电状态，请先执行 `power_off`",
                   "exit refused: motor power is ON; run `power_off` first"));
            return -1;
        }
        return 1;
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
        if (ret > 0) {
            return 0;
        }
    }
    return state->motor_power_on ? -1 : 0;
}

int main(int argc, char **argv)
{
    const char *config_path = DEFAULT_FIRMWARE_MAP;
    flash_state state;
    int opt;
    int ret;
    bool check_config = false;
    bool show_help = false;
    const char *language_override = NULL;

    static const struct option options[] = {
        {"config", required_argument, NULL, 'c'},
        {"check-config", no_argument, NULL, 'C'},
        {"language", required_argument, NULL, 'l'},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0},
    };

    while ((opt = getopt_long(argc, argv, "c:Cl:h", options, NULL)) != -1) {
        if (opt == 'c') {
            config_path = optarg;
        } else if (opt == 'C') {
            check_config = true;
        } else if (opt == 'l') {
            if (strcmp(optarg, "zh") != 0 && strcmp(optarg, "en") != 0) {
                printf("用法：--language zh|en\n");
                return 1;
            }
            language_override = optarg;
        } else if (opt == 'h') {
            show_help = true;
        } else {
            printf("用法：%s [-c config.yaml] [--language zh|en] [--check-config]\n", argv[0]);
            return 1;
        }
    }
    if (show_help) {
        console_print_help(language_override != NULL && strcmp(language_override, "zh") == 0);
        return 0;
    }
    if (optind != argc) {
        printf("用法：%s [-c config.yaml] [--language zh|en] [--check-config]\n", argv[0]);
        return 1;
    }

    memset(&state, 0, sizeof(state));
    state.bus = 0u;
    state.boot_id = 1u;
    state.mode = TOOL_MODE_FLASH;
    state.debug_use_canfd = true;
    state.quiet_tx = true;
    state.show_can_output = true;
    state.limits = bxi_motor_default_limits;
    custom_command_words = console_command_words;
    custom_command_word_count = sizeof(console_command_words) /
                                sizeof(console_command_words[0]);
    update_boot_ids(&state);
    rx_ring_init(&state.rx);
    frame_ring_init(&state.frames);
    if (language_override != NULL) {
        state.language_override_active = true;
        state.chinese_override = strcmp(language_override, "zh") == 0;
    }
    if (console_reload_config(&state, config_path) != 0) {
        return 1;
    }
    if (check_config) {
        console_print_config(&state);
        console_print_motors(&state);
        printf("%s\n", console_text(&state,
               "配置检查通过，未初始化 PCI/CAN，也未操作电机",
               "configuration check passed; no PCI/CAN initialization performed"));
        return 0;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    if (bxi_pci_init(can_rx_callback, &state, -1) == -1) {
        fprintf(stderr, "%s\n", console_text(&state,
                "PCI/CAN 初始化失败：bxi_pci_init",
                "bxi_pci_init failed"));
        return 1;
    }
    state.pci_started = true;
    ret = console_terminal(&state);
    if (state.motor_power_on) {
        fprintf(stderr, "%s\n", console_text(&state,
                "警告：终端停止时电机仍处于上电状态，正在执行安全下电",
                "warning: console stopped while motor power state is ON; powering off"));
        console_power_off(&state);
    }
    bxi_pci_exit();
    return ret == 0 ? 0 : 1;
}
