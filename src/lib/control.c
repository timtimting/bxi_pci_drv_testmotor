/*
 * Power, MIT control, scanning and CAN statistics commands for main.c.
 *
 * Included by main.c; do not add as a standalone CMake source.
 */

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

static int load_flash_plan_config(const char *path, flash_plan_config *plan);
static const motor_map_entry *console_motor_by_bus_id(flash_state *state,
                                                       unsigned int bus,
                                                       unsigned int id,
                                                       size_t *slot);

static int console_probe_motors(flash_state *state, unsigned int timeout_ms)
{
    unsigned int before[MOTOR_MAP_MAX];
    unsigned int old_bus = state->bus;
    unsigned int old_id = state->boot_id;
    bool old_monitor = state->show_can_output;
    bool old_input = state->show_motor_input;
    int saved_stdout = -1;
    int saved_stderr = -1;
    bool output_silenced = false;
    uint64_t deadline;
    size_t i;
    size_t found = 0u;
    uint64_t timeout_before[CANFD_DEVICE_NUM];
    unsigned int bus;

    if (console_require_power(state) != 0) {
        return -1;
    }
    state->show_can_output = false;
    state->show_motor_input = false;
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
               (unsigned int)CANFD_DEVICE_NUM, state->config.entry_count);
    } else {
        printf("probing %zu configured motors across %u CAN buses...\n",
               state->config.entry_count, (unsigned int)CANFD_DEVICE_NUM);
    }
    if (console_silence_process_output(&saved_stdout, &saved_stderr) == 0) {
        output_silenced = true;
    }
    for (i = 0u; i < state->config.entry_count; i++) {
        const motor_map_entry *m = &state->config.entries[i];

        console_use_motor(state, m);
        console_expect_reply(state, m->bus);
        send_debug_mit(state, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        sleep_ms(5u);
    }
    if (output_silenced) {
        console_restore_process_output(saved_stdout, saved_stderr);
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
            printf(" %02u", state->config.entries[i].index);
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
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
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
                printf("%s 被拒绝：电机 index %02u 仍处于使能状态，请先失能\n",
                       name, state->config.entries[i].index);
            } else {
                printf("%s refused: motor index %02u is enabled; disable it first\n",
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

static int console_can_warmup(flash_state *state,
                              unsigned int count,
                              unsigned int period_ms,
                              bool online_only)
{
    unsigned int old_bus;
    unsigned int old_id;
    bool old_monitor;
    bool old_input;
    int saved_stdout = -1;
    int saved_stderr = -1;
    bool output_silenced = false;
    unsigned int sent = 0u;
    unsigned int before_rx = 0u;
    size_t candidate_count = 0u;
    size_t cursor = 0u;
    size_t i;

    if (state == NULL || state->config.entry_count == 0u || count == 0u) {
        return 0;
    }
    old_bus = state->bus;
    old_id = state->boot_id;
    old_monitor = state->show_can_output;
    old_input = state->show_motor_input;
    if (period_ms == 0u) {
        period_ms = 1u;
    }

    for (i = 0u; i < state->config.entry_count; i++) {
        before_rx += state->motors[i].rx_count;
        if (!online_only || state->motors[i].online) {
            candidate_count++;
        }
    }
    if (candidate_count == 0u) {
        return 0;
    }
    printf("%s%zu%s\n", console_text(state,
           "CAN warmup：只向 ", "CAN warmup: sending only to "),
           candidate_count,
           console_text(state,
           " 台已在线电机静默发送零 MIT 帧，让 CAN 错误计数自然恢复",
           " online motor(s), so CAN error counters recover"));

    state->show_can_output = false;
    state->show_motor_input = false;
    if (console_silence_process_output(&saved_stdout, &saved_stderr) == 0) {
        output_silenced = true;
    }

    while (!stop_requested && sent < count) {
        const motor_map_entry *m = NULL;
        size_t tries;

        for (tries = 0u; tries < state->config.entry_count; tries++) {
            size_t slot = (cursor + tries) % state->config.entry_count;

            if (!online_only || state->motors[slot].online) {
                m = &state->config.entries[slot];
                cursor = (slot + 1u) % state->config.entry_count;
                break;
            }
        }
        if (m == NULL) {
            break;
        }

        console_use_motor(state, m);
        if (send_debug_mit(state, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f) != 0) {
            break;
        }
        sent++;
        sleep_ms(period_ms);
    }

    if (output_silenced) {
        console_restore_process_output(saved_stdout, saved_stderr);
    }
    state->bus = old_bus;
    state->boot_id = old_id;
    update_boot_ids(state);
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;

    {
        unsigned int after_rx = 0u;

        for (i = 0u; i < state->config.entry_count; i++) {
            after_rx += state->motors[i].rx_count;
        }
        printf("%s %u/%u%s%u\n",
               console_text(state, "CAN warmup 完成，发送", "CAN warmup finished, sent"),
               sent,
               count,
               console_text(state, "，期间接收回复=", ", replies="),
               after_rx - before_rx);
    }
    return sent == count ? 0 : -1;
}

static size_t console_print_passive_power_on_scan(flash_state *state,
                                                  const unsigned int *before)
{
    size_t i;
    size_t found = 0u;

    printf("%s:", console_text(state,
                               "上电被动监听到回复的电机序号",
                               "motor indexes seen during passive power-on listen"));
    for (i = 0u; i < state->config.entry_count; i++) {
        if (state->motors[i].rx_count != before[i]) {
            state->motors[i].online = true;
            printf(" %02u", state->config.entries[i].index);
            found++;
        } else {
            state->motors[i].online = false;
        }
    }
    if (state->config.chinese_ui) {
        printf("\n被动扫描结果：%zu/%zu 台电机在线\n", found, state->config.entry_count);
    } else {
        printf("\npassive scan result: %zu/%zu motors online\n",
               found, state->config.entry_count);
    }
    return found;
}

static int console_power_on(flash_state *state)
{
    bool old_monitor = state->show_can_output;
    bool old_input = state->show_motor_input;
    unsigned int before[MOTOR_MAP_MAX];
    size_t i;
    size_t passive_found;
    int ret;

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
    state->show_can_output = false;
    state->show_motor_input = false;
    for (i = 0u; i < state->config.entry_count; i++) {
        before[i] = state->motors[i].rx_count;
        state->motors[i].online = false;
    }
    if (state->config.chinese_ui) {
        printf("电机电源已开启，等待 %u ms 完成软启动\n",
               state->config.power_on_wait_ms);
    } else {
        printf("motor power ON; waiting %u ms for soft start\n",
               state->config.power_on_wait_ms);
    }
    console_quiet_sleep_ms(state->config.power_on_wait_ms);
    passive_found = console_print_passive_power_on_scan(state, before);
    if (passive_found != 0u) {
        console_can_warmup(state,
                           state->config.can_warmup_count,
                           state->config.can_warmup_period_ms,
                           true);
        state->show_can_output = old_monitor;
        state->show_motor_input = old_input;
        return 0;
    }

    printf("%s\n", console_text(state,
           "被动监听没有收到电机输出，开始主动扫描；如果总线未接电机，可能产生 CAN TX error",
           "passive listen saw no motor output; starting active scan; empty buses may produce CAN TX errors"));
    ret = console_probe_motors(state, state->config.scan_timeout_ms);
    if (ret == 0) {
        console_can_warmup(state,
                           state->config.can_warmup_count,
                           state->config.can_warmup_period_ms,
                           true);
    }
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    return ret;
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
    console_quiet_sleep_ms(300u);
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
    const bxi_motor_limits *limits;

    if (argc != 7 || console_parse_index_arg(argv[1], &index) != 0 ||
        parse_float_arg(argv[2], &pos) != 0 ||
        parse_float_arg(argv[3], &torque) != 0 ||
        parse_float_arg(argv[4], &vel) != 0 ||
        parse_float_arg(argv[5], &kp) != 0 ||
        parse_float_arg(argv[6], &kd) != 0) {
        printf("%s: motor_set <index00> <pos> <torque> <vel> <kp> <kd>\n",
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
            printf("motor_set 被拒绝：电机 index %02u 尚未确认使能\n", index);
        } else {
            printf("motor_set refused: motor index %02u is not known to be enabled\n", index);
        }
        return -1;
    }
    limits = limits_for_entry(state, motor);
    if (!isfinite(pos) || !isfinite(torque) || !isfinite(vel) ||
        !isfinite(kp) || !isfinite(kd) ||
        pos < limits->p_min || pos > limits->p_max ||
        vel < limits->v_min || vel > limits->v_max ||
        torque < limits->t_min || torque > limits->t_max ||
        kp < limits->kp_min || kp > limits->kp_max ||
        kd < limits->kd_min || kd > limits->kd_max) {
        printf("%s: "
               "p[%g,%g] torque[%g,%g] vel[%g,%g] kp[%g,%g] kd[%g,%g]\n",
               console_text(state,
                   "motor_set 被拒绝，参数超出 MIT 范围",
                   "motor_set refused: values exceed MIT limits"),
               limits->p_min, limits->p_max,
               limits->t_min, limits->t_max,
               limits->v_min, limits->v_max,
               limits->kp_min, limits->kp_max,
               limits->kd_min, limits->kd_max);
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
                printf("stand_up 被拒绝：index %02u 未同时满足在线和使能状态\n",
                       state->config.entries[i].index);
            } else {
                printf("stand_up refused: index %02u is not online and enabled\n",
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
                printf("%s index %02u\n", console_text(state,
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

static int console_reg_send_one(flash_state *state,
                                const motor_map_entry *motor,
                                unsigned int cmd,
                                unsigned int reg,
                                unsigned int value,
                                unsigned int wait_ms)
{
    unsigned int frame_id;
    uint8_t data[8] = {0u};
    bool old_canfd;
    int ret;

    console_use_motor(state, motor);
    frame_id = (unsigned int)reg_frame_id(state, cmd);
    frame_ring_clear(&state->frames);
    old_canfd = state->debug_use_canfd;
    state->debug_use_canfd = false;
    if (cmd == CAN_CMD_REG_READ) {
        u32_to_data(reg, data);
        printf("%s index=%02u bus=%u id=%u reg=0x%08x\n",
               console_text(state, "读取寄存器", "reading register"),
               motor->index, motor->bus, motor->id, reg);
        ret = send_debug_packet(state, frame_id, data, 4u);
    } else if (cmd == CAN_CMD_REG_WRITE) {
        u32_to_data(reg, &data[0]);
        u32_to_data(value, &data[4]);
        printf("%s index=%02u bus=%u id=%u reg=0x%08x value=0x%08x\n",
               console_text(state, "写入寄存器", "writing register"),
               motor->index, motor->bus, motor->id, reg, value);
        ret = send_debug_packet(state, frame_id, data, 8u);
    } else {
        printf("%s index=%02u bus=%u id=%u\n",
               console_text(state, "保存寄存器配置", "saving register config"),
               motor->index, motor->bus, motor->id);
        ret = send_debug_packet(state, frame_id, data, 4u);
    }
    state->debug_use_canfd = old_canfd;

    if (ret == 0) {
        ret = wait_debug_reply(state, frame_id, wait_ms);
        if (ret != 0) {
            printf("%s\n", console_text(state,
                   "提示：底层电机固件在 mit_mode=1 时会把 CAN 帧交给 MIT 回调，"
                   "不会处理 0x11/0x12/0x13 配置寄存器命令；需要固件支持或切换 mit_mode 后才会回复。",
                   "hint: motor firmware routes CAN frames to the MIT callback when mit_mode=1, "
                   "so 0x11/0x12/0x13 config register commands are not handled; firmware support "
                   "or switching mit_mode is required."));
        }
    }
    return ret;
}

static int console_reg_command_all(flash_state *state,
                                   unsigned int cmd,
                                   unsigned int reg,
                                   unsigned int value,
                                   unsigned int wait_ms)
{
    flash_plan_config plan;
    unsigned int old_bus = state->bus;
    unsigned int old_id = state->boot_id;
    bool old_monitor = state->show_can_output;
    size_t i;
    size_t succeeded = 0u;
    size_t failed = 0u;

    if (load_flash_plan_config(DEFAULT_FLASH_PLAN, &plan) != 0) {
        return -1;
    }

    state->show_can_output = false;
    printf("%s plan=%s total=%zu start\n",
           console_text(state, "批量寄存器命令", "register batch"),
           DEFAULT_FLASH_PLAN, plan.target_count);
    for (i = 0u; i < plan.target_count && !stop_requested; i++) {
        const flash_plan_target *target = &plan.targets[i];
        const motor_map_entry *motor;
        motor_map_entry temporary_motor;

        motor = console_motor_by_bus_id(state, target->bus, target->id, NULL);
        if (motor == NULL) {
            memset(&temporary_motor, 0, sizeof(temporary_motor));
            temporary_motor.index = target->index;
            temporary_motor.bus = target->bus;
            temporary_motor.id = target->id;
            temporary_motor.line_no = target->line_no;
            strncpy(temporary_motor.type, target->version,
                    sizeof(temporary_motor.type) - 1u);
            motor = &temporary_motor;
        }

        if (console_reg_send_one(state, motor, cmd, reg, value, wait_ms) == 0) {
            succeeded++;
        } else {
            failed++;
        }
    }
    state->show_can_output = old_monitor;
    state->bus = old_bus;
    set_target_id(state, old_id);

    printf("%s total=%zu success=%zu failed=%zu%s\n",
           console_text(state, "批量寄存器完成", "register batch done"),
           plan.target_count, succeeded, failed,
           stop_requested ? console_text(state, " interrupted", " interrupted") : "");
    return failed == 0u && succeeded == plan.target_count ? 0 : -1;
}

static int console_reg_command(flash_state *state, int argc, char **argv, unsigned int cmd)
{
    unsigned int index;
    unsigned int reg = 0u;
    unsigned int value = 0u;
    unsigned int wait_ms = 1000u;
    unsigned int old_bus;
    unsigned int old_id;
    size_t slot;
    const motor_map_entry *motor;
    int ret;
    bool all_targets = false;

    if (cmd == CAN_CMD_REG_READ) {
        if (argc < 3 || argc > 4 ||
            parse_uint_arg(argv[2], &reg) != 0 ||
            (argc == 4 && parse_uint_arg(argv[3], &wait_ms) != 0)) {
            printf("%s: reg_read <index00|all> <reg_index> [wait_ms]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
    } else if (cmd == CAN_CMD_REG_WRITE) {
        if (argc < 4 || argc > 5 ||
            parse_uint_arg(argv[2], &reg) != 0 ||
            parse_uint_arg(argv[3], &value) != 0 ||
            (argc == 5 && parse_uint_arg(argv[4], &wait_ms) != 0)) {
            printf("%s: reg_write <index00|all> <reg_index> <value> [wait_ms]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
    } else if (cmd == CAN_CMD_REG_SAVE) {
        if (argc < 2 || argc > 3 ||
            (argc == 3 && parse_uint_arg(argv[2], &wait_ms) != 0)) {
            printf("%s: reg_save <index00|all> [wait_ms]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
    } else {
        return -1;
    }
    if (strcmp(argv[1], "all") == 0) {
        all_targets = true;
    } else if (console_parse_index_arg(argv[1], &index) != 0) {
        printf("%s: %s\n",
               console_text(state, "无效的电机序号，使用 00..99 或 all",
                            "invalid motor index, use 00..99 or all"),
               argv[1]);
        return -1;
    }

    if (console_require_power(state) != 0) {
        return -1;
    }
    if ((cmd == CAN_CMD_REG_WRITE || cmd == CAN_CMD_REG_SAVE) &&
        console_any_enabled(state)) {
        printf("%s\n", console_text(state,
               "寄存器写入/保存被拒绝：存在已知使能电机，请先失能",
               "register write/save refused: disable known enabled motors first"));
        return -1;
    }
    if (all_targets) {
        return console_reg_command_all(state, cmd, reg, value, wait_ms);
    }

    motor = console_motor_by_index(state, index, &slot);
    if (motor == NULL) {
        printf("%s: %02u\n",
               console_text(state, "未知的电机序号", "unknown motor index"),
               index);
        return -1;
    }

    old_bus = state->bus;
    old_id = state->boot_id;
    ret = console_reg_send_one(state, motor, cmd, reg, value, wait_ms);
    state->bus = old_bus;
    set_target_id(state, old_id);
    (void)slot;
    return ret;
}
