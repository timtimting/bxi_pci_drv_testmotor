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
static size_t console_print_motor_offline_rows(const flash_state *state);
static size_t console_update_online_from_rx_delta(flash_state *state,
                                                  const unsigned int *before);

static void console_home_gains_for_motor(const flash_state *state,
                                         const motor_map_entry *motor,
                                         float *kp,
                                         float *kd)
{
    unsigned int index = motor->index;

    *kp = state->config.home_kp;
    *kd = state->config.home_kd;
    if (index < MOTOR_MAP_MAX && state->config.home_kp_by_index_set[index]) {
        *kp = state->config.home_kp_by_index[index];
    }
    if (index < MOTOR_MAP_MAX && state->config.home_kd_by_index_set[index]) {
        *kd = state->config.home_kd_by_index[index];
    }
}

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
    printf("motor_scan: start total=%zu buses=%u\n",
           state->config.entry_count, (unsigned int)CANFD_DEVICE_NUM);
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
    found = console_update_online_from_rx_delta(state, before);
    console_print_motor_offline_rows(state);
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
    printf("motor_scan: done total=%zu success=%zu failed=%zu\n",
           state->config.entry_count, found, state->config.entry_count - found);
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
    bool sent_ok[MOTOR_MAP_MAX];
    unsigned int old_bus = state->bus;
    unsigned int old_id = state->boot_id;
    bool old_monitor = state->show_can_output;
    bool old_input = state->show_motor_input;
    int saved_stdout = -1;
    int saved_stderr = -1;
    bool output_silenced = false;
    bool quiet_all_enable_disable = selected_slot < 0 &&
                                    (command == BXI_MOTOR_CMD_ENABLE ||
                                     command == BXI_MOTOR_CMD_DISABLE);
    size_t i;
    size_t total = 0u;
    size_t sent = 0u;
    size_t failed = 0u;

    if (console_require_power(state) != 0) {
        return -1;
    }
    memset(sent_ok, 0, sizeof(sent_ok));
    for (i = 0u; i < state->config.entry_count; i++) {
        if (selected_slot < 0 || i == (size_t)selected_slot) {
            total++;
        }
    }
    for (i = 0u; i < state->config.entry_count; i++) {
        before[i] = state->motors[i].rx_count;
        if (selected_slot >= 0 && i != (size_t)selected_slot) {
            continue;
        }
        if (command == BXI_MOTOR_CMD_ZERO && state->motors[i].enabled) {
            printf("%s: start total=%zu\n", name, total);
            printf("[motor%02u]: failed reason=enabled bus=%u id=%u\n",
                   state->config.entries[i].index,
                   state->config.entries[i].bus,
                   state->config.entries[i].id);
            printf("%s: done total=%zu success=%zu failed=1\n",
                   name, total, total - 1u);
            return -1;
        }
    }
    printf("%s: start total=%zu\n", name, total);
    frame_ring_clear(&state->frames);
    if (quiet_all_enable_disable) {
        state->show_can_output = false;
        state->show_motor_input = false;
        output_silenced = console_silence_process_output(&saved_stdout, &saved_stderr) == 0;
    }
    for (i = 0u; i < state->config.entry_count; i++) {
        const motor_map_entry *m = &state->config.entries[i];

        if (selected_slot >= 0 && i != (size_t)selected_slot) {
            continue;
        }
        console_use_motor(state, m);
        console_expect_reply(state, m->bus);
        if (send_debug_special(state, command) == 0) {
            sent_ok[i] = true;
            sent++;
        }
    }
    state->bus = old_bus;
    state->boot_id = old_id;
    update_boot_ids(state);
    sleep_ms(state->config.scan_timeout_ms);
    if (output_silenced) {
        console_restore_process_output(saved_stdout, saved_stderr);
    }
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
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
    failed = total - sent;
    for (i = 0u; i < state->config.entry_count; i++) {
        const motor_map_entry *m = &state->config.entries[i];
        const char *status;

        if (selected_slot >= 0 && i != (size_t)selected_slot) {
            continue;
        }
        if (sent_ok[i]) {
            if (selected_slot < 0) {
                continue;
            }
            status = "success";
        } else {
            status = "failed";
        }
        printf("[motor%02u]: %s bus=%u id=%u\n", m->index, status, m->bus, m->id);
    }
    printf("%s: done total=%zu success=%zu failed=%zu\n",
           name, total, sent, failed);
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
        if (!online_only || state->motors[i].online) {
            candidate_count++;
        }
    }
    if (candidate_count == 0u) {
        return 0;
    }
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

    (void)candidate_count;
    return sent == count ? 0 : -1;
}

static size_t console_print_motor_offline_rows(const flash_state *state)
{
    size_t i;
    size_t success = 0u;
    size_t failed;
    bool print_online;

    for (i = 0u; i < state->config.entry_count; i++) {
        if (state->motors[i].online) {
            success++;
        }
    }
    failed = state->config.entry_count - success;
    if (failed == 0u) {
        return success;
    }
    print_online = success != 0u && success <= failed;

    for (i = 0u; i < state->config.entry_count; i++) {
        const motor_map_entry *m = &state->config.entries[i];
        const char *name = m->name[0] != '\0' ? m->name : "-";

        if (print_online && state->motors[i].online) {
            printf("[motor%02u]: online name=%s bus=%u id=%u\n",
                   m->index, name, m->bus, m->id);
        } else if (!print_online && !state->motors[i].online) {
            printf("[motor%02u]: offline name=%s bus=%u id=%u\n",
                   m->index, name, m->bus, m->id);
        }
    }

    return success;
}

static size_t console_update_online_from_rx_delta(flash_state *state,
                                                  const unsigned int *before)
{
    size_t i;
    size_t found = 0u;

    for (i = 0u; i < state->config.entry_count; i++) {
        if (state->motors[i].rx_count != before[i]) {
            state->motors[i].online = true;
            found++;
        } else {
            state->motors[i].online = false;
        }
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
    size_t success;
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
        if ((state->config.power_on_wait_ms % 1000u) == 0u) {
            printf("power_on: start 电源已开启 wait=%us\n",
                   state->config.power_on_wait_ms / 1000u);
        } else {
            printf("power_on: start 电源已开启 wait=%.3fs\n",
                   (double)state->config.power_on_wait_ms / 1000.0);
        }
    } else {
        if ((state->config.power_on_wait_ms % 1000u) == 0u) {
            printf("power_on: start power=on wait=%us\n",
                   state->config.power_on_wait_ms / 1000u);
        } else {
            printf("power_on: start power=on wait=%.3fs\n",
                   (double)state->config.power_on_wait_ms / 1000.0);
        }
    }
    console_quiet_sleep_ms(state->config.power_on_wait_ms);
    passive_found = console_update_online_from_rx_delta(state, before);
    if (passive_found != 0u) {
        success = console_print_motor_offline_rows(state);
        console_can_warmup(state,
                           state->config.can_warmup_count,
                           state->config.can_warmup_period_ms,
                           true);
        printf("power_on: done total=%zu success=%zu failed=%zu\n",
               state->config.entry_count, success, state->config.entry_count - success);
        state->show_can_output = old_monitor;
        state->show_motor_input = old_input;
        return 0;
    }

    printf("power_on: passive=0, active_scan=start\n");
    ret = console_probe_motors(state, state->config.scan_timeout_ms);
    if (ret == 0) {
        console_can_warmup(state,
                           state->config.can_warmup_count,
                           state->config.can_warmup_period_ms,
                           true);
    }
    {
        size_t online = 0u;

        for (i = 0u; i < state->config.entry_count; i++) {
            if (state->motors[i].online) {
                online++;
            }
        }
        printf("power_on: done total=%zu success=%zu failed=%zu\n",
               state->config.entry_count, online, state->config.entry_count - online);
    }
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    return ret;
}

static int console_power_off(flash_state *state)
{
    size_t i;
    bool any_enabled = false;

    printf("power_off: start total=1\n");
    if (!state->motor_power_on) {
        printf("power_off: done total=1 success=1 failed=0 already_off\n");
        return 0;
    }
    for (i = 0u; i < state->config.entry_count; i++) {
        any_enabled |= state->motors[i].enabled;
    }
    if (any_enabled) {
        console_send_special(state, -1, BXI_MOTOR_CMD_DISABLE, "mit_disable_all");
    }
    if (motor_pwr_set(0u) < 0) {
        fprintf(stderr, "[power]: failed reason=motor_pwr_set_off\n");
        fprintf(stderr, "power_off: done total=1 success=0 failed=1\n");
        return -1;
    }
    console_quiet_sleep_ms(300u);
    state->motor_power_on = false;
    for (i = 0u; i < state->config.entry_count; i++) {
        state->motors[i].online = false;
        state->motors[i].enabled = false;
    }
    printf("power_off: done total=1 success=1 failed=0\n");
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
    printf("motor_set: start total=1 index=%02u bus=%u id=%u\n",
           motor->index, motor->bus, motor->id);
    if (!state->motors[slot].enabled) {
        printf("[motor%02u]: failed bus=%u id=%u reason=not_enabled\n",
               motor->index, motor->bus, motor->id);
        printf("motor_set: done total=1 success=0 failed=1\n");
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
        printf("[motor%02u]: failed bus=%u id=%u reason=out_of_range "
               "p[%g,%g] torque[%g,%g] vel[%g,%g] kp[%g,%g] kd[%g,%g]\n",
               motor->index, motor->bus, motor->id,
               limits->p_min, limits->p_max,
               limits->t_min, limits->t_max,
               limits->v_min, limits->v_max,
               limits->kp_min, limits->kp_max,
               limits->kd_min, limits->kd_max);
        printf("motor_set: done total=1 success=0 failed=1\n");
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
        printf("[motor%02u]: %s bus=%u id=%u\n",
               motor->index, send_ret == 0 ? "success" : "failed",
               motor->bus, motor->id);
        printf("motor_set: done total=1 success=%u failed=%u\n",
               send_ret == 0 ? 1u : 0u, send_ret == 0 ? 0u : 1u);
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
    size_t failed = 0u;
    bool old_monitor = state->show_can_output;
    bool send_failed[MOTOR_MAP_MAX];
    uint64_t pending_before[CANFD_DEVICE_NUM];

    if (console_require_power(state) != 0) {
        return -1;
    }
    memset(send_failed, 0, sizeof(send_failed));
    printf("stand_up: start total=%zu duration_ms=%u\n",
           state->config.entry_count, state->config.home_soft_start_ms);
    for (i = 0u; i < state->config.entry_count; i++) {
        if (!state->motors[i].online || !state->motors[i].enabled) {
            const motor_map_entry *m = &state->config.entries[i];

            printf("[motor%02u]: failed bus=%u id=%u reason=%s%s%s\n",
                   m->index, m->bus, m->id,
                   !state->motors[i].online ? "offline" : "",
                   (!state->motors[i].online && !state->motors[i].enabled) ? "," : "",
                   !state->motors[i].enabled ? "not_enabled" : "");
            failed++;
        }
    }
    if (failed != 0u) {
        printf("stand_up: done total=%zu success=%zu failed=%zu\n",
               state->config.entry_count, state->config.entry_count - failed, failed);
        return -1;
    }
    steps = state->config.home_soft_start_ms / period_ms;
    if (steps == 0u) {
        steps = 1u;
    }
    if (old_monitor) {
        state->show_can_output = false;
    }
    for (bus = 0u; bus < CANFD_DEVICE_NUM; bus++) {
        pending_before[bus] = state->can_stats[bus].outstanding_replies;
    }
    for (step = 1u; step <= steps && !stop_requested; step++) {
        for (i = 0u; i < state->config.entry_count; i++) {
            const motor_map_entry *m = &state->config.entries[i];
            float target_kp;
            float kd;
            float kp;

            console_home_gains_for_motor(state, m, &target_kp, &kd);
            kp = target_kp * (float)step / (float)steps;
            console_use_motor(state, m);
            console_expect_reply(state, m->bus);
            if (send_debug_mit(state, 0.0f, 0.0f, kp, kd, 0.0f) != 0) {
                send_failed[i] = true;
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
    failed = 0u;
    for (i = 0u; i < state->config.entry_count; i++) {
        if (send_failed[i]) {
            const motor_map_entry *m = &state->config.entries[i];

            printf("[motor%02u]: failed bus=%u id=%u reason=send_error\n",
                   m->index, m->bus, m->id);
            failed++;
        }
    }
    if (stop_requested && failed == 0u) {
        failed = state->config.entry_count;
    }
    printf("stand_up: done total=%zu success=%zu failed=%zu%s\n",
           state->config.entry_count,
           state->config.entry_count - failed,
           failed,
           stop_requested ? " interrupted" : "");
    return failed == 0u && !stop_requested ? 0 : -1;
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

static const char *console_reg_command_name(unsigned int cmd)
{
    if (cmd == CAN_CMD_REG_READ) {
        return "reg_read";
    }
    if (cmd == CAN_CMD_REG_WRITE) {
        return "reg_write";
    }
    if (cmd == CAN_CMD_REG_SAVE) {
        return "reg_save";
    }
    return "reg";
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
        if (!state->brief_register_output) {
            printf("%s index=%02u bus=%u id=%u reg=0x%08x\n",
                   console_text(state, "读取寄存器", "reading register"),
                   motor->index, motor->bus, motor->id, reg);
        }
        ret = send_debug_packet(state, frame_id, data, 4u);
    } else if (cmd == CAN_CMD_REG_WRITE) {
        u32_to_data(reg, &data[0]);
        u32_to_data(value, &data[4]);
        if (!state->brief_register_output) {
            printf("%s index=%02u bus=%u id=%u reg=0x%08x value=0x%08x\n",
                   console_text(state, "写入寄存器", "writing register"),
                   motor->index, motor->bus, motor->id, reg, value);
        }
        ret = send_debug_packet(state, frame_id, data, 8u);
    } else {
        if (!state->brief_register_output) {
            printf("%s index=%02u bus=%u id=%u\n",
                   console_text(state, "保存寄存器配置", "saving register config"),
                   motor->index, motor->bus, motor->id);
        }
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
    bool old_input = state->show_motor_input;
    bool old_brief_register_output = state->brief_register_output;
    size_t i;
    size_t succeeded = 0u;
    size_t failed = 0u;

    if (load_flash_plan_config(DEFAULT_FLASH_PLAN, &plan) != 0) {
        return -1;
    }

    state->show_can_output = false;
    state->show_motor_input = false;
    state->brief_register_output = true;
    printf("%s: start plan=%s total=%zu\n",
           console_reg_command_name(cmd), DEFAULT_FLASH_PLAN, plan.target_count);
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
            printf("[motor%02u]: failed bus=%u id=%u reason=no_reply\n",
                   motor->index, motor->bus, motor->id);
            failed++;
        }
    }
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    state->brief_register_output = old_brief_register_output;
    state->bus = old_bus;
    set_target_id(state, old_id);

    printf("%s: done total=%zu success=%zu failed=%zu%s\n",
           console_reg_command_name(cmd), plan.target_count, succeeded, failed,
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
    bool old_brief_register_output;
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
    old_brief_register_output = state->brief_register_output;
    state->brief_register_output = true;
    printf("%s: start total=1 index=%02u bus=%u id=%u\n",
           console_reg_command_name(cmd), motor->index, motor->bus, motor->id);
    ret = console_reg_send_one(state, motor, cmd, reg, value, wait_ms);
    if (ret != 0) {
        printf("[motor%02u]: failed bus=%u id=%u reason=no_reply\n",
               motor->index, motor->bus, motor->id);
    }
    printf("%s: done total=1 success=%u failed=%u\n",
           console_reg_command_name(cmd), ret == 0 ? 1u : 0u, ret == 0 ? 0u : 1u);
    state->brief_register_output = old_brief_register_output;
    state->bus = old_bus;
    set_target_id(state, old_id);
    (void)slot;
    return ret;
}
