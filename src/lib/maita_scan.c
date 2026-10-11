typedef enum {
    MAITA_SCAN_UNTESTED,
    MAITA_SCAN_NO_REPLY,
    MAITA_SCAN_TX_FAILED,
    MAITA_SCAN_REPLIED,
    MAITA_SCAN_CANCELLED
} maita_scan_result;

static maita_scan_result maita_scan_query(flash_state *state, unsigned int bus,
                                          unsigned int id, unsigned int timeout_ms,
                                          rx_can_frame *reply, uint64_t *ignored)
{
    const uint8_t request[8] = {MAITA_CMD_STATUS1, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    uint64_t started;
    uint64_t deadline;

    if (stop_requested) return MAITA_SCAN_CANCELLED;
    frame_ring_clear(&state->frames);
    started = monotonic_time_us();
    deadline = started + (uint64_t)timeout_ms * 1000ULL;
    if (maita_send_frame_logged(state, bus, MAITA_SEND_BASE + id, request, false) != 0) {
        return MAITA_SCAN_TX_FAILED;
    }
    while (!stop_requested) {
        rx_can_frame frame;
        bool received = frame_ring_pop(&state->frames, &frame, 0u) != 0;

        if (received) {
            if (frame.monotonic_us >= started && frame.monotonic_us <= deadline &&
                frame.bus == bus && frame.raw_can_id == MAITA_REPLY_BASE + id &&
                frame.can_id == MAITA_REPLY_BASE + id && frame.len == 8u &&
                frame.flags == 0u && frame.data[0] == MAITA_CMD_STATUS1) {
                *reply = frame;
                return MAITA_SCAN_REPLIED;
            }
            (*ignored)++;
        }
        if (monotonic_time_us() >= deadline) break;
        if (!received) usleep(1000u);
    }
    return stop_requested ? MAITA_SCAN_CANCELLED : MAITA_SCAN_NO_REPLY;
}

static int console_maita_scan(flash_state *state, int argc, char **argv)
{
    maita_scan_result results[CANFD_DEVICE_NUM][MAITA_CAN_ID_MAX + 1u] = {{MAITA_SCAN_UNTESTED}};
    uint64_t errors_before[CANFD_DEVICE_NUM];
    uint64_t errors_after[CANFD_DEVICE_NUM];
    uint64_t ignored = 0u;
    unsigned int timeout_ms = 100u;
    unsigned int attempts = 2u;
    unsigned int replies = 0u;
    unsigned int tx_failed = 0u;
    unsigned int checked_total = 0u;
    unsigned int bus;
    unsigned int id;
    bool old_monitor;
    bool old_input;
    bool completed;

    if (argc > 3 ||
        (argc >= 2 && parse_uint_arg(argv[1], &timeout_ms) != 0) ||
        (argc == 3 && parse_uint_arg(argv[2], &attempts) != 0) ||
        timeout_ms == 0u || timeout_ms > 5000u || attempts == 0u || attempts > 10u) {
        printf("%s: %s [timeout_ms:1..5000] [attempts:1..10]\n",
               console_text(state, "用法", "usage"), argv[0]);
        return -1;
    }
    if (console_require_power(state) != 0) return -1;

    old_monitor = state->show_can_output;
    old_input = state->show_motor_input;
    state->show_can_output = false;
    state->show_motor_input = false;
    pthread_mutex_lock(&state->frames.lock);
    memcpy(errors_before, state->frames.can_errors, sizeof(errors_before));
    pthread_mutex_unlock(&state->frames.lock);
    printf("%s\n", console_text(state,
           "maita_scan: 仅发送 0x9A 状态查询；复用终端 PCI/CAN，不操作电源、不发送使能或运动。",
           "maita_scan: status 0x9A only; reusing console PCI/CAN, no power/enable/motion commands."));
    printf("maita_scan: start buses=0..%u ids=%u..%u timeout_ms=%u attempts=%u\n",
           (unsigned int)CANFD_DEVICE_NUM - 1u, MAITA_CAN_ID_MIN, MAITA_CAN_ID_MAX,
           timeout_ms, attempts);
    for (bus = 0u; bus < CANFD_DEVICE_NUM && !stop_requested; bus++) {
        for (id = MAITA_CAN_ID_MIN; id <= MAITA_CAN_ID_MAX && !stop_requested; id++) {
            rx_can_frame reply;
            maita_scan_result response = MAITA_SCAN_UNTESTED;
            unsigned int used_attempts = 0u;
            bool sent = false;

            while (used_attempts < attempts && !stop_requested) {
                used_attempts++;
                response = maita_scan_query(state, bus, id, timeout_ms, &reply, &ignored);
                if (response == MAITA_SCAN_TX_FAILED) tx_failed++;
                else if (response != MAITA_SCAN_CANCELLED) sent = true;
                if (response == MAITA_SCAN_REPLIED || response == MAITA_SCAN_CANCELLED) break;
                if (used_attempts < attempts) sleep_ms(2u);
            }
            if (stop_requested) {
                results[bus][id] = MAITA_SCAN_CANCELLED;
                break;
            }
            results[bus][id] = response == MAITA_SCAN_REPLIED ? MAITA_SCAN_REPLIED :
                (sent ? MAITA_SCAN_NO_REPLY : MAITA_SCAN_TX_FAILED);
            checked_total++;
            printf("[bus%u id%02u] tx=0x%03x rx=0x%03x attempts=%u result=%s",
                   bus, id, MAITA_SEND_BASE + id, MAITA_REPLY_BASE + id, used_attempts,
                   response == MAITA_SCAN_REPLIED ? "reply" : (sent ? "no_reply" : "tx_failed"));
            if (response == MAITA_SCAN_REPLIED) {
                int temperature_c;
                unsigned int brake_released;
                float voltage_v;
                uint16_t error;

                replies++;
                if (maita_unpack_temp_reply(&reply, id, &temperature_c, &brake_released,
                                            &voltage_v, &error) == 0) {
                    printf(" temp=%dC voltage=%.1fV brake=%s error=0x%04x",
                           temperature_c, voltage_v, brake_released ? "release" : "lock", error);
                }
                printf(" data:");
                maita_print_data(reply.data, reply.len);
            }
            printf("\n");
            fflush(stdout);
            sleep_ms(2u);
        }
    }
    completed = checked_total == CANFD_DEVICE_NUM * (MAITA_CAN_ID_MAX - MAITA_CAN_ID_MIN + 1u) &&
                !stop_requested;
    pthread_mutex_lock(&state->frames.lock);
    memcpy(errors_after, state->frames.can_errors, sizeof(errors_after));
    pthread_mutex_unlock(&state->frames.lock);
    for (bus = 0u; bus < CANFD_DEVICE_NUM; bus++) {
        unsigned int checked = 0u;
        unsigned int online = 0u;

        printf("[bus%u] responding_ids:", bus);
        for (id = MAITA_CAN_ID_MIN; id <= MAITA_CAN_ID_MAX; id++) {
            if (results[bus][id] == MAITA_SCAN_REPLIED) {
                printf(" %u", id);
                online++;
            }
            if (results[bus][id] != MAITA_SCAN_UNTESTED && results[bus][id] != MAITA_SCAN_CANCELLED) {
                checked++;
            }
        }
        if (online == 0u) printf(" none");
        printf("; responding=%u checked=%u/%u CAN_errors=%llu\n", online, checked,
               MAITA_CAN_ID_MAX - MAITA_CAN_ID_MIN + 1u,
               (unsigned long long)(errors_after[bus] - errors_before[bus]));
    }
    printf("maita_scan: %s checked=%u/%u responding=%u tx_failed=%u ignored_rx=%llu\n",
           completed ? "complete" : "incomplete", checked_total,
           CANFD_DEVICE_NUM * (MAITA_CAN_ID_MAX - MAITA_CAN_ID_MIN + 1u),
           replies, tx_failed, (unsigned long long)ignored);
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    return completed && replies > 0u && tx_failed == 0u ? 0 : -1;
}
