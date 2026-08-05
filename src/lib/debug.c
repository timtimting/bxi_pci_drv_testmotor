/*
 * Pass-through motor debug commands for main.c.
 *
 * Included by main.c; do not add as a standalone CMake source.
 */

static int console_motor_debug_loop(flash_state *state, unsigned int bus, unsigned int id)
{
    unsigned int old_bus = state->bus;
    unsigned int old_id = state->boot_id;
    bool old_monitor = state->show_can_output;
    bool old_input = state->show_motor_input;
    struct termios old_term;
    struct termios new_term;
    bool raw_enabled = false;
    int result = 0;

    if (bus >= CANFD_DEVICE_NUM || id == 0u || id > 8u) {
        printf("%s\n", console_text(state,
               "motor_dbg 参数错误：bus 必须在范围内，id 必须为 1..8",
               "motor_dbg argument error: bus must be valid and id must be 1..8"));
        return -1;
    }
    if (console_require_power(state) != 0) {
        return -1;
    }

    state->bus = bus;
    set_target_id(state, id);
    state->show_can_output = true;
    state->show_motor_input = false;
    rx_ring_clear(&state->rx);

    printf("%s bus=%u id=%u tx=0x%03x rx=0x%03x\n",
           console_text(state,
                        "进入电机直通调试，所有按键都会立即发送给电机；按 ` 退出",
                        "enter motor pass-through debug; every key is sent immediately; ` exits"),
           bus,
           id,
           state->tx_id,
           state->rx_id);

    if (isatty(STDIN_FILENO)) {
        if (tcgetattr(STDIN_FILENO, &old_term) != 0) {
            result = -1;
            goto out;
        }
        new_term = old_term;
        new_term.c_lflag &= (tcflag_t)~(ICANON | ECHO);
        new_term.c_cc[VMIN] = 1;
        new_term.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &new_term) != 0) {
            result = -1;
            goto out;
        }
        raw_enabled = true;
    }

    while (!stop_requested) {
        uint8_t byte;
        ssize_t n = read(STDIN_FILENO, &byte, 1u);

        if (n <= 0) {
            result = -1;
            break;
        }
        if (byte == '`') {
            break;
        }
        if (send_boot_byte(state, byte) != 0) {
            result = -1;
            break;
        }
    }
    if (raw_enabled) {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
        raw_enabled = false;
    }
    printf("\n%s\n", console_text(state,
           "已退出电机直通调试",
           "motor pass-through debug exited"));

out:
    if (raw_enabled) {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
    }
    state->bus = old_bus;
    set_target_id(state, old_id);
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    return result;
}

static int console_motor_dbg(flash_state *state, int argc, char **argv)
{
    unsigned int index;
    size_t slot;
    const motor_map_entry *motor;

    if (argc != 2 || console_parse_index_arg(argv[1], &index) != 0 ||
        (motor = console_motor_by_index(state, index, &slot)) == NULL) {
        printf("%s: motor_dbg <index00>\n", console_text(state, "用法", "usage"));
        return -1;
    }
    (void)slot;
    return console_motor_debug_loop(state, motor->bus, motor->id);
}

static bool console_motor_can_frame_matches(const rx_can_frame *frame,
                                            const motor_map_entry *motor)
{
    if (frame == NULL || motor == NULL || frame->bus != motor->bus) {
        return false;
    }
    if (boot_output_id_matches(frame->can_id, motor->id)) {
        return true;
    }
    if (mit_reply_frame_matches(frame, motor->id)) {
        return true;
    }
    if ((frame->can_id & 0x0fu) == (motor->id & 0x0fu)) {
        return true;
    }
    return false;
}

static void console_print_raw_motor_can_frame(flash_state *state,
                                              const motor_map_entry *motor,
                                              const rx_can_frame *frame,
                                              bool use_aux_decode);

static bool console_read_motor_fw_version(flash_state *state,
                                          const motor_map_entry *motor,
                                          uint32_t *major_value,
                                          uint32_t *minor_value)
{
    enum {
        CONFIG_VERSION_WAIT_MS = 300u,
    };
    uint8_t empty_data[1] = {0u};
    unsigned int frame_id = (unsigned int)reg_frame_id(state, CAN_CMD_REG_FW_VERSION);
    uint64_t deadline;
    bool old_canfd = state->debug_use_canfd;
    bool old_input = state->show_motor_input;

    if (major_value != NULL) {
        *major_value = 0u;
    }
    if (minor_value != NULL) {
        *minor_value = 0u;
    }
    state->debug_use_canfd = false;
    state->show_motor_input = false;
    frame_ring_clear(&state->frames);
    console_print_motor_tx_frame(motor, "fw_version", motor->bus, frame_id, 0u, 0u, empty_data);
    if (send_debug_packet(state, frame_id, empty_data, 0u) != 0) {
        state->debug_use_canfd = old_canfd;
        state->show_motor_input = old_input;
        return false;
    }

    deadline = time_us() + (uint64_t)CONFIG_VERSION_WAIT_MS * 1000ULL;
    while (!stop_requested && time_us() < deadline) {
        rx_can_frame frame;
        uint64_t now = time_us();
        unsigned int wait_ms = 20u;

        if (deadline > now) {
            uint64_t remain_ms = (deadline - now) / 1000ULL;
            if (remain_ms < wait_ms) {
                wait_ms = (unsigned int)remain_ms;
            }
        }
        if (wait_ms == 0u) {
            wait_ms = 1u;
        }
        if (!frame_ring_pop(&state->frames, &frame, wait_ms)) {
            continue;
        }
        if (frame.bus != motor->bus ||
            frame.can_id != frame_id ||
            (frame.can_id & 0x0fu) != (motor->id & 0x0fu)) {
            continue;
        }
        if (frame.len >= 8u) {
            uint32_t major = data_to_u32(&frame.data[0]);
            uint32_t minor = data_to_u32(&frame.data[4]);

            console_print_raw_motor_can_frame(state, motor, &frame, false);
            if (major_value != NULL) {
                *major_value = major;
            }
            if (minor_value != NULL) {
                *minor_value = minor;
            }
            state->debug_use_canfd = old_canfd;
            state->show_motor_input = old_input;
            return true;
        }
        console_print_raw_motor_can_frame(state, motor, &frame, false);
        if (frame.can_id == frame_id && frame.len == 0u) {
            break;
        }
    }

    state->debug_use_canfd = old_canfd;
    state->show_motor_input = old_input;
    return false;
}

static void console_print_raw_motor_can_frame(flash_state *state,
                                              const motor_map_entry *motor,
                                              const rx_can_frame *frame,
                                              bool use_aux_decode)
{
    const char *kind = "can";

    if ((frame->can_id >> 4) == CAN_CMD_REG_FW_VERSION) {
        kind = "fw_version";
    } else if (boot_output_id_matches(frame->can_id, motor->id)) {
        kind = "boot";
    } else if (mit_reply_frame_matches(frame, motor->id)) {
        kind = "mit";
    } else if (is_reg_cmd_id(frame->can_id)) {
        kind = "reg";
    }

    printf("[motor%02u]: rx kind=%s bus=%u id=%u can_id=0x%03x len=%u flags=0x%02x data:",
           motor->index,
           kind,
           frame->bus,
           motor->id,
           frame->can_id,
           frame->len,
           frame->flags);
    print_data(frame->data, frame->len);

    if (mit_reply_frame_matches(frame, motor->id)) {
        bxi_motor_reply reply;
        const bxi_motor_limits *limits = limits_for_entry(state, motor);
        int decode_ret;

        decode_ret = use_aux_decode ?
                     bxi_motor_unpack_reply(frame->data, frame->len, limits, &reply) :
                     bxi_motor_unpack_reply_legacy_ntc(frame->data, frame->len, limits, &reply);
        if (decode_ret == 0) {
            printf("  decode: pos=% .5f vel=% .5f torque=% .5f",
                   reply.position,
                   reply.velocity,
                   reply.torque);
            if (use_aux_decode) {
                print_mit_aux_decode(&state->config, &reply);
            } else {
                printf(" ntc1=% .1fC ntc2=% .1fC",
                   reply.mos_temperature,
                   reply.motor_temperature);
            }
        }
    } else if ((frame->can_id >> 4) == CAN_CMD_REG_FW_VERSION && frame->len >= 8u) {
        printf("  fw_version=%u.%u",
               data_to_u32(&frame->data[0]),
               data_to_u32(&frame->data[4]));
    }
    printf("\n");
}

static int console_motor_reply(flash_state *state, int argc, char **argv)
{
    unsigned int index;
    unsigned int timeout_ms = 3000u;
    bool probe = true;
    size_t slot;
    const motor_map_entry *motor;
    unsigned int old_bus;
    unsigned int old_id;
    bool old_monitor;
    bool old_input;
    uint32_t fw_major = 0u;
    uint32_t fw_minor = 0u;
    bool use_aux_decode = false;
    uint64_t deadline;
    size_t matched = 0u;
    int argi;

    if (argc < 2 || argc > 4 ||
        console_parse_index_arg(argv[1], &index) != 0 ||
        (motor = console_motor_by_index(state, index, &slot)) == NULL) {
        printf("%s: motor_reply <index00> [timeout_ms] [passive]\n",
               console_text(state, "用法", "usage"));
        return -1;
    }
    for (argi = 2; argi < argc; argi++) {
        if (strcmp(argv[argi], "probe") == 0) {
            probe = true;
        } else if (strcmp(argv[argi], "passive") == 0) {
            probe = false;
        } else if (parse_uint_arg(argv[argi], &timeout_ms) != 0) {
            printf("%s: motor_reply <index00> [timeout_ms] [passive]\n",
                   console_text(state, "用法", "usage"));
            return -1;
        }
    }
    if (timeout_ms == 0u) {
        timeout_ms = 1u;
    }
    if (console_require_power(state) != 0) {
        return -1;
    }

    old_bus = state->bus;
    old_id = state->boot_id;
    old_monitor = state->show_can_output;
    old_input = state->show_motor_input;
    state->show_can_output = false;
    state->show_motor_input = false;
    state->bus = motor->bus;
    set_target_id(state, motor->id);
    frame_ring_clear(&state->frames);

    printf("motor_reply: start total=1 index=%02u bus=%u id=%u timeout_ms=%u%s\n",
           motor->index,
           motor->bus,
           motor->id,
           timeout_ms,
           probe ? " probe" : "");
    if (console_read_motor_fw_version(state, motor, &fw_major, &fw_minor)) {
        use_aux_decode = fw_major == 0u && fw_minor <= 1u;
        printf("[motor%02u]: mit_decode=%s fw_version=%u.%u%s\n",
               motor->index,
               use_aux_decode ? "aux" : "legacy_ntc",
               fw_major,
               fw_minor,
               use_aux_decode ? " reason=polling_mit_supported" : "");
    } else {
        printf("[motor%02u]: mit_decode=legacy_ntc reason=config_version_empty\n",
               motor->index);
    }
    frame_ring_clear(&state->frames);
    if (probe) {
        uint8_t data[BXI_MOTOR_MIT_LEN];

        bxi_motor_pack_mit(data, limits_for_entry(state, motor),
                           0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        console_print_motor_tx_frame(motor,
                                     "mit",
                                     motor->bus,
                                     motor->id,
                                     BXI_MOTOR_MIT_LEN,
                                     state->debug_use_canfd ? (CANFD_BRS | CANFD_FDF) : 0u,
                                     data);
        console_expect_reply(state, motor->bus);
        if (send_debug_packet(state, motor->id, data, BXI_MOTOR_MIT_LEN) != 0) {
            printf("[motor%02u]: failed bus=%u id=%u reason=probe_send_error\n",
                   motor->index, motor->bus, motor->id);
        }
    }

    deadline = time_us() + (uint64_t)timeout_ms * 1000ULL;
    while (!stop_requested && time_us() < deadline) {
        rx_can_frame frame;
        uint64_t now = time_us();
        unsigned int wait_ms = 50u;

        if (deadline > now) {
            uint64_t remain_ms = (deadline - now) / 1000ULL;
            if (remain_ms < wait_ms) {
                wait_ms = (unsigned int)remain_ms;
            }
        }
        if (wait_ms == 0u) {
            wait_ms = 1u;
        }
        if (!frame_ring_pop(&state->frames, &frame, wait_ms)) {
            continue;
        }
        if (!console_motor_can_frame_matches(&frame, motor)) {
            continue;
        }
        console_print_raw_motor_can_frame(state, motor, &frame, use_aux_decode);
        matched++;
    }

    state->bus = old_bus;
    set_target_id(state, old_id);
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    (void)slot;

    printf("motor_reply: done total=1 success=%u failed=%u frames=%zu\n",
           matched != 0u ? 1u : 0u,
           matched != 0u ? 0u : 1u,
           matched);
    return matched == 0u ? 1 : 0;
}
