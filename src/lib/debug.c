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
                                              const rx_can_frame *frame)
{
    const char *kind = "can";

    if (boot_output_id_matches(frame->can_id, motor->id)) {
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

        if (bxi_motor_unpack_reply(frame->data, frame->len, limits, &reply) == 0) {
            printf("  decode: pos=% .5f vel=% .5f torque=% .5f",
                   reply.position,
                   reply.velocity,
                   reply.torque);
            print_mit_aux_decode(&state->config, &reply);
        }
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
    if (probe) {
        console_expect_reply(state, motor->bus);
        if (send_debug_mit(state, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f) != 0) {
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
        console_print_raw_motor_can_frame(state, motor, &frame);
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
