/*
 * Pass-through motor debug commands for motor_console.c.
 *
 * Included by motor_console.c; do not add as a standalone CMake source.
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

static int console_motor_all_dbg(flash_state *state, int argc, char **argv)
{
    unsigned int bus;
    unsigned int id;

    if (argc != 3 ||
        parse_uint_arg(argv[1], &bus) != 0 ||
        parse_uint_arg(argv[2], &id) != 0) {
        printf("%s: motor_all_dbg <bus> <id>\n", console_text(state, "用法", "usage"));
        return -1;
    }
    return console_motor_debug_loop(state, bus, id);
}
