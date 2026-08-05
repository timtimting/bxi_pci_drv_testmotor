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
                                              uint32_t config_version);

/*
 * 通过普通寄存器 CAN 通道读取一个 32 位电机配置寄存器。
 *
 * 这个函数只做很薄的一层封装：发送 CAN_CMD_REG_READ，等待 bus/id/reg
 * 都匹配的回复，打印原始 TX/RX 帧，并返回 uint32 原始值。寄存器含义
 * 由调用方决定。
 */
static bool console_read_motor_register(flash_state *state,
                                        const motor_map_entry *motor,
                                        uint32_t reg,
                                        uint32_t *value)
{
    enum {
        /* 版本探测不应阻塞太久，保持 can_dbg 响应比较快。 */
        REGISTER_READ_WAIT_MS = 300u,
    };
    /* 寄存器读请求的数据区是 4 字节小端寄存器地址。 */
    uint8_t data[4] = {0u};
    /* 寄存器 CAN ID 格式是 (command << 4) | motor_id。 */
    unsigned int frame_id = (unsigned int)reg_frame_id(state, CAN_CMD_REG_READ);
    /* 等待匹配寄存器回复的绝对超时时间。 */
    uint64_t deadline;
    /* 保存调试发送模式；寄存器协议需要使用 classic CAN。 */
    bool old_canfd = state->debug_use_canfd;
    /* 读取二进制寄存器回复时，临时隐藏电机文本输出，避免混杂。 */
    bool old_input = state->show_motor_input;

    /* 默认值置 0；调用方可以把读取失败安全地当作旧版本处理。 */
    if (value != NULL) {
        *value = 0u;
    }
    /* 将目标寄存器地址写入 4 字节读请求。 */
    u32_to_data(reg, data);
    /* 寄存器协议使用 classic CAN，不使用 CAN-FD/BRS 的 MIT 帧。 */
    state->debug_use_canfd = false;
    /* 避免本次一次性寄存器探测和电机文本输出混在一起。 */
    state->show_motor_input = false;
    /* 清掉旧帧，避免把上一次命令的回复误认为本次寄存器回复。 */
    frame_ring_clear(&state->frames);
    /* 调试输出：打印准确的 TX 帧，便于现场排查 bus/id/reg 是否正确。 */
#if 0
    console_print_motor_tx_frame(motor, "reg", motor->bus, frame_id, sizeof(data), 0u, data);
#endif
    /* 向指定电机发送寄存器读请求。 */
    if (send_debug_packet(state, frame_id, data, sizeof(data)) != 0) {
        /* 发送失败也必须恢复终端/调试状态。 */
        state->debug_use_canfd = old_canfd;
        state->show_motor_input = old_input;
        return false;
    }

    /* 将相对等待时间转换为绝对截止时间。 */
    deadline = time_us() + (uint64_t)REGISTER_READ_WAIT_MS * 1000ULL;
    /* 轮询共享帧队列，直到收到匹配的寄存器回复或超时。 */
    while (!stop_requested && time_us() < deadline) {
        rx_can_frame frame;
        uint64_t now = time_us();
        /* 小周期醒来一次，方便及时响应 Ctrl-C/stop_requested。 */
        unsigned int wait_ms = 20u;

        /* 最后一轮等待不能超过总截止时间。 */
        if (deadline > now) {
            uint64_t remain_ms = (deadline - now) / 1000ULL;
            if (remain_ms < wait_ms) {
                wait_ms = (unsigned int)remain_ms;
            }
        }
        /* frame_ring_pop 需要非零超时时间。 */
        if (wait_ms == 0u) {
            wait_ms = 1u;
        }
        /* 本小窗口没有收到帧，继续等到总超时。 */
        if (!frame_ring_pop(&state->frames, &frame, wait_ms)) {
            continue;
        }
        /* 忽略其它 bus、其它命令、其它电机 ID 的帧。 */
        if (frame.bus != motor->bus ||
            frame.can_id != frame_id ||
            (frame.can_id & 0x0fu) != (motor->id & 0x0fu)) {
            continue;
        }
        /* 有效寄存器回复格式为：[reg u32][value u32]。 */
        if (frame.len >= 8u &&
            data_to_u32(&frame.data[0]) == reg) {
            /* 从 bytes 4..7 取出寄存器返回值。 */
            uint32_t reply_value = data_to_u32(&frame.data[4]);
            /* 调试输出：复用已有的简洁寄存器输出，保持日志格式一致。 */
#if 0
            bool old_brief = state->brief_register_output;

            /* can_dbg 前置探测时，强制寄存器回复单行输出。 */
            state->brief_register_output = true;
            print_register_debug_frame(state, &frame);
            state->brief_register_output = old_brief;
#endif
            /* 返回原始值；具体版本含义由调用方解释。 */
            if (value != NULL) {
                *value = reply_value;
            }
            /* 成功读取后恢复终端/调试状态。 */
            state->debug_use_canfd = old_canfd;
            state->show_motor_input = old_input;
            return true;
        }
        /* 调试输出：打印非预期但非空的帧，便于发现协议不匹配。 */
#if 0
        if (frame.len > 0u) {
            console_print_raw_motor_can_frame(state, motor, &frame, 0u);
        }
#endif
        /* 空寄存器回复通常表示目标拒绝或忽略了本次请求。 */
        if (is_reg_cmd_id(frame.can_id) && frame.len == 0u) {
            break;
        }
    }

    /* 超时或未匹配到回复时，也要先恢复状态再返回失败。 */
    state->debug_use_canfd = old_canfd;
    state->show_motor_input = old_input;
    return false;
}

/*
 * 根据电机配置版本解析一帧 MIT 回复。
 *
 * 当前兼容规则：
 *   config_version != 0：新版固件，bytes 6..7 是轮询 AUX 遥测。
 *   config_version == 0：旧版固件，bytes 6..7 是 NTC 温度。
 */
static int console_unpack_motor_reply_by_version(const uint8_t *data,
                                                 size_t len,
                                                 const bxi_motor_limits *limits,
                                                 uint32_t config_version,
                                                 bxi_motor_reply *reply)
{
    /* 任何非零配置版本都认为电机使用轮询 AUX 遥测。 */
    if (config_version != 0u) {
        return bxi_motor_unpack_reply(data, len, limits, reply);
    }
    /* 版本 0，或者版本读取失败后保留的默认 0，都按旧版 NTC 字节解析。 */
    return bxi_motor_unpack_reply_legacy_ntc(data, len, limits, reply);
}

static void console_print_raw_motor_can_frame(flash_state *state,
                                              const motor_map_entry *motor,
                                              const rx_can_frame *frame,
                                              uint32_t config_version)
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
        int decode_ret;

        decode_ret = console_unpack_motor_reply_by_version(frame->data,
                                                           frame->len,
                                                           limits,
                                                           config_version,
                                                           &reply);
        if (decode_ret == 0) {
            printf("  decode: pos=% .5f vel=% .5f torque=% .5f",
                   reply.position,
                   reply.velocity,
                   reply.torque);
            if (config_version != 0u) {
                print_mit_aux_decode(&state->config, &reply);
            } else {
                printf(" ntc1=% .1fC ntc2=% .1fC",
                   reply.mos_temperature,
                   reply.motor_temperature);
            }
        }
    }
    printf("\n");
}

static int console_motor_reply(flash_state *state, int argc, char **argv)
{
    enum {
        CONFIG_VERSION_REG = 0x7cu,
    };
    unsigned int index;
    unsigned int timeout_ms = 3000u;
    bool probe = true;
    size_t slot;
    const motor_map_entry *motor;
    unsigned int old_bus;
    unsigned int old_id;
    bool old_monitor;
    bool old_input;
    uint32_t config_version = 0u;
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
    if (console_read_motor_register(state, motor, CONFIG_VERSION_REG, &config_version)) {
        printf("[motor%02u]: mit_decode=%s config_version=%u%s\n",
               motor->index,
               config_version != 0u ? "aux" : "legacy_ntc",
               config_version,
               config_version != 0u ? " reason=polling_mit_supported" : "");
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
        console_print_raw_motor_can_frame(state, motor, &frame, config_version);
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
