/*
 * Minimal MYACTUATOR/Maita CAN helpers for the maita_motor branch.
 *
 * Protocol reference: "CAN BUS Motor Motion Protocol V4.4 260520.pdf".
 * CAN is classic standard frame, 1 Mbps, DLC=8.
 */

typedef struct
{
    unsigned int id;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    int temperature_c;
    float current_a;
    float speed_dps;
    int angle_deg;
} maita_reply;

enum {
    MAITA_CAN_ID_MIN = 1u,
    MAITA_CAN_ID_MAX = 32u,
    MAITA_SEND_BASE = 0x140u,
    MAITA_REPLY_BASE = 0x240u,
    MAITA_MIT_BASE = 0x400u,
    MAITA_MIT_REPLY_BASE = 0x500u,
    MAITA_CMD_FUNC = 0x20u,
    MAITA_CMD_PID_READ = 0x30u,
    MAITA_CMD_ACCEL_READ = 0x42u,
    MAITA_CMD_ZERO_CURRENT = 0x64u,
    MAITA_CMD_RESET = 0x76u,
    MAITA_CMD_SHUTDOWN = 0x80u,
    MAITA_CMD_STOP = 0x81u,
    MAITA_CMD_TORQUE = 0xa1u,
    MAITA_CMD_SW_VERSION = 0xb2u,
    MAITA_CMD_MODEL = 0xb5u,
};

#define MAITA_POS_MIN_RAD (-12.566f)
#define MAITA_POS_MAX_RAD (12.566f)
#define MAITA_VEL_MIN_RAD_S (-45.0f)
#define MAITA_VEL_MAX_RAD_S (45.0f)
#define MAITA_KP_MAX (500.0f)
/* 供应商确认脉塔 MIT kd 通信范围为 0~5；手册 V4.4 修订记录写 0~50，与实际不一致。 */
#define MAITA_KD_MAX (5.0f)

static float maita_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint32_t maita_float_to_uint(float value,
                                    float min_value,
                                    float max_value,
                                    unsigned int bits)
{
    uint32_t max_int;
    float scaled;

    if (bits == 0u || bits >= 31u || max_value <= min_value) {
        return 0u;
    }
    max_int = (1u << bits) - 1u;
    value = maita_clamp_float(value, min_value, max_value);
    scaled = (value - min_value) * (float)max_int / (max_value - min_value);
    if (scaled <= 0.0f) {
        return 0u;
    }
    if (scaled >= (float)max_int) {
        return max_int;
    }
    return (uint32_t)(scaled + 0.5f);
}

static float maita_uint_to_float(uint32_t value,
                                  float min_value,
                                  float max_value,
                                  unsigned int bits)
{
    uint32_t max_int;

    if (bits == 0u || bits >= 31u || max_value <= min_value) {
        return min_value;
    }
    max_int = (1u << bits) - 1u;
    if (value > max_int) {
        value = max_int;
    }
    return (float)value * (max_value - min_value) / (float)max_int + min_value;
}

static void maita_pack_mit(uint8_t data[8],
                           float position_rad,
                           float velocity_rad_s,
                           float kp,
                           float kd,
                           float torque_nm,
                           float torque_max_nm)
{
    uint32_t p = maita_float_to_uint(position_rad, MAITA_POS_MIN_RAD, MAITA_POS_MAX_RAD, 16u);
    uint32_t v = maita_float_to_uint(velocity_rad_s, MAITA_VEL_MIN_RAD_S, MAITA_VEL_MAX_RAD_S, 12u);
    uint32_t kp_raw = maita_float_to_uint(kp, 0.0f, MAITA_KP_MAX, 12u);
    uint32_t kd_raw = maita_float_to_uint(kd, 0.0f, MAITA_KD_MAX, 12u);
    uint32_t t = maita_float_to_uint(torque_nm, -torque_max_nm, torque_max_nm, 12u);

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)(p & 0xffu);
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)(((v & 0x0fu) << 4) | (kp_raw >> 8));
    data[4] = (uint8_t)(kp_raw & 0xffu);
    data[5] = (uint8_t)(kd_raw >> 4);
    data[6] = (uint8_t)(((kd_raw & 0x0fu) << 4) | (t >> 8));
    data[7] = (uint8_t)(t & 0xffu);
}

static int maita_unpack_mit_reply(const rx_can_frame *frame,
                                  unsigned int id,
                                  float torque_max_nm,
                                  maita_reply *reply)
{
    uint32_t p;
    uint32_t v;
    uint32_t t;

    if (frame == NULL || reply == NULL || frame->len < 8u ||
        frame->can_id != MAITA_MIT_REPLY_BASE + id ||
        frame->data[0] != (uint8_t)id) {
        return -1;
    }
    p = ((uint32_t)frame->data[1] << 8) | frame->data[2];
    v = ((uint32_t)frame->data[3] << 4) | ((uint32_t)frame->data[4] >> 4);
    t = (((uint32_t)frame->data[4] & 0x0fu) << 8) | frame->data[5];
    memset(reply, 0, sizeof(*reply));
    reply->id = id;
    reply->position_rad = maita_uint_to_float(p, MAITA_POS_MIN_RAD, MAITA_POS_MAX_RAD, 16u);
    reply->velocity_rad_s = maita_uint_to_float(v, MAITA_VEL_MIN_RAD_S, MAITA_VEL_MAX_RAD_S, 12u);
    reply->torque_nm = maita_uint_to_float(t, -torque_max_nm, torque_max_nm, 12u);
    return 0;
}

static int maita_unpack_status_reply(const rx_can_frame *frame,
                                     unsigned int id,
                                     unsigned int expected_cmd,
                                     maita_reply *reply)
{
    int16_t iq;
    int16_t speed;
    int16_t angle;

    if (frame == NULL || reply == NULL || frame->len < 8u ||
        frame->can_id != MAITA_REPLY_BASE + id ||
        frame->data[0] != (uint8_t)expected_cmd) {
        return -1;
    }
    iq = (int16_t)((uint16_t)frame->data[2] | ((uint16_t)frame->data[3] << 8));
    speed = (int16_t)((uint16_t)frame->data[4] | ((uint16_t)frame->data[5] << 8));
    angle = (int16_t)((uint16_t)frame->data[6] | ((uint16_t)frame->data[7] << 8));
    memset(reply, 0, sizeof(*reply));
    reply->id = id;
    reply->temperature_c = (int8_t)frame->data[1];
    reply->current_a = (float)iq * 0.01f;
    reply->speed_dps = (float)speed;
    reply->angle_deg = angle;
    return 0;
}

static int maita_unpack_zero_reply(const rx_can_frame *frame,
                                   unsigned int id,
                                   int32_t *encoder_offset)
{
    uint32_t raw;

    if (frame == NULL || encoder_offset == NULL || frame->len < 8u ||
        frame->can_id != MAITA_REPLY_BASE + id ||
        frame->data[0] != MAITA_CMD_ZERO_CURRENT) {
        return -1;
    }
    raw = (uint32_t)frame->data[4] |
          ((uint32_t)frame->data[5] << 8) |
          ((uint32_t)frame->data[6] << 16) |
          ((uint32_t)frame->data[7] << 24);
    *encoder_offset = (int32_t)raw;
    return 0;
}

static int maita_unpack_version_reply(const rx_can_frame *frame,
                                      unsigned int id,
                                      uint32_t *version_date)
{
    if (frame == NULL || version_date == NULL || frame->len < 8u ||
        frame->can_id != MAITA_REPLY_BASE + id ||
        frame->data[0] != MAITA_CMD_SW_VERSION) {
        return -1;
    }
    *version_date = (uint32_t)frame->data[4] |
                    ((uint32_t)frame->data[5] << 8) |
                    ((uint32_t)frame->data[6] << 16) |
                    ((uint32_t)frame->data[7] << 24);
    return 0;
}

static const char *maita_pid_name(unsigned int index)
{
    switch (index) {
    case 0x01u:
        return "current_kp";
    case 0x02u:
        return "current_ki";
    case 0x04u:
        return "speed_kp";
    case 0x05u:
        return "speed_ki";
    case 0x07u:
        return "position_kp";
    case 0x08u:
        return "position_ki";
    case 0x09u:
        return "position_kd";
    default:
        return "unknown";
    }
}

static int maita_unpack_pid_reply(const rx_can_frame *frame,
                                  unsigned int id,
                                  unsigned int index,
                                  float *value)
{
    uint32_t raw;

    if (frame == NULL || value == NULL || frame->len < 8u ||
        frame->can_id != MAITA_REPLY_BASE + id ||
        frame->data[0] != MAITA_CMD_PID_READ ||
        frame->data[1] != (uint8_t)index) {
        return -1;
    }
    raw = (uint32_t)frame->data[4] |
          ((uint32_t)frame->data[5] << 8) |
          ((uint32_t)frame->data[6] << 16) |
          ((uint32_t)frame->data[7] << 24);
    memcpy(value, &raw, sizeof(*value));
    return 0;
}

static const char *maita_accel_name(unsigned int index)
{
    switch (index) {
    case 0x00u:
        return "pos_accel";
    case 0x01u:
        return "pos_decel";
    case 0x02u:
        return "speed_accel";
    case 0x03u:
        return "speed_decel";
    default:
        return "unknown";
    }
}

static int maita_unpack_accel_reply(const rx_can_frame *frame,
                                    unsigned int id,
                                    unsigned int index,
                                    int32_t *value)
{
    uint32_t raw;

    if (frame == NULL || value == NULL || frame->len < 8u ||
        frame->can_id != MAITA_REPLY_BASE + id ||
        frame->data[0] != MAITA_CMD_ACCEL_READ ||
        frame->data[1] != (uint8_t)index) {
        return -1;
    }
    raw = (uint32_t)frame->data[4] |
          ((uint32_t)frame->data[5] << 8) |
          ((uint32_t)frame->data[6] << 16) |
          ((uint32_t)frame->data[7] << 24);
    *value = (int32_t)raw;
    return 0;
}

static int maita_parse_bus_id(flash_state *state,
                              int argc,
                              char **argv,
                              unsigned int *bus,
                              unsigned int *id)
{
    if (argc < 3 ||
        parse_uint_arg(argv[1], bus) != 0 ||
        parse_uint_arg(argv[2], id) != 0 ||
        *bus >= CANFD_DEVICE_NUM ||
        *id < MAITA_CAN_ID_MIN ||
        *id > MAITA_CAN_ID_MAX) {
        printf("%s: <bus> <id1-32>\n", console_text(state, "用法", "usage"));
        return -1;
    }
    return 0;
}

static void maita_print_data(const uint8_t *data, unsigned int len)
{
    unsigned int i;

    for (i = 0u; i < len; i++) {
        printf(" %02x", data[i]);
    }
}

static void maita_print_tx(unsigned int bus,
                           unsigned int can_id,
                           const uint8_t *data,
                           unsigned int len)
{
    printf("[maita]: tx bus=%u can_id=0x%03x len=%u data:", bus, can_id, len);
    maita_print_data(data, len);
    printf("\n");
}

static void maita_print_rx(const rx_can_frame *frame)
{
    printf("[maita]: rx bus=%u can_id=0x%03x len=%u flags=0x%02x data:",
           frame->bus, frame->can_id, frame->len, frame->flags);
    maita_print_data(frame->data, frame->len);
    printf("\n");
}

static int maita_send_frame(flash_state *state,
                            unsigned int bus,
                            unsigned int can_id,
                            const uint8_t data[8])
{
    unsigned int old_bus = state->bus;
    bool old_canfd = state->debug_use_canfd;
    bool old_input = state->show_motor_input;
    bool old_quiet = state->quiet_tx;
    int ret;

    state->bus = bus;
    state->debug_use_canfd = false;
    state->show_motor_input = false;
    state->quiet_tx = true;
    maita_print_tx(bus, can_id, data, 8u);
    ret = send_debug_packet(state, can_id, data, 8u);
    state->quiet_tx = old_quiet;
    state->show_motor_input = old_input;
    state->debug_use_canfd = old_canfd;
    state->bus = old_bus;
    return ret;
}

static int maita_wait_reply(flash_state *state,
                            unsigned int bus,
                            unsigned int expected_id,
                            unsigned int timeout_ms,
                            rx_can_frame *out)
{
    uint64_t deadline = time_us() + (uint64_t)timeout_ms * 1000ULL;

    while (!stop_requested && time_us() < deadline) {
        rx_can_frame frame;
        uint64_t now = time_us();
        unsigned int step_ms = 20u;

        if (deadline > now) {
            uint64_t remain_ms = (deadline - now) / 1000ULL;

            if (remain_ms < step_ms) {
                step_ms = (unsigned int)remain_ms + 1u;
            }
        }
        if (!frame_ring_pop(&state->frames, &frame, step_ms)) {
            continue;
        }
        if (frame.bus != bus) {
            continue;
        }
        if (frame.can_id != expected_id) {
            printf("[maita]: rx_other bus=%u can_id=0x%03x len=%u flags=0x%02x data:",
                   frame.bus, frame.can_id, frame.len, frame.flags);
            maita_print_data(frame.data, frame.len);
            printf("\n");
            continue;
        }
        maita_print_rx(&frame);
        *out = frame;
        return 0;
    }
    return -1;
}

static int maita_wait_status_reply(flash_state *state,
                                   unsigned int bus,
                                   unsigned int id,
                                   unsigned int timeout_ms,
                                   rx_can_frame *out)
{
    return maita_wait_reply(state, bus, MAITA_REPLY_BASE + id, timeout_ms, out);
}

static int maita_wait_mit_reply(flash_state *state,
                                unsigned int bus,
                                unsigned int id,
                                unsigned int timeout_ms,
                                rx_can_frame *out)
{
    return maita_wait_reply(state, bus, MAITA_MIT_REPLY_BASE + id, timeout_ms, out);
}

static int maita_send_command_wait(flash_state *state,
                                   const char *name,
                                   unsigned int bus,
                                   unsigned int id,
                                   const uint8_t data[8],
                                   unsigned int timeout_ms,
                                   unsigned int expected_cmd)
{
    bool old_monitor;
    bool old_input;
    rx_can_frame frame;
    maita_reply reply;
    bool parsed = false;
    int ret = -1;

    if (console_require_power(state) != 0) {
        return -1;
    }
    old_monitor = state->show_can_output;
    old_input = state->show_motor_input;
    state->show_can_output = false;
    state->show_motor_input = false;
    frame_ring_clear(&state->frames);
    printf("%s: start bus=%u id=%u\n", name, bus, id);
    if (maita_send_frame(state, bus, MAITA_SEND_BASE + id, data) != 0) {
        printf("%s: done success=0 failed=1\n", name);
        goto out;
    }
    if (maita_wait_status_reply(state, bus, id, timeout_ms, &frame) != 0) {
        printf("[maita%02u]: failed reason=no_reply timeout_ms=%u\n", id, timeout_ms);
        printf("%s: done success=0 failed=1\n", name);
        goto out;
    }
    if (expected_cmd == MAITA_CMD_ZERO_CURRENT) {
        int32_t encoder_offset;

        if (maita_unpack_zero_reply(&frame, id, &encoder_offset) == 0) {
            printf("[maita%02u]: encoder_offset=%d raw=0x%08x note=reset_required\n",
                   id, encoder_offset, (uint32_t)encoder_offset);
            parsed = true;
        }
    } else if (expected_cmd == MAITA_CMD_SW_VERSION) {
        uint32_t version_date;

        if (maita_unpack_version_reply(&frame, id, &version_date) == 0) {
            printf("[maita%02u]: version_date=%08u raw=0x%08x\n",
                   id, version_date, version_date);
            parsed = true;
        }
    } else if (maita_unpack_status_reply(&frame, id, expected_cmd, &reply) == 0) {
        printf("[maita%02u]: temp=%dC iq=%.2fA speed=%.0fdps angle=%ddeg\n",
               id, reply.temperature_c, reply.current_a, reply.speed_dps, reply.angle_deg);
        parsed = true;
    }
    if (!parsed) {
        printf("[maita%02u]: failed reason=parse_failed expected_cmd=0x%02x\n",
               id, expected_cmd);
        printf("%s: done success=0 failed=1\n", name);
        goto out;
    }
    printf("%s: done success=1 failed=0\n", name);
    ret = 0;
out:
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    return ret;
}

static int console_maita_power_on(flash_state *state, int argc, char **argv)
{
    unsigned int wait_ms = state->config.power_on_wait_ms;

    if (argc > 2 || (argc == 2 && parse_uint_arg(argv[1], &wait_ms) != 0)) {
        printf("%s: maita_power_on [wait_ms]\n", console_text(state, "用法", "usage"));
        return -1;
    }
    if (state->motor_power_on) {
        printf("maita_power_on: done success=1 failed=0 already_on\n");
        return 0;
    }
    printf("maita_power_on: start wait=%.3fs\n", (double)wait_ms / 1000.0);
    if (motor_pwr_set(1u) < 0) {
        printf("maita_power_on: done success=0 failed=1\n");
        return -1;
    }
    state->motor_power_on = true;
    console_quiet_sleep_ms(wait_ms);
    printf("maita_power_on: done success=1 failed=0\n");
    return 0;
}

static int console_maita_power_off(flash_state *state, int argc, char **argv)
{
    if (argc != 1) {
        printf("%s: maita_power_off\n", console_text(state, "用法", "usage"));
        return -1;
    }
    printf("maita_power_off: start\n");
    if (!state->motor_power_on) {
        printf("maita_power_off: done success=1 failed=0 already_off\n");
        return 0;
    }
    if (motor_pwr_set(0u) < 0) {
        printf("maita_power_off: done success=0 failed=1\n");
        return -1;
    }
    console_quiet_sleep_ms(300u);
    state->motor_power_on = false;
    printf("maita_power_off: done success=1 failed=0\n");
    return 0;
}

static int console_maita_info(flash_state *state, int argc, char **argv)
{
    unsigned int bus;
    unsigned int id;
    unsigned int timeout_ms = 1000u;
    float torque_max_nm = 18.0f;
    uint8_t data[8];
    rx_can_frame frame;
    maita_reply reply;
    bool old_monitor;
    bool old_input;
    int ret = -1;

    if (maita_parse_bus_id(state, argc, argv, &bus, &id) != 0 ||
        argc > 5 ||
        (argc >= 4 && parse_uint_arg(argv[3], &timeout_ms) != 0) ||
        (argc == 5 && parse_float_arg(argv[4], &torque_max_nm) != 0)) {
        printf("%s: maita_info <bus> <id> [timeout_ms] [torque_max_nm]\n",
               console_text(state, "用法", "usage"));
        return -1;
    }
    if (console_require_power(state) != 0) {
        return -1;
    }
    old_monitor = state->show_can_output;
    old_input = state->show_motor_input;
    state->show_can_output = false;
    state->show_motor_input = false;
    maita_pack_mit(data, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, torque_max_nm);
    frame_ring_clear(&state->frames);
    printf("maita_info: start bus=%u id=%u\n", bus, id);
    if (maita_send_frame(state, bus, MAITA_MIT_BASE + id, data) != 0 ||
        maita_wait_mit_reply(state, bus, id, timeout_ms, &frame) != 0) {
        printf("[maita%02u]: failed reason=no_reply timeout_ms=%u\n", id, timeout_ms);
        printf("maita_info: done success=0 failed=1\n");
        goto out;
    }
    if (maita_unpack_mit_reply(&frame, id, torque_max_nm, &reply) == 0) {
        printf("[maita%02u]: pos=%.5frad vel=%.5frad/s torque=%.5fNm\n",
               id, reply.position_rad, reply.velocity_rad_s, reply.torque_nm);
    } else {
        printf("[maita%02u]: reply raw only\n", id);
    }
    printf("maita_info: done success=1 failed=0\n");
    ret = 0;
out:
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    return ret;
}

static int maita_send_mit_wait(flash_state *state,
                               const char *name,
                               unsigned int bus,
                               unsigned int id,
                               float pos,
                               float torque,
                               float vel,
                               float kp,
                               float kd,
                               unsigned int timeout_ms,
                               float torque_max_nm)
{
    uint8_t data[8];
    rx_can_frame frame;
    maita_reply reply;
    bool old_monitor;
    bool old_input;
    int ret = -1;

    if (!isfinite(pos) || !isfinite(torque) || !isfinite(vel) ||
        !isfinite(kp) || !isfinite(kd) || !isfinite(torque_max_nm) ||
        torque_max_nm <= 0.0f) {
        printf("[maita%02u]: failed reason=invalid_float\n", id);
        return -1;
    }
    if (console_require_power(state) != 0) {
        return -1;
    }
    old_monitor = state->show_can_output;
    old_input = state->show_motor_input;
    state->show_can_output = false;
    state->show_motor_input = false;
    maita_pack_mit(data, pos, vel, kp, kd, torque, torque_max_nm);
    frame_ring_clear(&state->frames);
    printf("%s: start bus=%u id=%u\n", name, bus, id);
    printf("[maita%02u]: set pos=%.5frad vel=%.5frad/s torque=%.5fNm kp=%.3f kd=%.3f torque_max=%.3fNm\n",
           id, pos, vel, torque, kp, kd, torque_max_nm);
    if (maita_send_frame(state, bus, MAITA_MIT_BASE + id, data) != 0 ||
        maita_wait_mit_reply(state, bus, id, timeout_ms, &frame) != 0) {
        printf("[maita%02u]: failed reason=no_reply timeout_ms=%u\n", id, timeout_ms);
        printf("%s: done success=0 failed=1\n", name);
        goto out;
    }
    if (maita_unpack_mit_reply(&frame, id, torque_max_nm, &reply) == 0) {
        printf("[maita%02u]: pos=%.5frad vel=%.5frad/s torque=%.5fNm\n",
               id, reply.position_rad, reply.velocity_rad_s, reply.torque_nm);
    }
    printf("%s: done success=1 failed=0\n", name);
    ret = 0;
out:
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    return ret;
}

static int console_maita_mit_set(flash_state *state, int argc, char **argv)
{
    unsigned int bus;
    unsigned int id;
    unsigned int timeout_ms = 1000u;
    float pos;
    float torque;
    float vel;
    float kp;
    float kd;
    float torque_max_nm = 18.0f;

    if (maita_parse_bus_id(state, argc, argv, &bus, &id) != 0 ||
        argc < 8 || argc > 10 ||
        parse_float_arg(argv[3], &pos) != 0 ||
        parse_float_arg(argv[4], &torque) != 0 ||
        parse_float_arg(argv[5], &vel) != 0 ||
        parse_float_arg(argv[6], &kp) != 0 ||
        parse_float_arg(argv[7], &kd) != 0 ||
        (argc >= 9 && parse_uint_arg(argv[8], &timeout_ms) != 0) ||
        (argc == 10 && parse_float_arg(argv[9], &torque_max_nm) != 0)) {
        printf("%s: %s <bus> <id> <pos_rad> <torque_Nm> <vel_rad_s> <kp> <kd> [timeout_ms] [torque_max_nm]\n",
               console_text(state, "用法", "usage"),
               argv[0]);
        return -1;
    }
    return maita_send_mit_wait(state, argv[0], bus, id, pos, torque, vel, kp, kd,
                               timeout_ms, torque_max_nm);
}

static int console_maita_pos(flash_state *state, int argc, char **argv)
{
    unsigned int bus;
    unsigned int id;
    unsigned int timeout_ms = 1000u;
    float pos;
    float kp = 20.0f;
    float kd = 1.0f;
    float torque_max_nm = 18.0f;

    if (maita_parse_bus_id(state, argc, argv, &bus, &id) != 0 ||
        argc < 4 || argc > 8 ||
        parse_float_arg(argv[3], &pos) != 0 ||
        (argc >= 5 && parse_float_arg(argv[4], &kp) != 0) ||
        (argc >= 6 && parse_float_arg(argv[5], &kd) != 0) ||
        (argc >= 7 && parse_uint_arg(argv[6], &timeout_ms) != 0) ||
        (argc == 8 && parse_float_arg(argv[7], &torque_max_nm) != 0)) {
        printf("%s: maita_pos <bus> <id> <pos_rad> [kp] [kd] [timeout_ms] [torque_max_nm]\n",
               console_text(state, "用法", "usage"));
        return -1;
    }
    return maita_send_mit_wait(state, "maita_pos", bus, id, pos, 0.0f, 0.0f, kp, kd,
                               timeout_ms, torque_max_nm);
}

static int console_maita_enable(flash_state *state, int argc, char **argv)
{
    unsigned int bus;
    unsigned int id;
    unsigned int timeout_ms = 1000u;
    uint8_t data[8] = {MAITA_CMD_STOP, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

    if (maita_parse_bus_id(state, argc, argv, &bus, &id) != 0 ||
        argc > 4 ||
        (argc == 4 && parse_uint_arg(argv[3], &timeout_ms) != 0)) {
        printf("%s: maita_enable <bus> <id> [timeout_ms]\n", console_text(state, "用法", "usage"));
        return -1;
    }
    return maita_send_command_wait(state, "maita_enable", bus, id, data, timeout_ms,
                                   MAITA_CMD_STOP);
}

static int console_maita_disable(flash_state *state, int argc, char **argv)
{
    unsigned int bus;
    unsigned int id;
    unsigned int timeout_ms = 1000u;
    uint8_t data[8] = {MAITA_CMD_SHUTDOWN, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

    if (maita_parse_bus_id(state, argc, argv, &bus, &id) != 0 ||
        argc > 4 ||
        (argc == 4 && parse_uint_arg(argv[3], &timeout_ms) != 0)) {
        printf("%s: maita_disable <bus> <id> [timeout_ms]\n", console_text(state, "用法", "usage"));
        return -1;
    }
    return maita_send_command_wait(state, "maita_disable", bus, id, data, timeout_ms,
                                   MAITA_CMD_SHUTDOWN);
}

static int console_maita_torque(flash_state *state, int argc, char **argv)
{
    unsigned int bus;
    unsigned int id;
    unsigned int timeout_ms = 1000u;
    float current_a;
    int current_raw;
    uint8_t data[8] = {MAITA_CMD_TORQUE, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

    if (maita_parse_bus_id(state, argc, argv, &bus, &id) != 0 ||
        argc < 4 || argc > 5 ||
        parse_float_arg(argv[3], &current_a) != 0 ||
        (argc == 5 && parse_uint_arg(argv[4], &timeout_ms) != 0)) {
        printf("%s: maita_torque <bus> <id> <iq_A> [timeout_ms]\n",
               console_text(state, "用法", "usage"));
        return -1;
    }
    if (!isfinite(current_a) || current_a < -100.0f || current_a > 100.0f) {
        printf("[maita%02u]: failed reason=iq_out_of_range range=[-100,100]A\n", id);
        return -1;
    }
    current_raw = (int)(current_a * 100.0f + (current_a >= 0.0f ? 0.5f : -0.5f));
    data[4] = (uint8_t)((uint16_t)current_raw & 0xffu);
    data[5] = (uint8_t)(((uint16_t)current_raw >> 8) & 0xffu);
    return maita_send_command_wait(state, "maita_torque", bus, id, data, timeout_ms,
                                   MAITA_CMD_TORQUE);
}

static int console_maita_zero(flash_state *state, int argc, char **argv)
{
    unsigned int bus;
    unsigned int id;
    unsigned int timeout_ms = 1000u;
    uint8_t data[8] = {MAITA_CMD_ZERO_CURRENT, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

    if (maita_parse_bus_id(state, argc, argv, &bus, &id) != 0 ||
        argc > 4 ||
        (argc == 4 && parse_uint_arg(argv[3], &timeout_ms) != 0)) {
        printf("%s: maita_zero <bus> <id> [timeout_ms]\n", console_text(state, "用法", "usage"));
        return -1;
    }
    return maita_send_command_wait(state, "maita_zero", bus, id, data, timeout_ms,
                                   MAITA_CMD_ZERO_CURRENT);
}

static int console_maita_reset(flash_state *state, int argc, char **argv)
{
    unsigned int bus;
    unsigned int id;
    unsigned int wait_ms = 1000u;
    uint8_t data[8] = {MAITA_CMD_RESET, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    bool old_monitor;
    bool old_input;
    int ret = -1;

    if (maita_parse_bus_id(state, argc, argv, &bus, &id) != 0 ||
        argc > 4 ||
        (argc == 4 && parse_uint_arg(argv[3], &wait_ms) != 0)) {
        printf("%s: maita_reset <bus> <id> [wait_ms]\n",
               console_text(state, "用法", "usage"));
        return -1;
    }
    if (console_require_power(state) != 0) {
        return -1;
    }

    old_monitor = state->show_can_output;
    old_input = state->show_motor_input;
    state->show_can_output = false;
    state->show_motor_input = false;
    frame_ring_clear(&state->frames);
    printf("maita_reset: start bus=%u id=%u wait_ms=%u\n", bus, id, wait_ms);
    if (maita_send_frame(state, bus, MAITA_SEND_BASE + id, data) != 0) {
        printf("maita_reset: done success=0 failed=1\n");
        goto out;
    }
    console_quiet_sleep_ms(wait_ms);
    printf("[maita%02u]: reset sent note=no_reply_expected\n", id);
    printf("maita_reset: done success=1 failed=0\n");
    ret = 0;
out:
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    return ret;
}

static int console_maita_version(flash_state *state, int argc, char **argv)
{
    unsigned int bus;
    unsigned int id;
    unsigned int timeout_ms = 1000u;
    uint8_t data[8] = {MAITA_CMD_SW_VERSION, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

    if (maita_parse_bus_id(state, argc, argv, &bus, &id) != 0 ||
        argc > 4 ||
        (argc == 4 && parse_uint_arg(argv[3], &timeout_ms) != 0)) {
        printf("%s: maita_version <bus> <id> [timeout_ms]\n",
               console_text(state, "用法", "usage"));
        return -1;
    }
    return maita_send_command_wait(state, "maita_version", bus, id, data, timeout_ms,
                                   MAITA_CMD_SW_VERSION);
}

static int maita_read_one_pid(flash_state *state,
                              unsigned int bus,
                              unsigned int id,
                              unsigned int index,
                              unsigned int timeout_ms)
{
    uint8_t data[8] = {MAITA_CMD_PID_READ, (uint8_t)index, 0u, 0u, 0u, 0u, 0u, 0u};
    rx_can_frame frame;
    float value;

    frame_ring_clear(&state->frames);
    if (maita_send_frame(state, bus, MAITA_SEND_BASE + id, data) != 0) {
        printf("[maita%02u]: pid index=0x%02x %s failed reason=send\n",
               id, index, maita_pid_name(index));
        return -1;
    }
    if (maita_wait_status_reply(state, bus, id, timeout_ms, &frame) != 0) {
        printf("[maita%02u]: pid index=0x%02x %s failed reason=no_reply timeout_ms=%u\n",
               id, index, maita_pid_name(index), timeout_ms);
        return -1;
    }
    if (maita_unpack_pid_reply(&frame, id, index, &value) != 0) {
        printf("[maita%02u]: pid index=0x%02x %s failed reason=parse_failed\n",
               id, index, maita_pid_name(index));
        return -1;
    }
    printf("[maita%02u]: pid index=0x%02x %-11s value=%g\n",
           id, index, maita_pid_name(index), value);
    return 0;
}

static int console_maita_pid(flash_state *state, int argc, char **argv)
{
    static const unsigned int all_indices[] = {
        0x01u, 0x02u, 0x04u, 0x05u, 0x07u, 0x08u, 0x09u,
    };
    unsigned int bus;
    unsigned int id;
    unsigned int index = 0u;
    unsigned int timeout_ms = 1000u;
    bool old_monitor;
    bool old_input;
    size_t i;
    unsigned int success = 0u;
    unsigned int failed = 0u;

    if (maita_parse_bus_id(state, argc, argv, &bus, &id) != 0 ||
        argc > 5 ||
        (argc >= 4 && parse_uint_arg(argv[3], &index) != 0) ||
        (argc == 5 && parse_uint_arg(argv[4], &timeout_ms) != 0)) {
        printf("%s: maita_pid <bus> <id> [index] [timeout_ms]\n",
               console_text(state, "用法", "usage"));
        return -1;
    }
    if (console_require_power(state) != 0) {
        return -1;
    }

    old_monitor = state->show_can_output;
    old_input = state->show_motor_input;
    state->show_can_output = false;
    state->show_motor_input = false;
    printf("maita_pid: start bus=%u id=%u\n", bus, id);
    if (argc >= 4) {
        if (maita_read_one_pid(state, bus, id, index, timeout_ms) == 0) {
            success++;
        } else {
            failed++;
        }
    } else {
        for (i = 0u; i < sizeof(all_indices) / sizeof(all_indices[0]); i++) {
            if (maita_read_one_pid(state, bus, id, all_indices[i], timeout_ms) == 0) {
                success++;
            } else {
                failed++;
            }
        }
    }
    printf("maita_pid: done success=%u failed=%u\n", success, failed);
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    return failed == 0u ? 0 : -1;
}

static int maita_read_one_accel(flash_state *state,
                                unsigned int bus,
                                unsigned int id,
                                unsigned int index,
                                unsigned int timeout_ms)
{
    uint8_t data[8] = {MAITA_CMD_ACCEL_READ, (uint8_t)index, 0u, 0u, 0u, 0u, 0u, 0u};
    rx_can_frame frame;
    int32_t value;

    frame_ring_clear(&state->frames);
    if (maita_send_frame(state, bus, MAITA_SEND_BASE + id, data) != 0) {
        printf("[maita%02u]: accel index=0x%02x %s failed reason=send\n",
               id, index, maita_accel_name(index));
        return -1;
    }
    if (maita_wait_status_reply(state, bus, id, timeout_ms, &frame) != 0) {
        printf("[maita%02u]: accel index=0x%02x %s failed reason=no_reply timeout_ms=%u\n",
               id, index, maita_accel_name(index), timeout_ms);
        return -1;
    }
    if (maita_unpack_accel_reply(&frame, id, index, &value) != 0) {
        printf("[maita%02u]: accel index=0x%02x %s failed reason=parse_failed\n",
               id, index, maita_accel_name(index));
        return -1;
    }
    printf("[maita%02u]: accel index=0x%02x %-11s value=%d dps/s\n",
           id, index, maita_accel_name(index), value);
    return 0;
}

static int console_maita_accel(flash_state *state, int argc, char **argv)
{
    static const unsigned int all_indices[] = {
        0x00u, 0x01u, 0x02u, 0x03u,
    };
    unsigned int bus;
    unsigned int id;
    unsigned int index = 0u;
    unsigned int timeout_ms = 1000u;
    bool old_monitor;
    bool old_input;
    size_t i;
    unsigned int success = 0u;
    unsigned int failed = 0u;

    if (maita_parse_bus_id(state, argc, argv, &bus, &id) != 0 ||
        argc > 5 ||
        (argc >= 4 && parse_uint_arg(argv[3], &index) != 0) ||
        (argc == 5 && parse_uint_arg(argv[4], &timeout_ms) != 0)) {
        printf("%s: maita_accel <bus> <id> [index] [timeout_ms]\n",
               console_text(state, "用法", "usage"));
        return -1;
    }
    if (console_require_power(state) != 0) {
        return -1;
    }

    old_monitor = state->show_can_output;
    old_input = state->show_motor_input;
    state->show_can_output = false;
    state->show_motor_input = false;
    printf("maita_accel: start bus=%u id=%u\n", bus, id);
    if (argc >= 4) {
        if (maita_read_one_accel(state, bus, id, index, timeout_ms) == 0) {
            success++;
        } else {
            failed++;
        }
    } else {
        for (i = 0u; i < sizeof(all_indices) / sizeof(all_indices[0]); i++) {
            if (maita_read_one_accel(state, bus, id, all_indices[i], timeout_ms) == 0) {
                success++;
            } else {
                failed++;
            }
        }
    }
    printf("maita_accel: done success=%u failed=%u\n", success, failed);
    state->show_can_output = old_monitor;
    state->show_motor_input = old_input;
    return failed == 0u ? 0 : -1;
}
