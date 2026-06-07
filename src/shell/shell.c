#define _DEFAULT_SOURCE

#include "shell/shell.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/time.h>
#include <unistd.h>

volatile sig_atomic_t shell_stop_requested = 0;

enum {
    CAN_CMD_REG_READ = 23,
    CAN_CMD_REG_WRITE = 24,
    CAN_CMD_REG_SAVE = 25,
    CAN_CMD_REG_INFO = 26,
};

enum {
    REG_TYPE_RAW = 0,
    REG_TYPE_INT,
    REG_TYPE_UINT,
    REG_TYPE_FLOAT,
};

typedef struct
{
    const char *name;
    unsigned int index;
    unsigned int count;
    unsigned int type;
} config_reg;

static const config_reg config_regs[] = {
    {"motor_pole_pairs", 0u, 1u, REG_TYPE_INT},
    {"motor_phase_resistance", 1u, 1u, REG_TYPE_FLOAT},
    {"motor_phase_inductance", 2u, 1u, REG_TYPE_FLOAT},
    {"inertia", 3u, 1u, REG_TYPE_FLOAT},
    {"reduction_ratio", 4u, 1u, REG_TYPE_FLOAT},
    {"encoder_dir_rev", 5u, 1u, REG_TYPE_INT},
    {"encoder_offset", 6u, 1u, REG_TYPE_INT},
    {"ldc1614_dir", 7u, 1u, REG_TYPE_INT},
    {"ldc1614_offset", 8u, 1u, REG_TYPE_FLOAT},
    {"ldc_channel1_max", 9u, 1u, REG_TYPE_UINT},
    {"ldc_channel1_min", 10u, 1u, REG_TYPE_UINT},
    {"ldc_channel1_middle", 11u, 1u, REG_TYPE_UINT},
    {"ldc_channel2_max", 12u, 1u, REG_TYPE_UINT},
    {"ldc_channel2_min", 13u, 1u, REG_TYPE_UINT},
    {"ldc_channel2_middle", 14u, 1u, REG_TYPE_UINT},
    {"offset_lut", 15u, 128u, REG_TYPE_INT},
    {"ldc1614_offset_lut", 143u, 360u, REG_TYPE_FLOAT},
    {"calib_valid", 503u, 1u, REG_TYPE_INT},
    {"ldc1614_calib_valid", 504u, 1u, REG_TYPE_INT},
    {"calib_current", 505u, 1u, REG_TYPE_FLOAT},
    {"calib_max_voltage", 506u, 1u, REG_TYPE_FLOAT},
    {"control_mode", 507u, 1u, REG_TYPE_INT},
    {"current_ramp_rate", 508u, 1u, REG_TYPE_FLOAT},
    {"vel_ramp_rate", 509u, 1u, REG_TYPE_FLOAT},
    {"traj_vel", 510u, 1u, REG_TYPE_FLOAT},
    {"traj_accel", 511u, 1u, REG_TYPE_FLOAT},
    {"traj_decel", 512u, 1u, REG_TYPE_FLOAT},
    {"pos_gain", 513u, 1u, REG_TYPE_FLOAT},
    {"vel_gain", 514u, 1u, REG_TYPE_FLOAT},
    {"vel_integrator_gain", 515u, 1u, REG_TYPE_FLOAT},
    {"vel_limit", 516u, 1u, REG_TYPE_FLOAT},
    {"current_limit", 517u, 1u, REG_TYPE_FLOAT},
    {"torquet_limit", 518u, 1u, REG_TYPE_FLOAT},
    {"torque_limit", 518u, 1u, REG_TYPE_FLOAT},
    {"current_ctrl_p_gain", 519u, 1u, REG_TYPE_FLOAT},
    {"current_ctrl_i_gain", 520u, 1u, REG_TYPE_FLOAT},
    {"current_ctrl_bandwidth", 521u, 1u, REG_TYPE_INT},
    {"protect_under_voltage", 522u, 1u, REG_TYPE_FLOAT},
    {"protect_over_voltage", 523u, 1u, REG_TYPE_FLOAT},
    {"protect_over_speed", 524u, 1u, REG_TYPE_FLOAT},
    {"protect_temperature_low", 525u, 1u, REG_TYPE_FLOAT},
    {"protect_temperature_high", 526u, 1u, REG_TYPE_FLOAT},
    {"can_id", 527u, 1u, REG_TYPE_INT},
    {"can_timeout_ms", 528u, 1u, REG_TYPE_INT},
    {"can_sync_target_enable", 529u, 1u, REG_TYPE_INT},
    {"mit_mode", 530u, 1u, REG_TYPE_INT},
    {"max_pos", 531u, 1u, REG_TYPE_FLOAT},
    {"max_vel", 532u, 1u, REG_TYPE_FLOAT},
    {"max_tor", 533u, 1u, REG_TYPE_FLOAT},
    {"kp_max", 534u, 1u, REG_TYPE_FLOAT},
    {"kd_max", 535u, 1u, REG_TYPE_FLOAT},
    {"usr_enc1_offset", 536u, 1u, REG_TYPE_INT},
    {"usr_enc2_offset", 537u, 1u, REG_TYPE_FLOAT},
    {"can_mode_switch", 538u, 1u, REG_TYPE_INT},
    {"master_id", 539u, 1u, REG_TYPE_INT},
    {"protect_i_abc_error", 540u, 1u, REG_TYPE_FLOAT},
    {"field_weaken_mode", 541u, 1u, REG_TYPE_INT},
    {"sync_boot_id_flag", 542u, 1u, REG_TYPE_INT},
    {"un_use", 543u, 16u, REG_TYPE_INT},
    {"crc", 559u, 1u, REG_TYPE_UINT},
};

static uint64_t timebase64_get(void)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

void shell_sleep_ms(unsigned int ms)
{
    while (ms > 0u && !shell_stop_requested) {
        unsigned int step = ms > 100u ? 100u : ms;
        usleep((useconds_t)step * 1000u);
        ms -= step;
    }
}

void shell_on_signal(int sig)
{
    (void)sig;
    shell_stop_requested = 1;
}

static unsigned int clean_can_id(canid_t can_id)
{
    if ((can_id & CAN_EFF_FLAG) != 0u) {
        return can_id & CAN_EFF_MASK;
    }
    return can_id & CAN_SFF_MASK;
}

static void print_data(const uint8_t *data, unsigned int len)
{
    unsigned int i;

    for (i = 0u; i < len; i++) {
        printf(" %02x", data[i]);
    }
}

static void u32_to_data(uint32_t value, uint8_t *data)
{
    data[0] = (uint8_t)((value >> 0) & 0xffu);
    data[1] = (uint8_t)((value >> 8) & 0xffu);
    data[2] = (uint8_t)((value >> 16) & 0xffu);
    data[3] = (uint8_t)((value >> 24) & 0xffu);
}

static uint32_t data_to_u32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 0) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static int32_t data_to_i32(const uint8_t *data)
{
    return (int32_t)data_to_u32(data);
}

static uint32_t float_to_u32(float value)
{
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    return raw;
}

static float u32_to_float(uint32_t raw)
{
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static const char *reg_type_name(unsigned int type)
{
    switch (type) {
    case REG_TYPE_INT:
        return "int";
    case REG_TYPE_UINT:
        return "uint";
    case REG_TYPE_FLOAT:
        return "float";
    default:
        return "raw";
    }
}

static int parse_i32_arg(const char *text, int32_t *value)
{
    char *end = NULL;
    long parsed;

    errno = 0;
    parsed = strtol(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' ||
        parsed < INT32_MIN || parsed > INT32_MAX) {
        return -1;
    }

    *value = (int32_t)parsed;
    return 0;
}

static const config_reg *find_config_reg(const char *name)
{
    size_t i;

    for (i = 0u; i < sizeof(config_regs) / sizeof(config_regs[0]); i++) {
        if (strcmp(config_regs[i].name, name) == 0) {
            return &config_regs[i];
        }
    }

    return NULL;
}

static int parse_reg_ref(const char *text,
                         unsigned int *index,
                         unsigned int *type,
                         char *name,
                         size_t name_len)
{
    unsigned int numeric_index;
    char base_name[96];
    const char *bracket;
    const config_reg *reg;
    unsigned int element = 0u;

    if (shell_parse_uint_arg(text, &numeric_index) == 0) {
        *index = numeric_index;
        *type = REG_TYPE_RAW;
        if (name_len > 0u) {
            snprintf(name, name_len, "%u", numeric_index);
        }
        return 0;
    }

    bracket = strchr(text, '[');
    if (bracket != NULL) {
        const char *end_bracket = strchr(bracket, ']');
        char element_text[32];
        size_t base_len = (size_t)(bracket - text);
        size_t element_len;

        if (end_bracket == NULL || end_bracket[1] != '\0' ||
            base_len == 0u || base_len >= sizeof(base_name)) {
            return -1;
        }

        element_len = (size_t)(end_bracket - bracket - 1);
        if (element_len == 0u || element_len >= sizeof(element_text)) {
            return -1;
        }

        memcpy(base_name, text, base_len);
        base_name[base_len] = '\0';
        memcpy(element_text, bracket + 1, element_len);
        element_text[element_len] = '\0';

        if (shell_parse_uint_arg(element_text, &element) != 0) {
            return -1;
        }
    } else {
        snprintf(base_name, sizeof(base_name), "%s", text);
    }

    reg = find_config_reg(base_name);
    if (reg == NULL) {
        return -1;
    }
    if (element >= reg->count) {
        return -1;
    }

    *index = reg->index + element;
    *type = reg->type;
    if (name_len > 0u) {
        if (reg->count > 1u || bracket != NULL) {
            snprintf(name, name_len, "%s[%u]", reg->name, element);
        } else {
            snprintf(name, name_len, "%s", reg->name);
        }
    }

    return 0;
}

static void remember_reg_request(app_state *state,
                                 unsigned int index,
                                 unsigned int type,
                                 const char *name)
{
    state->last_reg_index = index;
    state->last_reg_type = type;
    snprintf(state->last_reg_name, sizeof(state->last_reg_name), "%s", name != NULL ? name : "");
}

static void forget_reg_request(app_state *state)
{
    state->last_reg_index = 0u;
    state->last_reg_type = REG_TYPE_RAW;
    state->last_reg_name[0] = '\0';
}

static int is_reg_cmd_id(unsigned int can_id)
{
    unsigned int cmd = can_id >> 4;

    return cmd == CAN_CMD_REG_READ ||
           cmd == CAN_CMD_REG_WRITE ||
           cmd == CAN_CMD_REG_SAVE ||
           cmd == CAN_CMD_REG_INFO;
}

static int print_reg_reply(app_state *state, unsigned int can_id, const uint8_t *data, unsigned int len)
{
    unsigned int cmd = can_id >> 4;
    uint32_t value;
    int32_t status;

    if (!is_reg_cmd_id(can_id) || len < 8u) {
        return 0;
    }

    if (cmd == CAN_CMD_REG_INFO) {
        printf(" | reg_count:%u writable:%u", data_to_u32(&data[0]), data_to_u32(&data[4]));
        return 1;
    }

    status = data_to_i32(&data[0]);
    value = data_to_u32(&data[4]);
    if (state->last_reg_name[0] != '\0') {
        printf(" | reg:%s index:%u type:%s",
               state->last_reg_name,
               state->last_reg_index,
               reg_type_name(state->last_reg_type));
    }
    printf(" | status:%d value:0x%08x uint:%u int:%d float:%g",
           status,
           value,
           value,
           (int32_t)value,
           u32_to_float(value));
    return 1;
}

int shell_can_rx_callback(void *arg, canfd_packet *msg)
{
    app_state *state = (app_state *)arg;
    unsigned int can_id;
    bool candidate_reply;
    uint64_t now;
    uint64_t last_tx;

    if (state == NULL || msg == NULL) {
        return 0;
    }

    if (!state->print_all && msg->bus != state->bus) {
        return 0;
    }

    can_id = clean_can_id(msg->frame.can_id);
    candidate_reply = state->print_all || can_id == state->master_id ||
                      (state->last_tx_can_id != 0u && can_id == state->last_tx_can_id) ||
                      (msg->frame.len >= BXI_MOTOR_MIT_LEN && msg->frame.data[0] == state->motor_id);

    if (!candidate_reply) {
        return 0;
    }

    state->rx_count++;

    printf("rx can[%u] id:0x%03x len:%u flags:0x%02x data:",
           msg->bus,
           can_id,
           msg->frame.len,
           msg->frame.flags);
    print_data(msg->frame.data, msg->frame.len);

    if (print_reg_reply(state, can_id, msg->frame.data, msg->frame.len)) {
        /* Already decoded above. */
    } else if (msg->frame.len >= BXI_MOTOR_MIT_LEN) {
        bxi_motor_reply reply;

        if (bxi_motor_unpack_reply(msg->frame.data, msg->frame.len, &state->limits, &reply) == 0) {
            printf(" | motor:%u pos:% .5f vel:% .5f torque:% .5f t_mos:% .1fC t_motor:% .1fC",
                   reply.motor_id,
                   reply.position,
                   reply.velocity,
                   reply.torque,
                   reply.mos_temperature,
                   reply.motor_temperature);
        }
    }

    last_tx = state->last_tx_time_us;
    if (last_tx != 0u) {
        now = timebase64_get();
        printf(" dt:%llu us", (unsigned long long)(now - last_tx));
    }

    printf("\n");
    fflush(stdout);
    return 0;
}

int shell_parse_uint_arg(const char *text, unsigned int *value)
{
    char *end = NULL;
    unsigned long parsed;

    errno = 0;
    parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed > 0xfffffffful) {
        return -1;
    }

    *value = (unsigned int)parsed;
    return 0;
}

int shell_parse_float_arg(const char *text, float *value)
{
    char *end = NULL;
    float parsed;

    errno = 0;
    parsed = strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0') {
        return -1;
    }

    *value = parsed;
    return 0;
}

void shell_print_usage(const char *prog)
{
    printf("Usage:\n");
    printf("  sudo %s [options]\n", prog);
    printf("  sudo %s [options] terminal\n", prog);
    printf("  sudo %s [options] listen [seconds]\n", prog);
    printf("  sudo %s [options] ping [wait_ms]\n", prog);
    printf("  sudo %s [options] enable|disable|zero|terminal-on|terminal-off [wait_ms]\n", prog);
    printf("  sudo %s [options] cmd <pos> <vel> <kp> <kd> <torque> [duration_ms] [period_ms]\n", prog);
    printf("  sudo %s [options] scan [max_id]\n", prog);
    printf("  sudo %s [options] reg help|info|list|read|write|write-raw|write-float|save ...\n", prog);
    printf("\nOptions:\n");
    printf("  -b, --bus N          CANFD bus, default 0\n");
    printf("  -i, --id N           target motor can_id, default 1\n");
    printf("  -m, --master-id N    expected feedback frame ID, default id|0x10\n");
    printf("      --classic        send classic 8-byte CAN frames instead of CANFD+BRS\n");
    printf("      --power          turn motor power on before the command\n");
    printf("      --keep-power     do not turn motor power off on exit\n");
    printf("      --all            print all received frames on every bus\n");
    printf("      --p-min F        MIT position min, default -12.5\n");
    printf("      --p-max F        MIT position max, default 12.5\n");
    printf("      --v-min F        MIT velocity min, default -45\n");
    printf("      --v-max F        MIT velocity max, default 45\n");
    printf("      --t-min F        MIT torque min, default -18\n");
    printf("      --t-max F        MIT torque max, default 18\n");
    printf("      --kp-max F       MIT kp max, default 500\n");
    printf("      --kd-max F       MIT kd max, default 5\n");
    printf("  -h, --help           show this help\n");
    printf("\nExamples:\n");
    printf("  sudo %s --power --bus 0 --id 1\n", prog);
    printf("  sudo %s --power --bus 0 --id 1 ping\n", prog);
    printf("  sudo %s --bus 0 --id 1 enable\n", prog);
    printf("  sudo %s --bus 0 --id 1 cmd 0 0 0 0 0 1000 10\n", prog);
    printf("  sudo %s --bus 0 scan 8\n", prog);
    printf("  sudo %s --bus 0 --id 1 reg read can_id\n", prog);
}

static int send_packet(app_state *state, unsigned int can_id, const uint8_t *data, unsigned int len)
{
    canfd_packet packet;
    int ret;

    if (len > CANFD_MAX_DLEN) {
        fprintf(stderr, "frame length %u is too large\n", len);
        return -1;
    }

    memset(&packet, 0, sizeof(packet));
    packet.bus = state->bus;
    packet.frame.can_id = can_id;
    packet.frame.len = (uint8_t)len;
    packet.frame.flags = state->use_canfd ? (CANFD_BRS | CANFD_FDF) : 0u;
    memcpy(packet.frame.data, data, len);

    printf("tx can[%u] id:0x%03x len:%u flags:0x%02x data:",
           packet.bus,
           clean_can_id(packet.frame.can_id),
           packet.frame.len,
           packet.frame.flags);
    print_data(packet.frame.data, packet.frame.len);
    printf("\n");
    fflush(stdout);

    state->last_tx_can_id = clean_can_id(packet.frame.can_id);
    state->last_tx_time_us = timebase64_get();
    ret = canfd_send_packet(&packet, 1u);
    if (ret < 0) {
        fprintf(stderr, "canfd_send_packet failed: %d\n", ret);
        return -1;
    }

    return 0;
}

static int send_special(app_state *state, uint8_t command)
{
    uint8_t data[BXI_MOTOR_MIT_LEN];

    bxi_motor_pack_special(data, command);
    return send_packet(state, state->motor_id, data, BXI_MOTOR_MIT_LEN);
}

static int send_mit(app_state *state, float position, float velocity, float kp, float kd, float torque)
{
    uint8_t data[BXI_MOTOR_MIT_LEN];

    bxi_motor_pack_mit(data, &state->limits, position, velocity, kp, kd, torque);
    return send_packet(state, state->motor_id, data, BXI_MOTOR_MIT_LEN);
}

static int command_wait_reply(app_state *state, unsigned int wait_ms, unsigned int before)
{
    shell_sleep_ms(wait_ms);
    if (state->rx_count == before) {
        printf("no matching feedback received in %u ms\n", wait_ms);
        return 1;
    }
    return 0;
}

static int run_listen(app_state *state, int argc, char **argv)
{
    unsigned int seconds = 10u;

    if (argc > 0 && shell_parse_uint_arg(argv[0], &seconds) != 0) {
        fprintf(stderr, "invalid listen seconds: %s\n", argv[0]);
        return -1;
    }

    printf("listening for %u second(s), press Ctrl-C to stop\n", seconds);
    shell_sleep_ms(seconds * 1000u);
    printf("received %u matching frame(s)\n", state->rx_count);
    return 0;
}

static int run_ping(app_state *state, int argc, char **argv)
{
    unsigned int wait_ms = 1000u;
    unsigned int before;

    if (argc > 0 && shell_parse_uint_arg(argv[0], &wait_ms) != 0) {
        fprintf(stderr, "invalid wait_ms: %s\n", argv[0]);
        return -1;
    }

    before = state->rx_count;
    if (send_mit(state, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f) != 0) {
        return -1;
    }

    return command_wait_reply(state, wait_ms, before);
}

static int run_special(app_state *state, uint8_t command, int argc, char **argv)
{
    unsigned int wait_ms = 1000u;
    unsigned int before;

    if (argc > 0 && shell_parse_uint_arg(argv[0], &wait_ms) != 0) {
        fprintf(stderr, "invalid wait_ms: %s\n", argv[0]);
        return -1;
    }

    before = state->rx_count;
    if (send_special(state, command) != 0) {
        return -1;
    }

    return command_wait_reply(state, wait_ms, before);
}

static int run_cmd(app_state *state, int argc, char **argv)
{
    float position;
    float velocity;
    float kp;
    float kd;
    float torque;
    unsigned int duration_ms = 0u;
    unsigned int period_ms = 10u;
    uint64_t end_time;

    if (argc < 5) {
        fprintf(stderr, "cmd requires <pos> <vel> <kp> <kd> <torque>\n");
        return -1;
    }

    if (shell_parse_float_arg(argv[0], &position) != 0 ||
        shell_parse_float_arg(argv[1], &velocity) != 0 ||
        shell_parse_float_arg(argv[2], &kp) != 0 ||
        shell_parse_float_arg(argv[3], &kd) != 0 ||
        shell_parse_float_arg(argv[4], &torque) != 0) {
        fprintf(stderr, "invalid cmd float argument\n");
        return -1;
    }

    if (argc > 5 && shell_parse_uint_arg(argv[5], &duration_ms) != 0) {
        fprintf(stderr, "invalid duration_ms: %s\n", argv[5]);
        return -1;
    }
    if (argc > 6 && shell_parse_uint_arg(argv[6], &period_ms) != 0) {
        fprintf(stderr, "invalid period_ms: %s\n", argv[6]);
        return -1;
    }
    if (period_ms == 0u) {
        period_ms = 1u;
    }

    if (duration_ms == 0u) {
        unsigned int before = state->rx_count;

        if (send_mit(state, position, velocity, kp, kd, torque) != 0) {
            return -1;
        }
        return command_wait_reply(state, 1000u, before);
    }

    end_time = timebase64_get() + (uint64_t)duration_ms * 1000ULL;
    while (!shell_stop_requested && timebase64_get() < end_time) {
        if (send_mit(state, position, velocity, kp, kd, torque) != 0) {
            return -1;
        }
        shell_sleep_ms(period_ms);
    }

    printf("sent command stream for %u ms, received %u matching frame(s)\n",
           duration_ms,
           state->rx_count);
    return 0;
}

static int run_scan(app_state *state, int argc, char **argv)
{
    unsigned int max_id = 8u;
    unsigned int id;
    uint8_t data[BXI_MOTOR_MIT_LEN];

    if (argc > 0 && shell_parse_uint_arg(argv[0], &max_id) != 0) {
        fprintf(stderr, "invalid max_id: %s\n", argv[0]);
        return -1;
    }
    if (max_id > 8u) {
        max_id = 8u;
    }

    state->print_all = true;
    bxi_motor_pack_mit(data, &state->limits, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    for (id = 1u; id <= max_id && !shell_stop_requested; id++) {
        if (send_packet(state, id, data, BXI_MOTOR_MIT_LEN) != 0) {
            return -1;
        }
        shell_sleep_ms(20u);
    }

    shell_sleep_ms(1000u);
    printf("scan finished, received %u frame(s)\n", state->rx_count);
    return 0;
}

static int reg_frame_id(const app_state *state, unsigned int cmd, unsigned int *frame_id)
{
    if (state->motor_id == 0u || state->motor_id > 0x0fu) {
        fprintf(stderr, "reg commands require motor id in [1, 15], current id is 0x%x\n", state->motor_id);
        return -1;
    }

    *frame_id = (cmd << 4) | (state->motor_id & 0x0fu);
    return 0;
}

static int run_reg_info(app_state *state, int argc, char **argv)
{
    uint8_t data[4] = {0u, 0u, 0u, 0u};
    unsigned int frame_id;
    unsigned int wait_ms = 1000u;
    unsigned int before;

    if (argc > 0 && shell_parse_uint_arg(argv[0], &wait_ms) != 0) {
        fprintf(stderr, "invalid wait_ms: %s\n", argv[0]);
        return -1;
    }
    if (reg_frame_id(state, CAN_CMD_REG_INFO, &frame_id) != 0) {
        return -1;
    }

    forget_reg_request(state);
    before = state->rx_count;
    if (send_packet(state, frame_id, data, sizeof(data)) != 0) {
        return -1;
    }
    return command_wait_reply(state, wait_ms, before);
}

static int run_reg_read(app_state *state, int argc, char **argv)
{
    uint8_t data[4];
    unsigned int frame_id;
    unsigned int reg;
    unsigned int type;
    unsigned int wait_ms = 1000u;
    unsigned int before;
    char name[64];

    if (argc < 1 || parse_reg_ref(argv[0], &reg, &type, name, sizeof(name)) != 0) {
        fprintf(stderr, "usage: reg read <index|name|array[index]> [wait_ms]\n");
        return -1;
    }
    if (argc > 1 && shell_parse_uint_arg(argv[1], &wait_ms) != 0) {
        fprintf(stderr, "invalid wait_ms: %s\n", argv[1]);
        return -1;
    }
    if (reg_frame_id(state, CAN_CMD_REG_READ, &frame_id) != 0) {
        return -1;
    }

    u32_to_data(reg, data);
    remember_reg_request(state, reg, type, name);
    before = state->rx_count;
    if (send_packet(state, frame_id, data, sizeof(data)) != 0) {
        return -1;
    }
    return command_wait_reply(state, wait_ms, before);
}

static int send_reg_write_raw(app_state *state,
                              unsigned int reg,
                              uint32_t value,
                              unsigned int type,
                              const char *name,
                              unsigned int wait_ms)
{
    uint8_t data[8];
    unsigned int frame_id;
    unsigned int before;

    if (reg_frame_id(state, CAN_CMD_REG_WRITE, &frame_id) != 0) {
        return -1;
    }

    u32_to_data(reg, &data[0]);
    u32_to_data(value, &data[4]);
    remember_reg_request(state, reg, type, name);
    before = state->rx_count;
    if (send_packet(state, frame_id, data, sizeof(data)) != 0) {
        return -1;
    }
    return command_wait_reply(state, wait_ms, before);
}

static int run_reg_write(app_state *state, int argc, char **argv)
{
    unsigned int reg;
    unsigned int type;
    uint32_t value;
    unsigned int wait_ms = 1000u;
    char name[64];

    if (argc < 2 || parse_reg_ref(argv[0], &reg, &type, name, sizeof(name)) != 0) {
        fprintf(stderr, "usage: reg write <index|name|array[index]> <value> [wait_ms]\n");
        return -1;
    }

    if (type == REG_TYPE_FLOAT) {
        float parsed;

        if (shell_parse_float_arg(argv[1], &parsed) != 0) {
            fprintf(stderr, "invalid float value for %s: %s\n", name, argv[1]);
            return -1;
        }
        value = float_to_u32(parsed);
    } else if (type == REG_TYPE_INT) {
        int32_t parsed;

        if (parse_i32_arg(argv[1], &parsed) != 0) {
            fprintf(stderr, "invalid int value for %s: %s\n", name, argv[1]);
            return -1;
        }
        value = (uint32_t)parsed;
    } else {
        unsigned int parsed;

        if (shell_parse_uint_arg(argv[1], &parsed) != 0) {
            fprintf(stderr, "invalid u32 value for %s: %s\n", name, argv[1]);
            return -1;
        }
        value = (uint32_t)parsed;
    }

    if (argc > 2 && shell_parse_uint_arg(argv[2], &wait_ms) != 0) {
        fprintf(stderr, "invalid wait_ms: %s\n", argv[2]);
        return -1;
    }

    return send_reg_write_raw(state, reg, value, type, name, wait_ms);
}

static int run_reg_write_raw(app_state *state, int argc, char **argv)
{
    unsigned int reg;
    unsigned int type;
    unsigned int value;
    unsigned int wait_ms = 1000u;
    char name[64];

    if (argc < 2 ||
        parse_reg_ref(argv[0], &reg, &type, name, sizeof(name)) != 0 ||
        shell_parse_uint_arg(argv[1], &value) != 0) {
        fprintf(stderr, "usage: reg write-raw <index|name|array[index]> <u32_value> [wait_ms]\n");
        return -1;
    }
    if (argc > 2 && shell_parse_uint_arg(argv[2], &wait_ms) != 0) {
        fprintf(stderr, "invalid wait_ms: %s\n", argv[2]);
        return -1;
    }

    return send_reg_write_raw(state, reg, (uint32_t)value, type, name, wait_ms);
}

static int run_reg_write_float(app_state *state, int argc, char **argv)
{
    unsigned int reg;
    unsigned int type;
    unsigned int wait_ms = 1000u;
    float value;
    char name[64];

    if (argc < 2 ||
        parse_reg_ref(argv[0], &reg, &type, name, sizeof(name)) != 0 ||
        shell_parse_float_arg(argv[1], &value) != 0) {
        fprintf(stderr, "usage: reg write-float <index|name|array[index]> <float_value> [wait_ms]\n");
        return -1;
    }
    if (argc > 2 && shell_parse_uint_arg(argv[2], &wait_ms) != 0) {
        fprintf(stderr, "invalid wait_ms: %s\n", argv[2]);
        return -1;
    }

    return send_reg_write_raw(state, reg, float_to_u32(value), type, name, wait_ms);
}

static int run_reg_save(app_state *state, int argc, char **argv)
{
    uint8_t data[4] = {0u, 0u, 0u, 0u};
    unsigned int frame_id;
    unsigned int wait_ms = 1000u;
    unsigned int before;

    if (argc > 0 && shell_parse_uint_arg(argv[0], &wait_ms) != 0) {
        fprintf(stderr, "invalid wait_ms: %s\n", argv[0]);
        return -1;
    }
    if (reg_frame_id(state, CAN_CMD_REG_SAVE, &frame_id) != 0) {
        return -1;
    }

    forget_reg_request(state);
    before = state->rx_count;
    if (send_packet(state, frame_id, data, sizeof(data)) != 0) {
        return -1;
    }
    return command_wait_reply(state, wait_ms, before);
}

static int reg_matches_filter(const config_reg *reg, const char *filter)
{
    if (filter == NULL || filter[0] == '\0') {
        return 1;
    }

    return strstr(reg->name, filter) != NULL ||
           strstr(reg_type_name(reg->type), filter) != NULL;
}

static int run_reg_list(app_state *state, int argc, char **argv)
{
    const char *filter = argc > 0 ? argv[0] : NULL;
    size_t i;
    unsigned int shown = 0u;

    (void)state;

    printf("Configured registers from usr_config.h:\n");
    for (i = 0u; i < sizeof(config_regs) / sizeof(config_regs[0]); i++) {
        const config_reg *reg = &config_regs[i];
        unsigned int last_index = reg->index + reg->count - 1u;

        if (!reg_matches_filter(reg, filter)) {
            continue;
        }

        if (reg->count > 1u) {
            printf("  %-24s index:%3u..%-3u type:%-5s count:%u\n",
                   reg->name,
                   reg->index,
                   last_index,
                   reg_type_name(reg->type),
                   reg->count);
        } else {
            printf("  %-24s index:%3u     type:%-5s\n",
                   reg->name,
                   reg->index,
                   reg_type_name(reg->type));
        }
        shown++;
    }

    printf("shown %u register entr%s%s\n",
           shown,
           shown == 1u ? "y" : "ies",
           filter != NULL ? " matching filter" : "");
    return 0;
}

static void print_reg_help(void)
{
    printf("Register commands:\n");
    printf("  reg list [filter]                                      list usr_config.h registers\n");
    printf("  reg info [wait_ms]                                     read register count\n");
    printf("  reg read <index|name|array[index]> [wait_ms]           read one 32-bit register\n");
    printf("  reg write <index|name|array[index]> <value> [wait_ms]  write with known type\n");
    printf("  reg write-raw <index|name|array[index]> <u32> [wait_ms] write raw 32-bit value\n");
    printf("  reg write-float <index|name|array[index]> <f> [wait_ms] write IEEE-754 float bits\n");
    printf("  reg save [wait_ms]                                     save writable registers\n");
    printf("Examples: reg read can_id | reg write can_id 2 | reg write pos_gain 20 | reg read offset_lut[3]\n");
    printf("Reply status: 0=OK, -1=invalid addr, -2=read only, -3=save fail, -4=invalid len\n");
}

static int run_reg(app_state *state, int argc, char **argv)
{
    if (argc <= 0 || strcmp(argv[0], "help") == 0 || strcmp(argv[0], "?") == 0) {
        print_reg_help();
        return 0;
    }
    if (strcmp(argv[0], "info") == 0) {
        return run_reg_info(state, argc - 1, argv + 1);
    }
    if (strcmp(argv[0], "list") == 0) {
        return run_reg_list(state, argc - 1, argv + 1);
    }
    if (strcmp(argv[0], "read") == 0) {
        return run_reg_read(state, argc - 1, argv + 1);
    }
    if (strcmp(argv[0], "write") == 0) {
        return run_reg_write(state, argc - 1, argv + 1);
    }
    if (strcmp(argv[0], "write-raw") == 0) {
        return run_reg_write_raw(state, argc - 1, argv + 1);
    }
    if (strcmp(argv[0], "write-float") == 0) {
        return run_reg_write_float(state, argc - 1, argv + 1);
    }
    if (strcmp(argv[0], "save") == 0) {
        return run_reg_save(state, argc - 1, argv + 1);
    }

    fprintf(stderr, "unknown reg command: %s\n", argv[0]);
    print_reg_help();
    return -1;
}

static void print_config(const app_state *state)
{
    printf("config: bus=%u id=0x%x master_id=0x%x frame=%s print_all=%s\n",
           state->bus,
           state->motor_id,
           state->master_id,
           state->use_canfd ? "CANFD+BRS" : "classic CAN",
           state->print_all ? "on" : "off");
    printf("limits: p[%g,%g] v[%g,%g] kp[%g,%g] kd[%g,%g] torque[%g,%g]\n",
           state->limits.p_min,
           state->limits.p_max,
           state->limits.v_min,
           state->limits.v_max,
           state->limits.kp_min,
           state->limits.kp_max,
           state->limits.kd_min,
           state->limits.kd_max,
           state->limits.t_min,
           state->limits.t_max);
}

static void print_terminal_help(void)
{
    printf("Interactive commands:\n");
    printf("  Tab                             complete command or show candidates\n");
    printf("  q | quit | exit                 exit terminal\n");
    printf("  help                            show this help\n");
    printf("  config                          show current settings\n");
    printf("  bus <0-4>                       set CANFD bus\n");
    printf("  id <can_id>                     set target motor id, updates master_id=id|0x10\n");
    printf("  master-id <id>                  set feedback frame id\n");
    printf("  canfd | classic                 switch frame format\n");
    printf("  all on|off                      print all received frames\n");
    printf("  power on|off                    motor power control\n");
    printf("  listen [seconds]                print feedback\n");
    printf("  ping [wait_ms]                  send zero MIT frame and wait feedback\n");
    printf("  enable [wait_ms]                enter MIT motor mode\n");
    printf("  disable [wait_ms]               exit MIT motor mode\n");
    printf("  zero [wait_ms]                  save zero position, only in menu mode\n");
    printf("  terminal-on|terminal-off        toggle motor terminal output\n");
    printf("  cmd <p> <v> <kp> <kd> <t> [duration_ms] [period_ms]\n");
    printf("  scan [max_id]                   scan motor ids 1..max_id\n");
    printf("  reg help                        show CAN register commands from usr_config.h\n");
}

static int split_line(char *line, char **argv, int max_args)
{
    int argc = 0;
    char *p = line;

    while (*p != '\0' && argc < max_args) {
        while (*p != '\0' && isspace((unsigned char)*p)) {
            p++;
        }
        if (*p == '\0') {
            break;
        }

        argv[argc++] = p;
        while (*p != '\0' && !isspace((unsigned char)*p)) {
            p++;
        }
        if (*p != '\0') {
            *p++ = '\0';
        }
    }

    return argc;
}

static int parse_on_off(const char *text, bool *value)
{
    if (strcmp(text, "on") == 0 || strcmp(text, "1") == 0 || strcmp(text, "true") == 0) {
        *value = true;
        return 0;
    }
    if (strcmp(text, "off") == 0 || strcmp(text, "0") == 0 || strcmp(text, "false") == 0) {
        *value = false;
        return 0;
    }
    return -1;
}

static const char *const command_words[] = {
    "q",
    "quit",
    "exit",
    "help",
    "?",
    "config",
    "bus",
    "id",
    "master-id",
    "master",
    "canfd",
    "classic",
    "all",
    "power",
    "listen",
    "ping",
    "enable",
    "disable",
    "zero",
    "terminal-on",
    "terminal-off",
    "cmd",
    "scan",
    "terminal",
    "term",
    "reg",
};

static const char *const on_off_words[] = {
    "on",
    "off",
};

static const char *const reg_words[] = {
    "help",
    "info",
    "list",
    "read",
    "write",
    "write-raw",
    "write-float",
    "save",
};

static const char *const reg_name_words[] = {
    "motor_pole_pairs",
    "motor_phase_resistance",
    "motor_phase_inductance",
    "inertia",
    "reduction_ratio",
    "encoder_dir_rev",
    "encoder_offset",
    "ldc1614_dir",
    "ldc1614_offset",
    "ldc_channel1_max",
    "ldc_channel1_min",
    "ldc_channel1_middle",
    "ldc_channel2_max",
    "ldc_channel2_min",
    "ldc_channel2_middle",
    "offset_lut",
    "ldc1614_offset_lut",
    "calib_valid",
    "ldc1614_calib_valid",
    "calib_current",
    "calib_max_voltage",
    "control_mode",
    "current_ramp_rate",
    "vel_ramp_rate",
    "traj_vel",
    "traj_accel",
    "traj_decel",
    "pos_gain",
    "vel_gain",
    "vel_integrator_gain",
    "vel_limit",
    "current_limit",
    "torquet_limit",
    "torque_limit",
    "current_ctrl_p_gain",
    "current_ctrl_i_gain",
    "current_ctrl_bandwidth",
    "protect_under_voltage",
    "protect_over_voltage",
    "protect_over_speed",
    "protect_temperature_low",
    "protect_temperature_high",
    "can_id",
    "can_timeout_ms",
    "can_sync_target_enable",
    "mit_mode",
    "max_pos",
    "max_vel",
    "max_tor",
    "kp_max",
    "kd_max",
    "usr_enc1_offset",
    "usr_enc2_offset",
    "can_mode_switch",
    "master_id",
    "protect_i_abc_error",
    "field_weaken_mode",
    "sync_boot_id_flag",
    "un_use",
    "crc",
};

static int starts_with(const char *text, const char *prefix)
{
    return strncmp(text, prefix, strlen(prefix)) == 0;
}

static void redraw_prompt(const char *prompt, const char *line)
{
    printf("\r\033[2K%s%s", prompt, line);
    fflush(stdout);
}

static int current_token_start(const char *line, size_t len)
{
    int pos = (int)len;

    while (pos > 0 && !isspace((unsigned char)line[pos - 1])) {
        pos--;
    }

    return pos;
}

static unsigned int count_tokens_before(const char *line, int end)
{
    unsigned int count = 0u;
    int pos = 0;

    while (pos < end) {
        while (pos < end && isspace((unsigned char)line[pos])) {
            pos++;
        }
        if (pos >= end) {
            break;
        }
        count++;
        while (pos < end && !isspace((unsigned char)line[pos])) {
            pos++;
        }
    }

    return count;
}

static int copy_nth_token(const char *line, unsigned int target, char *out, size_t out_len)
{
    unsigned int count = 0u;
    size_t pos = 0u;

    if (out_len == 0u) {
        return -1;
    }

    while (line[pos] != '\0') {
        size_t out_pos = 0u;

        while (line[pos] != '\0' && isspace((unsigned char)line[pos])) {
            pos++;
        }
        if (line[pos] == '\0') {
            break;
        }

        if (count == target) {
            while (line[pos] != '\0' && !isspace((unsigned char)line[pos])) {
                if (out_pos + 1u < out_len) {
                    out[out_pos++] = line[pos];
                }
                pos++;
            }
            out[out_pos] = '\0';
            return 0;
        }

        count++;
        while (line[pos] != '\0' && !isspace((unsigned char)line[pos])) {
            pos++;
        }
    }

    out[0] = '\0';
    return -1;
}

static const char *const *completion_words_for_line(const char *line, size_t len, size_t *count)
{
    int start = current_token_start(line, len);
    char first[64];
    char second[64];
    unsigned int tokens_before = count_tokens_before(line, start);

    if (tokens_before == 0u) {
        *count = sizeof(command_words) / sizeof(command_words[0]);
        return command_words;
    }

    if (copy_nth_token(line, 0u, first, sizeof(first)) != 0) {
        *count = 0u;
        return NULL;
    }

    if ((strcmp(first, "power") == 0 || strcmp(first, "all") == 0) &&
        tokens_before == 1u) {
        *count = sizeof(on_off_words) / sizeof(on_off_words[0]);
        return on_off_words;
    }
    if (strcmp(first, "reg") == 0) {
        if (tokens_before == 1u) {
            *count = sizeof(reg_words) / sizeof(reg_words[0]);
            return reg_words;
        }

        if (tokens_before == 2u &&
            copy_nth_token(line, 1u, second, sizeof(second)) == 0 &&
            (strcmp(second, "read") == 0 ||
             strcmp(second, "write") == 0 ||
             strcmp(second, "write-raw") == 0 ||
             strcmp(second, "write-float") == 0 ||
             strcmp(second, "list") == 0)) {
            *count = sizeof(reg_name_words) / sizeof(reg_name_words[0]);
            return reg_name_words;
        }
    }

    *count = 0u;
    return NULL;
}

static void complete_line(char *line, size_t *len, size_t max_len, const char *prompt)
{
    const char *const *words;
    const char *matches[96];
    size_t word_count;
    size_t match_count = 0u;
    int token_start = current_token_start(line, *len);
    const char *prefix = &line[token_start];
    size_t prefix_len = *len - (size_t)token_start;
    size_t i;

    words = completion_words_for_line(line, *len, &word_count);
    if (words == NULL || word_count == 0u) {
        putchar('\a');
        fflush(stdout);
        return;
    }

    for (i = 0u; i < word_count && match_count < sizeof(matches) / sizeof(matches[0]); i++) {
        if (starts_with(words[i], prefix)) {
            matches[match_count++] = words[i];
        }
    }

    if (match_count == 0u) {
        putchar('\a');
        fflush(stdout);
        return;
    }

    if (match_count == 1u) {
        const char *match = matches[0];
        size_t match_len = strlen(match);
        size_t add_len;

        if (match_len < prefix_len) {
            return;
        }

        add_len = match_len - prefix_len;
        if (*len + add_len + 1u >= max_len) {
            putchar('\a');
            fflush(stdout);
            return;
        }

        memcpy(&line[*len], &match[prefix_len], add_len);
        *len += add_len;
        if (*len == match_len || line[*len - 1u] != ' ') {
            line[(*len)++] = ' ';
        }
        line[*len] = '\0';
        redraw_prompt(prompt, line);
        return;
    }

    printf("\n");
    for (i = 0u; i < match_count; i++) {
        printf("%s%s", matches[i], (i + 1u == match_count) ? "\n" : "  ");
    }
    redraw_prompt(prompt, line);
}

static int read_line_with_completion(const char *prompt, char *line, size_t max_len)
{
    struct termios old_term;
    struct termios new_term;
    size_t len = 0u;

    if (max_len == 0u) {
        return -1;
    }

    if (!isatty(STDIN_FILENO)) {
        if (fgets(line, (int)max_len, stdin) == NULL) {
            return -1;
        }
        return 0;
    }

    if (tcgetattr(STDIN_FILENO, &old_term) != 0) {
        return -1;
    }

    new_term = old_term;
    new_term.c_lflag &= (tcflag_t)~(ICANON | ECHO);
    new_term.c_cc[VMIN] = 1;
    new_term.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_term) != 0) {
        return -1;
    }

    line[0] = '\0';
    while (!shell_stop_requested) {
        unsigned char ch;
        ssize_t n = read(STDIN_FILENO, &ch, 1u);

        if (n <= 0) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
            return -1;
        }

        if (ch == '\r' || ch == '\n') {
            putchar('\n');
            line[len] = '\0';
            tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
            return 0;
        }
        if (ch == '\t') {
            complete_line(line, &len, max_len, prompt);
            continue;
        }
        if (ch == 4u) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
            return -1;
        }
        if (ch == 127u || ch == 8u) {
            if (len > 0u) {
                len--;
                line[len] = '\0';
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        }
        if (isprint(ch)) {
            if (len + 1u >= max_len) {
                putchar('\a');
                fflush(stdout);
                continue;
            }
            line[len++] = (char)ch;
            line[len] = '\0';
            putchar(ch);
            fflush(stdout);
            continue;
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
    return -1;
}

static int run_terminal_command(app_state *state, int argc, char **argv)
{
    const char *cmd;
    unsigned int value;
    bool enabled;

    if (argc == 0) {
        return 0;
    }

    cmd = argv[0];

    if (strcmp(cmd, "q") == 0 || strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) {
        return 1;
    }
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        print_terminal_help();
        return 0;
    }
    if (strcmp(cmd, "config") == 0) {
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "bus") == 0) {
        if (argc < 2 || shell_parse_uint_arg(argv[1], &value) != 0 || value >= CANFD_DEVICE_NUM) {
            printf("usage: bus <0-%u>\n", CANFD_DEVICE_NUM - 1u);
            return 0;
        }
        state->bus = value;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "id") == 0) {
        if (argc < 2 || shell_parse_uint_arg(argv[1], &value) != 0 || value == 0u || value > CAN_SFF_MASK) {
            printf("usage: id <1-0x%x>\n", CAN_SFF_MASK);
            return 0;
        }
        state->motor_id = value;
        state->master_id = value | 0x10u;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "master-id") == 0 || strcmp(cmd, "master") == 0) {
        if (argc < 2 || shell_parse_uint_arg(argv[1], &value) != 0 || value > CAN_SFF_MASK) {
            printf("usage: master-id <0-0x%x>\n", CAN_SFF_MASK);
            return 0;
        }
        state->master_id = value;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "canfd") == 0) {
        state->use_canfd = true;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "classic") == 0) {
        state->use_canfd = false;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "all") == 0) {
        if (argc < 2 || parse_on_off(argv[1], &enabled) != 0) {
            printf("usage: all on|off\n");
            return 0;
        }
        state->print_all = enabled;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "power") == 0) {
        if (argc < 2 || parse_on_off(argv[1], &enabled) != 0) {
            printf("usage: power on|off\n");
            return 0;
        }
        if (motor_pwr_set(enabled ? 1u : 0u) < 0) {
            printf("motor_pwr_set(%u) failed\n", enabled ? 1u : 0u);
            return 0;
        }
        state->power_enabled_by_tool = enabled;
        printf("motor power %s\n", enabled ? "on" : "off");
        if (enabled) {
            printf("waiting for soft start\n");
            shell_sleep_ms(2000u);
        }
        return 0;
    }
    if (strcmp(cmd, "listen") == 0) {
        run_listen(state, argc - 1, argv + 1);
        return 0;
    }
    if (strcmp(cmd, "ping") == 0) {
        run_ping(state, argc - 1, argv + 1);
        return 0;
    }
    if (strcmp(cmd, "enable") == 0) {
        run_special(state, BXI_MOTOR_CMD_ENABLE, argc - 1, argv + 1);
        return 0;
    }
    if (strcmp(cmd, "disable") == 0) {
        run_special(state, BXI_MOTOR_CMD_DISABLE, argc - 1, argv + 1);
        return 0;
    }
    if (strcmp(cmd, "zero") == 0) {
        run_special(state, BXI_MOTOR_CMD_ZERO, argc - 1, argv + 1);
        return 0;
    }
    if (strcmp(cmd, "terminal-on") == 0) {
        run_special(state, BXI_MOTOR_CMD_TERMINAL_ON, argc - 1, argv + 1);
        return 0;
    }
    if (strcmp(cmd, "terminal-off") == 0) {
        run_special(state, BXI_MOTOR_CMD_TERMINAL_OFF, argc - 1, argv + 1);
        return 0;
    }
    if (strcmp(cmd, "cmd") == 0) {
        run_cmd(state, argc - 1, argv + 1);
        return 0;
    }
    if (strcmp(cmd, "scan") == 0) {
        run_scan(state, argc - 1, argv + 1);
        return 0;
    }
    if (strcmp(cmd, "reg") == 0) {
        run_reg(state, argc - 1, argv + 1);
        return 0;
    }

    printf("unknown command: %s\n", cmd);
    printf("type help for commands, q to exit\n");
    return 0;
}

static int run_terminal(app_state *state)
{
    char line[256];
    char *argv[16];
    char prompt[64];

    printf("Entering interactive motor terminal. Type help for commands, Tab to complete, q to exit.\n");
    print_config(state);

    while (!shell_stop_requested) {
        int argc;

        snprintf(prompt, sizeof(prompt), "motor[%u:0x%x]> ", state->bus, state->motor_id);
        printf("%s", prompt);
        fflush(stdout);

        if (read_line_with_completion(prompt, line, sizeof(line)) != 0) {
            printf("\n");
            break;
        }

        argc = split_line(line, argv, (int)(sizeof(argv) / sizeof(argv[0])));
        if (run_terminal_command(state, argc, argv) != 0) {
            break;
        }
    }

    return 0;
}

int shell_run_command(app_state *state, int argc, char **argv)
{
    const char *cmd;

    if (argc <= 0) {
        return run_terminal(state);
    }

    cmd = argv[0];
    argc--;
    argv++;

    if (strcmp(cmd, "listen") == 0) {
        return run_listen(state, argc, argv);
    }
    if (strcmp(cmd, "ping") == 0) {
        return run_ping(state, argc, argv);
    }
    if (strcmp(cmd, "enable") == 0) {
        return run_special(state, BXI_MOTOR_CMD_ENABLE, argc, argv);
    }
    if (strcmp(cmd, "disable") == 0) {
        return run_special(state, BXI_MOTOR_CMD_DISABLE, argc, argv);
    }
    if (strcmp(cmd, "zero") == 0) {
        return run_special(state, BXI_MOTOR_CMD_ZERO, argc, argv);
    }
    if (strcmp(cmd, "terminal-on") == 0) {
        return run_special(state, BXI_MOTOR_CMD_TERMINAL_ON, argc, argv);
    }
    if (strcmp(cmd, "terminal-off") == 0) {
        return run_special(state, BXI_MOTOR_CMD_TERMINAL_OFF, argc, argv);
    }
    if (strcmp(cmd, "cmd") == 0) {
        return run_cmd(state, argc, argv);
    }
    if (strcmp(cmd, "scan") == 0) {
        return run_scan(state, argc, argv);
    }
    if (strcmp(cmd, "reg") == 0) {
        return run_reg(state, argc, argv);
    }
    if (strcmp(cmd, "terminal") == 0 || strcmp(cmd, "term") == 0) {
        return run_terminal(state);
    }

    fprintf(stderr, "unknown command: %s\n", cmd);
    return -1;
}
