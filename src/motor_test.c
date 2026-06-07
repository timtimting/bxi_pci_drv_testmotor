#define _DEFAULT_SOURCE

#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "bxi_motor_comm.h"
#include "bxi_pci_drv.h"

typedef struct
{
    unsigned int bus;
    unsigned int motor_id;
    unsigned int master_id;
    bool use_canfd;
    bool print_all;
    volatile unsigned int rx_count;
    volatile uint64_t last_tx_time_us;
    bxi_motor_limits limits;
} app_state;

static volatile sig_atomic_t stop_requested = 0;

static uint64_t timebase64_get(void)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static void sleep_ms(unsigned int ms)
{
    while (ms > 0u && !stop_requested) {
        unsigned int step = ms > 100u ? 100u : ms;
        usleep((useconds_t)step * 1000u);
        ms -= step;
    }
}

static void on_signal(int sig)
{
    (void)sig;
    stop_requested = 1;
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

static int can_rx_callback(void *arg, canfd_packet *msg)
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

    if (msg->frame.len >= BXI_MOTOR_MIT_LEN) {
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

static int parse_uint_arg(const char *text, unsigned int *value)
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

static int parse_float_arg(const char *text, float *value)
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

static void print_usage(const char *prog)
{
    printf("Usage:\n");
    printf("  sudo %s [options] listen [seconds]\n", prog);
    printf("  sudo %s [options] ping [wait_ms]\n", prog);
    printf("  sudo %s [options] enable|disable|zero|terminal-on|terminal-off [wait_ms]\n", prog);
    printf("  sudo %s [options] cmd <pos> <vel> <kp> <kd> <torque> [duration_ms] [period_ms]\n", prog);
    printf("  sudo %s [options] scan [max_id]\n", prog);
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
    printf("  sudo %s --power --bus 0 --id 1 ping\n", prog);
    printf("  sudo %s --bus 0 --id 1 enable\n", prog);
    printf("  sudo %s --bus 0 --id 1 cmd 0 0 0 0 0 1000 10\n", prog);
    printf("  sudo %s --bus 0 scan 8\n", prog);
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
    sleep_ms(wait_ms);
    if (state->rx_count == before) {
        printf("no matching feedback received in %u ms\n", wait_ms);
        return 1;
    }
    return 0;
}

static int run_listen(app_state *state, int argc, char **argv)
{
    unsigned int seconds = 10u;

    if (argc > 0 && parse_uint_arg(argv[0], &seconds) != 0) {
        fprintf(stderr, "invalid listen seconds: %s\n", argv[0]);
        return -1;
    }

    printf("listening for %u second(s), press Ctrl-C to stop\n", seconds);
    sleep_ms(seconds * 1000u);
    printf("received %u matching frame(s)\n", state->rx_count);
    return 0;
}

static int run_ping(app_state *state, int argc, char **argv)
{
    unsigned int wait_ms = 1000u;
    unsigned int before;

    if (argc > 0 && parse_uint_arg(argv[0], &wait_ms) != 0) {
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

    if (argc > 0 && parse_uint_arg(argv[0], &wait_ms) != 0) {
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

    if (parse_float_arg(argv[0], &position) != 0 ||
        parse_float_arg(argv[1], &velocity) != 0 ||
        parse_float_arg(argv[2], &kp) != 0 ||
        parse_float_arg(argv[3], &kd) != 0 ||
        parse_float_arg(argv[4], &torque) != 0) {
        fprintf(stderr, "invalid cmd float argument\n");
        return -1;
    }

    if (argc > 5 && parse_uint_arg(argv[5], &duration_ms) != 0) {
        fprintf(stderr, "invalid duration_ms: %s\n", argv[5]);
        return -1;
    }
    if (argc > 6 && parse_uint_arg(argv[6], &period_ms) != 0) {
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
    while (!stop_requested && timebase64_get() < end_time) {
        if (send_mit(state, position, velocity, kp, kd, torque) != 0) {
            return -1;
        }
        sleep_ms(period_ms);
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

    if (argc > 0 && parse_uint_arg(argv[0], &max_id) != 0) {
        fprintf(stderr, "invalid max_id: %s\n", argv[0]);
        return -1;
    }
    if (max_id > 8u) {
        max_id = 8u;
    }

    state->print_all = true;
    bxi_motor_pack_mit(data, &state->limits, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    for (id = 1u; id <= max_id && !stop_requested; id++) {
        if (send_packet(state, id, data, BXI_MOTOR_MIT_LEN) != 0) {
            return -1;
        }
        sleep_ms(20u);
    }

    sleep_ms(1000u);
    printf("scan finished, received %u frame(s)\n", state->rx_count);
    return 0;
}

static int run_command(app_state *state, int argc, char **argv)
{
    const char *cmd;

    if (argc <= 0) {
        print_usage("motor_test");
        return -1;
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

    fprintf(stderr, "unknown command: %s\n", cmd);
    return -1;
}

int main(int argc, char **argv)
{
    enum {
        OPT_CLASSIC = 1000,
        OPT_POWER,
        OPT_KEEP_POWER,
        OPT_ALL,
        OPT_P_MIN,
        OPT_P_MAX,
        OPT_V_MIN,
        OPT_V_MAX,
        OPT_T_MIN,
        OPT_T_MAX,
        OPT_KP_MAX,
        OPT_KD_MAX,
    };
    static const struct option long_options[] = {
        {"bus", required_argument, NULL, 'b'},
        {"id", required_argument, NULL, 'i'},
        {"master-id", required_argument, NULL, 'm'},
        {"classic", no_argument, NULL, OPT_CLASSIC},
        {"power", no_argument, NULL, OPT_POWER},
        {"keep-power", no_argument, NULL, OPT_KEEP_POWER},
        {"all", no_argument, NULL, OPT_ALL},
        {"p-min", required_argument, NULL, OPT_P_MIN},
        {"p-max", required_argument, NULL, OPT_P_MAX},
        {"v-min", required_argument, NULL, OPT_V_MIN},
        {"v-max", required_argument, NULL, OPT_V_MAX},
        {"t-min", required_argument, NULL, OPT_T_MIN},
        {"t-max", required_argument, NULL, OPT_T_MAX},
        {"kp-max", required_argument, NULL, OPT_KP_MAX},
        {"kd-max", required_argument, NULL, OPT_KD_MAX},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0},
    };
    app_state state;
    bool master_id_set = false;
    bool power_requested = false;
    bool keep_power = false;
    bool pci_started = false;
    int opt;
    int ret = 0;

    memset(&state, 0, sizeof(state));
    state.bus = 0u;
    state.motor_id = 1u;
    state.use_canfd = true;
    state.limits = bxi_motor_default_limits;

    while ((opt = getopt_long(argc, argv, "+b:i:m:h", long_options, NULL)) != -1) {
        switch (opt) {
        case 'b':
            if (parse_uint_arg(optarg, &state.bus) != 0) {
                fprintf(stderr, "invalid bus: %s\n", optarg);
                return 1;
            }
            break;
        case 'i':
            if (parse_uint_arg(optarg, &state.motor_id) != 0) {
                fprintf(stderr, "invalid id: %s\n", optarg);
                return 1;
            }
            break;
        case 'm':
            if (parse_uint_arg(optarg, &state.master_id) != 0) {
                fprintf(stderr, "invalid master-id: %s\n", optarg);
                return 1;
            }
            master_id_set = true;
            break;
        case OPT_CLASSIC:
            state.use_canfd = false;
            break;
        case OPT_POWER:
            power_requested = true;
            break;
        case OPT_KEEP_POWER:
            keep_power = true;
            break;
        case OPT_ALL:
            state.print_all = true;
            break;
        case OPT_P_MIN:
            if (parse_float_arg(optarg, &state.limits.p_min) != 0) {
                fprintf(stderr, "invalid p-min: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_P_MAX:
            if (parse_float_arg(optarg, &state.limits.p_max) != 0) {
                fprintf(stderr, "invalid p-max: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_V_MIN:
            if (parse_float_arg(optarg, &state.limits.v_min) != 0) {
                fprintf(stderr, "invalid v-min: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_V_MAX:
            if (parse_float_arg(optarg, &state.limits.v_max) != 0) {
                fprintf(stderr, "invalid v-max: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_T_MIN:
            if (parse_float_arg(optarg, &state.limits.t_min) != 0) {
                fprintf(stderr, "invalid t-min: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_T_MAX:
            if (parse_float_arg(optarg, &state.limits.t_max) != 0) {
                fprintf(stderr, "invalid t-max: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_KP_MAX:
            if (parse_float_arg(optarg, &state.limits.kp_max) != 0) {
                fprintf(stderr, "invalid kp-max: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_KD_MAX:
            if (parse_float_arg(optarg, &state.limits.kd_max) != 0) {
                fprintf(stderr, "invalid kd-max: %s\n", optarg);
                return 1;
            }
            break;
        case 'h':
            print_usage(argv[0]);
            return 0;
        default:
            print_usage(argv[0]);
            return 1;
        }
    }

    if (!master_id_set) {
        state.master_id = state.motor_id | 0x10u;
    }
    if (state.bus >= CANFD_DEVICE_NUM) {
        fprintf(stderr, "bus must be in [0, %u]\n", CANFD_DEVICE_NUM - 1u);
        return 1;
    }
    if (state.motor_id == 0u || state.motor_id > CAN_SFF_MASK) {
        fprintf(stderr, "id must be in [1, 0x%x]\n", CAN_SFF_MASK);
        return 1;
    }
    if (state.master_id > CAN_SFF_MASK) {
        fprintf(stderr, "master-id must be <= 0x%x\n", CAN_SFF_MASK);
        return 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    printf("config: bus=%u id=0x%x master_id=0x%x frame=%s\n",
           state.bus,
           state.motor_id,
           state.master_id,
           state.use_canfd ? "CANFD+BRS" : "classic CAN");

    if (bxi_pci_init(can_rx_callback, &state, -1) == -1) {
        fprintf(stderr, "bxi_pci_init failed\n");
        return 1;
    }
    pci_started = true;

    if (power_requested) {
        if (motor_pwr_set(1u) < 0) {
            fprintf(stderr, "motor_pwr_set(1) failed\n");
            ret = 1;
            goto out;
        }
        printf("motor power on, waiting for soft start\n");
        sleep_ms(2000u);
    }

    ret = run_command(&state, argc - optind, &argv[optind]);
    if (ret < 0) {
        ret = 1;
    }

out:
    if (power_requested && !keep_power) {
        motor_pwr_set(0u);
        printf("motor power off\n");
    }
    if (pci_started) {
        bxi_pci_exit();
    }

    return ret;
}
