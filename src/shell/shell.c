#define _DEFAULT_SOURCE

#include "shell/shell.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

volatile sig_atomic_t shell_stop_requested = 0;

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

    printf("unknown command: %s\n", cmd);
    printf("type help for commands, q to exit\n");
    return 0;
}

static int run_terminal(app_state *state)
{
    char line[256];
    char *argv[16];

    printf("Entering interactive motor terminal. Type help for commands, q to exit.\n");
    print_config(state);

    while (!shell_stop_requested) {
        int argc;

        printf("motor[%u:0x%x]> ", state->bus, state->motor_id);
        fflush(stdout);

        if (fgets(line, sizeof(line), stdin) == NULL) {
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
    if (strcmp(cmd, "terminal") == 0 || strcmp(cmd, "term") == 0) {
        return run_terminal(state);
    }

    fprintf(stderr, "unknown command: %s\n", cmd);
    return -1;
}
