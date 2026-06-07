#define _DEFAULT_SOURCE

#include <getopt.h>
#include <stdbool.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>

#include "bxi_motor_comm.h"
#include "bxi_pci_drv.h"
#include "shell/shell.h"

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
            if (shell_parse_uint_arg(optarg, &state.bus) != 0) {
                fprintf(stderr, "invalid bus: %s\n", optarg);
                return 1;
            }
            break;
        case 'i':
            if (shell_parse_uint_arg(optarg, &state.motor_id) != 0) {
                fprintf(stderr, "invalid id: %s\n", optarg);
                return 1;
            }
            break;
        case 'm':
            if (shell_parse_uint_arg(optarg, &state.master_id) != 0) {
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
            if (shell_parse_float_arg(optarg, &state.limits.p_min) != 0) {
                fprintf(stderr, "invalid p-min: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_P_MAX:
            if (shell_parse_float_arg(optarg, &state.limits.p_max) != 0) {
                fprintf(stderr, "invalid p-max: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_V_MIN:
            if (shell_parse_float_arg(optarg, &state.limits.v_min) != 0) {
                fprintf(stderr, "invalid v-min: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_V_MAX:
            if (shell_parse_float_arg(optarg, &state.limits.v_max) != 0) {
                fprintf(stderr, "invalid v-max: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_T_MIN:
            if (shell_parse_float_arg(optarg, &state.limits.t_min) != 0) {
                fprintf(stderr, "invalid t-min: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_T_MAX:
            if (shell_parse_float_arg(optarg, &state.limits.t_max) != 0) {
                fprintf(stderr, "invalid t-max: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_KP_MAX:
            if (shell_parse_float_arg(optarg, &state.limits.kp_max) != 0) {
                fprintf(stderr, "invalid kp-max: %s\n", optarg);
                return 1;
            }
            break;
        case OPT_KD_MAX:
            if (shell_parse_float_arg(optarg, &state.limits.kd_max) != 0) {
                fprintf(stderr, "invalid kd-max: %s\n", optarg);
                return 1;
            }
            break;
        case 'h':
            shell_print_usage(argv[0]);
            return 0;
        default:
            shell_print_usage(argv[0]);
            return 1;
        }
    }

    if (!master_id_set) {
        state.master_id = state.motor_id | 0x10u;
    }
    state.keep_power = keep_power;
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

    signal(SIGINT, shell_on_signal);
    signal(SIGTERM, shell_on_signal);

    printf("config: bus=%u id=0x%x master_id=0x%x frame=%s\n",
           state.bus,
           state.motor_id,
           state.master_id,
           state.use_canfd ? "CANFD+BRS" : "classic CAN");

    if (bxi_pci_init(shell_can_rx_callback, &state, -1) == -1) {
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
        state.power_enabled_by_tool = true;
        printf("motor power on, waiting for soft start\n");
        shell_sleep_ms(2000u);
        printf("running CAN warmup after power on\n");
        if (shell_can_warmup(&state, 130u, 10u) != 0) {
            ret = 1;
            goto out;
        }
    }

    ret = shell_run_command(&state, argc - optind, &argv[optind]);
    if (ret < 0) {
        ret = 1;
    }

out:
    if ((power_requested || state.power_enabled_by_tool) && !state.keep_power) {
        motor_pwr_set(0u);
        printf("motor power off\n");
    }
    if (pci_started) {
        bxi_pci_exit();
    }

    return ret;
}
