#ifndef BXI_MOTOR_SHELL_MODULE_H
#define BXI_MOTOR_SHELL_MODULE_H

#include <stdbool.h>
#include <signal.h>
#include <stdint.h>

#include "bxi_motor_comm.h"
#include "bxi_pci_drv.h"

typedef struct
{
    unsigned int bus;
    unsigned int motor_id;
    unsigned int master_id;
    bool use_canfd;
    bool print_all;
    bool keep_power;
    bool power_enabled_by_tool;
    volatile unsigned int rx_count;
    volatile uint64_t last_tx_time_us;
    bxi_motor_limits limits;
} app_state;

extern volatile sig_atomic_t shell_stop_requested;

void shell_on_signal(int sig);
void shell_sleep_ms(unsigned int ms);
int shell_parse_uint_arg(const char *text, unsigned int *value);
int shell_parse_float_arg(const char *text, float *value);
void shell_print_usage(const char *prog);
int shell_can_rx_callback(void *arg, canfd_packet *msg);
int shell_run_command(app_state *state, int argc, char **argv);

#endif
