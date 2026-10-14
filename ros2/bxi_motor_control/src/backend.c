/* Embed the same backend as motor_console; ROS owns process lifecycle. */
#define MOTOR_CONSOLE_EMBEDDED
#include "../../../src/main.c"
#include "backend.h"

static flash_state ros_state;
static char ros_interface[IFNAMSIZ];
static bool ros_initialized;

int bxi_ros_init(const char *config, const char *interface, int init_pci)
{
    if (ros_initialized || console_ethercat_validate_interface(interface) != 0) return -1;
    memset(&ros_state, 0, sizeof(ros_state));
    stop_requested = 0;
    ros_state.boot_id = 1;
    ros_state.limits = bxi_motor_default_limits;
    ros_state.debug_use_canfd = true;
    ros_state.quiet_tx = true;
    ros_state.language_override_active = true;
    update_boot_ids(&ros_state);
    rx_ring_init(&ros_state.rx);
    frame_ring_init(&ros_state.frames);
    if (console_reload_config(&ros_state, config) != 0) return -1;
    snprintf(ros_interface, sizeof(ros_interface), "%s", interface);
    if (init_pci) {
        if (bxi_pci_init(can_rx_callback, &ros_state, -1) == -1) return -1;
        ros_state.pci_started = true;
    }
    ros_initialized = true;
    return 0;
}

int bxi_ros_scan(void)
{
    return console_ethercat_background_submit(ETHERCAT_TASK_SCAN, false,
        ros_interface, "all", 0, 0, 0, NULL, NULL);
}

int bxi_ros_command(const struct bxi_ros_command *command)
{
    if (!ros_initialized || !command || command->slave_id >= EC_MAXSLAVE) return -1;
    char selection[16];
    if (command->slave_id) snprintf(selection, sizeof(selection), "%u", command->slave_id);
    else snprintf(selection, sizeof(selection), "all");
    ethercat_task task;
    unsigned int value = 0;
    unsigned int duration = 0;
    switch (command->action) {
    case 1: task = ETHERCAT_TASK_ENABLE; duration = command->duration_ms; break;
    case 2: task = ETHERCAT_TASK_DISABLE; break;
    case 3: {
        float values[4] = {command->velocity, command->torque, command->kp, command->kd};
        return console_ethercat_background_submit_ex(ETHERCAT_TASK_MIT, false,
            ros_interface, selection, command->duration_ms, command->position, 0,
            NULL, NULL, ETHERCAT_CIA402_MODE_MIT, 0, 0, 0, values);
    }
    case 4: task = ETHERCAT_TASK_PN082; value = command->pn082; break;
    case 5: task = ETHERCAT_TASK_SAVE; break;
    case 6: task = ETHERCAT_TASK_QUALITY; duration = command->duration_ms; break;
    case 7: return ros_state.pci_started ? console_power_on(&ros_state) : -1;
    case 8: return ros_state.pci_started ? console_power_off(&ros_state) : -1;
    default: return -1;
    }
    return console_ethercat_background_submit(task, false, ros_interface,
        selection, duration, 0, value, NULL, NULL);
}

size_t bxi_ros_feedback(struct bxi_ros_state *out, size_t capacity)
{
    size_t count = 0;
    uint64_t now = time_us();
    pthread_mutex_lock(&ethercat_background_mutex);
    for (int slave = 1; slave < EC_MAXSLAVE && count < capacity; ++slave) {
        const ethercat_cached_slave *cached = &ethercat_cached_slaves[slave];
        if (!cached->valid) continue;
        struct bxi_ros_state *s = &out[count++];
        memset(s, 0, sizeof(*s));
        s->slave_id = slave;
        s->sample_age_ms = cached->live_valid && now >= cached->live_timestamp_us ?
            (now - cached->live_timestamp_us) / 1000.0 : -1.0;
        s->valid = cached->live_valid && cached->obytes == 23 && cached->ibytes == 16 &&
                   cached->outputs[22] == ETHERCAT_CIA402_MODE_MIT && s->sample_age_ms <= 100.0;
        s->pn082_valid = (cached->config_valid_mask & ETHERCAT_CACHE_PN082) != 0;
        s->pn082 = cached->pn082;
        if (!cached->live_valid) continue;
        s->status_word = console_ethercat_read_u16(cached->inputs);
        s->control_word = console_ethercat_read_u16(cached->outputs);
        s->error_code = cached->live_error_code;
        s->mode = cached->outputs[22];
        if (!s->valid) continue;
        s->position = console_ethercat_read_float(cached->inputs + 2);
        s->velocity = console_ethercat_read_float(cached->inputs + 6);
        s->torque = console_ethercat_read_float(cached->inputs + 10);
        s->target_position = console_ethercat_read_float(cached->outputs + 2);
        s->target_velocity = console_ethercat_read_float(cached->outputs + 6);
        s->target_torque = console_ethercat_read_float(cached->outputs + 10);
        s->kp = console_ethercat_read_float(cached->outputs + 14);
        s->kd = console_ethercat_read_float(cached->outputs + 18);
    }
    pthread_mutex_unlock(&ethercat_background_mutex);
    return count;
}

void bxi_ros_request_stop(void)
{
    stop_requested = 1;
}

void bxi_ros_shutdown(void)
{
    if (!ros_initialized) return;
    console_ethercat_background_stop();
    if (ros_state.motor_power_on) console_power_off(&ros_state);
    if (ros_state.pci_started) bxi_pci_exit();
    ros_initialized = false;
}
