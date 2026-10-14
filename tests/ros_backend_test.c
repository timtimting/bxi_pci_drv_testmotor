#include "../ros2/bxi_motor_control/src/backend.c"
#include <assert.h>

int main(void)
{
    assert(bxi_ros_init("config/motor_console.yaml", "enp86s0", 0) == 0);
    struct bxi_ros_command command = {.action = 255};
    assert(bxi_ros_command(&command) == -1);
    console_output_context context = console_output_get_context();
    assert(context.sequence != 0); /* ROS calls must carry a notification/log ID. */
    ethercat_scan_wait_sequence = context.sequence;
    ethercat_scan_wait_done = false;
    console_ethercat_scan_notify(0);
    assert(ethercat_scan_wait_done && ethercat_scan_wait_result == 0);
    ethercat_scan_wait_sequence = 0;
    ethercat_scan_wait_done = false;
    command.action = 7;
    assert(bxi_ros_command(&command) == -1); /* No PCI access unless enabled. */
    assert(console_output_get_context().sequence > context.sequence);
    command.action = 3; command.slave_id = 256;
    assert(bxi_ros_command(&command) == -1);
    command.slave_id = 1; command.position = NAN;
    assert(bxi_ros_command(&command) == -1);
    struct bxi_ros_state state[2];
    assert(bxi_ros_feedback(state, 2) == 0);
    ethercat_cached_slave *cached = &ethercat_cached_slaves[1];
    cached->valid = cached->live_valid = true;
    cached->obytes = 23; cached->ibytes = 16;
    cached->live_timestamp_us = time_us();
    cached->outputs[22] = 10;
    cached->pn082 = 10; cached->config_valid_mask = ETHERCAT_CACHE_PN082;
    console_ethercat_write_float(cached->inputs + 2, 0.75f);
    console_ethercat_write_float(cached->outputs + 14, 50.5f);
    assert(bxi_ros_feedback(state, 2) == 1);
    assert(state[0].valid && state[0].slave_id == 1);
    assert(state[0].position == 0.75f && state[0].kp == 50.5f);
    assert(state[0].pn082_valid && state[0].pn082 == 10);
    cached->live_timestamp_us -= 200000;
    assert(bxi_ros_feedback(state, 2) == 1 && !state[0].valid);
    struct bxi_ros_command frame[2] = {{.slave_id=1,.position=0.5f,.kp=10},
                                      {.slave_id=2,.position=-0.5f,.kp=20}};
    assert(bxi_ros_batch(frame, 2, 1000) == -1); /* No active PDO worker. */
    cached->live_timestamp_us = time_us();
    ethercat_cached_slaves[2] = *cached;
    ethercat_background_started = true; /* Mock the worker; inspect one atomic queued frame. */
    assert(bxi_ros_batch(frame, 2, 1000) == 0 && ethercat_command_count == 1);
    assert(ethercat_command_queue[ethercat_command_head].batch_count == 2);
    assert(ethercat_command_queue[ethercat_command_head].batch[1].position == -0.5f);
    frame[1].slave_id = 1;
    assert(bxi_ros_batch(frame, 2, 1000) == -1 && ethercat_command_count == 1);
    frame[1].slave_id = 2; frame[1].kd = -1;
    assert(bxi_ros_batch(frame, 2, 1000) == -1 && ethercat_command_count == 1);
    ethercat_background_started = false;
    ethercat_command_count = 0;
    bxi_ros_request_stop();
    assert(stop_requested);
    bxi_ros_shutdown();
    puts("PASS: ROS backend initialization without PCI, command validation, feedback and stale-sample handling");
    return 0;
}
