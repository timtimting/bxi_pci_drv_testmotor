#define main runtime_embedded_main
#include "../src/lib/runtime.c"
#undef main
#include "../src/lib/ethercat_output.c"
#undef printf
#define printf(...) console_ethercat_printf(__VA_ARGS__)
#include "../src/lib/ethercat_scan.c"
#include "../src/lib/ethercat_background.c"
#undef printf
#include <assert.h>


static uint8_t domain_data[14 * 39];
int ecrt_master_activate(ec_master_t *m) { (void)m; return 0; }
uint8_t *ecrt_domain_data(const ec_domain_t *d) { (void)d; return domain_data; }
int ecrt_master_receive(ec_master_t *m) { (void)m; return 0; }
int ecrt_domain_process(ec_domain_t *d) { (void)d; return 0; }
int ecrt_domain_state(const ec_domain_t *d, ec_domain_state_t *s) {
    (void)d; s->working_counter = (unsigned int)ec_slavecount * 3; return 0;
}
int ecrt_slave_config_state(const ec_slave_config_t *c, ec_slave_config_state_t *s) {
    (void)c; memset(s, 0, sizeof(*s)); s->al_state = EC_STATE_OPERATIONAL;
    s->online = 1; s->operational = 1; return 0;
}
int ecrt_domain_queue(ec_domain_t *d) { (void)d; return 0; }
int ecrt_master_application_time(ec_master_t *m, uint64_t t) { (void)m; (void)t; return 0; }
int ecrt_master_send(ec_master_t *m) {
    (void)m;
    for (int slave = 1; slave <= ec_slavecount; ++slave) {
        uint16_t word = console_ethercat_read_u16(ec_slave[slave].outputs);
        uint16_t state = word == 15 ? 0x27 : word == 7 ? 0x23 : word == 6 ? 0x21 : 0x40;
        console_ethercat_write_u16(ec_slave[slave].inputs, state);
    }
    return 78;
}
int ecrt_master_sync_reference_clock(ec_master_t *m) { (void)m; return 0; }
int ecrt_master_sync_slave_clocks(ec_master_t *m) { (void)m; return 0; }
int ecrt_master_reference_clock_time(const ec_master_t *m, uint32_t *t) {
    (void)m; *t = 0; return 0;
}

int main(void) {
    ethercat_mit_mapping = true;
    ethercat_session.active = true;
    etherlab_master = (ec_master_t *)1;
    etherlab_domain = (ec_domain_t *)1;
    etherlab_activated = true;
    etherlab_domain_data = domain_data;
    ec_slavecount = 2;
    ec_group[0].outputsWKC = 2; ec_group[0].inputsWKC = 2;
    ethercat_expected_work_counter = 6;
    ethercat_control_period_ns = 1000000;
    strcpy(ethercat_background_arguments.interface, "enp86s0");
    for (int slave = 1; slave <= 2; ++slave) {
        ethercat_session.configured[slave] = 1;
        etherlab_slaves[slave].config = (ec_slave_config_t *)1;
        etherlab_slaves[slave].output_offset = (slave - 1) * 39;
        ec_slave[slave].eep_man = ETHERCAT_KAIXUAN_VENDOR_ID;
        ec_slave[slave].eep_id = ETHERCAT_KAIXUAN_PRODUCT_CODE;
        ec_slave[slave].state = EC_STATE_OPERATIONAL;
        ec_slave[slave].outputs = etherlab_slaves[slave].output_shadow;
        ec_slave[slave].inputs = domain_data + (slave - 1) * 39 + 23;
        ec_slave[slave].Obits = 184; ec_slave[slave].Obytes = 23;
        ec_slave[slave].Ibits = 128; ec_slave[slave].Ibytes = 16;
        console_ethercat_write_u16(ec_slave[slave].inputs, 0x40);
        console_ethercat_write_float((uint8_t *)ec_slave[slave].inputs + 2, 0.75f * slave);
    }
    ethercat_background_args cmd = {.task = ETHERCAT_TASK_ENABLE,
        .position_mode = ETHERCAT_CIA402_MODE_CSP, .interface = "enp86s0", .selection = "1"};
    assert(console_ethercat_session_command(&cmd) == 0);
    uint8_t *out = ec_slave[1].outputs;
    assert(out[22] == 10 && console_ethercat_read_u16(out) == 15);
    assert(console_ethercat_read_float(out + 2) == 0.75f);
    assert(console_ethercat_read_float(out + 14) == 0 && console_ethercat_read_float(out + 18) == 0);
    assert(console_ethercat_read_u16(ec_slave[2].outputs) == 0);
    cmd.task = ETHERCAT_TASK_POSITION; cmd.output_rad = 0.5;
    assert(console_ethercat_session_command(&cmd) == -1); /* Zero KP cannot hold position. */
    assert(console_ethercat_read_float(out + 2) == 0.75f);
    cmd.task = ETHERCAT_TASK_MIT; cmd.mit_kp = 50.5f; cmd.mit_kd = 10.25f;
    assert(console_ethercat_session_command(&cmd) == 0);
    cmd.task = ETHERCAT_TASK_ENABLE;
    assert(console_ethercat_session_command(&cmd) == 0); /* Repeated enable retains gains. */
    assert(console_ethercat_read_float(out + 14) == 50.5f);
    cmd.task = ETHERCAT_TASK_POSITION; cmd.output_rad = -0.25;
    assert(console_ethercat_session_command(&cmd) == 0);
    assert(console_ethercat_read_float(out + 2) == -0.25f);
    assert(console_ethercat_read_float(out + 14) == 50.5f && console_ethercat_read_float(out + 18) == 10.25f);
    strcpy(cmd.selection, "all");
    assert(console_ethercat_session_command(&cmd) == -1); /* Validate every slave before changing any. */
    assert(console_ethercat_read_float(out + 2) == -0.25f);
    cmd.task = ETHERCAT_TASK_ENABLE;
    assert(console_ethercat_session_command(&cmd) == 0);
    assert(console_ethercat_read_u16(ec_slave[2].outputs) == 15);
    cmd.task = ETHERCAT_TASK_DISABLE;
    assert(console_ethercat_session_command(&cmd) == 0);
    assert(console_ethercat_read_u16(out) == 0 && console_ethercat_read_u16(ec_slave[2].outputs) == 0);
    cmd.task = ETHERCAT_TASK_MIT_BATCH; cmd.batch_count = 2; cmd.hold_ms = 1000;
    cmd.batch[0] = (ethercat_mit_target){1, 0.25f, 0.5f, 1.0f, 50.5f, 10.25f};
    cmd.batch[1] = (ethercat_mit_target){2, -0.5f, -0.25f, 2.0f, 20.5f, 3.25f};
    assert(console_ethercat_session_command(&cmd) == 0);
    assert(console_ethercat_read_float(out + 2) == 0.25f);
    assert(console_ethercat_read_float((uint8_t *)ec_slave[2].outputs + 2) == -0.5f);
    assert(console_ethercat_read_float((uint8_t *)ec_slave[2].outputs + 14) == 20.5f);
    cmd.batch[0].position = 0.75f; cmd.batch[1].kp = NAN;
    assert(console_ethercat_session_command(&cmd) == -1);
    assert(console_ethercat_read_float(out + 2) == 0.25f); /* Whole invalid frame rejected. */
    cmd.batch[1].kp = 20.5f; cmd.batch[1].slave_id = 1;
    assert(console_ethercat_session_command(&cmd) == -1);
    assert(console_ethercat_read_float(out + 2) == 0.25f);
    cmd.batch[1].slave_id = 2;
    console_ethercat_write_u16(ec_slave[2].inputs, 0x0008); /* One drive faults. */
    assert(console_ethercat_session_mit(&cmd, (uint8_t[EC_MAXSLAVE]){[1]=1,[2]=1}) == -1);
    assert(console_ethercat_read_float(out + 2) == 0.25f);
    cmd.task = ETHERCAT_TASK_DISABLE;
    assert(console_ethercat_session_command(&cmd) == 0);
    cmd.batch_count = 0;
    puts("PASS: two-motor batch parameters, duplicate/invalid frame rejection and no partial target changes");
    uint8_t saved_outputs[2][23];
    memcpy(saved_outputs[0], ec_slave[1].outputs, 23);
    memcpy(saved_outputs[1], ec_slave[2].outputs, 23);
    cmd.task = ETHERCAT_TASK_QUALITY; cmd.hold_ms = 10000;
    assert(console_ethercat_session_command(&cmd) == 0);
    assert(ethercat_quality.active);
    assert(console_ethercat_quality_start(&cmd) == -1);
    assert(memcmp(saved_outputs[0], ec_slave[1].outputs, 23) == 0);
    assert(memcmp(saved_outputs[1], ec_slave[2].outputs, 23) == 0);
    ethercat_quality.deadline_us = 4000;
    console_ethercat_quality_record(1000, 6, 6, 0);
    console_ethercat_quality_record(2000, 6, 6, 0);
    console_ethercat_quality_record(4000, 6, 6, 0);
    assert(!ethercat_quality.active && ethercat_quality.cycles == 3);
    assert(ethercat_quality.intervals == 2 && ethercat_quality.sum_interval_us == 3000);
    assert(ethercat_quality.min_interval_us == 1000 && ethercat_quality.max_interval_us == 2000);
    assert(ethercat_quality.late_intervals == 1 && ethercat_quality.bad_wkc == 0);
    assert(console_ethercat_quality_start(&cmd) == 0);
    console_ethercat_quality_record(1000, 5, 6, -1);
    assert(!ethercat_quality.active && ethercat_quality.bad_wkc == 1 && ethercat_quality.send_errors == 1);
    assert(ethercat_quality.incomplete_cycles == 1 && ethercat_quality.no_response_cycles == 0);
    assert(console_ethercat_quality_start(&cmd) == 0);
    console_ethercat_quality_record(1000, 0, 6, 0);
    assert(!ethercat_quality.active && ethercat_quality.no_response_cycles == 1);
    assert(ethercat_quality.incomplete_cycles == 0);
    assert(console_ethercat_quality_start(&cmd) == 0);
    console_ethercat_quality_finish("session_stopped");
    assert(!ethercat_quality.active);
    cmd.hold_ms = 1000;
    uint64_t test_start = console_ethercat_monotonic_us();
    assert(console_ethercat_quality_start(&cmd) == 0);
    while (ethercat_quality.active) assert(console_ethercat_mailbox_cycle() == 0);
    assert(console_ethercat_monotonic_us() - test_start >= 1000000);
    assert(ethercat_quality.cycles > 1 && ethercat_quality.bad_wkc == 0 && ethercat_quality.send_errors == 0);
    assert(memcmp(saved_outputs[0], ec_slave[1].outputs, 23) == 0);
    assert(memcmp(saved_outputs[1], ec_slave[2].outputs, 23) == 0);
    assert(console_ethercat_mailbox_cycle() == 0); /* Communication continues after test finishes. */
    ethercat_background_owner = true;
    ethercat_dc_sync_enabled = true;
    ethercat_dc_sync_adjustment_ns = 500000;
    clock_gettime(CLOCK_MONOTONIC, &ethercat_next_cycle);
    ethercat_next_cycle.tv_nsec += 2000000;
    if (ethercat_next_cycle.tv_nsec >= 1000000000) {
        ethercat_next_cycle.tv_nsec -= 1000000000; ethercat_next_cycle.tv_sec++;
    }
    uint64_t before_ns = (uint64_t)ethercat_next_cycle.tv_sec * 1000000000u + ethercat_next_cycle.tv_nsec;
    console_ethercat_wait_next_cycle();
    uint64_t after_ns = (uint64_t)ethercat_next_cycle.tv_sec * 1000000000u + ethercat_next_cycle.tv_nsec;
    assert(after_ns - before_ns == 1000000); /* DC adjustment must not change 1kHz grid. */
    clock_gettime(CLOCK_MONOTONIC, &ethercat_next_cycle);
    ethercat_next_cycle.tv_sec--; /* Deliberately simulate a long scheduling stall. */
    uint64_t skipped_before = ethercat_skipped_cycles;
    console_ethercat_wait_next_cycle();
    assert(ethercat_skipped_cycles > skipped_before);
    assert(console_ethercat_background_submit_ex(ETHERCAT_TASK_ENABLE, false, "enp86s0", "1",
        0, 0, 0, NULL, "0.5", ETHERCAT_CIA402_MODE_CSP, 0, 0, 0, NULL) == -1);
    /* Full confirmed dual-arm frame: 14 independent targets in one PDO cycle. */
    ec_slavecount = 14; ec_group[0].outputsWKC = 14; ec_group[0].inputsWKC = 14;
    ethercat_expected_work_counter = 42;
    cmd.task = ETHERCAT_TASK_MIT_BATCH; cmd.batch_count = 14; cmd.hold_ms = 1000;
    for (int slave = 1; slave <= 14; ++slave) {
        ethercat_session.configured[slave] = 1;
        etherlab_slaves[slave].config = (ec_slave_config_t *)1;
        etherlab_slaves[slave].output_offset = (slave - 1) * 39;
        ec_slave[slave].outputs = etherlab_slaves[slave].output_shadow;
        memset(ec_slave[slave].outputs, 0, 23);
        ec_slave[slave].inputs = domain_data + (slave - 1) * 39 + 23;
        memset(ec_slave[slave].inputs, 0, 16);
        ec_slave[slave].state = EC_STATE_OPERATIONAL;
        ec_slave[slave].Obits = 184; ec_slave[slave].Ibits = 128;
        console_ethercat_write_u16(ec_slave[slave].inputs, 0x40);
        cmd.batch[slave - 1] = (ethercat_mit_target){slave, slave * 0.01f, 0, slave * 0.1f, slave * 1.0f, 0.5f};
    }
    assert(console_ethercat_session_command(&cmd) == 0);
    for (int slave = 1; slave <= 14; ++slave) {
        const uint8_t *target = ec_slave[slave].outputs;
        assert(console_ethercat_read_u16(target) == 15 && target[22] == 10);
        assert(console_ethercat_read_float(target + 2) == slave * 0.01f);
        assert(console_ethercat_read_float(target + 10) == slave * 0.1f);
        assert(console_ethercat_read_float(target + 14) == slave * 1.0f);
    }
    cmd.task = ETHERCAT_TASK_DISABLE; strcpy(cmd.selection, "all");
    assert(console_ethercat_session_command(&cmd) == 0);
    puts("PASS: 14-motor dual-arm frame and disable-all, expected WKC=42");
    puts("PASS: fixed 1ms background grid, scheduling skips, WKC loss metrics and communication after test");
    puts("PASS: communication quality duration, interval/jitter counters, failure reporting and unchanged PDO targets");
    puts("PASS: MIT PDO enable, repeated enable preserves gains, float position updates, two-slave isolation and disable-all");
    return 0;
}
