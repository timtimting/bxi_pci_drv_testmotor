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


static uint8_t domain_data[78];
int ecrt_master_activate(ec_master_t *m) { (void)m; return 0; }
uint8_t *ecrt_domain_data(const ec_domain_t *d) { (void)d; return domain_data; }
int ecrt_master_receive(ec_master_t *m) { (void)m; return 0; }
int ecrt_domain_process(ec_domain_t *d) { (void)d; return 0; }
int ecrt_domain_state(const ec_domain_t *d, ec_domain_state_t *s) {
    (void)d; s->working_counter = 6; return 0;
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
    puts("PASS: MIT PDO enable, repeated enable preserves gains, float position updates, two-slave isolation and disable-all");
    return 0;
}
