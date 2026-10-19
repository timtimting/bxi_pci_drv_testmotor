/* Hardware-free test. Link with -Wl,--wrap=canfd_send_packet. */
#define MOTOR_CONSOLE_TEST
#include "../src/main.c"
#include <assert.h>

static flash_state transport_state;
static unsigned int sent_count, respond_after;
static bool wrong_reply;
static bool fail_send;
static uint8_t first_payload[8];

int __wrap_canfd_send_packet(canfd_packet *packet, unsigned int count)
{
    assert(count == 1 && packet->bus == 1 && packet->frame.can_id == 1);
    assert(packet->frame.len == 8);
    if (!sent_count) memcpy(first_payload, packet->frame.data, 8);
    else assert(!memcmp(first_payload, packet->frame.data, 8));
    ++sent_count;
    if (fail_send) return -7;
    if (respond_after && sent_count >= respond_after) {
        rx_can_frame frame = {0};
        frame.bus = 1; frame.can_id = 0x11; frame.len = 8; frame.time_us = time_us();
        frame.data[0] = wrong_reply ? 0x11 : 1;
        frame.data[1] = frame.data[3] = frame.data[6] = 0x80;
        frame_ring_push(&transport_state.frames, &frame);
    }
    return 0;
}

int main(void)
{
    flash_state *state = &transport_state;
    state->config.entry_count = 1;
    motor_map_entry *motor = &state->config.entries[0];
    motor->bus = 1; motor->id = 1;
    state->config.mit_limits = bxi_motor_default_limits;
    state->quiet_tx = true; state->debug_use_canfd = true;
    frame_ring_init(&state->frames);
    FILE *log = tmpfile(); assert(log); boot_capture_file = log;
    boot_capture_bus=1; boot_capture_id=1;
    bxi_motor_reply reply;
    respond_after = 1;
    assert(!zero_cycle_sample_velocity(state, motor, .1, .2, 50, 5, &reply));
    assert(sent_count == 1);
    sent_count = 0; respond_after = 3;
    assert(!zero_cycle_sample_velocity(state, motor, .1, .2, 50, 5, &reply));
    assert(sent_count == 3);
    sent_count = 0; respond_after = 0;
    assert(zero_cycle_sample_velocity(state, motor, .1, .2, 50, 5, &reply) < 0);
    assert(sent_count >= 2 && sent_count <= 10);
    sent_count = 0; respond_after = 1; wrong_reply = true;
    assert(zero_cycle_sample_velocity(state, motor, .1, .2, 50, 5, &reply) < 0);
    sent_count=0; fail_send=true;
    assert(zero_cycle_sample_velocity(state,motor,.1,.2,50,5,&reply)<0 && sent_count==1);
    rewind(log); char text[8192] = {0}; assert(fread(text, 1, sizeof(text)-1, log));
    assert(strstr(text, "MIT_RECOVERED") && strstr(text, "MIT_TIMEOUT") && strstr(text, "MIT_RX"));
    assert(strstr(text,"TX_FAILED") && strstr(text,"ret=-7"));
    boot_capture_file = NULL; fclose(log);
    puts("PASS: immediate reply, dropped requests, bounded timeout, strict reply filtering and raw diagnostics");
    return 0;
}
