#define MOTOR_CONSOLE_TEST
#include "../src/main.c"
#include <assert.h>
int main(void)
{
    unsigned int seed = 1234, replay = seed;
    int sectors[36] = {0};
    unsigned int order[36], replay_order[36];
    for (unsigned int i = 0; i < 72; ++i) {
        if (i % 36 == 0) memset(sectors, 0, sizeof(sectors));
        double target = zero_cycle_target(i, &seed, 9, order);
        assert(target > -9 && target < 9);
        assert(target == zero_cycle_target(i, &replay, 9, replay_order));
        int sector = (int)((target + 9) * 2);
        assert(sector >= 0 && sector < 36);
        assert(!sectors[sector]++);
    }
    char path[] = "/tmp/zero-cycle-config-XXXXXX";
    int fd = mkstemp(path); assert(fd >= 0);
    FILE *f = fdopen(fd, "w"); assert(f);
    fprintf(f, "gear_ratio: 19.54570637\ncount: 0\nduration_s: 60\n"); fclose(f);
    zero_cycle_config c;
    assert(zero_cycle_load(path, &c) == 0 && c.count == 0 && c.duration_s == 60);
    bxi_motor_limits limits = bxi_motor_default_limits;
    limits.v_min = -45; limits.v_max = 45;
    double decoded_zero_speed = (limits.v_max - limits.v_min) / 4095. / 2 * c.ratio / (2 * M_PI);
    assert(decoded_zero_speed > c.tolerance); /* Old criterion never settled. */
    assert(decoded_zero_speed < zero_cycle_speed_threshold(&limits, &c));
    assert(decoded_zero_speed * 2 > zero_cycle_speed_threshold(&limits, &c));
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 18\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0); /* Half output turn is ambiguous. */
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 20\nmax_motor_turns: 9.1\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0);
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 20\ncount: 0\nduration_s: 0\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0);
    f = fopen(path, "w"); fprintf(f, "gear_ratio: nan\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0);
    unlink(path);
    assert(zero_cycle_load("config/zero_cycle_test.yaml", &c) == 0);
    assert(c.count == 1000 && c.speed == 10 && c.kp == 50 && c.kd == 5 && c.tolerance == .1);
    custom_command_words = console_command_words;
    custom_command_word_count = sizeof(console_command_words) / sizeof(console_command_words[0]);
    const char *line = "zero_cycle_test 01 config/zero_c";
    size_t count = 0;
    const char *const *words = completion_words_for_line(line, strlen(line), &count);
    assert(words && count == 1 && !strcmp(words[0], "config/zero_cycle_test.yaml"));
    line = "zero_cycle_test 01 confi";
    words = completion_words_for_line(line, strlen(line), &count);
    assert(words && count == 1 && !strcmp(words[0], "config/"));
    char input[256] = "zero_cycle_test 01 confi";
    size_t length = strlen(input), cursor = length;
    complete_line(input, &length, &cursor, sizeof(input), "");
    assert(!strcmp(input, "zero_cycle_test 01 config/")); /* No space after directory. */
    /* Raw boot capture works while terminal output is disabled, and excludes
     * other buses and motor IDs. No hardware initialization is performed. */
    static flash_state state;
    rx_ring_init(&state.rx); frame_ring_init(&state.frames);
    FILE *capture = tmpfile(); assert(capture);
    boot_capture_file = capture; boot_capture_bus = 0; boot_capture_id = 2;
    canfd_packet packet = {0};
    packet.bus = 0; packet.frame.can_id = 0x7f2; packet.frame.len = 5;
    memcpy(packet.frame.data, "BOOT\n", 5);
    can_rx_callback(&state, &packet);
    packet.bus = 1; memcpy(packet.frame.data, "OTHER", 5);
    can_rx_callback(&state, &packet);
    packet.bus = 0; packet.frame.can_id = 0x7f3;
    can_rx_callback(&state, &packet);
    boot_capture_file = NULL;
    rewind(capture); char captured[1024] = {0};
    assert(fread(captured, 1, sizeof(captured) - 1, capture) > 0);
    assert(strstr(captured, "BOOT\n") && !strstr(captured, "hex:"));
    assert(!strstr(captured, "OTHER")); fclose(capture);
    zero_encoder_snapshot enc_before = zero_encoder_parse("ENC1: raw=62000 pos=0.951134\r\nENC2: angle=17.5 deg pos=0.048661 turn\r\nENC2_RAW_DEG=12.3\n");
    zero_encoder_snapshot enc_after = zero_encoder_parse("ENC1: raw=62001 pos=6.951052\nENC2: angle=128.0 deg pos=0.355631 turn\nENC2_RAW_DEG=12.4\n");
    assert(enc_before.enc1_valid && enc_before.enc2_valid && enc_before.raw2_valid);
    assert(fabs(enc_after.pos1-enc_before.pos1-5.999918) < 1e-6);
    assert(!zero_encoder_parse("old firmware without enc info").enc1_valid);
    assert(fabs(zero_shortest_deg(-359.8)-.2) < 1e-6);
    capture = tmpfile(); assert(capture);
    zero_encoder_compare(capture, &enc_before, &enc_after, 19.54570637);
    rewind(capture); memset(captured, 0, sizeof(captured));
    assert(fread(captured, 1, sizeof(captured)-1, capture) > 0);
    assert(strstr(captured, "ENC1 motor:") && strstr(captured, "ENC2 output:")); fclose(capture);
    puts("PASS: random coverage, config, completion and silent raw boot capture filtering");
    return 0;
}
