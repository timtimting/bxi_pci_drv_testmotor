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
    c.settle_speed = .01;
    limits.v_min = -45; limits.v_max = 45;
    double decoded_zero_speed = (limits.v_max - limits.v_min) / 4095. / 2 * c.ratio / (2 * M_PI);
    assert(decoded_zero_speed > c.tolerance); /* Old criterion never settled. */
    assert(decoded_zero_speed < zero_cycle_speed_threshold(&limits, &c));
    assert(decoded_zero_speed * 2 > zero_cycle_speed_threshold(&limits, &c));
    c.kp = 50; c.kd = 5; c.retry_gain_factor = 1.5;
    c.retry_kp_max = 100; c.retry_kd_max = 7.5;
    limits.kp_max = 500; limits.kd_max = 5;
    double retry_kp, retry_kd;
    zero_cycle_retry_gains(&c, &limits, 1, &retry_kp, &retry_kd);
    assert(retry_kp == 75 && retry_kd == 5); /* KD constrained by motor protocol. */
    limits.kd_max = 20;
    zero_cycle_retry_gains(&c, &limits, 2, &retry_kp, &retry_kd);
    assert(retry_kp == 100 && retry_kd == 7.5); /* Config caps. */
    zero_cycle_retry_gains(&c, &limits, 10, &retry_kp, &retry_kd);
    assert(retry_kp == 100 && retry_kd == 7.5);
    for (int direction = -1; direction <= 1; direction += 2) {
        double distance = direction * 9.;
        double duration = zero_cycle_move_duration(distance, 10, 20);
        float position, velocity, previous_velocity = 0;
        for (int i = 0; i <= 10000; ++i) {
            zero_cycle_trajectory(2, 2+distance, duration*i/10000, duration, &position, &velocity);
            assert(position >= fmin(2, 2+distance)-1e-5 && position <= fmax(2, 2+distance)+1e-5);
            assert(fabs(velocity) <= 10.0001);
            if (i) assert(fabs(velocity-previous_velocity)/(duration/10000) <= 20.01);
            previous_velocity = velocity;
        }
        assert(fabs(position-(2+distance)) < 1e-5 && fabs(velocity) < 1e-5);
    }
    float static_position, static_velocity;
    zero_cycle_trajectory(2, 2, 1, 0, &static_position, &static_velocity);
    assert(static_position == 2 && static_velocity == 0);
    assert(zero_cycle_parse_state("STATE : 02 MOTOR") == 2);
    assert(zero_cycle_parse_state("STATE : 00 STARTUP\r\nSTATE : 01 MENU") == 1);
    assert(zero_cycle_parse_state("position reply received") == -1);
    assert(zero_cycle_parse_state("Error Code: 20000000") == -1);
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 18\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0); /* Half output turn is ambiguous. */
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 20\nmax_motor_turns: 9.1\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0);
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 20\ncount: 0\nduration_s: 0\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0);
    f = fopen(path, "w"); fprintf(f, "gear_ratio: nan\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0);
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 20\ntest_mode: degree_scan\narrival_tolerance_motor_turns: 0.01\ncount: 0\nscan_rounds: 0\nscan_repeats_per_angle: 10\n"); fclose(f);
    assert(!zero_cycle_load(path,&c) && c.scan_rounds==0 && c.scan_repeats==10);
    f = fopen(path, "a"); fprintf(f, "scan_repeats_per_angle: 0\n"); fclose(f);
    assert(zero_cycle_load(path,&c));
    f=fopen(path,"w"); fprintf(f,"gear_ratio: 20\ntest_mode: degree_scan\narrival_tolerance_motor_turns: 0.01\nscan_start_direction: positive\n"); fclose(f);
    assert(!zero_cycle_load(path,&c) && c.scan_start_positive==1);
    assert(!zero_cycle_direction_override(&c,0) && c.scan_start_positive==0);
    assert(!zero_cycle_direction_override(&c,-1) && c.scan_start_positive==0);
    f=fopen(path,"a"); fprintf(f,"scan_start_direction: sideways\n"); fclose(f);
    assert(zero_cycle_load(path,&c));
    unlink(path);
    assert(zero_cycle_load("config/zero_cycle_test.yaml", &c) == 0);
    assert(c.count == 100 && c.speed == 10 && c.kp == 50 && c.kd == 5 && c.tolerance == .1);
    assert(c.acceleration == 20 && c.arrival_tolerance == .1 && c.settle_speed == .1);
    assert(c.move_retry_count == 2 && c.arrival_wait_ms == 5000 && c.retry_hold_ms == 3000);
    assert(c.continue_after_position_failure == 1 && c.revisit_after_random_cycles == 3);
    zero_cycle_config scan;
    assert(zero_cycle_load("config/zero_cycle_degree_scan.yaml", &scan) == 0);
    assert(scan.test_mode == 1 && scan.count == 0 && zero_cycle_scan_steps(&scan) == 175);
    assert(scan.scan_repeats==10 && scan.scan_rounds==1 && zero_cycle_scan_total(&scan)==3500);
    assert(scan.scan_start_positive==0);
    assert(scan.scan_return_speed_deg_s == 5 && zero_cycle_physical_limit(&scan) > 9);
    for (unsigned int i = 0; i < 350; ++i) {
        double target;
        assert(zero_cycle_scan_target(i, &scan, &target) == 0);
        double expected_deg = i < 175 ? -(double)(i+1) : (double)(i-174);
        assert(fabs(target*360/scan.ratio-expected_deg) < 1e-9);
    }
    double scan_target;
    assert(zero_cycle_scan_target(350, &scan, &scan_target) != 0);
    scan.scan_rounds=2;
    assert(zero_cycle_scan_total(&scan)==7000);
    for (unsigned int i=0;i<7000;++i) {
        double target;
        assert(!zero_cycle_scan_cycle_target(i,&scan,&target));
        unsigned int point=(i/10)%350;
        double expected=point<175 ? -(double)(point+1) : (double)(point-174);
        assert(fabs(target*360/scan.ratio-expected)<1e-9);
        assert(zero_cycle_scan_boundary(i,&scan)==(i>0 && i%1750==0));
    }
    assert(zero_cycle_scan_cycle_target(7000,&scan,&scan_target));
    scan.scan_rounds=0;
    assert(zero_cycle_scan_total(&scan)==UINT64_MAX);
    assert(!zero_cycle_scan_cycle_target(7000,&scan,&scan_target));
    scan.scan_start_positive=1;
    for (unsigned int i=0;i<7000;++i) {
        double target;
        assert(!zero_cycle_scan_cycle_target(i,&scan,&target));
        unsigned int point=(i/10)%350;
        double expected=point<175 ? (double)(point+1) : -(double)(point-174);
        assert(fabs(target*360/scan.ratio-expected)<1e-9);
    }
    char *args[]={"zero_cycle_test","03","config/zero_cycle_degree_scan.yaml","--direction","positive"};
    char *filtered[64]; int filtered_count,direction;
    assert(!zero_cycle_direction_args(5,args,filtered,&filtered_count,&direction));
    assert(filtered_count==3 && direction==1 && filtered[2]==args[2]);
    assert(!strcmp(args[3],"--direction")); /* Terminal-owned argv stays intact. */
    char *early[]={"zero_cycle_group_test","--direction=negative","auto","config/zero_cycle_degree_scan.yaml"};
    assert(!zero_cycle_direction_args(4,early,filtered,&filtered_count,&direction) && filtered_count==3 && direction==0);
    char *missing[]={"zero_cycle_test","03","--direction"};
    assert(zero_cycle_direction_args(3,missing,filtered,&filtered_count,&direction));
    char *invalid[]={"zero_cycle_test","03","--direction","sideways"};
    assert(zero_cycle_direction_args(4,invalid,filtered,&filtered_count,&direction));
    char *duplicate[]={"zero_cycle_test","03","--direction=positive","--direction=negative"};
    assert(zero_cycle_direction_args(4,duplicate,filtered,&filtered_count,&direction));
    float scan_command;
    assert(zero_cycle_compensated_target(zero_cycle_physical_limit(&scan), 0, &scan, &limits, &scan_command) == 0);
    /* Regression: -13 motor-turn restart jump must command a displacement
     * of -6.235 turns to return a physical +6.235-turn point to original zero. */
    limits.p_min = -12.5; limits.p_max = 12.5;
    double scale = 2*M_PI/c.ratio, current = -6.765*scale;
    double offset = current-6.235*scale;
    float compensated;
    assert(zero_cycle_compensated_target(0, offset, &c, &limits, &compensated) == 0);
    assert(fabs((compensated-current)/scale + 6.235) < 1e-5);
    assert(zero_cycle_compensated_target(6.235, offset, &c, &limits, &compensated) == 0);
    assert(fabs(compensated-current) < 1e-6);
    assert(zero_cycle_compensated_target(9.1, offset, &c, &limits, &compensated) != 0);
    assert(zero_cycle_compensated_target(0, 20, &c, &limits, &compensated) != 0);
    assert(zero_cycle_zero_recheck_limits(-.0445, -.0445, .1, .1));
    assert(!zero_cycle_zero_recheck_limits(-.0445, 5.955418, .1, .1)); /* Old offset must not mask a failed zero reboot. */
    assert(!zero_cycle_zero_recheck_limits(0, 6, .1, .1));
    assert(!zero_cycle_zero_recheck_limits(.09, -.09, .1, .1));
    assert(!zero_cycle_zero_recheck_limits(0, NAN, .1, .1));
    double zero_limit=zero_cycle_return_tolerance(&scan);
    assert(fabs(zero_limit-scan.ratio/360)<1e-9);
    assert(zero_limit>scan.arrival_tolerance);
    assert(zero_cycle_zero_recheck_limits(-.032634,-.032634,zero_limit,.1));
    assert(!zero_cycle_zero_recheck_limits(-.032634,-.08,zero_limit,.1));
    assert(!zero_cycle_zero_recheck_limits(-.032634,-.030634,zero_limit,.001));
    assert(!zero_cycle_zero_recheck_limits(-.032634,5.967366,zero_limit,.1));
    zero_cycle_anomaly points[2] = {{.motor_turns=6.235}, {.motor_turns=-2}};
    assert(zero_cycle_find_anomaly(points, 2, 6.24, .1) == 0);
    assert(zero_cycle_find_anomaly(points, 2, -2.01, .1) == 1);
    assert(zero_cycle_find_anomaly(points, 2, 0, .1) == -1);
    points[0].observations = 3; points[0].failures = 2;
    points[0].planned_returns = 2; points[0].reproduced_returns = 1;
    points[0].fixed_trials = 20; points[0].fixed_failures = 20;
    zero_cycle_summary(path, points, 1, 1, 2, 0, true, 10, .1);
    char summary_path[512]; snprintf(summary_path, sizeof(summary_path), "%s.summary.csv", path);
    FILE *summary_file = fopen(summary_path, "r"); assert(summary_file);
    char summary_text[2048] = {0};
    assert(fread(summary_text, 1, sizeof(summary_text)-1, summary_file) > 0);
    fclose(summary_file); unlink(summary_path);
    assert(strstr(summary_text, "passed=1 position_failures=2"));
    assert(strstr(summary_text, "reproduced_returns") && strstr(summary_text, "fixed_retest_trials"));
    assert(strstr(summary_text, ",3,2,2,1,0,20,20,0,"));
    custom_command_words = console_command_words;
    custom_command_word_count = sizeof(console_command_words) / sizeof(console_command_words[0]);
    const char *line = "zero_cycle_test 01 config/zero_cycle_t";
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
    rx_can_frame mit_frame = {0}; mit_frame.can_id = 0x11; mit_frame.len = 8; mit_frame.data[0] = 1;
    assert(mit_reply_frame_matches(&mit_frame, 1));
    mit_frame.data[0] = 0x11; assert(!mit_reply_frame_matches(&mit_frame, 1));
    mit_frame.data[0] = 1; mit_frame.len = 6; assert(!mit_reply_frame_matches(&mit_frame, 1));
    mit_frame.len = 8; mit_frame.can_id = 0x7f1; assert(!mit_reply_frame_matches(&mit_frame, 1));
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
