/* Included by main.c. All angles in this test configuration are motor-side turns. */
typedef struct {
    double ratio, max_turns, speed, kp, kd, tolerance;
    double acceleration, arrival_tolerance, settle_speed;
    double retry_gain_factor, retry_kp_max, retry_kd_max;
    unsigned int arrival_wait_ms, move_retry_count, retry_hold_ms;
    unsigned int continue_after_position_failure, revisit_after_random_cycles;
    unsigned int count, duration_s, off_ms, settle_ms, move_timeout_ms, seed;
} zero_cycle_config;

static int zero_cycle_load(const char *path, zero_cycle_config *c)
{
    *c = (zero_cycle_config){.ratio=0, .max_turns=9, .speed=1, .kp=5, .kd=1,
        .tolerance=.01, .acceleration=20, .arrival_tolerance=.1, .settle_speed=.1,
        .retry_gain_factor=1.5, .retry_kp_max=100, .retry_kd_max=7.5,
        .arrival_wait_ms=5000, .move_retry_count=2, .retry_hold_ms=3000,
        .continue_after_position_failure=1, .revisit_after_random_cycles=3,
        .count=100, .off_ms=1000, .settle_ms=500, .move_timeout_ms=120000};
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return -1; }
    char line[256]; int bad = 0;
    while (fgets(line, sizeof(line), f)) {
        char *comment = strchr(line, '#'); if (comment) *comment = 0;
        char *p = trim_space(line); if (!*p) continue;
        char *colon = strchr(p, ':'); if (!colon) { bad = 1; break; }
        *colon++ = 0; char *key = trim_space(p), *value = trim_space(colon);
        double d; unsigned int u;
#define ZDOUBLE(k, field) if (!strcmp(key, k)) { char *end; d = strtod(value, &end); if (!*value || *end || !isfinite(d)) { bad = 1; break; } c->field = d; }
#define ZUINT(k, field) if (!strcmp(key, k)) { if (parse_uint_arg(value, &u)) { bad = 1; break; } c->field = u; }
        ZDOUBLE("gear_ratio", ratio)
        else ZDOUBLE("max_motor_turns", max_turns)
        else ZDOUBLE("motor_turns_per_second", speed)
        else ZDOUBLE("motor_acceleration_turns_per_second2", acceleration)
        else ZDOUBLE("arrival_tolerance_motor_turns", arrival_tolerance)
        else ZDOUBLE("settle_speed_motor_turns_per_second", settle_speed)
        else ZDOUBLE("retry_gain_factor", retry_gain_factor)
        else ZDOUBLE("retry_kp_max", retry_kp_max)
        else ZDOUBLE("retry_kd_max", retry_kd_max)
        else ZDOUBLE("kp", kp)
        else ZDOUBLE("kd", kd)
        else ZDOUBLE("tolerance_motor_turns", tolerance)
        else ZUINT("count", count)
        else ZUINT("duration_s", duration_s)
        else ZUINT("off_ms", off_ms)
        else ZUINT("settle_ms", settle_ms)
        else ZUINT("move_timeout_ms", move_timeout_ms)
        else ZUINT("arrival_wait_ms", arrival_wait_ms)
        else ZUINT("move_retry_count", move_retry_count)
        else ZUINT("retry_hold_ms", retry_hold_ms)
        else ZUINT("continue_after_position_failure", continue_after_position_failure)
        else ZUINT("revisit_after_random_cycles", revisit_after_random_cycles)
        else ZUINT("seed", seed)
        else { bad = 1; break; }
#undef ZDOUBLE
#undef ZUINT
    }
    if (ferror(f)) bad = 1;
    fclose(f);
    if (bad || c->ratio <= 1 || c->max_turns <= 0 || c->max_turns > 9 ||
        c->max_turns / c->ratio >= .5 || c->speed <= 0 || c->speed > 10 ||
        c->acceleration <= 0 || c->acceleration > 1000 ||
        c->arrival_tolerance <= 0 || c->arrival_tolerance >= .25 ||
        c->settle_speed <= 0 || c->settle_speed > 1 ||
        c->retry_gain_factor <= 1 || c->retry_gain_factor > 3 ||
        c->retry_kp_max < c->kp || c->retry_kd_max < c->kd ||
        c->move_retry_count > 10 || c->arrival_wait_ms < c->settle_ms || c->arrival_wait_ms > 60000 ||
        c->retry_hold_ms < c->settle_ms || c->retry_hold_ms < 1000 || c->retry_hold_ms > 60000 ||
        c->continue_after_position_failure > 1 || !c->revisit_after_random_cycles || c->revisit_after_random_cycles > 10000 ||
        c->kp <= 0 || c->kd < 0 || c->tolerance <= 0 || c->tolerance >= .25 ||
        (!c->count && !c->duration_s) || c->off_ms < 300 || c->off_ms > 60000 ||
        c->settle_ms < 100 || c->settle_ms > 60000 || !c->move_timeout_ms || c->move_timeout_ms > 3600000) {
        fprintf(stderr, "zero_cycle_test: invalid config; set actual gear_ratio; require max_motor_turns<=9 and max_motor_turns/gear_ratio<0.5\n");
        return -1;
    }
    return 0;
}

static double zero_cycle_target(unsigned int n, unsigned int *seed, double range, unsigned int order[36])
{
    /* Reshuffle each block, then jitter inside every sector once. */
    if (n % 36u == 0) {
        for (unsigned int i = 0; i < 36; ++i) order[i] = i;
        for (unsigned int i = 35; i > 0; --i) {
            unsigned int j = (unsigned int)((rand_r(seed) / ((double)RAND_MAX + 1)) * (i + 1));
            unsigned int swap = order[i]; order[i] = order[j]; order[j] = swap;
        }
    }
    unsigned int sector = order[n % 36u];
    double jitter = (rand_r(seed) + .5) / ((double)RAND_MAX + 1.0);
    return -range + 2 * range * (sector + jitter) / 36.0;
}

/* Keep routine power/scan/handshake chatter out of the per-cycle report. */
static int zero_cycle_operation(flash_state *state, int operation, size_t slot)
{
    int out = -1, err = -1;
    if (console_silence_process_output(&out, &err)) {
        fprintf(stderr, "zero_cycle_test: ERROR cannot suppress routine output\n"); return -1;
    }
    int result;
    if (operation == 1) result = console_power_on(state);
    else if (operation == 0) result = console_power_off(state);
    else result = console_send_special(state, slot, (uint8_t)operation, "zero_cycle_handshake");
    console_restore_process_output(out, err);
    if (result) fprintf(stderr, "zero_cycle_test: ERROR operation=%d failed\n", operation);
    return result;
}

static int zero_cycle_sample_velocity(flash_state *state, const motor_map_entry *m,
                             float target, float velocity, float kp, float kd, bxi_motor_reply *reply)
{
    frame_ring_clear(&state->frames);
    console_use_motor(state, m);
    uint64_t sent = time_us();
    unsigned int transmissions = 1, received = 0, text_frames = 0, decode_failures = 0;
    rx_can_frame observed[8]; size_t observed_count = 0;
    console_expect_reply(state, m->bus);
    if (send_debug_mit(state, target, velocity, kp, kd, 0)) return -1;
    while (!stop_requested && time_us() - sent < 100000) {
        rx_can_frame frame;
        if (!frame_ring_pop(&state->frames, &frame, 10)) continue;
        if (frame.time_us < sent || frame.bus != m->bus) continue;
        ++received;
        if (boot_output_id_matches(frame.can_id, m->id)) ++text_frames;
        else if (observed_count < sizeof(observed)/sizeof(observed[0])) observed[observed_count++] = frame;
        if (!mit_reply_frame_matches(&frame, m->id)) continue;
        int result = state->motors[m - state->config.entries].mit_aux_enabled ?
            bxi_motor_unpack_reply(frame.data, frame.len, &state->limits, reply) :
            bxi_motor_unpack_reply_legacy_ntc(frame.data, frame.len, &state->limits, reply);
        if (!result && isfinite(reply->position) && isfinite(reply->velocity)) return 0;
        ++decode_failures;
    }
    fprintf(stderr, "zero_cycle_test: ERROR %s: bus=%u motor_id=%u expected_CAN_ID=0x%03X byte0=0x%02X len>=8; sends=%u received=%u boot_text=%u decode_failures=%u. Raw diagnostics in .boot.log\n",
            stop_requested ? "interrupted" : "fresh MIT feedback timeout (100ms)", m->bus, m->id,
            0x10u | m->id, m->id, transmissions, received, text_frames, decode_failures);
    pthread_mutex_lock(&boot_capture_mutex);
    if (boot_capture_file) {
        uint8_t tx[BXI_MOTOR_MIT_LEN];
        bxi_motor_pack_mit(tx, &state->limits, target, velocity, kp, kd, 0);
        fprintf(boot_capture_file, "\nMIT_TIMEOUT bus=%u motor_id=%u expected_id=0x%03X expected_byte0=0x%02X sends=%u rx_bus=%u rx_text=%u decode_failures=%u\n"
                "MIT_TX canfd=%d p=%.9g v=%.9g kp=%.9g kd=%.9g CAN_ID=0x%03X data=",
                m->bus, m->id, 0x10u|m->id, m->id, transmissions, received, text_frames, decode_failures,
                state->debug_use_canfd, target, velocity, kp, kd, m->id);
        for (unsigned int i = 0; i < sizeof(tx); ++i) fprintf(boot_capture_file, "%02X ", tx[i]);
        fputc('\n', boot_capture_file);
        for (size_t i = 0; i < observed_count; ++i) {
            const rx_can_frame *frame = &observed[i];
            fprintf(boot_capture_file, "MIT_RX time_us=%llu CAN_ID=0x%03X len=%u flags=0x%02X data=",
                    (unsigned long long)frame->time_us, frame->can_id, frame->len, frame->flags);
            for (unsigned int j = 0; j < frame->len; ++j) fprintf(boot_capture_file, "%02X ", frame->data[j]);
            fputc('\n', boot_capture_file);
        }
        fflush(boot_capture_file);
    }
    pthread_mutex_unlock(&boot_capture_mutex);
    return -1;
}

static int zero_cycle_sample(flash_state *state, const motor_map_entry *m,
                             float target, float kp, float kd, bxi_motor_reply *reply)
{
    return zero_cycle_sample_velocity(state, m, target, 0, kp, kd, reply);
}

static int zero_cycle_parse_state(const char *text)
{
    int state = -1;
    for (const char *p = text; (p = strstr(p, "STATE")) != NULL; ++p) {
        unsigned int value;
        if (sscanf(p, "STATE : %u", &value) == 1) state = (int)value;
    }
    return state;
}

static int zero_cycle_enable_confirmed(flash_state *state, const motor_map_entry *m, size_t slot)
{
    console_use_motor(state, m);
    frame_ring_clear(&state->frames);
    if (send_debug_special(state, BXI_MOTOR_CMD_ENABLE)) return -1;
    /* Generic console_send_special sleeps scan_timeout_ms after enabling.
     * Keep feeding zero-gain MIT here instead, so an enabled CAN watchdog
     * cannot expire during that handshake wait. State query 0xC8 is read-only. */
    char text[1024] = {0}; size_t used = 0;
    uint64_t begin = time_us(), next_send = 0, next_query = 0;
    int reported_state = -1;
    while (!stop_requested && time_us() - begin < 2000000) {
        uint64_t now = time_us();
        if (now >= next_send) {
            if (send_debug_mit(state, 0, 0, 0, 0, 0)) return -1;
            next_send = now + 10000;
        }
        if (now >= next_query) {
            const uint8_t query = 0xc8;
            if (send_boot_data(state, &query, 1)) return -1;
            next_query = now + 100000;
        }
        rx_can_frame frame;
        if (!frame_ring_pop(&state->frames, &frame, 5)) continue;
        if (frame.bus != m->bus || frame.time_us < begin || !boot_output_id_matches(frame.can_id, m->id)) continue;
        if (used + frame.len >= sizeof(text)) {
            memmove(text, text + 512, used - 512 + 1); used -= 512;
        }
        memcpy(text + used, frame.data, frame.len); used += frame.len; text[used] = 0;
        /* Wait for a line terminator so a fragmented state number cannot
         * prematurely look like state 2. */
        char *end = strpbrk(text, "\r\n");
        while (end) {
            *end = 0;
            int parsed = zero_cycle_parse_state(text);
            if (parsed >= 0) reported_state = parsed;
            if (reported_state == 2) { state->motors[slot].enabled = true; return 0; }
            size_t consumed = (size_t)(end-text) + 1;
            memmove(text, text+consumed, used-consumed+1); used -= consumed;
            end = strpbrk(text, "\r\n");
        }
    }
    fprintf(stderr, "zero_cycle_test: ERROR enable not confirmed: STATE=%d (expected 02 MOTOR). Motor may reject enable due to fault/calibration, startup not ready, or unsupported state query; inspect .boot.log.\n", reported_state);
    return -1;
}

static double zero_cycle_speed_threshold(const bxi_motor_limits *limits, const zero_cycle_config *c)
{
    /* Symmetric 12-bit velocity has no exact zero code. Allow its half-LSB
     * residual, without relaxing the independent position comparison. */
    double half_lsb = (limits->v_max - limits->v_min) / 4095.0 / 2;
    return fmax(c->settle_speed, half_lsb * c->ratio / (2 * M_PI) * 1.01);
}

/* Quintic trajectory: zero velocity/acceleration at both ends; duration
 * bounds peak velocity (1.875*d/T) and acceleration (5.773503*d/T^2). */
static double zero_cycle_move_duration(double distance, double speed, double acceleration)
{
    return fmax(1.875 * fabs(distance) / speed, sqrt(5.773503 * fabs(distance) / acceleration));
}

static void zero_cycle_trajectory(double start, double target, double t, double duration,
                                  float *position, float *velocity)
{
    double u = duration > 0 ? fmax(0, fmin(t / duration, 1)) : 1;
    double s = u*u*u*(10 + u*(-15 + 6*u));
    *position = start + (target-start)*s;
    *velocity = duration > 0 ? (target-start)*30*u*u*(1-u)*(1-u)/duration : 0;
}

static void zero_cycle_retry_gains(const zero_cycle_config *c, const bxi_motor_limits *limits,
                                    unsigned int retry, double *kp, double *kd)
{
    double factor = pow(c->retry_gain_factor, retry);
    *kp = fmin(c->kp * factor, fmin(c->retry_kp_max, limits->kp_max));
    *kd = fmin(c->kd * factor, fmin(c->retry_kd_max, limits->kd_max));
}

/* 0=arrived, 1=healthy feedback but arrival attempts exhausted, -1=abort. */
static int zero_cycle_move(flash_state *state, const motor_map_entry *m,
                           const zero_cycle_config *c, float target, bxi_motor_reply *reply,
                           unsigned int *retries_used)
{
    *retries_used = 0;
    if (zero_cycle_sample(state, m, 0, 0, 0, reply)) return -1;
    float start = reply->position;
    uint64_t begin = time_us(), stable = 0, next_report = 0;
    double speed_threshold = zero_cycle_speed_threshold(limits_for_entry(state, m), c);
    double scale = 2 * M_PI / c->ratio;
    double ramp_s = zero_cycle_move_duration(target-start, c->speed*scale, c->acceleration*scale);
    if (ramp_s * 1000 + c->settle_ms >= c->move_timeout_ms) {
        fprintf(stderr, "zero_cycle_test: ERROR move_timeout_ms too short: trajectory=%.1f ms plus settle=%u ms\n", ramp_s*1000, c->settle_ms);
        return -1;
    }
    double next_attempt = ramp_s + c->arrival_wait_ms / 1000.;
    double previous_kp = c->kp, previous_kd = c->kd, desired_kp = c->kp, desired_kd = c->kd;
    double gain_start = 0;
    while (!stop_requested && time_us() - begin < (uint64_t)c->move_timeout_ms * 1000) {
        double t = (time_us() - begin) / 1e6;
        float setpoint, set_velocity;
        zero_cycle_trajectory(start, target, t, ramp_s, &setpoint, &set_velocity);
        double gain_fraction = fmin(1, fmax(0, (t-gain_start)/.5));
        double active_kp = previous_kp + (desired_kp-previous_kp)*gain_fraction;
        double active_kd = previous_kd + (desired_kd-previous_kd)*gain_fraction;
        if (zero_cycle_sample_velocity(state, m, setpoint, set_velocity, active_kp, active_kd, reply)) return -1;
        double error = fabs(reply->position - target) * c->ratio / (2 * M_PI);
        double speed = fabs(reply->velocity) * c->ratio / (2 * M_PI);
        if (t >= ramp_s && error <= c->arrival_tolerance && speed <= speed_threshold) {
            if (!stable) stable = time_us();
            if (time_us() - stable >= (uint64_t)c->settle_ms * 1000) return 0;
        } else stable = 0;
        if (t >= next_attempt) {
            printf(console_text(state,
                   "目标仍未到位：目标 %+.6f 圈，实际 %+.6f 圈，差值 %+.6f 圈，速度 %.5f 圈/秒。\n",
                   "Target not reached: target %+.6f actual %+.6f delta %+.6f turns, speed %.5f turns/s.\n"),
                   target/scale, reply->position/scale, (reply->position-target)/scale, speed);
            if (*retries_used >= c->move_retry_count) {
                fputs(console_text(state, "增益重试已耗尽，尝试在当前位置稳定后进行上下电测试。\n",
                                          "Gain retries exhausted; attempt to settle at current position for power-cycle test.\n"), stdout);
                fflush(stdout); return 1;
            }
            ++*retries_used;
            previous_kp = active_kp; previous_kd = active_kd;
            zero_cycle_retry_gains(c, limits_for_entry(state, m), *retries_used, &desired_kp, &desired_kd);
            gain_start = t; next_attempt = t + c->retry_hold_ms/1000.; stable = 0;
            printf(console_text(state, "增益重试 %u/%u：KP %.3f → %.3f，KD %.3f → %.3f（500ms 平滑调整）。\n",
                                      "Gain retry %u/%u: KP %.3f -> %.3f, KD %.3f -> %.3f (500ms ramp).\n"),
                   *retries_used, c->move_retry_count, previous_kp, desired_kp, previous_kd, desired_kd);
            fflush(stdout);
        }
        if (time_us() >= next_report) {
            pthread_mutex_lock(&boot_capture_mutex);
            if (boot_capture_file) {
                fprintf(boot_capture_file, "\nMOTION elapsed_s=%.3f phase=%s target_motor_turns=%.7f command_motor_turns=%.7f command_motor_speed=%.5f actual_motor_turns=%.7f actual_motor_speed=%.5f error_motor_turns=%.7f torque=%.5f stable_ms=%.1f\n",
                        t, t < ramp_s ? "trajectory" : "settle", target/scale, setpoint/scale,
                        set_velocity/scale, reply->position/scale, reply->velocity/scale, error, reply->torque,
                        stable ? (time_us()-stable)/1000. : 0.);
                fprintf(boot_capture_file, "MOTION_GAINS retry=%u kp=%.5f kd=%.5f\n", *retries_used, active_kp, active_kd);
                fflush(boot_capture_file);
            }
            pthread_mutex_unlock(&boot_capture_mutex);
            next_report = time_us() + 1000000;
        }
        sleep_ms(10);
    }
    fprintf(stderr, "zero_cycle_test: %s %s target=%.7f actual=%.7f error=%.7f turns (arrival_limit=%.7f); speed=%.5f turns/s (limit=%.5f). Inspect .boot.log MOTION records.\n",
            stop_requested ? "ERROR" : "WARNING", stop_requested ? "interrupted" : "move timeout; attempt current-position fallback", target/scale, reply->position/scale,
            fabs(reply->position-target)/scale, c->arrival_tolerance, fabs(reply->velocity)/scale, speed_threshold);
    return stop_requested ? -1 : 1;
}

static int zero_cycle_settle_current(flash_state *state, const motor_map_entry *m,
                                     const zero_cycle_config *c, bxi_motor_reply *reply)
{
    float hold = reply->position;
    uint64_t begin = time_us(), stable = 0;
    double speed_threshold = zero_cycle_speed_threshold(limits_for_entry(state, m), c);
    while (!stop_requested && time_us()-begin < 5000000) {
        if (zero_cycle_sample(state, m, hold, c->kp, c->kd, reply)) return -1;
        double speed = fabs(reply->velocity)*c->ratio/(2*M_PI);
        double error = fabs(reply->position-hold)*c->ratio/(2*M_PI);
        if (speed <= speed_threshold && error <= c->arrival_tolerance) {
            if (!stable) stable = time_us();
            if (time_us()-stable >= (uint64_t)c->settle_ms*1000) return 0;
        } else stable = 0;
        sleep_ms(10);
    }
    fprintf(stderr, "zero_cycle_test: ERROR current position did not settle within 5s; skip power-cycle comparison\n");
    return -1;
}

typedef struct {
    bool enc1_valid, enc2_valid, raw2_valid;
    int raw1;
    unsigned int cpr1;
    double pos1, angle2, pos2, raw2;
} zero_encoder_snapshot;

static zero_encoder_snapshot zero_encoder_parse(const char *text)
{
    zero_encoder_snapshot s = {0}; s.cpr1 = 65536; /* Reference firmware fallback. */
    const char *p = strstr(text, "ENC1: raw=");
    if (p) s.enc1_valid = sscanf(p, "ENC1: raw=%d pos=%lf", &s.raw1, &s.pos1) == 2;
    p = strstr(text, "ENC1_CPR=");
    if (p) { unsigned int cpr; if (sscanf(p, "ENC1_CPR=%u", &cpr) == 1 && cpr) s.cpr1 = cpr; }
    p = strstr(text, "ENC2: angle=");
    if (p) s.enc2_valid = sscanf(p, "ENC2: angle=%lf deg pos=%lf turn", &s.angle2, &s.pos2) == 2;
    p = strstr(text, "ENC2_RAW_DEG=");
    if (p) s.raw2_valid = sscanf(p, "ENC2_RAW_DEG=%lf", &s.raw2) == 1 && isfinite(s.raw2);
    s.enc1_valid = s.enc1_valid && isfinite(s.pos1);
    s.enc2_valid = s.enc2_valid && isfinite(s.angle2) && isfinite(s.pos2);
    return s;
}

static zero_encoder_snapshot zero_encoder_query(flash_state *state, const motor_map_entry *m, FILE *log, const char *stage)
{
    pthread_mutex_lock(&boot_capture_mutex);
    fprintf(log, "\n--- %s: send menu string 'p' (read-only, disabled) ---\n", stage); fflush(log);
    pthread_mutex_unlock(&boot_capture_mutex);
    char text[8192] = {0}; size_t used = 0;
    console_use_motor(state, m);
    frame_ring_clear(&state->frames);
    if (send_boot_data(state, (const uint8_t *)"p", 1) == 0) {
        uint64_t until = time_us() + 1500000;
        while (!stop_requested && time_us() < until) {
            rx_can_frame frame;
            if (!frame_ring_pop(&state->frames, &frame, 20)) continue;
            if (frame.bus != m->bus || !boot_output_id_matches(frame.can_id, m->id)) continue;
            if (used + frame.len < sizeof(text)) {
                memcpy(text + used, frame.data, frame.len); used += frame.len; text[used] = 0;
            }
        }
    }
    zero_encoder_snapshot s = zero_encoder_parse(text);
    pthread_mutex_lock(&boot_capture_mutex);
    fprintf(log, "\nQUERY_SUMMARY stage=%s ENC1=%s ENC2=%s ENC2_RAW=%s\n", stage,
            s.enc1_valid ? "received" : "unavailable", s.enc2_valid ? "received" : "unavailable",
            s.raw2_valid ? "received" : "unavailable (requires diagnostic firmware)"); fflush(log);
    pthread_mutex_unlock(&boot_capture_mutex);
    return s;
}

static double zero_shortest_deg(double deg) { return remainder(deg, 360.0); }

static void zero_encoder_compare(FILE *log, const zero_encoder_snapshot *before,
                                  const zero_encoder_snapshot *after, double ratio)
{
    pthread_mutex_lock(&boot_capture_mutex);
    if (before->enc1_valid && after->enc1_valid) {
        /* 65536 CPR is specific to the inspected reference firmware. */
        double raw_before = before->raw1 * 360.0 / before->cpr1;
        double raw_after = after->raw1 * 360.0 / after->cpr1;
        fprintf(log, "ENC1 motor: pos_before=%.6f turns pos_after=%.6f turns delta=%.6f turns / %.4f deg\n"
                     "ENC1 raw: before=%d / %.4f deg after=%d / %.4f deg shortest_delta=%.4f deg\n",
                before->pos1, after->pos1, after->pos1-before->pos1, (after->pos1-before->pos1)*360,
                before->raw1, raw_before, after->raw1, raw_after, zero_shortest_deg(raw_after-raw_before));
    } else fputs("ENC1 comparison unavailable: firmware 'p' output missing.\n", log);
    if (before->enc2_valid && after->enc2_valid) {
        fprintf(log, "ENC2 output: pos_before=%.6f turns pos_after=%.6f turns delta=%.6f turns / %.4f output deg / %.4f motor deg\n"
                     "ENC2 calibrated angle: before=%.4f deg after=%.4f deg shortest_delta=%.4f output deg\n",
                before->pos2, after->pos2, after->pos2-before->pos2, (after->pos2-before->pos2)*360,
                (after->pos2-before->pos2)*360*ratio, before->angle2, after->angle2, zero_shortest_deg(after->angle2-before->angle2));
        if (before->enc1_valid && after->enc1_valid) {
            double d1 = after->pos1-before->pos1, d2 = (after->pos2-before->pos2)*ratio;
            fprintf(log, "RESTORE comparison motor turns: ENC1_delta=%.6f ENC2_delta_scaled=%.6f residual=%.6f\n", d1, d2, d1-d2);
            if (fabs(d1) > .5 && fabs(zero_shortest_deg((after->raw1-before->raw1)*360.0/before->cpr1)) < 1)
                fputs("HINT: ENC1 single-turn raw angle is nearly unchanged but restored multi-turn position jumped; inspect ENC2 angle/config/offset and startup turn reconstruction. This does not identify a hardware fault by itself.\n", log);
        }
    } else fputs("ENC2 comparison unavailable: firmware 'p' output missing.\n", log);
    if (before->raw2_valid && after->raw2_valid)
        fprintf(log, "ENC2 raw angle: before=%.6f deg after=%.6f deg shortest_delta=%.6f output deg\n",
                before->raw2, after->raw2, zero_shortest_deg(after->raw2-before->raw2));
    else fputs("ENC2 RAW angle unavailable; calibrated angle is NOT raw angle. Apply firmware diagnostic patch to expose it.\n", log);
    fflush(log); pthread_mutex_unlock(&boot_capture_mutex);
}

typedef struct { unsigned int completed, valid, matched; } zero_cycle_fixed_stats;

static int zero_cycle_retest(flash_state *state, const motor_map_entry *m, size_t slot,
                              const zero_cycle_config *c, unsigned int failed_cycle,
                              double target_turns, float baseline, const char *csv_path,
                              const zero_encoder_snapshot *before_snapshot, zero_cycle_fixed_stats *stats)
{
    *stats = (zero_cycle_fixed_stats){0};
    char log_path[512];
    snprintf(log_path, sizeof(log_path), "%s.cycle-%u.retries.log", csv_path, failed_cycle);
    FILE *log = fopen(log_path, "wx");
    if (!log) { perror(log_path); return -1; }
    double baseline_turns = baseline * c->ratio / (2 * M_PI);
    fprintf(log, "Failed cycle=%u motor_index=%u bus=%u id=%u gear_ratio=%.12g\n"
                 "target_motor_turns=%.9g original_before_motor_turns=%.9g tolerance_motor_turns=%.9g\n"
                 "10 additional power cycles, NO motion/enable/zero; all errors relative to original baseline.\n",
            failed_cycle, m->index, m->bus, m->id, c->ratio, target_turns, baseline_turns, c->tolerance);
    pthread_mutex_lock(&boot_capture_mutex);
    FILE *previous_capture = boot_capture_file;
    boot_capture_bus = m->bus; boot_capture_id = m->id; boot_capture_file = log;
    pthread_mutex_unlock(&boot_capture_mutex);
    printf(console_text(state,
           "\n失败点复测：保持当前位置，不使能、不运动、不置零；额外上下电 10 次。\n上电原始输出及角度记录：%s\n",
           "\nFailed-point retest: 10 additional power cycles, no enable/motion/zero.\nRaw boot output and angle results: %s\n"), log_path);
    unsigned int completed = 0, valid = 0, matched = 0;
    for (unsigned int i = 1; i <= 10 && !stop_requested; ++i) {
        pthread_mutex_lock(&boot_capture_mutex);
        fprintf(log, "\n\n========== RETEST %u/10 POWER OFF/ON ==========\n", i); fflush(log);
        pthread_mutex_unlock(&boot_capture_mutex);
        uint64_t off_begin = time_us();
        if (zero_cycle_operation(state, 0, slot)) break;
        while (!stop_requested && time_us() - off_begin < (uint64_t)c->off_ms * 1000) sleep_ms(10);
        if (stop_requested) break;
        bxi_motor_reply reply;
        int powered = zero_cycle_operation(state, 1, slot);
        bool readable = powered == 0 && state->motors[slot].online &&
                        zero_cycle_sample(state, m, 0, 0, 0, &reply) == 0;
        ++completed;
        if (readable) {
            ++valid;
            double turns = reply.position * c->ratio / (2 * M_PI);
            double delta = turns - baseline_turns;
            bool ok = fabs(delta) <= c->tolerance;
            matched += ok;
            printf(console_text(state,
                   "复测 %2u/10：上电电机端 %+.6f 圈 / %+.3f°，相对原断电前差值 %+.7f 圈 / %+.4f°，%s\n",
                   "Retest %2u/10: motor %+.6f turns / %+.3f deg; delta from original baseline %+.7f turns / %+.4f deg; %s\n"),
                   i, turns, turns * 360, delta, delta * 360, ok ? "PASS" : "FAIL");
            pthread_mutex_lock(&boot_capture_mutex);
            fprintf(log, "\nRESULT retry=%u output_rad=%.9g motor_turns=%.9g motor_deg=%.9g delta_motor_turns=%.9g delta_motor_deg=%.9g %s\n",
                    i, reply.position, turns, turns * 360, delta, delta * 360, ok ? "PASS" : "FAIL");
            pthread_mutex_unlock(&boot_capture_mutex);
        } else {
            printf("zero_cycle_test: retest=%u/10 ERROR no valid position after power-on\n", i);
            pthread_mutex_lock(&boot_capture_mutex);
            fprintf(log, "\nRESULT retry=%u ERROR no valid position\n", i);
            pthread_mutex_unlock(&boot_capture_mutex);
        }
        if (powered == 0) {
            zero_encoder_snapshot snapshot = zero_encoder_query(state, m, log, "RETEST_AFTER_BOOT");
            zero_encoder_compare(log, before_snapshot, &snapshot, c->ratio);
        }
        /* Keep capturing late startup messages, without motion commands. */
        console_quiet_sleep_ms(1000);
        fflush(stdout);
    }
    pthread_mutex_lock(&boot_capture_mutex);
    boot_capture_file = previous_capture;
    fprintf(log, "\nSUMMARY completed=%u valid=%u baseline_matches=%u interrupted=%d\n", completed, valid, matched, stop_requested != 0);
    fclose(log);
    pthread_mutex_unlock(&boot_capture_mutex);
    printf(console_text(state, "失败点复测结束：完成 %u/10，有效读取 %u 次，与原断电前一致 %u 次。日志：%s\n",
                              "Retest finished: completed %u/10, valid %u, baseline matches %u. Log: %s\n"),
           completed, valid, matched, log_path);
    *stats = (zero_cycle_fixed_stats){completed, valid, matched};
    return completed == 10 && valid == 10 && !stop_requested ? 0 : -1;
}

static int zero_cycle_compensated_target(double motor_turns, double offset_rad,
                                         const zero_cycle_config *c, const bxi_motor_limits *limits,
                                         float *command)
{
    double value = motor_turns * 2 * M_PI / c->ratio + offset_rad;
    if (!isfinite(value) || fabs(motor_turns) > c->max_turns ||
        value < limits->p_min || value > limits->p_max) return -1;
    *command = (float)value;
    return 0;
}

static int zero_cycle_return_original_zero(flash_state *state, const motor_map_entry *m, size_t slot,
                                           const zero_cycle_config *c, double offset_rad, FILE *log,
                                           const zero_encoder_snapshot *initial_zero)
{
    float target;
    if (zero_cycle_compensated_target(0, offset_rad, c, limits_for_entry(state, m), &target)) {
        fprintf(stderr, "zero_cycle_test: ERROR compensated zero outside MIT range\n"); return -1;
    }
    printf(console_text(state,
           "相对补偿返回原零位：原零位目标 0 圈，固件坐标目标 %+.6f 圈；不发送置零或保存命令。\n",
           "Return to original zero by relative compensation: physical target 0 turns, firmware target %+.6f turns; no ZERO/save.\n"),
           target*c->ratio/(2*M_PI)); fflush(stdout);
    bxi_motor_reply reply; unsigned int retries;
    if (zero_cycle_enable_confirmed(state, m, slot) ||
        zero_cycle_move(state, m, c, target, &reply, &retries) != 0 ||
        zero_cycle_operation(state, BXI_MOTOR_CMD_DISABLE, slot) ||
        zero_cycle_sample(state, m, 0, 0, 0, &reply)) return -1;
    double corrected = (reply.position-offset_rad)*c->ratio/(2*M_PI);
    if (fabs(corrected) > c->arrival_tolerance) {
        fprintf(stderr, "zero_cycle_test: ERROR original-zero recovery did not hold after disable: %.6f motor turns\n", corrected);
        return -1;
    }
    zero_encoder_snapshot snapshot = zero_encoder_query(state, m, log, "RECOVERED_ORIGINAL_ZERO_NO_REZERO");
    printf(console_text(state, "原零位恢复读数：固件位置 %+.6f 圈，补偿后位置 %+.6f 圈（%+.3f°）。\n",
                              "Recovered zero readback: firmware %+.6f turns, compensated %+.6f turns (%+.3f deg).\n"),
           reply.position*c->ratio/(2*M_PI), corrected, corrected*360);
    if (snapshot.enc1_valid) printf("  ENC1 raw=%d raw_deg=%.6f firmware_pos=%.6f turns\n", snapshot.raw1, snapshot.raw1*360./snapshot.cpr1, snapshot.pos1);
    if (snapshot.enc2_valid) printf("  ENC2 calibrated_deg=%.6f output_pos=%.6f turns\n", snapshot.angle2, snapshot.pos2);
    if (snapshot.raw2_valid) printf("  ENC2_RAW_DEG=%.9f\n", snapshot.raw2);
    else puts("  ENC2_RAW_DEG unavailable (requires diagnostic firmware)");
    if (initial_zero->enc1_valid && snapshot.enc1_valid) {
        double delta = zero_shortest_deg(snapshot.raw1*360./snapshot.cpr1-initial_zero->raw1*360./initial_zero->cpr1);
        printf("  原零位 ENC1 raw angle delta=%.6f motor deg\n", delta);
        if (fabs(delta) > c->arrival_tolerance*360 + 360./snapshot.cpr1) {
            fprintf(stderr, "zero_cycle_test: ERROR recovered zero contradicts initial ENC1 raw phase; stop compensation\n"); return -1;
        }
    }
    if (initial_zero->raw2_valid && snapshot.raw2_valid) {
        double delta = zero_shortest_deg(snapshot.raw2-initial_zero->raw2);
        printf("  原零位 ENC2 raw angle delta=%.6f output deg\n", delta);
        if (fabs(delta) > c->arrival_tolerance*360/c->ratio + .01) {
            fprintf(stderr, "zero_cycle_test: ERROR recovered zero contradicts initial ENC2 raw angle; stop compensation\n"); return -1;
        }
    }
    pthread_mutex_lock(&boot_capture_mutex);
    fprintf(log, "RECOVERY firmware_output_rad=%.9g offset_output_rad=%.9g corrected_motor_turns=%.9g\n", reply.position, offset_rad, corrected);
    fflush(log); pthread_mutex_unlock(&boot_capture_mutex);
    return 0;
}

enum { ZERO_ANOMALY_MAX = 128 };
typedef struct {
    double motor_turns, enc2_raw_deg;
    unsigned int first_cycle, observations, failures, planned_returns, reproduced_returns, missed_returns;
    unsigned int fixed_trials, fixed_failures, fixed_errors;
    bool raw_valid;
} zero_cycle_anomaly;

static int zero_cycle_find_anomaly(const zero_cycle_anomaly *points, unsigned int count, double turns, double tolerance)
{
    int found = -1; double distance = tolerance;
    for (unsigned int i = 0; i < count; ++i) {
        double delta = fabs(points[i].motor_turns-turns);
        if (delta <= distance) { found = (int)i; distance = delta; }
    }
    return found;
}

static void zero_cycle_summary(const char *path, const zero_cycle_anomaly *points, unsigned int count,
                               unsigned int passed, unsigned int failures, unsigned int fallbacks, bool completed,
                               double elapsed_s, double grouping_tolerance)
{
    char summary_path[512]; snprintf(summary_path, sizeof(summary_path), "%s.summary.csv", path);
    FILE *file = fopen(summary_path, "wx");
    printf("\n========== 测试汇总 / TEST SUMMARY ==========\n"
           "完成流程 completed=%s；比较轮数=%u；位置一致 PASS=%u；零位丢失 FAIL=%u；目标未到位兜底=%u；耗时=%.1fs\n"
           "异常点分组容差：%.6f 电机圈；固定位置10次重启不计入离开后返回复现次数。\n",
           completed ? "YES" : "NO", passed+failures, passed, failures, fallbacks, elapsed_s, grouping_tolerance);
    if (file) {
        fprintf(file, "# completed=%d passed=%u position_failures=%u fallbacks=%u elapsed_s=%.3f grouping_tolerance_motor_turns=%.9g\n", completed, passed, failures, fallbacks, elapsed_s, grouping_tolerance);
        fputs("anomaly_id,motor_turns,motor_deg,first_failed_cycle,observations_since_discovery,failures,planned_returns,reproduced_returns,missed_returns,fixed_retest_trials,fixed_retest_failures,fixed_retest_errors,enc2_raw_deg\n", file);
    } else perror(summary_path);
    for (unsigned int i = 0; i < count; ++i) {
        const zero_cycle_anomaly *p = &points[i];
        printf("异常点 #%u：%+.6f 圈 / %+.3f°，记录后比较 %u 次，失败 %u 次；安排返回 %u 次，返回后复现 %u 次，返回未到位 %u 次；ENCO raw=",
               i+1, p->motor_turns, p->motor_turns*360, p->observations, p->failures,
               p->planned_returns, p->reproduced_returns, p->missed_returns);
        if (p->raw_valid) printf("%.6f°\n", p->enc2_raw_deg); else puts("unavailable");
        printf("  原地重启复测：%u 次，位置不一致 %u 次，读取失败 %u 次。\n", p->fixed_trials, p->fixed_failures, p->fixed_errors);
        if (file) fprintf(file, "%u,%.9g,%.9g,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.9g\n", i+1, p->motor_turns, p->motor_turns*360,
                          p->first_cycle, p->observations, p->failures, p->planned_returns, p->reproduced_returns,
                          p->missed_returns, p->fixed_trials, p->fixed_failures, p->fixed_errors, p->raw_valid ? p->enc2_raw_deg : NAN);
    }
    if (file) { fclose(file); printf("汇总文件：%s\n", summary_path); }
    fflush(stdout);
}

static int console_zero_cycle_test(flash_state *state, int argc, char **argv)
{
    unsigned int index; size_t slot; zero_cycle_config c;
    if ((argc != 2 && argc != 3) || console_parse_index_arg(argv[1], &index)) {
        printf("usage: zero_cycle_test <index00> [config.yaml]\n"); return -1;
    }
    const char *config_path = argc == 3 ? argv[2] : "config/zero_cycle_test.yaml";
    if (zero_cycle_load(config_path, &c)) return -1;
    const motor_map_entry *m = console_motor_by_index(state, index, &slot);
    if (!m || strstr(m->type, "hand")) { fprintf(stderr, "zero_cycle_test: unknown motor or unsupported hand\n"); return -1; }
    const bxi_motor_limits *limits = limits_for_entry(state, m);
    double max_rad = c.max_turns * 2 * M_PI / c.ratio;
    if (-max_rad < limits->p_min || max_rad > limits->p_max ||
        c.speed * 2 * M_PI / c.ratio > fmin(limits->v_max, -limits->v_min) ||
        c.kp < limits->kp_min || c.kp > limits->kp_max || c.kd < limits->kd_min || c.kd > limits->kd_max ||
        2 * fmin(c.tolerance, c.arrival_tolerance) * 2 * M_PI / c.ratio < (limits->p_max - limits->p_min) / 65535.) {
        fprintf(stderr, "zero_cycle_test: range/gains exceed MIT limits or tolerance below quantization\n"); return -1;
    }
    if (state->motor_power_on || console_any_enabled(state)) {
        fprintf(stderr, "zero_cycle_test: start with power OFF; shared power cycles affect every connected motor\n"); return -1;
    }
    char path[256]; snprintf(path, sizeof(path), "/tmp/zero-cycle-%ld-%llu.csv", (long)getpid(), (unsigned long long)time_us());
    FILE *csv = fopen(path, "wx"); if (!csv) { perror(path); return -1; }
    char readable_path[512]; snprintf(readable_path, sizeof(readable_path), "%s.boot.log", path);
    FILE *readable = fopen(readable_path, "wx");
    if (!readable) { perror(readable_path); fclose(csv); return -1; }
    fprintf(csv, "cycle,target_motor_turns,before_output_rad,after_output_rad,error_motor_turns,result,target_reached,gain_retries,physical_before_motor_turns,physical_after_motor_turns_before_correction,coordinate_offset_motor_turns,test_kind,anomaly_id,before_enc2_raw_deg,after_enc2_raw_deg,requested_anomaly_id,revisit_hit\n"); fflush(csv);
    unsigned int seed = c.seed ? c.seed : (unsigned int)time_us();
    fprintf(csv, "# motor_index=%u bus=%u id=%u gear_ratio=%.12g seed=%u\n", index, m->bus, m->id, c.ratio, seed); fflush(csv);
    printf("zero_cycle_test: index=%02u ratio=%.9g range=+/-%.6g motor turns seed=%u csv=%s\n", index, c.ratio, c.max_turns, seed, path);
    printf("zero_cycle_test: readable boot/encoder diagnostics: %s\n", readable_path);
    pthread_mutex_lock(&boot_capture_mutex);
    boot_capture_bus = m->bus; boot_capture_id = m->id; boot_capture_file = readable;
    pthread_mutex_unlock(&boot_capture_mutex);
    bool old_output = state->show_can_output, old_input = state->show_motor_input;
    unsigned int old_bus = state->bus, old_id = state->boot_id;
    state->show_can_output = state->show_motor_input = false;
    int result = -1; unsigned int passed = 0, failures = 0, fallback_cycles = 0; bxi_motor_reply reply;
    unsigned int order[36], random_index = 0, random_since_revisit = 0, revisit_cursor = 0, anomaly_count = 0;
    zero_cycle_anomaly anomalies[ZERO_ANOMALY_MAX] = {0};
    double offset_rad = 0, scale = 2*M_PI/c.ratio;
    bool completed = false;
    uint64_t begin = time_us();
    if (zero_cycle_operation(state, 1, slot) || !state->motors[slot].online ||
        zero_cycle_operation(state, BXI_MOTOR_CMD_DISABLE, slot) ||
        zero_cycle_sample(state, m, 0, 0, 0, &reply)) goto done;
    float initial_before = reply.position;
    if (zero_cycle_operation(state, BXI_MOTOR_CMD_ZERO, slot)) goto done;
    /* The reference firmware only accepts ZERO in menu/disabled state.
     * Keep its save response visible independently of quiet terminal output. */
    char save_output[512] = {0};
    rx_can_frame save_frame;
    while (frame_ring_pop(&state->frames, &save_frame, 0)) {
        if (save_frame.bus != m->bus || !boot_output_id_matches(save_frame.can_id, m->id)) continue;
        size_t used = strlen(save_output);
        if (used + save_frame.len >= sizeof(save_output)) {
            memmove(save_output, save_output + 256, used - 256 + 1);
            used -= 256;
        }
        memcpy(save_output + used, save_frame.data, save_frame.len);
        save_output[used + save_frame.len] = 0;
    }
    bool save_failed = strstr(save_output, "User Config Update FAIL!") != NULL;
    bool save_confirmed = strstr(save_output, "User Config Update OK!") != NULL;
    console_quiet_sleep_ms(500);
    if (save_failed || zero_cycle_sample(state, m, 0, 0, 0, &reply) ||
        fabs(reply.position) * c.ratio / (2 * M_PI) > c.tolerance) {
        fprintf(stderr, "zero_cycle_test: ERROR initial zero not confirmed\n"); goto done;
    }
    printf(console_text(state,
           "\n初始置零确认：置零前 %+.6f 圈，置零后 %+.6f 圈（电机端）；反馈零位检查 PASS。\n保存回复：%s\n",
           "\nInitial zero: before %+.6f turns, after %+.6f turns (motor); feedback check PASS.\nSave response: %s\n"),
           initial_before * c.ratio / (2 * M_PI), reply.position * c.ratio / (2 * M_PI),
           save_confirmed ? "User Config Update OK!" : console_text(state, "未收到明确保存确认，持久化尚未验证", "No explicit save ACK; persistence unverified"));
    fprintf(csv, "# initial_zero_before_rad=%.9g after_rad=%.9g save_ack=%s\n", initial_before, reply.position,
            save_confirmed ? "OK" : "UNCONFIRMED"); fflush(csv); fflush(stdout);
    zero_encoder_snapshot initial_zero_snapshot = zero_encoder_query(state, m, readable, "INITIAL_ORIGINAL_ZERO");
    for (unsigned int n = 0; !stop_requested && (!c.count || n < c.count) &&
         (!c.duration_s || time_us() - begin < (uint64_t)c.duration_s * 1000000); ++n) {
        bool revisit = anomaly_count && random_since_revisit >= c.revisit_after_random_cycles;
        unsigned int requested_anomaly = 0;
        double motor_target;
        if (revisit) {
            requested_anomaly = revisit_cursor++ % anomaly_count + 1;
            motor_target = anomalies[requested_anomaly-1].motor_turns;
            ++anomalies[requested_anomaly-1].planned_returns;
            random_since_revisit = 0;
            printf("\n异常点返回复现测试 / REVISIT #%u\n", requested_anomaly);
        } else {
            motor_target = zero_cycle_target(random_index++, &seed, c.max_turns, order);
            if (anomaly_count) ++random_since_revisit;
        }
        float target;
        if (zero_cycle_compensated_target(motor_target, offset_rad, &c, limits, &target)) {
            fprintf(stderr, "zero_cycle_test: ERROR compensated command outside physical/MIT range; stopping without rezero\n"); goto done;
        }
        printf(console_text(state,
               "\n========== 第 %u 轮开始：目标电机端 %+.6f 圈（%+.3f°），输出端 %+.3f° ==========\n",
               "\n========== Cycle %u start: motor target %+.6f turns (%+.3f deg), output target %+.3f deg ==========\n"),
               n + 1, motor_target, motor_target * 360, motor_target * 360 / c.ratio);
        fflush(stdout);
        if (zero_cycle_enable_confirmed(state, m, slot)) {
            if (!stop_requested && zero_cycle_operation(state, BXI_MOTOR_CMD_DISABLE, slot) == 0)
                zero_encoder_query(state, m, readable, "ENABLE_FAILURE_AFTER_DISABLE");
            goto done;
        }
        unsigned int retries_used;
        int motion_result = zero_cycle_move(state, m, &c, target, &reply, &retries_used);
        if (motion_result < 0 ||
            (motion_result > 0 && zero_cycle_settle_current(state, m, &c, &reply))) {
            if (!stop_requested && zero_cycle_operation(state, BXI_MOTOR_CMD_DISABLE, slot) == 0)
                zero_encoder_query(state, m, readable, "MOVE_FAILURE_AFTER_DISABLE");
            goto done;
        }
        if (motion_result > 0) {
            ++fallback_cycles;
            printf(console_text(state, "本轮目标未到达；使用当前稳定位置 %+.6f 电机圈作为上下电基准，不计为目标到位成功。\n",
                                      "Target NOT reached; using current stable motor position %+.6f turns as power-cycle baseline, not a motion success.\n"),
                   reply.position*c.ratio/(2*M_PI));
            fflush(stdout);
        }
        /* Sample after disabling, so the baseline includes any release motion. */
        if (zero_cycle_operation(state, BXI_MOTOR_CMD_DISABLE, slot) ||
            zero_cycle_sample(state, m, 0, 0, 0, &reply)) goto done;
        float before = reply.position;
        double physical_before = (before-offset_rad)/scale;
        zero_encoder_snapshot before_snapshot = zero_encoder_query(state, m, readable, "BEFORE_POWER_OFF");
        uint64_t off_begin = time_us();
        if (zero_cycle_operation(state, 0, slot)) goto done;
        while (!stop_requested && time_us() - off_begin < (uint64_t)c.off_ms * 1000) sleep_ms(10);
        if (stop_requested || zero_cycle_operation(state, 1, slot) || !state->motors[slot].online ||
            zero_cycle_sample(state, m, 0, 0, 0, &reply)) {
            printf(console_text(state, "第 %u 轮失败：重新上电后未能获取有效位置，无法比较。\n",
                                      "Cycle %u FAILED: no valid position after reboot; comparison unavailable.\n"), n + 1);
            goto done;
        }
        double error = (reply.position - before) * c.ratio / (2 * M_PI);
        zero_encoder_snapshot after_snapshot = zero_encoder_query(state, m, readable, "AFTER_POWER_ON");
        zero_encoder_compare(readable, &before_snapshot, &after_snapshot, c.ratio);
        bool ok = fabs(error) <= c.tolerance;
        int point = zero_cycle_find_anomaly(anomalies, anomaly_count, physical_before, c.arrival_tolerance);
        bool revisit_hit = revisit && fabs(physical_before-motor_target) <= c.arrival_tolerance;
        if (revisit && !revisit_hit) ++anomalies[requested_anomaly-1].missed_returns;
        if (revisit_hit) point = (int)requested_anomaly-1;
        if (!ok) {
            ++failures;
            if (point < 0 && anomaly_count < ZERO_ANOMALY_MAX) {
                point = (int)anomaly_count++;
                anomalies[point].motor_turns = physical_before;
                anomalies[point].first_cycle = n+1;
            }
        }
        if (point >= 0) {
            ++anomalies[point].observations;
            if (!ok) ++anomalies[point].failures;
            if (!ok && revisit_hit) ++anomalies[point].reproduced_returns;
            if (!anomalies[point].raw_valid && before_snapshot.raw2_valid) {
                anomalies[point].raw_valid = true; anomalies[point].enc2_raw_deg = before_snapshot.raw2;
            }
        }
        fprintf(csv, "%u,%.9g,%.9g,%.9g,%.9g,%s,%s,%u,%.9g,%.9g,%.9g,%s,%u,%.9g,%.9g,%u,%s\n", n + 1, motor_target, before, reply.position, error,
                ok ? "PASS" : "FAIL", motion_result == 0 ? "true" : "false", retries_used, physical_before,
                (reply.position-offset_rad)/scale, offset_rad/scale, revisit ? "revisit" : "random", point >= 0 ? (unsigned)point+1 : 0,
                before_snapshot.raw2_valid ? before_snapshot.raw2 : NAN, after_snapshot.raw2_valid ? after_snapshot.raw2 : NAN,
                requested_anomaly, revisit_hit ? "true" : "false"); fflush(csv);
        double before_turns = physical_before;
        double after_turns = (reply.position-offset_rad)/scale;
        printf(console_text(state,
               "\n========== 第 %u 轮上下电零位识别结果：%s ==========\n"
               "  目标位置（电机端）：%+.6f 圈 / %+.3f°\n"
               "  断电之前（电机端）：%+.6f 圈 / %+.3f°\n"
               "  重新上电（电机端）：%+.6f 圈 / %+.3f°\n"
               "  上电后 - 断电前 ：%+.7f 圈 / %+.4f°\n"
               "  允许绝对差值    ：%.6f 圈 / %.3f°\n"
               "  目标跟踪误差    ：%+.6f 圈（断电前实际位置 - 目标）\n"
               "====================================================\n\n",
               "\n========== Cycle %u power-cycle recognition: %s ==========\n"
               "  Motor target : %+.6f turns / %+.3f deg\n"
               "  Before off   : %+.6f turns / %+.3f deg\n"
               "  After reboot : %+.6f turns / %+.3f deg\n"
               "  After-before : %+.7f turns / %+.4f deg\n"
               "  Absolute tolerance: %.6f turns / %.3f deg\n"
               "  Tracking error: %+.6f turns (before-target)\n"
               "====================================================\n\n"),
               n + 1, ok ? console_text(state, "成功 PASS", "PASS") : console_text(state, "失败 FAIL", "FAIL"),
               motor_target, motor_target * 360, before_turns, before_turns * 360,
               after_turns, after_turns * 360, error, error * 360,
               c.tolerance, c.tolerance * 360, before_turns - motor_target);
        fflush(stdout);
        printf("  软件坐标偏移 / firmware-minus-original: %+.6f 电机圈；异常点 #%u；revisit_hit=%s\n",
               offset_rad/scale, point >= 0 ? (unsigned)point+1 : 0, revisit_hit ? "YES" : "NO");
        printf(console_text(state, "  目标到位：%s；增益重试 %u 次。本轮 PASS/FAIL 仅判断上下电位置一致性。\n",
                                  "  Target reached: %s; gain retries=%u. PASS/FAIL above tests power-cycle position consistency only.\n"),
               motion_result == 0 ? console_text(state, "是", "YES") : console_text(state, "否（当前位置兜底）", "NO (current-position fallback)"), retries_used);
        fflush(stdout);
        if (!ok) {
            zero_encoder_compare(stdout, &before_snapshot, &after_snapshot, c.ratio);
            zero_cycle_fixed_stats fixed;
            int fixed_result = zero_cycle_retest(state, m, slot, &c, n + 1, motor_target, before, path, &before_snapshot, &fixed);
            if (point >= 0) {
                anomalies[point].fixed_trials += fixed.completed;
                anomalies[point].fixed_failures += fixed.valid-fixed.matched;
                anomalies[point].fixed_errors += fixed.completed-fixed.valid;
            }
            if (fixed_result) goto done;
            if (!c.continue_after_position_failure) goto done;
            if (point < 0) {
                fprintf(stderr, "zero_cycle_test: ERROR anomaly table full; stopping\n"); goto done;
            }
            /* No motion during fixed-point retries: re-anchor the latest raw
             * coordinate to the original trusted physical baseline. Never
             * change the raw error result or send ZERO/save to the drive. */
            if (!state->motor_power_on || zero_cycle_sample(state, m, 0, 0, 0, &reply)) goto done;
            offset_rad = reply.position - physical_before*scale;
            fprintf(csv, "# compensation_after_cycle=%u offset_output_rad=%.9g trusted_motor_turns=%.9g latest_firmware_output_rad=%.9g\n",
                    n+1, offset_rad, physical_before, reply.position); fflush(csv);
            if (zero_cycle_return_original_zero(state, m, slot, &c, offset_rad, readable, &initial_zero_snapshot)) goto done;
            if (anomaly_count == 1 && anomalies[0].first_cycle == n+1) random_since_revisit = 0;
            continue;
        }
        ++passed;
    }
    completed = !stop_requested;
    result = stop_requested || failures ? -1 : 0;
done:
    if (state->motor_power_on && zero_cycle_operation(state, 0, slot)) { result = -1; completed = false; }
    pthread_mutex_lock(&boot_capture_mutex);
    boot_capture_file = NULL;
    fclose(readable);
    pthread_mutex_unlock(&boot_capture_mutex);
    state->show_can_output = old_output; state->show_motor_input = old_input;
    state->bus = old_bus; state->boot_id = old_id; update_boot_ids(state);
    const char *outcome = completed ? (failures ? "COMPLETED_WITH_POSITION_FAILURES" : "PASS") : "STOPPED";
    if (result != 0 && !failures) outcome = "STOPPED";
    fprintf(csv, "# result=%s passed=%u position_failures=%u current_position_fallbacks=%u\n", outcome, passed, failures, fallback_cycles); fclose(csv);
    zero_cycle_summary(path, anomalies, anomaly_count, passed, failures, fallback_cycles, completed, (time_us()-begin)/1e6, c.arrival_tolerance);
    printf("zero_cycle_test: %s passed=%u position_failures=%u current_position_fallbacks=%u csv=%s\n", outcome, passed, failures, fallback_cycles, path);
    return result;
}
