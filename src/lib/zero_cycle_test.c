/* Included by main.c. All angles in this test configuration are motor-side turns. */
typedef struct {
    double ratio, max_turns, speed, kp, kd, tolerance;
    unsigned int count, duration_s, off_ms, settle_ms, move_timeout_ms, seed;
} zero_cycle_config;

static int zero_cycle_load(const char *path, zero_cycle_config *c)
{
    *c = (zero_cycle_config){0, 9, 1, 5, 1, .01, 100, 0, 1000, 500, 120000, 0};
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
        else ZDOUBLE("kp", kp)
        else ZDOUBLE("kd", kd)
        else ZDOUBLE("tolerance_motor_turns", tolerance)
        else ZUINT("count", count)
        else ZUINT("duration_s", duration_s)
        else ZUINT("off_ms", off_ms)
        else ZUINT("settle_ms", settle_ms)
        else ZUINT("move_timeout_ms", move_timeout_ms)
        else ZUINT("seed", seed)
        else { bad = 1; break; }
#undef ZDOUBLE
#undef ZUINT
    }
    if (ferror(f)) bad = 1;
    fclose(f);
    if (bad || c->ratio <= 1 || c->max_turns <= 0 || c->max_turns > 9 ||
        c->max_turns / c->ratio >= .5 || c->speed <= 0 || c->speed > 10 ||
        c->kp <= 0 || c->kd < 0 || c->tolerance <= 0 || c->tolerance >= .25 ||
        (!c->count && !c->duration_s) || c->off_ms < 300 || c->off_ms > 60000 ||
        c->settle_ms < 100 || c->settle_ms > 60000 || !c->move_timeout_ms || c->move_timeout_ms > 3600000) {
        fprintf(stderr, "zero_cycle_test: invalid config; set actual gear_ratio; require max_motor_turns<=9 and max_motor_turns/gear_ratio<0.5\n");
        return -1;
    }
    return 0;
}

static double zero_cycle_target(unsigned int n, unsigned int *seed, double range)
{
    /* Visit all 36 equal sectors per block, jitter within each sector. */
    unsigned int sector = (n % 36u * 17u) % 36u;
    double jitter = (rand_r(seed) + .5) / ((double)RAND_MAX + 1.0);
    return -range + 2 * range * (sector + jitter) / 36.0;
}

static int zero_cycle_sample(flash_state *state, const motor_map_entry *m,
                             float target, float kp, float kd, bxi_motor_reply *reply)
{
    frame_ring_clear(&state->frames);
    console_use_motor(state, m);
    uint64_t sent = time_us();
    console_expect_reply(state, m->bus);
    if (send_debug_mit(state, target, 0, kp, kd, 0)) return -1;
    while (!stop_requested && time_us() - sent < 100000) {
        rx_can_frame frame;
        if (!frame_ring_pop(&state->frames, &frame, 10)) continue;
        if (frame.time_us < sent || frame.bus != m->bus || !mit_reply_frame_matches(&frame, m->id)) continue;
        int result = state->motors[m - state->config.entries].mit_aux_enabled ?
            bxi_motor_unpack_reply(frame.data, frame.len, &state->limits, reply) :
            bxi_motor_unpack_reply_legacy_ntc(frame.data, frame.len, &state->limits, reply);
        if (!result && isfinite(reply->position) && isfinite(reply->velocity)) return 0;
    }
    fprintf(stderr, "zero_cycle_test: ERROR fresh MIT feedback timeout\n");
    return -1;
}

static double zero_cycle_speed_threshold(const bxi_motor_limits *limits, const zero_cycle_config *c)
{
    /* Symmetric 12-bit velocity has no exact zero code. Allow its half-LSB
     * residual, without relaxing the independent position comparison. */
    double half_lsb = (limits->v_max - limits->v_min) / 4095.0 / 2;
    return fmax(c->tolerance, half_lsb * c->ratio / (2 * M_PI) * 1.01);
}

static int zero_cycle_move(flash_state *state, const motor_map_entry *m,
                           const zero_cycle_config *c, float target, bxi_motor_reply *reply)
{
    if (zero_cycle_sample(state, m, 0, 0, 0, reply)) return -1;
    float start = reply->position;
    uint64_t begin = time_us(), stable = 0, next_report = 0;
    double speed_threshold = zero_cycle_speed_threshold(limits_for_entry(state, m), c);
    double ramp_s = fabs(target - start) / (c->speed * 2 * M_PI / c->ratio);
    printf("zero_cycle_test: move target_motor_turns=%.6f target_output_rad=%.7f ramp_s=%.2f speed_threshold_motor_turns_s=%.6f\n",
           target * c->ratio / (2 * M_PI), target, ramp_s, speed_threshold);
    fflush(stdout);
    while (!stop_requested && time_us() - begin < (uint64_t)c->move_timeout_ms * 1000) {
        double t = (time_us() - begin) / 1e6;
        double fraction = ramp_s > 0 ? fmin(t / ramp_s, 1) : 1;
        float setpoint = start + (target - start) * fraction;
        if (zero_cycle_sample(state, m, setpoint, c->kp, c->kd, reply)) return -1;
        double error = fabs(reply->position - target) * c->ratio / (2 * M_PI);
        double speed = fabs(reply->velocity) * c->ratio / (2 * M_PI);
        if (fraction == 1 && error <= c->tolerance && speed <= speed_threshold) {
            if (!stable) stable = time_us();
            if (time_us() - stable >= (uint64_t)c->settle_ms * 1000) return 0;
        } else stable = 0;
        if (time_us() >= next_report) {
            printf("zero_cycle_test: moving elapsed_s=%.1f phase=%s actual_motor_turns=%.6f error_motor_turns=%.6f speed_motor_turns_s=%.6f stable_ms=%.0f\n",
                   t, fraction < 1 ? "ramp" : "settle", reply->position * c->ratio / (2 * M_PI),
                   error, speed, stable ? (time_us() - stable) / 1000.0 : 0.0);
            fflush(stdout);
            next_report = time_us() + 1000000;
        }
        sleep_ms(10);
    }
    fprintf(stderr, "zero_cycle_test: ERROR move/settle timeout or interrupted\n");
    return -1;
}

static int console_zero_cycle_test(flash_state *state, int argc, char **argv)
{
    unsigned int index; size_t slot; zero_cycle_config c;
    if (argc != 3 || console_parse_index_arg(argv[1], &index) ||
        zero_cycle_load(argv[2], &c)) {
        printf("usage: zero_cycle_test <index00> <config.yaml>\n"); return -1;
    }
    const motor_map_entry *m = console_motor_by_index(state, index, &slot);
    if (!m || strstr(m->type, "hand")) { fprintf(stderr, "zero_cycle_test: unknown motor or unsupported hand\n"); return -1; }
    const bxi_motor_limits *limits = limits_for_entry(state, m);
    double max_rad = c.max_turns * 2 * M_PI / c.ratio;
    if (-max_rad < limits->p_min || max_rad > limits->p_max ||
        c.kp < limits->kp_min || c.kp > limits->kp_max || c.kd < limits->kd_min || c.kd > limits->kd_max ||
        2 * c.tolerance * 2 * M_PI / c.ratio < (limits->p_max - limits->p_min) / 65535.) {
        fprintf(stderr, "zero_cycle_test: range/gains exceed MIT limits or tolerance below quantization\n"); return -1;
    }
    if (state->motor_power_on || console_any_enabled(state)) {
        fprintf(stderr, "zero_cycle_test: start with power OFF; shared power cycles affect every connected motor\n"); return -1;
    }
    char path[256]; snprintf(path, sizeof(path), "/tmp/zero-cycle-%ld-%llu.csv", (long)getpid(), (unsigned long long)time_us());
    FILE *csv = fopen(path, "wx"); if (!csv) { perror(path); return -1; }
    fprintf(csv, "cycle,target_motor_turns,before_output_rad,after_output_rad,error_motor_turns,result\n"); fflush(csv);
    unsigned int seed = c.seed ? c.seed : (unsigned int)time_us();
    printf("zero_cycle_test: index=%02u ratio=%.9g range=+/-%.6g motor turns seed=%u csv=%s\n", index, c.ratio, c.max_turns, seed, path);
    bool old_output = state->show_can_output, old_input = state->show_motor_input;
    unsigned int old_bus = state->bus, old_id = state->boot_id;
    state->show_can_output = state->show_motor_input = false;
    int result = -1; unsigned int passed = 0; bxi_motor_reply reply;
    uint64_t begin = time_us();
    if (console_power_on(state) || !state->motors[slot].online ||
        console_send_special(state, slot, BXI_MOTOR_CMD_ZERO, "zero_cycle_initial_zero")) goto done;
    console_quiet_sleep_ms(500);
    if (zero_cycle_sample(state, m, 0, 0, 0, &reply) ||
        fabs(reply.position) * c.ratio / (2 * M_PI) > c.tolerance) {
        fprintf(stderr, "zero_cycle_test: ERROR initial zero not confirmed\n"); goto done;
    }
    for (unsigned int n = 0; !stop_requested && (!c.count || n < c.count) &&
         (!c.duration_s || time_us() - begin < (uint64_t)c.duration_s * 1000000); ++n) {
        double motor_target = zero_cycle_target(n, &seed, c.max_turns);
        float target = motor_target * 2 * M_PI / c.ratio;
        if (console_send_special(state, slot, BXI_MOTOR_CMD_ENABLE, "zero_cycle_enable") ||
            zero_cycle_move(state, m, &c, target, &reply)) goto done;
        /* Sample after disabling, so the baseline includes any release motion. */
        if (console_send_special(state, slot, BXI_MOTOR_CMD_DISABLE, "zero_cycle_disable") ||
            zero_cycle_sample(state, m, 0, 0, 0, &reply)) goto done;
        float before = reply.position;
        uint64_t off_begin = time_us();
        if (console_power_off(state)) goto done;
        while (!stop_requested && time_us() - off_begin < (uint64_t)c.off_ms * 1000) sleep_ms(10);
        if (stop_requested || console_power_on(state) || !state->motors[slot].online ||
            zero_cycle_sample(state, m, 0, 0, 0, &reply)) goto done;
        double error = (reply.position - before) * c.ratio / (2 * M_PI);
        bool ok = fabs(error) <= c.tolerance;
        fprintf(csv, "%u,%.9g,%.9g,%.9g,%.9g,%s\n", n + 1, motor_target, before, reply.position, error, ok ? "PASS" : "FAIL"); fflush(csv);
        printf("zero_cycle_test: cycle=%u target_motor_turns=%.6f before_rad=%.7f after_rad=%.7f error_motor_turns=%.7f %s\n", n + 1, motor_target, before, reply.position, error, ok ? "PASS" : "ERROR position mismatch");
        if (!ok) goto done;
        ++passed;
    }
    result = stop_requested ? -1 : 0;
done:
    if (state->motor_power_on && console_power_off(state)) result = -1;
    state->show_can_output = old_output; state->show_motor_input = old_input;
    state->bus = old_bus; state->boot_id = old_id; update_boot_ids(state);
    fprintf(csv, "# result=%s passed=%u\n", result ? "FAIL_OR_INTERRUPTED" : "PASS", passed); fclose(csv);
    printf("zero_cycle_test: %s passed=%u csv=%s\n", result ? "FAIL_OR_INTERRUPTED" : "PASS", passed, path);
    return result;
}
