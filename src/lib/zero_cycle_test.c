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
    uint64_t begin = time_us(), stable = 0;
    double speed_threshold = zero_cycle_speed_threshold(limits_for_entry(state, m), c);
    double ramp_s = fabs(target - start) / (c->speed * 2 * M_PI / c->ratio);
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
        sleep_ms(10);
    }
    fprintf(stderr, "zero_cycle_test: ERROR move/settle timeout or interrupted\n");
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

static void zero_cycle_retest(flash_state *state, const motor_map_entry *m, size_t slot,
                              const zero_cycle_config *c, unsigned int failed_cycle,
                              double target_turns, float baseline, const char *csv_path,
                              const zero_encoder_snapshot *before_snapshot)
{
    char log_path[512];
    snprintf(log_path, sizeof(log_path), "%s.retries.log", csv_path);
    FILE *log = fopen(log_path, "wx");
    if (!log) { perror(log_path); return; }
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
        c.kp < limits->kp_min || c.kp > limits->kp_max || c.kd < limits->kd_min || c.kd > limits->kd_max ||
        2 * c.tolerance * 2 * M_PI / c.ratio < (limits->p_max - limits->p_min) / 65535.) {
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
    fprintf(csv, "cycle,target_motor_turns,before_output_rad,after_output_rad,error_motor_turns,result\n"); fflush(csv);
    unsigned int seed = c.seed ? c.seed : (unsigned int)time_us();
    printf("zero_cycle_test: index=%02u ratio=%.9g range=+/-%.6g motor turns seed=%u csv=%s\n", index, c.ratio, c.max_turns, seed, path);
    printf("zero_cycle_test: readable boot/encoder diagnostics: %s\n", readable_path);
    pthread_mutex_lock(&boot_capture_mutex);
    boot_capture_bus = m->bus; boot_capture_id = m->id; boot_capture_file = readable;
    pthread_mutex_unlock(&boot_capture_mutex);
    bool old_output = state->show_can_output, old_input = state->show_motor_input;
    unsigned int old_bus = state->bus, old_id = state->boot_id;
    state->show_can_output = state->show_motor_input = false;
    int result = -1; unsigned int passed = 0; bxi_motor_reply reply;
    unsigned int order[36];
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
    for (unsigned int n = 0; !stop_requested && (!c.count || n < c.count) &&
         (!c.duration_s || time_us() - begin < (uint64_t)c.duration_s * 1000000); ++n) {
        double motor_target = zero_cycle_target(n, &seed, c.max_turns, order);
        float target = motor_target * 2 * M_PI / c.ratio;
        printf(console_text(state,
               "\n========== 第 %u 轮开始：目标电机端 %+.6f 圈（%+.3f°），输出端 %+.3f° ==========\n",
               "\n========== Cycle %u start: motor target %+.6f turns (%+.3f deg), output target %+.3f deg ==========\n"),
               n + 1, motor_target, motor_target * 360, motor_target * 360 / c.ratio);
        fflush(stdout);
        if (zero_cycle_operation(state, BXI_MOTOR_CMD_ENABLE, slot) ||
            zero_cycle_move(state, m, &c, target, &reply)) goto done;
        /* Sample after disabling, so the baseline includes any release motion. */
        if (zero_cycle_operation(state, BXI_MOTOR_CMD_DISABLE, slot) ||
            zero_cycle_sample(state, m, 0, 0, 0, &reply)) goto done;
        float before = reply.position;
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
        fprintf(csv, "%u,%.9g,%.9g,%.9g,%.9g,%s\n", n + 1, motor_target, before, reply.position, error, ok ? "PASS" : "FAIL"); fflush(csv);
        double before_turns = before * c.ratio / (2 * M_PI);
        double after_turns = reply.position * c.ratio / (2 * M_PI);
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
        if (!ok) {
            zero_encoder_compare(stdout, &before_snapshot, &after_snapshot, c.ratio);
            zero_cycle_retest(state, m, slot, &c, n + 1, motor_target, before, path, &before_snapshot);
            goto done;
        }
        ++passed;
    }
    result = stop_requested ? -1 : 0;
done:
    if (state->motor_power_on && zero_cycle_operation(state, 0, slot)) result = -1;
    pthread_mutex_lock(&boot_capture_mutex);
    boot_capture_file = NULL;
    fclose(readable);
    pthread_mutex_unlock(&boot_capture_mutex);
    state->show_can_output = old_output; state->show_motor_input = old_input;
    state->bus = old_bus; state->boot_id = old_id; update_boot_ids(state);
    fprintf(csv, "# result=%s passed=%u\n", result ? "FAIL_OR_INTERRUPTED" : "PASS", passed); fclose(csv);
    printf("zero_cycle_test: %s passed=%u csv=%s\n", result ? "FAIL_OR_INTERRUPTED" : "PASS", passed, path);
    return result;
}
