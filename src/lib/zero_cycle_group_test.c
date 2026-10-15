/* Included by main.c. A group shares one test configuration and power rail. */
typedef struct {
    const motor_map_entry *motor;
    size_t slot;
    bxi_motor_reply reply;
    float before, start;
    unsigned int passed, failed;
    char text[512];
    size_t text_used;
    int reported_state;
} zero_group_motor;

static bool zero_group_type_matches(const flash_state *state, const motor_map_entry *m)
{
    const motor_runtime *runtime=&state->motors[m-state->config.entries];
    char expected[MOTOR_TYPE_LEN];
    return runtime->detected_type_valid && !normalize_detected_motor_type(m->type,expected,sizeof(expected)) &&
           !strcmp(expected,runtime->detected_type);
}

/* Only addresses present in the loaded robot map are probed. Unknown IDs
 * cannot be assigned a model or mechanical configuration safely. */
static int zero_group_discover(flash_state *state, char *selection, size_t capacity)
{
    size_t used=0,selected=0; bool valid=true;
    selection[0]=0;
    for (unsigned int bus=0;bus<CANFD_DEVICE_NUM;++bus) {
        size_t configured=0,online=0;
        for (size_t i=0;i<state->config.entry_count;++i) {
            const motor_map_entry *m=&state->config.entries[i];
            if (m->bus!=bus) continue;
            ++configured;
            if (!state->motors[i].online) continue;
            ++online;
            bool hand=strstr(m->type,"hand")!=NULL;
            bool matches=zero_group_type_matches(state,m);
            printf("  bus=%u id=%u index=%02u name=%s configured=%s detected=%s %s\n",bus,m->id,m->index,m->name,m->type,
                   state->motors[i].detected_type_valid?state->motors[i].detected_type:"UNKNOWN",
                   hand?"SKIP_HAND":matches?"MODEL_OK":"MODEL_CHECK_FAILED");
            if (hand) continue;
            if (!matches) { valid=false; continue; }
            int n=snprintf(selection+used,capacity-used,"%s%u:%u",selected?",":"",bus,m->id);
            if (selected>=32 || n<0 || (size_t)n>=capacity-used) { valid=false; continue; }
            used+=(size_t)n; ++selected;
        }
        if (configured) printf("bus=%u：配置 %zu 台，在线 %zu 台；其余离线。\n",bus,configured,online);
    }
    if (!valid || !selected) {
        fputs("group: discovery failed: no eligible motors, unknown/mismatched model, or >32 motors; no zero/enable commands sent\n",stderr);
        return -1;
    }
    return 0;
}

static int console_zero_cycle_group_test(flash_state *state,int argc,char **argv);

static int zero_group_auto_test(flash_state *state,const char *config_path)
{
    zero_cycle_config config;
    if (zero_cycle_load(config_path,&config)) return -1;
    if (state->motor_power_on || console_any_enabled(state)) { fputs("group: start with power OFF\n",stderr); return -1; }
    if (zero_cycle_input_begin()) return -1;
    for (size_t i=0;i<state->config.entry_count;++i) state->motors[i].detected_type_valid=false;
    puts("自动发现：读取机器人 bus/id/型号配置，上电后扫描全部已配置地址，核对本次启动输出 HW/PRO。Ctrl+X 停止。");
    char selection[512]; int rc=-1;
    if (!zero_cycle_operation(state,1,0) && !zero_cycle_cancel_requested()) {
        int out=-1,err=-1;
        if (!console_silence_process_output(&out,&err)) {
            console_probe_motors(state,state->config.scan_timeout_ms);
            console_restore_process_output(out,err);
            if (!zero_cycle_cancel_requested()) rc=zero_group_discover(state,selection,sizeof(selection));
        }
    }
    if (state->motor_power_on && zero_cycle_operation(state,0,0)) rc=-1;
    if (zero_cycle_cancel_requested()) rc=-1;
    zero_cycle_input_end();
    if (rc) return -1;
    printf("自动选择：%s；测试参数：%s。重新上电确认型号后开始测试。\n",selection,config_path);
    char *arguments[]={"zero_cycle_group_test",selection,(char *)config_path};
    return console_zero_cycle_group_test(state,3,arguments);
}

static int zero_group_select(flash_state *state, const char *selection,
                             zero_group_motor *motors, size_t *count)
{
    char copy[512];
    if (!*selection || strlen(selection) >= sizeof(copy)) return -1;
    strcpy(copy, selection); *count = 0;
    char *p = copy;
    while (p) {
        char *next = strchr(p, ','); if (next) *next++ = 0;
        char *colon = strchr(p, ':'); unsigned int bus, id;
        if (!colon || *count >= 32) return -1;
        *colon++ = 0;
        if (parse_uint_arg(p, &bus) || parse_uint_arg(colon, &id)) return -1;
        const motor_map_entry *m = find_motor_by_bus_id(state, bus, id);
        if (!m || strstr(m->type, "hand")) return -1;
        for (size_t i = 0; i < *count; ++i) if (motors[i].motor == m) return -1;
        motors[*count] = (zero_group_motor){.motor=m, .slot=(size_t)(m-state->config.entries), .reported_state=-1};
        ++*count; p = next;
    }
    return *count ? 0 : -1;
}

static int zero_group_special(flash_state *state, zero_group_motor *motors, size_t count, uint8_t command)
{
    int result = 0;
    for (size_t i = 0; i < count; ++i) {
        console_use_motor(state, motors[i].motor);
        if (send_debug_special(state, command)) result = -1;
        if (command == BXI_MOTOR_CMD_DISABLE) state->motors[motors[i].slot].enabled = false;
    }
    return result;
}

/* Send to every member before collecting replies; never wait for one motor
 * while starving the others' CAN watchdogs. Match both bus and wire ID. */
static int zero_group_exchange(flash_state *state, zero_group_motor *motors, size_t count,
                                const float *positions, const float *velocities, double kp, double kd)
{
    bool received[32] = {0}; size_t ready = 0;
    uint64_t begin = time_us(), next_send = begin;
    while (!zero_cycle_cancel_requested() && time_us()-begin < 100000) {
        uint64_t now = time_us();
        if (now >= next_send) {
            for (size_t i = 0; i < count; ++i) {
                console_use_motor(state, motors[i].motor);
                console_expect_reply(state, motors[i].motor->bus);
                if (send_debug_mit(state, positions ? positions[i] : 0,
                                   velocities ? velocities[i] : 0, kp, kd, 0)) return -1;
            }
            next_send = time_us()+10000;
        }
        rx_can_frame frame;
        if (frame_ring_pop(&state->frames, &frame, 1)) {
            for (size_t i = 0; i < count; ++i) {
                zero_group_motor *g = &motors[i];
                if (frame.bus != g->motor->bus) continue;
                if (boot_output_id_matches(frame.can_id, g->motor->id)) {
                    for (unsigned int k = 0; k < frame.len; ++k) {
                        char ch = frame.data[k];
                        if (ch == '\n' || ch == '\r') {
                            g->text[g->text_used] = 0;
                            int parsed = zero_cycle_parse_state(g->text);
                            if (parsed >= 0) g->reported_state = parsed;
                            g->text_used = 0;
                        } else if (g->text_used < sizeof(g->text)-1) g->text[g->text_used++] = ch;
                    }
                }
                if (frame.time_us < begin || !mit_reply_frame_matches(&frame, g->motor->id)) continue;
                const bxi_motor_limits *limits = limits_for_entry(state, g->motor);
                int rc = state->motors[g->slot].mit_aux_enabled ?
                    bxi_motor_unpack_reply(frame.data, frame.len, limits, &g->reply) :
                    bxi_motor_unpack_reply_legacy_ntc(frame.data, frame.len, limits, &g->reply);
                if (!rc && isfinite(g->reply.position) && isfinite(g->reply.velocity) && !received[i]) {
                    received[i] = true; ++ready;
                }
            }
        }
        if (ready == count && time_us()-begin >= 10000) return 0;
    }
    for (size_t i = 0; i < count; ++i) if (!received[i])
        fprintf(stderr, "group: fresh MIT feedback timeout bus=%u id=%u\n", motors[i].motor->bus, motors[i].motor->id);
    return -1;
}

static int zero_group_enable(flash_state *state, zero_group_motor *motors, size_t count)
{
    frame_ring_clear(&state->frames);
    for (size_t i = 0; i < count; ++i) { motors[i].reported_state=-1; motors[i].text_used=0; }
    if (zero_group_special(state, motors, count, BXI_MOTOR_CMD_ENABLE)) return -1;
    uint64_t begin=time_us(), next_query=0;
    while (!zero_cycle_cancel_requested() && time_us()-begin < 2000000) {
        if (time_us() >= next_query) {
            for (size_t i=0; i<count; ++i) {
                console_use_motor(state, motors[i].motor);
                const uint8_t query=0xc8;
                if (send_boot_data(state, &query, 1)) return -1;
            }
            next_query=time_us()+100000;
        }
        if (zero_group_exchange(state, motors, count, NULL, NULL, 0, 0)) return -1;
        bool all=true;
        for (size_t i=0; i<count; ++i) if (motors[i].reported_state != 2) all=false;
        if (all) {
            for (size_t i=0; i<count; ++i) state->motors[motors[i].slot].enabled=true;
            return 0;
        }
    }
    fputs("group: enable not confirmed for all motors\n", stderr); return -1;
}

static int zero_group_move(flash_state *state, zero_group_motor *motors, size_t count,
                            const zero_cycle_config *c, float target, bool slow)
{
    if (zero_group_enable(state, motors, count)) return -1;
    double scale=2*M_PI/c->ratio, duration=0;
    double speed=slow ? fmin(c->speed, c->scan_return_speed_deg_s*c->ratio/360) : c->speed;
    for (size_t i=0; i<count; ++i) {
        motors[i].start=motors[i].reply.position;
        duration=fmax(duration, zero_cycle_move_duration(target-motors[i].start, speed*scale, c->acceleration*scale));
    }
    uint64_t begin=time_us(), stable=0;
    while (!zero_cycle_cancel_requested() && time_us()-begin < (uint64_t)c->move_timeout_ms*1000) {
        double elapsed=(time_us()-begin)/1e6; float positions[32], velocities[32];
        for (size_t i=0; i<count; ++i)
            zero_cycle_trajectory(motors[i].start, target, elapsed, duration, &positions[i], &velocities[i]);
        if (zero_group_exchange(state, motors, count, positions, velocities, c->kp, c->kd)) return -1;
        bool all=elapsed>=duration;
        for (size_t i=0; i<count; ++i)
            if (fabs(motors[i].reply.position-target)/scale > c->arrival_tolerance ||
                fabs(motors[i].reply.velocity)/scale > zero_cycle_speed_threshold(limits_for_entry(state, motors[i].motor), c)) all=false;
        if (all) {
            if (!stable) stable=time_us();
            if (time_us()-stable >= (uint64_t)c->settle_ms*1000) return zero_group_special(state, motors, count, BXI_MOTOR_CMD_DISABLE);
        } else stable=0;
    }
    fputs("group: target not reached; stop group without power-cycle comparison\n", stderr); return -1;
}

static int zero_group_power_cycle(flash_state *state, size_t slot, const zero_cycle_config *c)
{
    if (zero_cycle_operation(state, 0, slot)) return -1;
    uint64_t begin=time_us();
    while (!zero_cycle_cancel_requested() && time_us()-begin < (uint64_t)(c->off_ms-300)*1000) console_quiet_sleep_ms(10);
    if (zero_cycle_cancel_requested()) return -1;
    return zero_cycle_operation(state, 1, slot);
}

static bool zero_group_compare(zero_group_motor *g, const zero_cycle_config *c, bool *frozen)
{
    bool ok=fabs(g->reply.position-g->before)*c->ratio/(2*M_PI)<=c->tolerance;
    if (ok) ++g->passed; else { ++g->failed; *frozen=true; }
    return ok;
}

static int console_zero_cycle_group_test(flash_state *state, int argc, char **argv)
{
    if (argc==1 || (argc==2 && (!strcmp(argv[1],"auto") || !strchr(argv[1],':'))) ||
        (argc==3 && !strcmp(argv[1],"auto")))
        return zero_group_auto_test(state,argc==3?argv[2]:argc==2 && strcmp(argv[1],"auto")?argv[1]:"config/zero_cycle_test.yaml");
    zero_group_motor motors[32]; size_t count;
    zero_cycle_config c;
    if ((argc != 2 && argc != 3) || zero_group_select(state, argv[1], motors, &count)) {
        puts("usage: zero_cycle_group_test [auto|bus:id,bus:id,...] [config.yaml]"); return -1;
    }
    if (zero_cycle_load(argc==3 ? argv[2] : "config/zero_cycle_test.yaml", &c)) return -1;
    if (state->motor_power_on || console_any_enabled(state)) { fputs("group: start with power OFF\n", stderr); return -1; }
    double scale=2*M_PI/c.ratio, max_rad=zero_cycle_physical_limit(&c)*scale;
    for (size_t i=0; i<count; ++i) {
        const bxi_motor_limits *l=limits_for_entry(state, motors[i].motor);
        if (-max_rad<l->p_min || max_rad>l->p_max || c.speed*scale>fmin(l->v_max,-l->v_min) ||
            c.kp<l->kp_min || c.kp>l->kp_max || c.kd<l->kd_min || c.kd>l->kd_max ||
            2*fmin(c.tolerance,c.arrival_tolerance)*scale < (l->p_max-l->p_min)/65535.) {
            fprintf(stderr,"group: invalid MIT limits bus=%u id=%u\n",motors[i].motor->bus,motors[i].motor->id); return -1;
        }
    }
    char path[256]; snprintf(path,sizeof(path),"/tmp/zero-group-%ld-%llu.csv",(long)getpid(),(unsigned long long)time_us());
    FILE *csv=fopen(path,"wx"); if (!csv) return -1;
    bool old_output=state->show_can_output, old_input=state->show_motor_input;
    unsigned int old_bus=state->bus,old_id=state->boot_id;
    int result=-1; bool frozen=false; unsigned int passed=0,failed=0,rounds=0;
    unsigned int seed=c.seed ? c.seed : (unsigned int)time_us(),order[36],scan_index=0;
    uint64_t begin=time_us();
    fprintf(csv,"cycle,bus,id,index,phase,target_output_deg,baseline_output_rad,before_output_rad,after_output_rad,delta_baseline_motor_turns,delta_cycle_motor_turns,result\n");
    fprintf(csv,"# gear_ratio=%.12g seed=%u mode=%s tolerance_motor_turns=%.9g\n",c.ratio,seed,c.test_mode?"degree_scan":"random",c.tolerance);
    fprintf(csv,"# scan_repeats_per_angle=%u scan_rounds=%u\n",c.scan_repeats,c.scan_rounds);
    if (zero_cycle_input_begin()) goto done;
    state->show_can_output=state->show_motor_input=false;
    pthread_mutex_lock(&boot_capture_mutex);
    for (size_t i=0; i<count; ++i) {
        char log[384]; snprintf(log,sizeof(log),"%s.bus-%u.id-%u.boot.log",path,motors[i].motor->bus,motors[i].motor->id);
        group_boot_capture[i].bus=motors[i].motor->bus; group_boot_capture[i].id=motors[i].motor->id;
        group_boot_capture[i].file=fopen(log,"wx");
        if (!group_boot_capture[i].file) { pthread_mutex_unlock(&boot_capture_mutex); goto done; }
        ++group_boot_capture_count;
    }
    pthread_mutex_unlock(&boot_capture_mutex);
    printf("多电机测试：%zu 台，共用 gear_ratio=%.9g；Ctrl+X 停止。CSV: %s\n",count,c.ratio,path);
    for (size_t i=0;i<count;++i)
        printf("  index=%02u bus=%u id=%u\n",motors[i].motor->index,motors[i].motor->bus,motors[i].motor->id);
    for (size_t i=0;i<count;++i) state->motors[motors[i].slot].detected_type_valid=false;
    if (zero_cycle_operation(state,1,motors[0].slot)) goto done;
    /* Validate the whole group before changing any member's zero. */
    for (size_t i=0;i<count;++i) {
        const motor_map_entry *m=motors[i].motor;
        bool matches=zero_group_type_matches(state,m);
        fprintf(csv,"# model bus=%u id=%u expected=%s detected=%s match=%d\n",m->bus,m->id,m->type,
                state->motors[motors[i].slot].detected_type_valid?state->motors[motors[i].slot].detected_type:"UNKNOWN",matches);
        if (!matches) {
            fprintf(stderr,"group: fresh boot model check failed bus=%u id=%u configured=%s detected=%s; no zero/enable sent\n",m->bus,m->id,m->type,
                    state->motors[motors[i].slot].detected_type_valid?state->motors[motors[i].slot].detected_type:"UNKNOWN"); goto done;
        }
    }
    for (size_t i=0; i<count; ++i) {
        if (!state->motors[motors[i].slot].online) {
            fprintf(stderr,"group: selected motor offline bus=%u id=%u\n",motors[i].motor->bus,motors[i].motor->id); goto done;
        }
        if (zero_cycle_operation(state,BXI_MOTOR_CMD_DISABLE,motors[i].slot) ||
            zero_cycle_operation(state,BXI_MOTOR_CMD_ZERO,motors[i].slot)) goto done;
    }
    console_quiet_sleep_ms(500);
    if (zero_group_exchange(state,motors,count,NULL,NULL,0,0)) goto done;
    for (size_t i=0;i<count;++i) if (fabs(motors[i].reply.position)/scale>c.tolerance) {
        fprintf(stderr,"group: initial zero failed bus=%u id=%u\n",motors[i].motor->bus,motors[i].motor->id); goto done;
    }
    double target_turns=0;
    for (unsigned int n=0; !zero_cycle_cancel_requested() && (!c.count || n<c.count) &&
         (!c.test_mode || (uint64_t)n<zero_cycle_scan_total(&c)) &&
         (!c.duration_s || time_us()-begin<(uint64_t)c.duration_s*1000000); ++n) {
        bool new_point=!c.test_mode || scan_index%c.scan_repeats==0;
        if (!frozen) {
            if (c.test_mode && zero_cycle_scan_boundary(scan_index,&c)) {
                puts("全组单侧扫描结束，缓慢回零并上下电验证，然后继续扫描。");
                if (zero_group_move(state,motors,count,&c,0,true) || zero_group_exchange(state,motors,count,NULL,NULL,0,0)) goto done;
                float zero_before[32];
                for (size_t i=0;i<count;++i) zero_before[i]=motors[i].reply.position;
                if (zero_group_power_cycle(state,motors[0].slot,&c) ||
                    zero_group_exchange(state,motors,count,NULL,NULL,0,0)) goto done;
                for (size_t i=0;i<count;++i) {
                    bool ok=zero_cycle_zero_recheck_ok(zero_before[i]/scale,motors[i].reply.position/scale,c.tolerance);
                    if (!ok) frozen=true;
                    fprintf(csv,"# midpoint_zero bus=%u id=%u before_rad=%.9g after_rad=%.9g result=%s\n",
                            motors[i].motor->bus,motors[i].motor->id,zero_before[i],motors[i].reply.position,ok?"PASS":"FAIL");
                }
                if (frozen) {
                    puts("回零重启验证失败：全组锁定当前位置，仅上下电。");
                    for (size_t i=0;i<count;++i) motors[i].before=zero_before[i];
                    target_turns=0;
                }
            }
            if (!frozen) {
                if (c.test_mode) {
                    printf("扫描第 %u 轮，点内上下电 %u/%u\n",scan_index/(2*zero_cycle_scan_steps(&c)*c.scan_repeats)+1,scan_index%c.scan_repeats+1,c.scan_repeats);
                    zero_cycle_scan_cycle_target(scan_index++,&c,&target_turns);
                }
                else target_turns=zero_cycle_target(n,&seed,c.max_turns,order);
                printf("\n第 %u 轮，全组目标输出端 %+.3f°\n",n+1,target_turns*360/c.ratio);
                if ((new_point && zero_group_move(state,motors,count,&c,target_turns*scale,false)) ||
                    zero_group_exchange(state,motors,count,NULL,NULL,0,0)) goto done;
                if (new_point) for (size_t i=0;i<count;++i) motors[i].before=motors[i].reply.position;
            }
        }
        float previous[32];
        if (zero_group_exchange(state,motors,count,NULL,NULL,0,0)) goto done;
        for (size_t i=0;i<count;++i) {
            previous[i]=motors[i].reply.position;
            if (!frozen && new_point) motors[i].before=previous[i];
        }
        if (zero_group_power_cycle(state,motors[0].slot,&c) || zero_group_exchange(state,motors,count,NULL,NULL,0,0)) goto done;
        bool round_ok=true, was_frozen=frozen;
        for (size_t i=0;i<count;++i) {
            zero_group_motor *g=&motors[i];
            double delta=(g->reply.position-g->before)/scale,cycle_delta=(g->reply.position-previous[i])/scale;
            bool ok=zero_group_compare(g,&c,&frozen);
            if (!ok) round_ok=false;
            printf("第 %u 轮 bus=%u id=%u：基准 %+.3f°，上电 %+.3f°，相对基准差 %+.3f°，本次重启差 %+.3f°（电机端）；%s；累计成功 %u 失败 %u\n",
                   n+1,g->motor->bus,g->motor->id,g->before/scale*360,g->reply.position/scale*360,delta*360,cycle_delta*360,ok?"PASS":"FAIL",g->passed,g->failed);
            fprintf(csv,"%u,%u,%u,%u,%s,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%s\n",n+1,g->motor->bus,g->motor->id,g->motor->index,
                    was_frozen?"power_only":new_point?"motion":"angle_repeat",target_turns*360/c.ratio,g->before,previous[i],g->reply.position,delta,cycle_delta,ok?"PASS":"FAIL");
        }
        ++rounds; if (round_ok) ++passed; else ++failed;
        if (frozen && !was_frozen) {
            puts("检测到位置超差：全组停止后续运动，保留本轮断电前基准，仅继续上下电。");
            for (size_t i=0;i<count;++i)
                zero_encoder_query(state,motors[i].motor,group_boot_capture[i].file,"GROUP_FIRST_POSITION_FAILURE");
        }
        printf("全组累计：%u 轮成功，%u 轮失败。%s\n",passed,failed,frozen?"已锁定：仅上下电，不运动、不置零、不补偿。":"");
        fflush(csv); fflush(stdout);
    }
    result=zero_cycle_cancel_requested() || failed || frozen ? -1 : 0;
done:
    if (state->motor_power_on) {
        zero_group_special(state,motors,count,BXI_MOTOR_CMD_DISABLE);
        if (zero_cycle_operation(state,0,motors[0].slot)) result=-1;
    }
    zero_cycle_input_end();
    pthread_mutex_lock(&boot_capture_mutex);
    for (size_t i=0;i<group_boot_capture_count;++i) fclose(group_boot_capture[i].file);
    group_boot_capture_count=0;
    pthread_mutex_unlock(&boot_capture_mutex);
    state->show_can_output=old_output; state->show_motor_input=old_input;
    state->bus=old_bus; state->boot_id=old_id; update_boot_ids(state);
    fprintf(csv,"# completed_rounds=%u passed=%u failed=%u frozen=%d status=%s\n",rounds,passed,failed,frozen,result?"FAILED_OR_STOPPED":"PASS");
    for (size_t i=0;i<count;++i) {
        fprintf(csv,"# motor bus=%u id=%u passed=%u failed=%u\n",motors[i].motor->bus,motors[i].motor->id,motors[i].passed,motors[i].failed);
        printf("  bus=%u id=%u：累计成功 %u，失败 %u\n",motors[i].motor->bus,motors[i].motor->id,motors[i].passed,motors[i].failed);
    }
    fclose(csv);
    printf("多电机测试结束：成功 %u 轮，失败 %u 轮；%s；日志 %s\n",passed,failed,result?"FAILED_OR_STOPPED":"PASS",path);
    return result;
}
