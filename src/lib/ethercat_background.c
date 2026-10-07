static const char *console_ethercat_task_name(ethercat_task task)
{
    static const char *names[] = {
        "ethercat_scan", "ethercat_enable", "ethercat_disable",
        "ethercat_position", "ethercat_zero", "ethercat_info",
        "ethercat_pnread", "ethercat_pn077", "ethercat_save"
    };
    return names[task];
}

static bool console_ethercat_background_pop(ethercat_background_args *command)
{
    bool available = false;

    pthread_mutex_lock(&ethercat_background_mutex);
    if (ethercat_command_count > 0u && !ethercat_background_stop_requested) {
        *command = ethercat_command_queue[ethercat_command_head];
        ethercat_command_head = (ethercat_command_head + 1u) % ETHERCAT_COMMAND_QUEUE_SIZE;
        ethercat_command_count--;
        available = true;
    }
    pthread_mutex_unlock(&ethercat_background_mutex);
    return available;
}

#ifdef HAVE_ETHERLAB
static void console_ethercat_session_tick(void)
{
    int slave;
    uint64_t now = console_ethercat_monotonic_us();

    if (!ethercat_session.active) return;
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (ethercat_session.disable_at_us[slave] != 0u &&
            now >= ethercat_session.disable_at_us[slave]) {
            ethercat_session.disable_at_us[slave] = 0u;
            ethercat_session.monitor[slave] = false;
            console_ethercat_write_u16(ec_slave[slave].outputs, 0u);
            printf("[slave%d] hold_ms: %s\n", slave,
                   ethercat_background_arguments.chinese ?
                   "到期，发送失能；后台主站保持通信" :
                   "expired, sending disable; background master keeps communicating");
        }
    }
}

static int console_ethercat_mailbox_cycle(void)
{
    if (stop_requested || console_ethercat_background_should_stop() ||
        (ethercat_session.active && ethercat_session.failed)) return -1;
    return console_ethercat_exchange();
}

static int console_ethercat_session_selection(const char *selection,
                                              uint8_t selected[EC_MAXSLAVE])
{
    unsigned int slave_id;
    bool all_slaves;
    int slave;

    if (console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0 ||
        (!all_slaves && slave_id > (unsigned int)ec_slavecount)) return -1;
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (!all_slaves && (unsigned int)slave != slave_id) continue;
        if (!ethercat_session.configured[slave] ||
            ec_slave[slave].outputs == NULL || ec_slave[slave].inputs == NULL ||
            ec_slave[slave].Obits != ETHERCAT_KAIXUAN_RXPDO_BITS ||
            ec_slave[slave].Ibits < ETHERCAT_KAIXUAN_TXPDO_MIN_BITS) return -1;
        selected[slave] = 1u;
    }
    return 0;
}

static bool console_ethercat_session_disabled(const uint8_t selected[EC_MAXSLAVE])
{
    int slave;

    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] &&
            (console_ethercat_read_u16(ec_slave[slave].outputs) != 0u ||
             (console_ethercat_read_u16(ec_slave[slave].inputs) & 0x006fu) != 0x0040u)) {
            return false;
        }
    }
    return true;
}

static int console_ethercat_session_disable(const uint8_t selected[EC_MAXSLAVE])
{
    int slave;

    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (!selected[slave]) continue;
        ethercat_session.disable_at_us[slave] = 0u;
        ethercat_session.monitor[slave] = false;
    }
    console_ethercat_set_control_word(selected, 0u);
    return console_ethercat_wait_for_status(selected, 0x0040u);
}

static int console_ethercat_session_enable(const ethercat_background_args *command,
                                          const uint8_t selected[EC_MAXSLAVE])
{
    uint8_t to_enable[EC_MAXSLAVE] = {0};
    int32_t targets[EC_MAXSLAVE] = {0};
    uint32_t cycle_ns = ethercat_control_period_ns;
    int32_t shift_ns = etherlab_sync_shift_ns;
    int slave;
    uint16_t control_word = 0u;
    bool needs_enable = false;

    if ((command->has_sync0_cycle_ms &&
         console_ethercat_parse_cycle_ms(command->sync0_cycle_ms, &cycle_ns) != 0) ||
        (command->has_sync0_shift &&
         console_ethercat_parse_sync0_shift(command->sync0_shift, &shift_ns) != 0) ||
        cycle_ns != ethercat_control_period_ns || shift_ns != etherlab_sync_shift_ns) {
        printf("%s\n", command->chinese ?
               "不能在运行中修改周期/shift；请退出终端后重新配置。" :
               "Cannot change cycle/shift in a running session; exit the console first.");
        return -1;
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        uint16_t status;
        uint16_t control;
        if (!selected[slave]) continue;
        status = console_ethercat_read_u16(ec_slave[slave].inputs);
        control = console_ethercat_read_u16(ec_slave[slave].outputs);
        if (console_ethercat_read_u16((uint8_t *)ec_slave[slave].inputs + 12u) != 0u ||
            (ec_slave[slave].state & 0x0fu) != EC_STATE_OPERATIONAL) return -1;
        if ((status & 0x006fu) == 0x0027u && control == 0x000fu) continue;
        if ((status & 0x006fu) != 0x0040u || control != 0u) return -1;
        to_enable[slave] = 1u;
        needs_enable = true;
        targets[slave] = console_ethercat_read_i32((uint8_t *)ec_slave[slave].inputs + 2u);
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (!selected[slave]) continue;
        ethercat_session.disable_at_us[slave] = to_enable[slave] || command->hold_ms == 0u ? 0u :
            console_ethercat_monotonic_us() + (uint64_t)command->hold_ms * 1000u;
    }
    if (!needs_enable) goto enabled;
    if (console_ethercat_selected_ready(to_enable, targets) != 0) return -1;
    control_word = 0x0006u;
    console_ethercat_set_control_word(to_enable, control_word);
    if (console_ethercat_wait_for_status(to_enable, 0x0021u) != 0) goto failed;
    control_word = 0x0007u;
    console_ethercat_set_control_word(to_enable, control_word);
    if (console_ethercat_wait_for_status(to_enable, 0x0023u) != 0) goto failed;
    control_word = 0x000fu;
    console_ethercat_set_control_word(to_enable, control_word);
    if (console_ethercat_wait_for_status(to_enable, 0x0027u) != 0) goto failed;
enabled:
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (!selected[slave]) continue;
        ethercat_session.monitor[slave] = true;
        ethercat_session.disable_at_us[slave] = command->hold_ms == 0u ? 0u :
            console_ethercat_monotonic_us() + (uint64_t)command->hold_ms * 1000u;
    }
    printf("%s\n", command->chinese ?
           "ethercat_enable: 已确认使能，复用后台主站。" :
           "ethercat_enable: operation-enabled confirmed on the existing master.");
    return 0;

failed:
    printf("ethercat_enable: CW=0x%04x WKC=%d/%d DC_error=%lldns DC_adjust=%lldns\n",
           control_word, ethercat_last_work_counter, ethercat_expected_work_counter,
           (long long)ethercat_dc_sync_phase_error_ns, (long long)ethercat_dc_sync_adjustment_ns);
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (!to_enable[slave]) continue;
        printf("[slave%d] SW=0x%04x PDO_error=0x%04x\n", slave,
               console_ethercat_read_u16(ec_slave[slave].inputs),
               console_ethercat_read_u16((uint8_t *)ec_slave[slave].inputs + 12u));
    }
    console_ethercat_session_disable(to_enable);
    return -1;
}

static int console_ethercat_session_position(const ethercat_background_args *command,
                                            const uint8_t selected[EC_MAXSLAVE])
{
    double counts = command->position_rad * ETHERCAT_KAIXUAN_COUNTS_PER_REV /
                    ETHERCAT_KAIXUAN_TWO_PI;
    int32_t target;
    int slave;

    if (!isfinite(counts) || counts < INT32_MIN || counts > INT32_MAX) return -1;
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (!selected[slave]) continue;
        if ((ec_slave[slave].state & 0x0fu) != EC_STATE_OPERATIONAL ||
            console_ethercat_read_u16(ec_slave[slave].outputs) != 0x000fu ||
            (console_ethercat_read_u16(ec_slave[slave].inputs) & 0x006fu) != 0x0027u ||
            console_ethercat_read_u16((uint8_t *)ec_slave[slave].inputs + 12u) != 0u) {
            printf("[slave%d] %s\n", slave, command->chinese ?
                   "未使能或有故障，拒绝位置目标；请先检查状态并使能。" :
                   "Not enabled or faulted; refusing target. Check status and enable first.");
            return -1;
        }
    }
    target = (int32_t)llround(counts);
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (!selected[slave]) continue;
        console_ethercat_write_i32((uint8_t *)ec_slave[slave].outputs + 2u, target);
        ethercat_session.disable_at_us[slave] = command->hold_ms == 0u ? 0u :
            console_ethercat_monotonic_us() + (uint64_t)command->hold_ms * 1000u;
    }
    if (console_ethercat_mailbox_cycle() != 0) return -1;
    printf("ethercat_position: slave=%s target_rad=%.6f target_count=%d hold_ms=%u; %s\n",
           command->selection, command->position_rad, target, command->hold_ms,
           command->chinese ? "目标已交给原 PDO 循环（非到位确认），未重开主站" :
           "target sent by existing PDO loop (not arrival confirmation); master unchanged");
    return 0;
}

static int console_ethercat_session_zero(bool chinese, const uint8_t selected[EC_MAXSLAVE])
{
    int slave;
    int failed = 0;

    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (!selected[slave]) continue;
        if (console_ethercat_write_pn101_verified(chinese, slave, 0, "1/3") != 0 ||
            console_ethercat_write_pn101_verified(chinese, slave, 1, "2/3") != 0 ||
            console_ethercat_write_pn101_verified(chinese, slave, 0, "3/3") != 0) {
            console_ethercat_write_pn101_verified(chinese, slave, 0, "recovery");
            failed++;
        }
        if (stop_requested || console_ethercat_background_should_stop() ||
            ethercat_session.failed) return -1;
    }
    if (failed) return -1;
    printf("%s\n", chinese ?
           "ethercat_zero: Pn101 0->1->0 已回读确认；按手册重启主电和 USB 电源后验证零位。" :
           "ethercat_zero: Pn101 0->1->0 verified; restart main and USB power per manual and verify zero.");
    return 0;
}

static int console_ethercat_session_command(const ethercat_background_args *command)
{
    uint8_t selected[EC_MAXSLAVE] = {0};

    if (strcmp(command->interface, ethercat_background_arguments.interface) != 0) {
        printf("%s\n", command->chinese ?
               "网卡与后台主站不匹配；现有循环不变。" :
               "Interface does not match background master; existing cycle unchanged.");
        return -1;
    }
    if (command->task == ETHERCAT_TASK_SCAN)
        return console_ethercat_scan(command->chinese, command->interface);
    if (console_ethercat_session_selection(command->selection, selected) != 0) {
        printf("%s\n", command->chinese ?
               "从站不存在或未配置到后台 PDO；现有循环不变。" :
               "Slave absent or not configured for background PDO; existing cycle unchanged.");
        return -1;
    }
    if (console_ethercat_mailbox_cycle() != 0) return -1;
    if ((command->task == ETHERCAT_TASK_ZERO || command->task == ETHERCAT_TASK_PN077 ||
         command->task == ETHERCAT_TASK_SAVE) && !console_ethercat_session_disabled(selected)) {
        printf("%s\n", command->chinese ?
               "参数写入/保存/设零要求失能；请先执行 ethercat_disable，不会自动失能。" :
               "Write/save/zero requires disabled drives; use ethercat_disable first. No implicit disable.");
        return -1;
    }
    switch (command->task) {
    case ETHERCAT_TASK_ENABLE:
        return console_ethercat_session_enable(command, selected);
    case ETHERCAT_TASK_DISABLE:
        if (console_ethercat_session_disable(selected) != 0) return -1;
        printf("%s\n", command->chinese ?
               "ethercat_disable: 失能已确认；主站和 PDO 循环保留。" :
               "ethercat_disable: disabled confirmed; master and PDO loop retained.");
        return 0;
    case ETHERCAT_TASK_POSITION:
        return console_ethercat_session_position(command, selected);
    case ETHERCAT_TASK_INFO:
        return console_ethercat_info(command->chinese, command->interface, command->selection);
    case ETHERCAT_TASK_PNREAD:
        return console_ethercat_pnread(command->chinese, command->interface,
                                       command->selection, (uint16_t)command->value);
    case ETHERCAT_TASK_PN077:
        return console_ethercat_pn077(command->chinese, command->interface,
                                      command->selection, command->value);
    case ETHERCAT_TASK_SAVE:
        return console_ethercat_save(command->chinese, command->interface, command->selection);
    case ETHERCAT_TASK_ZERO:
        return console_ethercat_session_zero(command->chinese, selected);
    default:
        return -1;
    }
}

static void console_ethercat_cancel_pending(void)
{
    pthread_mutex_lock(&ethercat_background_mutex);
    if (ethercat_command_count != 0u) {
        printf("EtherCAT: %u %s\n", ethercat_command_count,
               ethercat_background_arguments.chinese ?
               "条未执行命令已取消（主站停止/通信失败）" :
               "pending commands cancelled (master stopped/communication failed)");
    }
    ethercat_command_count = 0u;
    pthread_mutex_unlock(&ethercat_background_mutex);
}

static int console_ethercat_run_session(const uint8_t selected[EC_MAXSLAVE],
                                        unsigned int hold_ms)
{
    ethercat_background_args command;
    int slave;
    int result = 0;

    ethercat_session.active = true;
    ethercat_session.failed = false;
    memset(ethercat_session.disable_at_us, 0, sizeof(ethercat_session.disable_at_us));
    memset(ethercat_session.monitor, 0, sizeof(ethercat_session.monitor));
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (!selected[slave]) continue;
        ethercat_session.monitor[slave] = true;
        ethercat_session.disable_at_us[slave] = hold_ms == 0u ? 0u :
            console_ethercat_monotonic_us() + (uint64_t)hold_ms * 1000u;
    }
    if (ethercat_background_arguments.task == ETHERCAT_TASK_POSITION &&
        console_ethercat_session_position(&ethercat_background_arguments, selected) != 0) {
        result = -1;
    }
    while (result == 0 && !stop_requested && !console_ethercat_background_should_stop()) {
        if (console_ethercat_mailbox_cycle() != 0) {
            result = -1;
            break;
        }
        if (console_ethercat_background_pop(&command)) {
            int command_result = console_ethercat_session_command(&command);
            printf("%s: %s\n", console_ethercat_task_name(command.task),
                   command_result == 0 ? (command.chinese ? "后台执行完成" : "background complete") :
                   (command.chinese ? "后台执行失败" : "background failed"));
        }
        if (ethercat_session.failed) result = -1;
    }
    ethercat_session.active = false;
    console_ethercat_cancel_pending();
    return result;
}
#endif

static int console_ethercat_background_execute(const ethercat_background_args *command)
{
    switch (command->task) {
    case ETHERCAT_TASK_SCAN:
        return console_ethercat_scan(command->chinese, command->interface);
    case ETHERCAT_TASK_ENABLE:
    case ETHERCAT_TASK_POSITION:
        return console_ethercat_enable(command->chinese, command->interface, command->selection,
                                        command->hold_ms,
                                        command->has_sync0_shift ? command->sync0_shift : NULL,
                                        command->has_sync0_cycle_ms ? command->sync0_cycle_ms : NULL);
    case ETHERCAT_TASK_DISABLE:
        return console_ethercat_disable(command->chinese, command->interface, command->selection);
    case ETHERCAT_TASK_ZERO:
        return console_ethercat_zero(command->chinese, command->interface, command->selection);
    case ETHERCAT_TASK_INFO:
        return console_ethercat_info(command->chinese, command->interface, command->selection);
    case ETHERCAT_TASK_PNREAD:
        return console_ethercat_pnread(command->chinese, command->interface,
                                       command->selection, (uint16_t)command->value);
    case ETHERCAT_TASK_PN077:
        return console_ethercat_pn077(command->chinese, command->interface,
                                      command->selection, command->value);
    case ETHERCAT_TASK_SAVE:
        return console_ethercat_save(command->chinese, command->interface, command->selection);
    }
    return -1;
}

static void *console_ethercat_background_worker(void *argument)
{
    ethercat_background_args command;
    (void)argument;
    ethercat_background_owner = true;
    while (!stop_requested && !console_ethercat_background_should_stop()) {
        if (console_ethercat_background_pop(&command)) {
            int result;
            ethercat_background_arguments = command;
            result = console_ethercat_background_execute(&command);
            printf("%s: %s\n", console_ethercat_task_name(command.task),
                   result == 0 ? (command.chinese ? "后台执行完成" : "background complete") :
                   (command.chinese ? "后台执行失败" : "background failed"));
#ifdef HAVE_ETHERLAB
            if (result != 0) console_ethercat_cancel_pending();
#endif
            continue;
        }
        pthread_mutex_lock(&ethercat_background_mutex);
        if (!ethercat_background_stop_requested && ethercat_command_count == 0u) {
            struct timespec deadline;
            clock_gettime(CLOCK_REALTIME, &deadline);
            deadline.tv_sec++;
            pthread_cond_timedwait(&ethercat_background_condition,
                                   &ethercat_background_mutex, &deadline);
        }
        pthread_mutex_unlock(&ethercat_background_mutex);
    }
    ethercat_background_owner = false;
    return NULL;
}

static int console_ethercat_background_stop(void)
{
    bool started;

    pthread_mutex_lock(&ethercat_background_mutex);
    started = ethercat_background_started;
    ethercat_background_stop_requested = true;
    ethercat_command_count = 0u;
    pthread_cond_broadcast(&ethercat_background_condition);
    pthread_mutex_unlock(&ethercat_background_mutex);
    if (started) pthread_join(ethercat_background_thread, NULL);
    console_ethercat_output_stop();
    pthread_mutex_lock(&ethercat_background_mutex);
    ethercat_background_started = false;
    ethercat_background_stop_requested = false;
    pthread_mutex_unlock(&ethercat_background_mutex);
    return started ? 1 : 0;
}

static int console_ethercat_background_submit(ethercat_task task, bool chinese,
                                              const char *interface, const char *selection,
                                              unsigned int hold_ms, double position_rad,
                                              unsigned int value, const char *shift,
                                              const char *cycle)
{
    ethercat_background_args command = {0};
    unsigned int slave_id;
    bool all_slaves;
    int32_t parsed_shift;
    uint32_t parsed_cycle;
    double counts = position_rad * ETHERCAT_KAIXUAN_COUNTS_PER_REV / ETHERCAT_KAIXUAN_TWO_PI;

    if (console_ethercat_validate_interface(interface) != 0 ||
        (task != ETHERCAT_TASK_SCAN &&
         (selection == NULL || strlen(selection) >= sizeof(command.selection) ||
          console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0)) ||
        hold_ms > ETHERCAT_KAIXUAN_ENABLE_MAX_HOLD_MS ||
        !isfinite(counts) || counts < INT32_MIN || counts > INT32_MAX ||
        (task == ETHERCAT_TASK_PN077 && value > 1u) ||
        (task == ETHERCAT_TASK_PNREAD && value > 0xdfffu) ||
        (shift != NULL && (strlen(shift) >= sizeof(command.sync0_shift) ||
         console_ethercat_parse_sync0_shift(shift, &parsed_shift) != 0)) ||
        (cycle != NULL && (strlen(cycle) >= sizeof(command.sync0_cycle_ms) ||
         console_ethercat_parse_cycle_ms(cycle, &parsed_cycle) != 0))) {
        printf("%s\n", chinese ? "EtherCAT 参数无效，后台状态未改变。" :
               "Invalid EtherCAT arguments; background state unchanged.");
        return -1;
    }
#ifndef HAVE_ETHERLAB
    printf("%s\n", chinese ? "当前程序未编译 EtherLab，请安装开发库后重新构建。" :
           "EtherLab is unavailable; install its development library and rebuild.");
    return -1;
#endif
    command.task = task;
    command.chinese = chinese;
    command.hold_ms = hold_ms;
    command.position_rad = position_rad;
    command.value = value;
    snprintf(command.interface, sizeof(command.interface), "%s", interface);
    snprintf(command.selection, sizeof(command.selection), "%s", selection ? selection : "all");
    if (shift) {
        command.has_sync0_shift = true;
        snprintf(command.sync0_shift, sizeof(command.sync0_shift), "%s", shift);
    }
    if (cycle) {
        command.has_sync0_cycle_ms = true;
        snprintf(command.sync0_cycle_ms, sizeof(command.sync0_cycle_ms), "%s", cycle);
    }
    pthread_mutex_lock(&ethercat_background_mutex);
    if (ethercat_command_count == ETHERCAT_COMMAND_QUEUE_SIZE ||
        ethercat_background_stop_requested || stop_requested) {
        pthread_mutex_unlock(&ethercat_background_mutex);
        printf("%s\n", chinese ? "EtherCAT 队列已满或正在停止；命令未提交。" :
               "EtherCAT queue full or stopping; command not submitted.");
        return -1;
    }
    if (!ethercat_background_started) {
        if (console_ethercat_output_start() != 0) {
            pthread_mutex_unlock(&ethercat_background_mutex);
            return -1;
        }
        if (pthread_create(&ethercat_background_thread, NULL,
                            console_ethercat_background_worker, NULL) != 0) {
            pthread_mutex_unlock(&ethercat_background_mutex);
            console_ethercat_output_stop();
            return -1;
        }
        ethercat_background_started = true;
    }
    ethercat_command_queue[(ethercat_command_head + ethercat_command_count) %
                           ETHERCAT_COMMAND_QUEUE_SIZE] = command;
    ethercat_command_count++;
    printf("%s: %s\n", console_ethercat_task_name(task),
           chinese ? "已提交后台队列（不代表执行成功）" : "queued (not execution confirmation)");
    pthread_cond_broadcast(&ethercat_background_condition);
    pthread_mutex_unlock(&ethercat_background_mutex);
    return 0;
}
