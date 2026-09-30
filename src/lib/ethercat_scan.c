#include <net/if.h>

#ifdef HAVE_SOEM
#include <soem/ethercat.h>
#endif

static int console_ethercat_validate_interface(const char *interface)
{
    size_t length;

    if (interface == NULL || interface[0] == '\0') {
        return -1;
    }
    length = strlen(interface);
    return length < IFNAMSIZ ? 0 : -1;
}

static int console_ethercat_parse_slave_selection(const char *selection,
                                                   unsigned int *slave_id,
                                                   bool *all_slaves)
{
    char *end;
    unsigned long parsed;

    if (selection == NULL || slave_id == NULL || all_slaves == NULL) {
        return -1;
    }
    if (strcmp(selection, "all") == 0) {
        *slave_id = 0u;
        *all_slaves = true;
        return 0;
    }
    errno = 0;
    parsed = strtoul(selection, &end, 10);
    if (errno != 0 || end == selection || *end != '\0' || parsed == 0ul ||
        parsed > UINT_MAX) {
        return -1;
    }
    *slave_id = (unsigned int)parsed;
    *all_slaves = false;
    return 0;
}

enum {
    ETHERCAT_KAIXUAN_VENDOR_ID = 0x00010203u,
    ETHERCAT_KAIXUAN_PRODUCT_CODE = 0x00000402u,
    ETHERCAT_KAIXUAN_RXPDO_BITS = 104u,
    ETHERCAT_KAIXUAN_TXPDO_MIN_BITS = 112u,
    ETHERCAT_KAIXUAN_CONTROL_PERIOD_MS = 2u,
    ETHERCAT_KAIXUAN_ENABLE_MAX_HOLD_MS = 60000u,
    ETHERCAT_KAIXUAN_PROCESS_IMAGE_SIZE = 8192u,
};

#ifdef HAVE_SOEM
static const char *console_ethercat_state_name(uint16_t state)
{
    switch (state & 0x0fU) {
    case EC_STATE_INIT:
        return "INIT";
    case EC_STATE_PRE_OP:
        return "PRE-OP";
    case EC_STATE_BOOT:
        return "BOOT";
    case EC_STATE_SAFE_OP:
        return "SAFE-OP";
    case EC_STATE_OPERATIONAL:
        return "OP";
    default:
        return "UNKNOWN";
    }
}

static uint16_t console_ethercat_read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8u);
}

static int32_t console_ethercat_read_i32(const uint8_t *data)
{
    uint32_t value = (uint32_t)data[0] |
                     ((uint32_t)data[1] << 8u) |
                     ((uint32_t)data[2] << 16u) |
                     ((uint32_t)data[3] << 24u);

    return (int32_t)value;
}

static void console_ethercat_write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xffu);
    data[1] = (uint8_t)(value >> 8u);
}

static void console_ethercat_write_i32(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)(raw & 0xffu);
    data[1] = (uint8_t)((raw >> 8u) & 0xffu);
    data[2] = (uint8_t)((raw >> 16u) & 0xffu);
    data[3] = (uint8_t)((raw >> 24u) & 0xffu);
}

static int console_ethercat_exchange(void)
{
    int work_counter;

    ec_send_processdata();
    work_counter = ec_receive_processdata(EC_TIMEOUTRET);
    return work_counter > 0 ? 0 : -1;
}

static int console_ethercat_selected_ready(const uint8_t selected[EC_MAXSLAVE],
                                           const int32_t target_positions[EC_MAXSLAVE])
{
    int slave;

    for (slave = 1; slave <= ec_slavecount; slave++) {
        uint8_t *outputs;

        if (selected[slave] == 0u) {
            continue;
        }
        outputs = (uint8_t *)ec_slave[slave].outputs;
        if (outputs == NULL || ec_slave[slave].Obits != ETHERCAT_KAIXUAN_RXPDO_BITS ||
            ec_slave[slave].Ibits < ETHERCAT_KAIXUAN_TXPDO_MIN_BITS ||
            ec_slave[slave].eep_man != ETHERCAT_KAIXUAN_VENDOR_ID ||
            ec_slave[slave].eep_id != ETHERCAT_KAIXUAN_PRODUCT_CODE) {
            return -1;
        }
        console_ethercat_write_u16(outputs, 0u);
        console_ethercat_write_i32(outputs + 2u, target_positions[slave]);
        console_ethercat_write_i32(outputs + 6u, 0);
        console_ethercat_write_u16(outputs + 10u, 0u);
        outputs[12] = 8u;
    }
    return 0;
}

static void console_ethercat_set_control_word(const uint8_t selected[EC_MAXSLAVE],
                                               uint16_t control_word)
{
    int slave;

    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] != 0u) {
            if (ec_slave[slave].outputs != NULL) {
                console_ethercat_write_u16((uint8_t *)ec_slave[slave].outputs, control_word);
            }
        }
    }
}

static bool console_ethercat_selected_status_matches(const uint8_t selected[EC_MAXSLAVE],
                                                      uint16_t expected_state)
{
    int slave;

    for (slave = 1; slave <= ec_slavecount; slave++) {
        uint16_t status_word;

        if (selected[slave] == 0u) {
            continue;
        }
        status_word = console_ethercat_read_u16((const uint8_t *)ec_slave[slave].inputs);
        if ((status_word & 0x006fu) != expected_state) {
            return false;
        }
    }
    return true;
}

static int console_ethercat_wait_for_status(const uint8_t selected[EC_MAXSLAVE],
                                            uint16_t expected_state)
{
    unsigned int attempt;

    for (attempt = 0u; attempt < 100u && !stop_requested; attempt++) {
        if (console_ethercat_exchange() != 0) {
            return -1;
        }
        if (console_ethercat_selected_status_matches(selected, expected_state)) {
            return 0;
        }
        sleep_ms(ETHERCAT_KAIXUAN_CONTROL_PERIOD_MS);
    }
    return -1;
}

static void console_ethercat_disable_selected(const uint8_t selected[EC_MAXSLAVE])
{
    int slave;
    unsigned int cycle;

    console_ethercat_set_control_word(selected, 0u);
    for (cycle = 0u; cycle < 10u; cycle++) {
        if (console_ethercat_exchange() != 0) {
            break;
        }
        sleep_ms(ETHERCAT_KAIXUAN_CONTROL_PERIOD_MS);
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] != 0u) {
            ec_slave[slave].state = EC_STATE_SAFE_OP;
            ec_writestate((uint16)slave);
        }
    }
}
#endif

static int console_ethercat_scan(bool chinese, const char *interface)
{
    if (console_ethercat_validate_interface(interface) != 0) {
        printf("%s: %s\n", chinese ? "用法" : "usage",
               chinese ? "ethercat_scan <network_interface>" :
               "ethercat_scan <network_interface>");
        return -1;
    }

#ifndef HAVE_SOEM
    printf("%s\n", chinese ?
           "当前程序未编译 SOEM，无法扫描 EtherCAT。从 SOEM 源码编译后，执行 make "
           "FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" 重新构建。" :
           "EtherCAT scan is unavailable because this build has no SOEM support. Build again "
           "with make FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" after building SOEM.");
    return -1;
#else
    int slave;
    int slave_count;

    printf("%s: interface=%s\n", chinese ? "ethercat_scan: 开始扫描" :
           "ethercat_scan: scan start", interface);
    if (ec_init((char *)interface) == 0) {
        printf("%s: %s\n", chinese ? "ethercat_scan: 打开网卡失败" :
               "ethercat_scan: failed to open interface", interface);
        return -1;
    }

    slave_count = ec_config_init(FALSE);
    if (slave_count <= 0) {
        printf("%s: %s\n", chinese ? "ethercat_scan: 未发现 EtherCAT 从站" :
               "ethercat_scan: no EtherCAT slaves found", interface);
        ec_close();
        return -1;
    }

    ec_readstate();
    printf("%s: total=%d\n", chinese ? "ethercat_scan: 完成" :
           "ethercat_scan: complete", slave_count);
    for (slave = 1; slave <= slave_count; slave++) {
        printf("  slave=%d name=%s state=0x%02x(%s) addr=0x%04x "
               "vendor=0x%08x product=0x%08x revision=0x%08x dc=%s\n",
               slave,
               ec_slave[slave].name,
               (unsigned int)ec_slave[slave].state,
               console_ethercat_state_name(ec_slave[slave].state),
               (unsigned int)ec_slave[slave].configadr,
               (unsigned int)ec_slave[slave].eep_man,
               (unsigned int)ec_slave[slave].eep_id,
               (unsigned int)ec_slave[slave].eep_rev,
               ec_slave[slave].hasdc ? "yes" : "no");
    }
    printf("%s\n", chinese ?
           "扫描未配置 PDO、未请求 OP 状态，也未向电机发送使能或运动指令。" :
           "The scan did not configure PDOs, request OP state, or send drive-enable/motion commands.");
    ec_close();
    return 0;
#endif
}

static int console_ethercat_enable(bool chinese,
                                   const char *interface,
                                   const char *selection,
                                   unsigned int hold_ms)
{
    unsigned int slave_id;
    bool all_slaves;

    if (console_ethercat_validate_interface(interface) != 0 ||
        console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0 ||
        hold_ms > ETHERCAT_KAIXUAN_ENABLE_MAX_HOLD_MS) {
        printf("%s: ethercat_enable <network_interface> <slave_id|all> [hold_ms:1..%u]\n",
               chinese ? "用法" : "usage", ETHERCAT_KAIXUAN_ENABLE_MAX_HOLD_MS);
        return -1;
    }

#ifndef HAVE_SOEM
    printf("%s\n", chinese ?
           "当前程序未编译 SOEM，无法使能 EtherCAT 电机。从 SOEM 源码编译后，执行 make "
           "FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" 重新构建。" :
           "EtherCAT enable is unavailable because this build has no SOEM support. Build again "
           "with make FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" after building SOEM.");
    return -1;
#else
    uint8_t process_image[ETHERCAT_KAIXUAN_PROCESS_IMAGE_SIZE];
    uint8_t selected[EC_MAXSLAVE] = {0};
    int32_t target_positions[EC_MAXSLAVE] = {0};
    uint64_t deadline;
    int slave;
    int result = -1;
    bool mapped = false;

    if (hold_ms == 0u) {
        printf("%s: interface=%s slave=%s hold=until-Ctrl-C\n",
               chinese ? "ethercat_enable: 开始" : "ethercat_enable: start",
               interface, selection);
    } else {
        printf("%s: interface=%s slave=%s hold_ms=%u\n",
               chinese ? "ethercat_enable: 开始" : "ethercat_enable: start",
               interface, selection, hold_ms);
    }
    if (ec_init((char *)interface) == 0) {
        printf("%s: %s\n", chinese ? "ethercat_enable: 打开网卡失败" :
               "ethercat_enable: failed to open interface", interface);
        return -1;
    }
    if (ec_config_init(FALSE) <= 0) {
        printf("%s\n", chinese ? "ethercat_enable: 未发现 EtherCAT 从站" :
               "ethercat_enable: no EtherCAT slaves found");
        goto close_socket;
    }
    if (!all_slaves && slave_id > (unsigned int)ec_slavecount) {
        printf("%s: %u (1..%d)\n", chinese ? "ethercat_enable: 从站序号不存在" :
               "ethercat_enable: slave id is out of range", slave_id, ec_slavecount);
        goto close_socket;
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (all_slaves || (unsigned int)slave == slave_id) {
            selected[slave] = 1u;
        }
    }

    ec_config_map(process_image);
    mapped = true;
    ec_configdc();
    if ((ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) & 0x0fu) != EC_STATE_SAFE_OP) {
        printf("%s\n", chinese ? "ethercat_enable: 从站未进入 SAFE-OP" :
               "ethercat_enable: slaves did not reach SAFE-OP");
        goto cleanup;
    }
    if (console_ethercat_exchange() != 0 || console_ethercat_exchange() != 0 ||
        console_ethercat_exchange() != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 初始 PDO 通信失败" :
               "ethercat_enable: initial PDO exchange failed");
        goto cleanup;
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] != 0u) {
            if (ec_slave[slave].inputs == NULL ||
                ec_slave[slave].Obits != ETHERCAT_KAIXUAN_RXPDO_BITS ||
                ec_slave[slave].Ibits < ETHERCAT_KAIXUAN_TXPDO_MIN_BITS ||
                ec_slave[slave].eep_man != ETHERCAT_KAIXUAN_VENDOR_ID ||
                ec_slave[slave].eep_id != ETHERCAT_KAIXUAN_PRODUCT_CODE) {
                printf("%s: %d\n", chinese ? "ethercat_enable: 从站 PDO 或型号不匹配" :
                       "ethercat_enable: slave PDO or model does not match", slave);
                goto cleanup;
            }
            target_positions[slave] = console_ethercat_read_i32(
                (const uint8_t *)ec_slave[slave].inputs + 2u);
        }
    }
    if (console_ethercat_selected_ready(selected, target_positions) != 0) {
        printf("%s\n", chinese ?
               "ethercat_enable: PDO 映射不是当前开璇驱动器要求的 13B 输出/14B 输入，已拒绝使能" :
               "ethercat_enable: PDO mapping is not the required Kaixuan 13B output/14B input layout; enable refused");
        goto cleanup;
    }
    if (console_ethercat_exchange() != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 写入初始位置失败" :
               "ethercat_enable: failed to write initial positions");
        goto cleanup;
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] != 0u) {
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate((uint16)slave);
        }
    }
    for (slave = 0; slave < 100 && !stop_requested; slave++) {
        if (console_ethercat_exchange() != 0) {
            break;
        }
        if (ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTRET) == EC_STATE_OPERATIONAL) {
            break;
        }
        sleep_ms(ETHERCAT_KAIXUAN_CONTROL_PERIOD_MS);
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] != 0u &&
            (ec_statecheck((uint16)slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET) & 0x0fu) !=
            EC_STATE_OPERATIONAL) {
            printf("%s: %d\n", chinese ? "ethercat_enable: 从站未进入 OP" :
                   "ethercat_enable: slave did not reach OP", slave);
            goto cleanup;
        }
    }
    console_ethercat_set_control_word(selected, 0x0006u);
    if (console_ethercat_wait_for_status(selected, 0x0021u) != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 0x0006 状态确认失败" :
               "ethercat_enable: 0x0006 state confirmation failed");
        goto cleanup;
    }
    console_ethercat_set_control_word(selected, 0x0007u);
    if (console_ethercat_wait_for_status(selected, 0x0023u) != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 0x0007 状态确认失败" :
               "ethercat_enable: 0x0007 state confirmation failed");
        goto cleanup;
    }
    console_ethercat_set_control_word(selected, 0x000fu);
    if (console_ethercat_wait_for_status(selected, 0x0027u) != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 0x000F 状态确认失败" :
               "ethercat_enable: 0x000F state confirmation failed");
        goto cleanup;
    }
    printf("%s\n", chinese ?
           "ethercat_enable: 已使能，正在保持当前位置；Ctrl-C 时自动失能。" :
           "ethercat_enable: enabled and holding current positions; automatically disables on Ctrl-C.");
    deadline = hold_ms == 0u ? UINT64_MAX : time_us() + (uint64_t)hold_ms * 1000u;
    while (!stop_requested && time_us() < deadline) {
        if (console_ethercat_exchange() != 0) {
            printf("%s\n", chinese ? "ethercat_enable: PDO 通信中断" :
                   "ethercat_enable: PDO communication lost");
            goto cleanup;
        }
        sleep_ms(ETHERCAT_KAIXUAN_CONTROL_PERIOD_MS);
    }
    result = stop_requested ? -1 : 0;

cleanup:
    if (mapped) {
        console_ethercat_disable_selected(selected);
    }
    printf("%s\n", chinese ? "ethercat_enable: 已发送失能并关闭 EtherCAT 主站" :
           "ethercat_enable: disable sent and EtherCAT master closed");
close_socket:
    ec_close();
    return result;
#endif
}

static int console_ethercat_disable(bool chinese,
                                    const char *interface,
                                    const char *selection)
{
    unsigned int slave_id;
    bool all_slaves;

    if (console_ethercat_validate_interface(interface) != 0 ||
        console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0) {
        printf("%s: ethercat_disable <network_interface> <slave_id|all>\n",
               chinese ? "用法" : "usage");
        return -1;
    }

#ifndef HAVE_SOEM
    printf("%s\n", chinese ?
           "当前程序未编译 SOEM，无法失能 EtherCAT 电机。从 SOEM 源码编译后，执行 make "
           "FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" 重新构建。" :
           "EtherCAT disable is unavailable because this build has no SOEM support. Build again "
           "with make FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" after building SOEM.");
    return -1;
#else
    uint8_t process_image[ETHERCAT_KAIXUAN_PROCESS_IMAGE_SIZE];
    uint8_t selected[EC_MAXSLAVE] = {0};
    int32_t zero_positions[EC_MAXSLAVE] = {0};
    int slave;
    int result = -1;
    bool mapped = false;
    bool operational = false;

    printf("%s: interface=%s slave=%s\n",
           chinese ? "ethercat_disable: 开始" : "ethercat_disable: start",
           interface, selection);
    if (ec_init((char *)interface) == 0) {
        printf("%s: %s\n", chinese ? "ethercat_disable: 打开网卡失败" :
               "ethercat_disable: failed to open interface", interface);
        return -1;
    }
    if (ec_config_init(FALSE) <= 0) {
        printf("%s\n", chinese ? "ethercat_disable: 未发现 EtherCAT 从站" :
               "ethercat_disable: no EtherCAT slaves found");
        goto close_socket;
    }
    if (!all_slaves && slave_id > (unsigned int)ec_slavecount) {
        printf("%s: %u (1..%d)\n", chinese ? "ethercat_disable: 从站序号不存在" :
               "ethercat_disable: slave id is out of range", slave_id, ec_slavecount);
        goto close_socket;
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (all_slaves || (unsigned int)slave == slave_id) {
            selected[slave] = 1u;
        }
    }

    ec_config_map(process_image);
    mapped = true;
    ec_configdc();
    if ((ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) & 0x0fu) != EC_STATE_SAFE_OP) {
        printf("%s\n", chinese ? "ethercat_disable: 从站未进入 SAFE-OP" :
               "ethercat_disable: slaves did not reach SAFE-OP");
        goto cleanup;
    }
    if (console_ethercat_selected_ready(selected, zero_positions) != 0) {
        printf("%s\n", chinese ?
               "ethercat_disable: PDO 映射不是当前开璇驱动器要求的 13B 输出/14B 输入，已拒绝操作" :
               "ethercat_disable: PDO mapping is not the required Kaixuan 13B output/14B input layout; operation refused");
        goto cleanup;
    }
    console_ethercat_set_control_word(selected, 0u);
    if (console_ethercat_exchange() != 0) {
        printf("%s\n", chinese ? "ethercat_disable: 初始 PDO 通信失败" :
               "ethercat_disable: initial PDO exchange failed");
        goto cleanup;
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] != 0u) {
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate((uint16)slave);
        }
    }
    for (slave = 0; slave < 100 && !stop_requested; slave++) {
        bool all_operational = true;
        int target;

        if (console_ethercat_exchange() != 0) {
            break;
        }
        for (target = 1; target <= ec_slavecount; target++) {
            if (selected[target] != 0u &&
                (ec_statecheck((uint16)target, EC_STATE_OPERATIONAL, EC_TIMEOUTRET) & 0x0fu) !=
                EC_STATE_OPERATIONAL) {
                all_operational = false;
                break;
            }
        }
        if (all_operational) {
            operational = true;
            break;
        }
        sleep_ms(ETHERCAT_KAIXUAN_CONTROL_PERIOD_MS);
    }
    if (!operational) {
        printf("%s\n", chinese ? "ethercat_disable: 从站未进入 OP，已尝试安全失能" :
               "ethercat_disable: slave did not reach OP; attempted safe disable");
        goto cleanup;
    }
    console_ethercat_disable_selected(selected);
    printf("%s\n", chinese ? "ethercat_disable: 已发送 0x0000 并切回 SAFE-OP" :
           "ethercat_disable: sent 0x0000 and returned to SAFE-OP");
    result = 0;

cleanup:
    if (mapped && !operational) {
        console_ethercat_disable_selected(selected);
    }
close_socket:
    ec_close();
    return result;
#endif
}
