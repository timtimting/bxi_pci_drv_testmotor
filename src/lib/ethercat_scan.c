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
