#include "etherlab_compat.h"

#include <errno.h>
#include <net/if.h>
#include <netinet/ether.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

ec_compat_slave ec_slave[EC_MAXSLAVE];
ec_compat_group ec_group[1];
int ec_slavecount;
int64_t ec_DCtime;

typedef struct {
    ec_slave_config_t *config;
    ec_reg_request_t *register_request;
    ec_sdo_request_t *sdo_read_request;
    ec_sdo_request_t *sdo_write_request;
    ec_slave_config_state_t state;
    unsigned int output_offset;
    unsigned int input_offset;
    uint8_t output_shadow[23];
} etherlab_slave_context;

static ec_master_t *etherlab_master;
static ec_domain_t *etherlab_domain;
static uint8_t *etherlab_domain_data;
static etherlab_slave_context etherlab_slaves[EC_MAXSLAVE];
static bool etherlab_activated;
static bool etherlab_mapped;
static bool etherlab_dc_configured;
static bool etherlab_reference_clock_selected;
static uint32_t etherlab_sync_cycle_ns;
static int32_t etherlab_sync_shift_ns;
static int etherlab_working_counter;
static unsigned int etherlab_master_index;
static const ec_pdo_entry_info_t etherlab_rx_entries[] = {
    {0x6040, 0x00, 16}, {0x607a, 0x00, 32}, {0x60ff, 0x00, 32},
    {0x6071, 0x00, 16}, {0x6060, 0x00, 8},
};
static const ec_pdo_entry_info_t etherlab_tx_entries[] = {
    {0x6041, 0x00, 16}, {0x6064, 0x00, 32}, {0x606c, 0x00, 32},
    {0x6077, 0x00, 16}, {0x603f, 0x00, 16},
};
static const ec_pdo_info_t etherlab_pdos[] = {
    {0x1601, sizeof(etherlab_rx_entries) / sizeof(etherlab_rx_entries[0]), etherlab_rx_entries},
    {0x1a01, sizeof(etherlab_tx_entries) / sizeof(etherlab_tx_entries[0]), etherlab_tx_entries},
};
static const ec_sync_info_t etherlab_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, &etherlab_pdos[0], EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, &etherlab_pdos[1], EC_WD_DISABLE},
    {0xff, EC_DIR_INVALID, 0, NULL, EC_WD_DEFAULT},
};

static const ec_pdo_entry_info_t etherlab_mit_rx[] = {
    {0x6040,0,16}, {0x4010,0,32}, {0x4011,0,32}, {0x4012,0,32},
    {0x4013,0,32}, {0x4014,0,32}, {0x6060,0,8},
};
static const ec_pdo_entry_info_t etherlab_mit_tx[] = {
    {0x6041,0,16}, {0x4000,0,32}, {0x4001,0,32}, {0x4002,0,32}, {0x603f,0,16},
};
static const ec_pdo_info_t etherlab_mit_pdos[] = {
    {0x1601,7,etherlab_mit_rx}, {0x1a01,5,etherlab_mit_tx},
};
static const ec_sync_info_t etherlab_mit_syncs[] = {
    {0,EC_DIR_OUTPUT,0,NULL,EC_WD_DISABLE}, {1,EC_DIR_INPUT,0,NULL,EC_WD_DISABLE},
    {2,EC_DIR_OUTPUT,1,&etherlab_mit_pdos[0],EC_WD_ENABLE},
    {3,EC_DIR_INPUT,1,&etherlab_mit_pdos[1],EC_WD_DISABLE},
    {0xff,EC_DIR_INVALID,0,NULL,EC_WD_DEFAULT},
};

static int etherlab_master_index_for_mac(char *configured_devices,
                                        const char *mac_address)
{
    char *save_pointer;
    char *device;
    int index = 0;

    for (device = strtok_r(configured_devices, ", \t\r\n", &save_pointer);
         device != NULL;
         device = strtok_r(NULL, ", \t\r\n", &save_pointer), index++) {
        if (strcasecmp(device, mac_address) == 0) return index;
    }
    return -1;
}

static int etherlab_master_index_for_interface(const char *interface)
{
    struct ifreq request;
    char configured_devices[256];
    char mac_address[ETHER_ADDR_LEN * 3];
    FILE *file;
    int socket_fd;

    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) return -1;
    memset(&request, 0, sizeof(request));
    snprintf(request.ifr_name, sizeof(request.ifr_name), "%s", interface);
    if (ioctl(socket_fd, SIOCGIFHWADDR, &request) != 0) {
        close(socket_fd);
        return -1;
    }
    close(socket_fd);
    snprintf(mac_address, sizeof(mac_address), "%02x:%02x:%02x:%02x:%02x:%02x",
             (unsigned char)request.ifr_hwaddr.sa_data[0],
             (unsigned char)request.ifr_hwaddr.sa_data[1],
             (unsigned char)request.ifr_hwaddr.sa_data[2],
             (unsigned char)request.ifr_hwaddr.sa_data[3],
             (unsigned char)request.ifr_hwaddr.sa_data[4],
             (unsigned char)request.ifr_hwaddr.sa_data[5]);
    file = fopen("/sys/module/ec_master/parameters/main_devices", "r");
    if (file == NULL) return -1;
    if (fgets(configured_devices, sizeof(configured_devices), file) == NULL) {
        fclose(file);
        return -1;
    }
    fclose(file);
    return etherlab_master_index_for_mac(configured_devices, mac_address);
}

static void etherlab_update_slave_states(void)
{
    int slave;

    if (etherlab_master == NULL) return;
    for (slave = 1; slave <= ec_slavecount; ++slave) {
        if (etherlab_activated && etherlab_slaves[slave].config != NULL) {
            ecrt_slave_config_state(etherlab_slaves[slave].config,
                                    &etherlab_slaves[slave].state);
            ec_slave[slave].state = etherlab_slaves[slave].state.al_state;
        }
    }
}

static bool etherlab_reference_clock_ready(void)
{
    int slave;

    for (slave = 1; slave <= ec_slavecount; ++slave) {
        unsigned int state = (unsigned int)ec_slave[slave].state & 0x0fu;
        if (ec_slave[slave].DCactive != 0u &&
            (state == EC_STATE_SAFE_OP || state == EC_STATE_OPERATIONAL)) {
            return true;
        }
    }
    return false;
}

static void etherlab_set_application_time(void)
{
    struct timespec now;
    struct timeval time_value;

    if (etherlab_master == NULL || clock_gettime(CLOCK_REALTIME, &now) != 0) return;
    time_value.tv_sec = now.tv_sec;
    time_value.tv_usec = now.tv_nsec / 1000;
    ecrt_master_application_time(etherlab_master, EC_TIMEVAL2NANO(time_value));
}

static int etherlab_activate(void)
{
    unsigned int slave;
    if (etherlab_activated) return 0;
    if (etherlab_master == NULL) return -1;
    etherlab_set_application_time();
    if (ecrt_master_activate(etherlab_master) != 0) return -1;
    etherlab_activated = true;
    etherlab_domain_data = etherlab_domain == NULL ? NULL : ecrt_domain_data(etherlab_domain);
    if (etherlab_mapped && etherlab_domain_data == NULL) return -1;
    for (slave = 1; slave <= (unsigned int)ec_slavecount; ++slave) {
        if (ec_slave[slave].Obits == 0) continue;
        ec_slave[slave].outputs = etherlab_slaves[slave].output_shadow;
        ec_slave[slave].inputs = etherlab_domain_data + etherlab_slaves[slave].input_offset;
    }
    return 0;
}

int ec_init(char *interface)
{
    int master_index;

    if (interface == NULL) return 0;
    master_index = etherlab_master_index_for_interface(interface);
    if (master_index < 0) {
        fprintf(stderr, "EtherLab: %s is not bound to a main master in /sys/module/ec_master/parameters/main_devices\n",
                interface);
        return 0;
    }
    etherlab_master = ecrt_request_master((unsigned int)master_index);
    if (etherlab_master == NULL) {
        fprintf(stderr, "EtherLab: cannot request master %d for %s\n", master_index, interface);
        return 0;
    }
    etherlab_master_index = (unsigned int)master_index;
    memset(ec_slave, 0, sizeof(ec_slave));
    memset(etherlab_slaves, 0, sizeof(etherlab_slaves));
    memset(ec_group, 0, sizeof(ec_group));
    ec_slavecount = 0;
    etherlab_domain = NULL;
    etherlab_domain_data = NULL;
    etherlab_activated = false;
    etherlab_mapped = false;
    etherlab_dc_configured = false;
    etherlab_reference_clock_selected = false;
    etherlab_sync_cycle_ns = 0;
    etherlab_sync_shift_ns = 0;
    etherlab_working_counter = 0;
    ec_DCtime = 0;
    return 1;
}

int ec_config_init(boolean use_table)
{
    ec_master_info_t master_info;
    ec_master_state_t master_state;
    ec_slave_info_t slave_info;
    ec_slave_info_t slave_infos[EC_MAXSLAVE];
    unsigned int attempt;
    unsigned int position;
    int unready_slave = -1;
    bool identities_ready = false;
    (void)use_table;

    if (etherlab_master == NULL) return 0;
    for (attempt = 0; attempt < 500; ++attempt) {
        memset(&master_info, 0, sizeof(master_info));
        if (ecrt_master(etherlab_master, &master_info) == 0 &&
            master_info.slave_count > 0 &&
            master_info.slave_count < EC_MAXSLAVE &&
            master_info.scan_busy == 0) {
            identities_ready = true;
            for (position = 0; position < master_info.slave_count; ++position) {
                if (ecrt_master_get_slave(etherlab_master, (uint16_t)position,
                                          &slave_infos[position]) != 0 ||
                    slave_infos[position].vendor_id == 0 ||
                    slave_infos[position].product_code == 0) {
                    identities_ready = false;
                    unready_slave = (int)position + 1;
                    break;
                }
            }
            if (identities_ready) break;
        }
        usleep(20000);
    }
    if (!identities_ready) {
        memset(&master_state, 0, sizeof(master_state));
        ecrt_master_state(etherlab_master, &master_state);
        fprintf(stderr, "EtherLab: master %u slave identities not ready after 10s (slaves=%u, responding=%u, link=%s, scan_busy=%u, first_unready_slave=%d); refusing zero-identity configuration\n",
                etherlab_master_index, (unsigned int)master_info.slave_count,
                (unsigned int)master_state.slaves_responding,
                master_state.link_up ? "UP" : "DOWN",
                (unsigned int)master_info.scan_busy, unready_slave);
        return 0;
    }
    ec_slavecount = (int)master_info.slave_count;
    for (position = 0; position < master_info.slave_count; ++position) {
        int slave = (int)position + 1;
        slave_info = slave_infos[position];
        snprintf(ec_slave[slave].name, sizeof(ec_slave[slave].name), "%s", slave_info.name);
        ec_slave[slave].configadr = (uint16_t)slave;
        ec_slave[slave].eep_man = slave_info.vendor_id;
        ec_slave[slave].eep_id = slave_info.product_code;
        ec_slave[slave].eep_rev = slave_info.revision_number;
        ec_slave[slave].state = slave_info.al_state;
        ec_slave[slave].hasdc = slave_info.vendor_id == 0x00010203u &&
                                slave_info.product_code == 0x00000402u;
        /* Console IDs are absolute ring positions, regardless of EEPROM aliases. */
        etherlab_slaves[slave].config = ecrt_master_slave_config(
            etherlab_master, 0, (uint16_t)position,
            slave_info.vendor_id, slave_info.product_code);
        if (etherlab_slaves[slave].config == NULL) return 0;
        etherlab_slaves[slave].register_request =
            ecrt_slave_config_create_reg_request(etherlab_slaves[slave].config, 8);
        etherlab_slaves[slave].sdo_read_request =
            ecrt_slave_config_create_sdo_request(etherlab_slaves[slave].config, 0x6041, 0, 256);
        etherlab_slaves[slave].sdo_write_request =
            ecrt_slave_config_create_sdo_request(etherlab_slaves[slave].config, 0x2077, 0, 4);
        if (etherlab_slaves[slave].sdo_read_request == NULL ||
            etherlab_slaves[slave].sdo_write_request == NULL ||
            ecrt_sdo_request_timeout(etherlab_slaves[slave].sdo_read_request, 700) != 0 ||
            ecrt_sdo_request_timeout(etherlab_slaves[slave].sdo_write_request, 700) != 0) return 0;
    }
    return ec_slavecount;
}

int ec_config_map(void *process_image)
{
    unsigned int slave;
    (void)process_image;
    if (etherlab_master == NULL || ec_slavecount <= 0) return 0;
    etherlab_domain = ecrt_master_create_domain(etherlab_master);
    if (etherlab_domain == NULL) return 0;
    for (slave = 1; slave <= (unsigned int)ec_slavecount; ++slave) {
        if (ec_slave[slave].eep_man != 0x00010203u ||
            ec_slave[slave].eep_id != 0x00000402u) continue;
        if (ecrt_slave_config_pdos(etherlab_slaves[slave].config,
                                   EC_END, ethercat_mit_mapping ? etherlab_mit_syncs : etherlab_syncs) != 0) return 0;
        int output_offset = ecrt_slave_config_reg_pdo_entry(
            etherlab_slaves[slave].config, 0x6040, 0, etherlab_domain, NULL);
        int input_offset = ecrt_slave_config_reg_pdo_entry(
            etherlab_slaves[slave].config, 0x6041, 0, etherlab_domain, NULL);
        if (output_offset < 0 || input_offset < 0) return 0;
        etherlab_slaves[slave].output_offset = (unsigned int)output_offset;
        etherlab_slaves[slave].input_offset = (unsigned int)input_offset;
        ec_slave[slave].Obytes = ethercat_mit_mapping ? 23 : 13;
        ec_slave[slave].Ibytes = ethercat_mit_mapping ? 16 : 14;
        ec_slave[slave].Obits = ec_slave[slave].Obytes * 8;
        ec_slave[slave].Ibits = ec_slave[slave].Ibytes * 8;
        ec_group[0].outputsWKC++;
        ec_group[0].inputsWKC++;
    }
    etherlab_mapped = true;
    return 1;
}

int ec_configdc(void)
{
    return 1;
}

int ec_dcsync0(uint16 slave, boolean activate, uint32_t cycle_ns, int32_t shift_ns)
{
    int result;

    if (slave == 0 || slave > ec_slavecount || !activate) return -1;
    result = ecrt_slave_config_dc(etherlab_slaves[slave].config, 0x0300,
                                  cycle_ns, shift_ns, 0, 0);
    if (result != 0) return result;
    if (!etherlab_reference_clock_selected) {
        result = ecrt_master_select_reference_clock(
            etherlab_master, etherlab_slaves[slave].config);
        if (result != 0) return result;
        etherlab_reference_clock_selected = true;
    }
    etherlab_sync_cycle_ns = cycle_ns;
    etherlab_sync_shift_ns = shift_ns;
    etherlab_dc_configured = true;
    ec_slave[slave].DCactive = 1;
    ec_slave[slave].DCcycle = cycle_ns;
    ec_slave[slave].DCshift = shift_ns;
    return 0;
}

int ec_activate(void)
{
    return etherlab_activate();
}

int ec_send_processdata(void)
{
    unsigned int slave;

    if (etherlab_master == NULL || etherlab_activate() != 0) return -1;
    if (etherlab_domain_data != NULL) {
        for (slave = 1; slave <= (unsigned int)ec_slavecount; ++slave) {
            if (ec_slave[slave].Obits == 0) continue;
            memcpy(etherlab_domain_data + etherlab_slaves[slave].output_offset,
                   etherlab_slaves[slave].output_shadow,
                   ec_slave[slave].Obytes);
        }
    }
    etherlab_set_application_time();
    if (etherlab_dc_configured) {
        ecrt_master_sync_reference_clock(etherlab_master);
        ecrt_master_sync_slave_clocks(etherlab_master);
    }
    if (etherlab_domain != NULL) ecrt_domain_queue(etherlab_domain);
    return ecrt_master_send(etherlab_master);
}

int ec_receive_processdata(int timeout_us)
{
    ec_domain_state_t domain_state;
    uint32_t reference_time;
    (void)timeout_us;
    if (etherlab_master == NULL || etherlab_activate() != 0) return 0;
    if (ecrt_master_receive(etherlab_master) < 0) {
        etherlab_working_counter = 0;
        ec_DCtime = 0;
        return 0;
    }
    if (etherlab_domain != NULL) {
        ecrt_domain_process(etherlab_domain);
        ecrt_domain_state(etherlab_domain, &domain_state);
        etherlab_working_counter = (int)domain_state.working_counter;
    }
    etherlab_update_slave_states();
    ec_DCtime = 0;
    if (etherlab_dc_configured && etherlab_reference_clock_ready() &&
        ec_group[0].outputsWKC + ec_group[0].inputsWKC > 0 &&
        etherlab_working_counter >=
            ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC &&
        ecrt_master_reference_clock_time(etherlab_master, &reference_time) == 0) {
        ec_DCtime = reference_time;
    }
    return etherlab_working_counter;
}

static const char *etherlab_al_state_name(uint8_t state)
{
    switch (state & 0x0fu) {
    case EC_STATE_INIT: return "INIT";
    case EC_STATE_PRE_OP: return "PRE-OP";
    case EC_STATE_BOOT: return "BOOT";
    case EC_STATE_SAFE_OP: return "SAFE-OP";
    case EC_STATE_OPERATIONAL: return "OP";
    default: return "UNKNOWN";
    }
}

static int etherlab_safe_op_timeout_us(void)
{
    unsigned int count = ec_slavecount > 0 ? (unsigned int)ec_slavecount : 1u;
    uint64_t budget = (uint64_t)EC_TIMEOUTSTATE + 15000000u * (count - 1u);
    return (int)(budget > 120000000u ? 120000000u : budget);
}

static int etherlab_statecheck(uint16 slave, uint16 requested_state, int timeout_us,
                               bool report_progress, bool chinese,
                               const char *command_name)
{
    struct timespec delay = {0, 1000000L};
    uint64_t deadline;
    uint64_t now;
    uint64_t started_us;
    uint64_t next_progress_us;
    struct timespec clock_now;
    int target = requested_state & 0x0f;
    if (etherlab_master == NULL) return 0;
    if (!etherlab_activated && etherlab_mapped && etherlab_activate() != 0) return 0;
    clock_gettime(CLOCK_MONOTONIC, &clock_now);
    started_us = (uint64_t)clock_now.tv_sec * 1000000u +
                 (uint64_t)clock_now.tv_nsec / 1000u;
    deadline = started_us + (uint64_t)(timeout_us > 0 ? timeout_us : 1);
    next_progress_us = started_us;
    do {
        if (stop_requested || console_ethercat_background_should_stop()) return 0;
        ec_receive_processdata(EC_TIMEOUTRET);
        if (report_progress) {
            ec_master_state_t master_state;
            int candidate;

            clock_gettime(CLOCK_MONOTONIC, &clock_now);
            now = (uint64_t)clock_now.tv_sec * 1000000u +
                  (uint64_t)clock_now.tv_nsec / 1000u;
            if (now >= next_progress_us) {
                ecrt_master_state(etherlab_master, &master_state);
                printf("%s: %s %.3fs master=%u/%d link=%s\n", command_name,
                       chinese ? "SAFE-OP 等待进度:" : "SAFE-OP wait progress:",
                       (double)(now - started_us) / 1000000.0,
                       master_state.slaves_responding, ec_slavecount,
                       master_state.link_up ? "UP" : "DOWN");
                for (candidate = 1; candidate <= ec_slavecount; ++candidate) {
                    printf("  slave=%d AL=%s(0x%02x) online=%u operational=%u\n", candidate,
                           etherlab_al_state_name(ec_slave[candidate].state),
                           (unsigned int)ec_slave[candidate].state,
                           etherlab_slaves[candidate].state.online,
                           etherlab_slaves[candidate].state.operational);
                }
                next_progress_us = now + 1000000u;
            }
        }
        if (slave == 0) {
            int candidate;
            bool all_ready = true;
            for (candidate = 1; candidate <= ec_slavecount; ++candidate) {
                int state = ec_slave[candidate].state & 0x0f;
                if (state != target &&
                    !(target == EC_STATE_SAFE_OP && state == EC_STATE_OPERATIONAL)) {
                    all_ready = false;
                    break;
                }
            }
            if (all_ready) return target;
        } else if (slave <= ec_slavecount) {
            etherlab_update_slave_states();
            if ((ec_slave[slave].state & 0x0f) == target ||
                (target == EC_STATE_SAFE_OP &&
                 (ec_slave[slave].state & 0x0f) == EC_STATE_OPERATIONAL)) return target;
        }
        if (ec_send_processdata() < 0) return 0;
        if (ethercat_background_owner) console_ethercat_wait_next_cycle();
        else nanosleep(&delay, NULL);
        clock_gettime(CLOCK_MONOTONIC, &clock_now);
        now = (uint64_t)clock_now.tv_sec * 1000000u + (uint64_t)clock_now.tv_nsec / 1000u;
    } while (now < deadline);
    return slave > 0 && slave <= ec_slavecount ? ec_slave[slave].state : 0;
}

int ec_statecheck(uint16 slave, uint16 requested_state, int timeout_us)
{
    return etherlab_statecheck(slave, requested_state, timeout_us, false, false, NULL);
}

int ec_statecheck_with_progress(uint16 slave, uint16 requested_state, int timeout_us,
                                bool chinese, const char *command_name)
{
    return etherlab_statecheck(slave, requested_state, timeout_us, true, chinese,
                               command_name);
}

int ec_readstate(void)
{
    ec_master_state_t master_state;
    int slave;
    if (etherlab_master == NULL) return 0;
    ecrt_master_state(etherlab_master, &master_state);
    etherlab_update_slave_states();
    for (slave = 1; slave <= ec_slavecount; ++slave) {
        uint8_t al_code[2] = {0};
        if (etherlab_activated &&
            ec_FPRD(ec_slave[slave].configadr, 0x0134, sizeof(al_code),
                    al_code, EC_TIMEOUTRET) > 0) {
            ec_slave[slave].ALstatuscode = (uint16_t)al_code[0] |
                                           ((uint16_t)al_code[1] << 8u);
        }
    }
    return (int)master_state.slaves_responding;
}

int ec_writestate(uint16 slave)
{
    (void)slave;
    return 1;
}

int ec_FPRD(uint16 configadr, uint16 reg, uint16 length, void *data, int timeout_us)
{
    ec_reg_request_t *request;
    unsigned int slave = configadr;
    unsigned int attempt;
    (void)timeout_us;
    if (etherlab_master == NULL || slave == 0 || slave > ec_slavecount ||
        length > 8 || data == NULL) return 0;
    request = etherlab_slaves[slave].register_request;
    if (request == NULL) return 0;
    if (!etherlab_activated) {
        if (etherlab_activate() != 0) return 0;
    }
    if (ecrt_reg_request_state(request) == EC_REQUEST_BUSY) return 0;
    if (ecrt_reg_request_read(request, reg, length) != 0) return 0;
    for (attempt = 0; attempt < 20; ++attempt) {
        ec_receive_processdata(EC_TIMEOUTRET);
        if (ecrt_reg_request_state(request) == EC_REQUEST_SUCCESS) {
            memcpy(data, ecrt_reg_request_data(request), length);
            return 1;
        }
        if (ec_send_processdata() < 0) return 0;
        usleep(1000);
    }
    return 0;
}

static int etherlab_wait_sdo(ec_sdo_request_t *request, int timeout_us)
{
    uint64_t deadline = console_ethercat_monotonic_us() +
                        (uint64_t)(timeout_us > 0 ? timeout_us : 700000);

    do {
        ec_request_state_t state = ecrt_sdo_request_state(request);
        if (state == EC_REQUEST_SUCCESS) return 1;
        if (state != EC_REQUEST_BUSY || console_ethercat_mailbox_cycle() != 0) return 0;
    } while (console_ethercat_monotonic_us() < deadline);
    return 0;
}

int ec_SDOread(uint16 slave, uint16 index, uint8 subindex, boolean complete_access,
               int *size, void *data, int timeout_us)
{
    size_t actual = 0;
    uint32_t abort_code = 0;
    int result;
    (void)complete_access;
    (void)timeout_us;
    if (etherlab_master == NULL || slave == 0 ||
        slave > ec_slavecount || size == NULL ||
        *size <= 0 || data == NULL) return 0;
    if (etherlab_activated) {
        ec_sdo_request_t *request = etherlab_slaves[slave].sdo_read_request;
        if (stop_requested || console_ethercat_background_should_stop() ||
            complete_access || request == NULL ||
            ecrt_sdo_request_state(request) == EC_REQUEST_BUSY ||
            ecrt_sdo_request_index(request, index, subindex) != 0 ||
            ecrt_sdo_request_read(request) != 0 ||
            !etherlab_wait_sdo(request, timeout_us)) return 0;
        actual = ecrt_sdo_request_data_size(request);
        if (actual == 0 || actual > (size_t)*size) return 0;
        memcpy(data, ecrt_sdo_request_data(request), actual);
        *size = (int)actual;
        return (int)actual;
    }
    result = ecrt_master_sdo_upload(etherlab_master, (uint16_t)(slave - 1), index,
        subindex, data, (size_t)*size, &actual, &abort_code);
    if (result != 0 || actual > INT_MAX) {
        printf("SDO upload failed: slave=%u ring_position=%u object=0x%04x:%02x result=%d abort=0x%08x\n",
               (unsigned int)slave, (unsigned int)(slave - 1), (unsigned int)index,
               (unsigned int)subindex, result, (unsigned int)abort_code);
        return 0;
    }
    *size = (int)actual;
    return (int)actual;
}

int ec_SDOwrite(uint16 slave, uint16 index, uint8 subindex, boolean complete_access,
                int size, const void *data, int timeout_us)
{
    uint32_t abort_code = 0;
    (void)complete_access;
    (void)timeout_us;
    if (etherlab_master == NULL || slave == 0 ||
        slave > ec_slavecount || size <= 0 || data == NULL) return 0;
    if (etherlab_activated) {
        ec_sdo_request_t *request = etherlab_slaves[slave].sdo_write_request;
        if (stop_requested || console_ethercat_background_should_stop() ||
            complete_access || size > 4 || request == NULL ||
            ecrt_sdo_request_state(request) == EC_REQUEST_BUSY ||
            ecrt_sdo_request_index(request, index, subindex) != 0) return 0;
        memcpy(ecrt_sdo_request_data(request), data, (size_t)size);
        if (ecrt_sdo_request_write(request) != 0 ||
            !etherlab_wait_sdo(request, timeout_us)) return 0;
        return size;
    }
    return ecrt_master_sdo_download(etherlab_master, (uint16_t)(slave - 1), index,
        subindex, data, (size_t)size, &abort_code) == 0 ? size : 0;
}

int ec_close(void)
{
    if (etherlab_master == NULL) return 0;
    if (etherlab_activated) ecrt_master_deactivate(etherlab_master);
    ecrt_release_master(etherlab_master);
    etherlab_master = NULL;
    etherlab_activated = false;
    etherlab_mapped = false;
    etherlab_dc_configured = false;
    etherlab_reference_clock_selected = false;
    etherlab_domain = NULL;
    etherlab_domain_data = NULL;
    return 1;
}
