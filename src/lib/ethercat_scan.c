#include <net/if.h>
#include <time.h>

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

static int console_ethercat_parse_position_rad(const char *text, double *position_rad)
{
    char *end;
    double parsed;

    if (text == NULL || position_rad == NULL) {
        return -1;
    }
    errno = 0;
    parsed = strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed)) {
        return -1;
    }
    *position_rad = parsed;
    return 0;
}

static int console_ethercat_parse_sync0_shift(const char *text, int32_t *shift_ns)
{
    char *end;
    long parsed;

    if (text == NULL || shift_ns == NULL) {
        return -1;
    }
    errno = 0;
    parsed = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' ||
        parsed < -4000000L || parsed > 4000000L) {
        return -1;
    }
    *shift_ns = (int32_t)parsed;
    return 0;
}

enum {
    ETHERCAT_KAIXUAN_VENDOR_ID = 0x00010203u,
    ETHERCAT_KAIXUAN_PRODUCT_CODE = 0x00000402u,
    ETHERCAT_KAIXUAN_RXPDO_BITS = 104u,
    ETHERCAT_KAIXUAN_TXPDO_MIN_BITS = 112u,
    ETHERCAT_KAIXUAN_CONTROL_PERIOD_MS = 4u,
    ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS = 4000000u,
    ETHERCAT_KAIXUAN_ENABLE_MAX_HOLD_MS = 60000u,
    ETHERCAT_KAIXUAN_PROCESS_IMAGE_SIZE = 8192u,
    ETHERCAT_COMPLETION_INTERFACE_MAX = 32u,
    ETHERCAT_COMPLETION_SLAVE_MAX = 100u,
};

#define ETHERCAT_DEFAULT_INTERFACE "enp86s0"

#define ETHERCAT_KAIXUAN_COUNTS_PER_REV 1048576.0
#define ETHERCAT_KAIXUAN_TWO_PI 6.28318530717958647692

static const char *const ethercat_hold_ms_words[] = {
    "1000", "2000", "5000", "10000", "30000", "60000",
};

static const char *const ethercat_sync0_shift_words[] = {
    "0", "100000", "-100000",
};

static const char *const ethercat_position_rad_words[] = {
    "0",
};

static const char *const ethercat_all_word[] = {
    "all",
};

static const char *const ethercat_pn077_value_words[] = {
    "0", "1",
};

static char ethercat_interface_storage[ETHERCAT_COMPLETION_INTERFACE_MAX][IFNAMSIZ];
static const char *ethercat_interface_words[ETHERCAT_COMPLETION_INTERFACE_MAX];
static char ethercat_slave_storage[ETHERCAT_COMPLETION_SLAVE_MAX][4];
static const char *ethercat_slave_words[ETHERCAT_COMPLETION_SLAVE_MAX + 1u];
static bool ethercat_slave_words_initialized = false;

static const char *const *console_ethercat_interface_completion_words(size_t *count)
{
    DIR *directory;
    struct dirent *entry;
    size_t interface_count = 0u;

    directory = opendir("/sys/class/net");
    if (directory == NULL) {
        *count = 0u;
        return NULL;
    }
    while ((entry = readdir(directory)) != NULL &&
           interface_count < ETHERCAT_COMPLETION_INTERFACE_MAX) {
        size_t name_length;

        if (entry->d_name[0] == '.') {
            continue;
        }
        name_length = strnlen(entry->d_name, IFNAMSIZ);
        if (name_length == IFNAMSIZ) {
            continue;
        }
        memcpy(ethercat_interface_storage[interface_count], entry->d_name, name_length + 1u);
        ethercat_interface_words[interface_count] = ethercat_interface_storage[interface_count];
        interface_count++;
    }
    closedir(directory);
    *count = interface_count;
    return ethercat_interface_words;
}

static const char *const *console_ethercat_slave_completion_words(size_t *count)
{
    size_t slave;

    if (!ethercat_slave_words_initialized) {
        ethercat_slave_words[0] = ethercat_all_word[0];
        for (slave = 1u; slave <= ETHERCAT_COMPLETION_SLAVE_MAX; slave++) {
            snprintf(ethercat_slave_storage[slave - 1u],
                     sizeof(ethercat_slave_storage[slave - 1u]), "%zu", slave);
            ethercat_slave_words[slave] = ethercat_slave_storage[slave - 1u];
        }
        ethercat_slave_words_initialized = true;
    }
    *count = sizeof(ethercat_slave_words) / sizeof(ethercat_slave_words[0]);
    return ethercat_slave_words;
}

static const char *const *console_ethercat_completion_words(const char *line,
                                                             size_t len,
                                                             size_t *count)
{
    int start;
    unsigned int tokens_before;
    char first[64];

    start = current_token_start(line, len);
    tokens_before = count_tokens_before(line, start);
    if (copy_nth_token(line, 0u, first, sizeof(first)) != 0) {
        *count = 0u;
        return NULL;
    }
    if (strcmp(first, "ethercat_scan") == 0 && tokens_before == 1u) {
        return console_ethercat_interface_completion_words(count);
    }
    if ((strcmp(first, "ethercat_enable") == 0 ||
         strcmp(first, "ethercat_disable") == 0 ||
         strcmp(first, "ethercat_zero") == 0 ||
         strcmp(first, "ethercat_info") == 0 ||
         strcmp(first, "ethercat_save") == 0 ||
         strcmp(first, "ethercat_pn077") == 0) && tokens_before == 1u) {
        return console_ethercat_slave_completion_words(count);
    }
    if (strcmp(first, "ethercat_disable") == 0 ||
        strcmp(first, "ethercat_zero") == 0 ||
        strcmp(first, "ethercat_info") == 0 ||
        strcmp(first, "ethercat_save") == 0) {
        return console_ethercat_interface_completion_words(count);
    }
    if (strcmp(first, "ethercat_enable") == 0 && tokens_before == 2u) {
        *count = sizeof(ethercat_hold_ms_words) / sizeof(ethercat_hold_ms_words[0]);
        return ethercat_hold_ms_words;
    }
    if (strcmp(first, "ethercat_enable") == 0 && tokens_before == 3u) {
        *count = sizeof(ethercat_sync0_shift_words) /
                 sizeof(ethercat_sync0_shift_words[0]);
        return ethercat_sync0_shift_words;
    }
    if (strcmp(first, "ethercat_enable") == 0 && tokens_before >= 4u) {
        return console_ethercat_interface_completion_words(count);
    }
    if (strcmp(first, "ethercat_position") == 0 && tokens_before == 1u) {
        return console_ethercat_slave_completion_words(count);
    }
    if (strcmp(first, "ethercat_position") == 0 && tokens_before == 2u) {
        *count = sizeof(ethercat_position_rad_words) /
                 sizeof(ethercat_position_rad_words[0]);
        return ethercat_position_rad_words;
    }
    if (strcmp(first, "ethercat_position") == 0 && tokens_before == 3u) {
        *count = sizeof(ethercat_hold_ms_words) / sizeof(ethercat_hold_ms_words[0]);
        return ethercat_hold_ms_words;
    }
    if (strcmp(first, "ethercat_position") == 0 && tokens_before >= 4u) {
        return console_ethercat_interface_completion_words(count);
    }
    if (strcmp(first, "ethercat_pn077") == 0 && tokens_before == 2u) {
        *count = sizeof(ethercat_pn077_value_words) /
                 sizeof(ethercat_pn077_value_words[0]);
        return ethercat_pn077_value_words;
    }
    if (strcmp(first, "ethercat_pn077") == 0 && tokens_before >= 3u) {
        return console_ethercat_interface_completion_words(count);
    }
    *count = 0u;
    return NULL;
}

#ifdef HAVE_SOEM
static int ethercat_last_work_counter;
static int ethercat_expected_work_counter;
static int ethercat_min_work_counter;
static unsigned int ethercat_incomplete_work_counter_count;
static uint64_t ethercat_last_exchange_us;
static uint64_t ethercat_max_exchange_interval_us;
static struct timespec ethercat_next_cycle;
static bool ethercat_cycle_initialized;

static void console_ethercat_wait_next_cycle(void)
{
    struct timespec now;
    int wait_result;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        sleep_ms(ETHERCAT_KAIXUAN_CONTROL_PERIOD_MS);
        return;
    }
    if (!ethercat_cycle_initialized) {
        ethercat_next_cycle = now;
        ethercat_cycle_initialized = true;
    }
    ethercat_next_cycle.tv_nsec += (long)ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS;
    if (ethercat_next_cycle.tv_nsec >= 1000000000L) {
        ethercat_next_cycle.tv_sec++;
        ethercat_next_cycle.tv_nsec -= 1000000000L;
    }
    while ((ethercat_next_cycle.tv_sec < now.tv_sec) ||
           (ethercat_next_cycle.tv_sec == now.tv_sec &&
            ethercat_next_cycle.tv_nsec <= now.tv_nsec)) {
        ethercat_next_cycle.tv_nsec += (long)ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS;
        if (ethercat_next_cycle.tv_nsec >= 1000000000L) {
            ethercat_next_cycle.tv_sec++;
            ethercat_next_cycle.tv_nsec -= 1000000000L;
        }
    }
    do {
        wait_result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                      &ethercat_next_cycle, NULL);
    } while (wait_result == EINTR && !stop_requested);
}

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
    struct timespec cycle_start;
    uint64_t now_us = time_us();
    int work_counter;

    if (!ethercat_cycle_initialized &&
        clock_gettime(CLOCK_MONOTONIC, &cycle_start) == 0) {
        ethercat_next_cycle = cycle_start;
        ethercat_cycle_initialized = true;
    }
    if (ethercat_last_exchange_us != 0u && now_us >= ethercat_last_exchange_us) {
        uint64_t interval_us = now_us - ethercat_last_exchange_us;

        if (interval_us > ethercat_max_exchange_interval_us) {
            ethercat_max_exchange_interval_us = interval_us;
        }
    }
    ethercat_last_exchange_us = now_us;
    ec_send_processdata();
    work_counter = ec_receive_processdata(EC_TIMEOUTRET);
    ethercat_last_work_counter = work_counter;
    if (work_counter < ethercat_min_work_counter) {
        ethercat_min_work_counter = work_counter;
    }
    if (ethercat_expected_work_counter > 0 &&
        work_counter < ethercat_expected_work_counter) {
        ethercat_incomplete_work_counter_count++;
    }
    return work_counter > 0 ? 0 : -1;
}

static void console_ethercat_reset_exchange_diagnostics(void)
{
    ethercat_last_work_counter = 0;
    ethercat_expected_work_counter = 0;
    ethercat_min_work_counter = INT_MAX;
    ethercat_incomplete_work_counter_count = 0u;
    ethercat_last_exchange_us = 0u;
    ethercat_max_exchange_interval_us = 0u;
    ethercat_cycle_initialized = false;
}

static void console_ethercat_print_bytes(const uint8_t *data, unsigned int length)
{
    unsigned int byte;

    if (length > 32u) {
        length = 32u;
    }
    for (byte = 0u; byte < length; byte++) {
        printf("%s%02x", byte == 0u ? "" : " ", (unsigned int)data[byte]);
    }
}

static const char *console_ethercat_cia402_state_name(uint16_t status_word)
{
    switch (status_word & 0x006fu) {
    case 0x0040u:
        return "switch-on-disabled";
    case 0x0021u:
        return "ready-to-switch-on";
    case 0x0023u:
        return "switched-on";
    case 0x0027u:
        return "operation-enabled";
    case 0x0007u:
        return "quick-stop-active";
    case 0x000fu:
        return "fault-reaction-active";
    case 0x0008u:
        return "fault";
    default:
        return "unknown";
    }
}

static int console_ethercat_enable_dc_sync(const uint8_t selected[EC_MAXSLAVE],
                                           int32_t shift_ns)
{
    int slave;

    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] == 0u) {
            continue;
        }
        if (ec_slave[slave].hasdc == 0u) {
            return -1;
        }
        ec_dcsync0((uint16)slave, TRUE, ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS, shift_ns);
    }
    return 0;
}

static void console_ethercat_print_selected_status(const uint8_t selected[EC_MAXSLAVE],
                                                   uint16_t requested_control_word)
{
    int slave;

    ec_readstate();
    printf("[EtherCAT diag]: requested_control_word=0x%04x slaves=%d "
           "expected_wkc=%d last_wkc=%d min_wkc=%d incomplete_wkc_count=%u "
           "max_cycle_interval_us=%llu target_cycle_us=%u\n",
           (unsigned int)requested_control_word, ec_slavecount,
           ethercat_expected_work_counter, ethercat_last_work_counter,
           ethercat_min_work_counter, ethercat_incomplete_work_counter_count,
           (unsigned long long)ethercat_max_exchange_interval_us,
           ETHERCAT_KAIXUAN_CONTROL_PERIOD_MS * 1000u);
    for (slave = 1; slave <= ec_slavecount; slave++) {
        uint16_t status_word;
        uint16_t al_status;
        uint16_t error_code;
        uint16_t pn077;
        uint16_t pn078;
        uint16_t sync_type;
        uint32_t sync_cycle_ns;
        uint16_t tx_sync_type;
        uint32_t tx_sync_cycle_ns;
        uint8_t dc_activation;
        uint8_t al_status_data[2];
        uint8_t dc_cycle_data[4];
        uint8_t dc_start_data[8];
        uint32_t dc_cycle_ns;
        int8_t mode_display;
        int size;

        if (selected[slave] == 0u) {
            continue;
        }
        status_word = ec_slave[slave].inputs != NULL && ec_slave[slave].Ibits >= 16u ?
                      console_ethercat_read_u16((const uint8_t *)ec_slave[slave].inputs) :
                      0xffffu;
        al_status = 0u;
        if (ec_FPRD(ec_slave[slave].configadr, 0x0130u,
                    (uint16)sizeof(al_status_data), al_status_data, EC_TIMEOUTRET) > 0) {
            al_status = console_ethercat_read_u16(al_status_data);
        }
        printf("[slave%d]: ec_state=0x%02x(%s) ALstatus=0x%04x "
               "ALstatuscode=0x%04x vendor=0x%08x product=0x%08x revision=0x%08x "
               "hasdc=%u DCactive=%u DCcycle=%u DCshift=%d "
               "PDO_out=%uB/%ubit PDO_in=%uB/%ubit status_word=0x%04x cia402=%s",
               slave, (unsigned int)ec_slave[slave].state,
               console_ethercat_state_name(ec_slave[slave].state),
               (unsigned int)al_status,
               (unsigned int)ec_slave[slave].ALstatuscode,
               (unsigned int)ec_slave[slave].eep_man, (unsigned int)ec_slave[slave].eep_id,
               (unsigned int)ec_slave[slave].eep_rev, (unsigned int)ec_slave[slave].hasdc,
               (unsigned int)ec_slave[slave].DCactive, (unsigned int)ec_slave[slave].DCcycle,
               (int)ec_slave[slave].DCshift,
               (unsigned int)ec_slave[slave].Obytes, (unsigned int)ec_slave[slave].Obits,
               (unsigned int)ec_slave[slave].Ibytes, (unsigned int)ec_slave[slave].Ibits,
               (unsigned int)status_word, console_ethercat_cia402_state_name(status_word));
        if (ec_slave[slave].outputs != NULL) {
            printf(" PDO_out_raw=[");
            console_ethercat_print_bytes((const uint8_t *)ec_slave[slave].outputs,
                                         ec_slave[slave].Obytes);
            printf("]");
        }
        if (ec_slave[slave].inputs != NULL) {
            printf(" PDO_in_raw=[");
            console_ethercat_print_bytes((const uint8_t *)ec_slave[slave].inputs,
                                         ec_slave[slave].Ibytes);
            printf("]");
        }

        size = (int)sizeof(mode_display);
        if (ec_SDOread((uint16)slave, 0x6061u, 0u, FALSE, &size, &mode_display,
                       EC_TIMEOUTRXM) > 0 && size == (int)sizeof(mode_display)) {
            printf(" mode_display=%d", (int)mode_display);
        } else {
            printf(" mode_display=unread");
        }
        size = (int)sizeof(error_code);
        if (ec_SDOread((uint16)slave, 0x603fu, 0u, FALSE, &size, &error_code,
                       EC_TIMEOUTRXM) > 0 && size == (int)sizeof(error_code)) {
            printf(" error_code=0x%04x", (unsigned int)error_code);
        } else {
            printf(" error_code=unread");
        }
        size = (int)sizeof(pn077);
        if (ec_SDOread((uint16)slave, 0x2077u, 0u, FALSE, &size, &pn077,
                       EC_TIMEOUTRXM) > 0 && size == (int)sizeof(pn077)) {
            printf(" Pn077=%u", (unsigned int)pn077);
        } else {
            printf(" Pn077=unread");
        }
        size = (int)sizeof(pn078);
        if (ec_SDOread((uint16)slave, 0x2078u, 0u, FALSE, &size, &pn078,
                       EC_TIMEOUTRXM) > 0 && size == (int)sizeof(pn078)) {
            printf(" Pn078=%u", (unsigned int)pn078);
        } else {
            printf(" Pn078=unread");
        }
        size = (int)sizeof(sync_type);
        if (ec_SDOread((uint16)slave, 0x1c32u, 1u, FALSE, &size, &sync_type,
                       EC_TIMEOUTRXM) > 0 && size == (int)sizeof(sync_type)) {
            printf(" SM2_sync_type=0x%04x", (unsigned int)sync_type);
        } else {
            printf(" SM2_sync_type=unread");
        }
        size = (int)sizeof(sync_cycle_ns);
        if (ec_SDOread((uint16)slave, 0x1c32u, 2u, FALSE, &size, &sync_cycle_ns,
                       EC_TIMEOUTRXM) > 0 && size == (int)sizeof(sync_cycle_ns)) {
            printf(" SM2_cycle_ns=%u", (unsigned int)sync_cycle_ns);
        } else {
            printf(" SM2_cycle_ns=unread");
        }
        size = (int)sizeof(tx_sync_type);
        if (ec_SDOread((uint16)slave, 0x1c33u, 1u, FALSE, &size, &tx_sync_type,
                       EC_TIMEOUTRXM) > 0 && size == (int)sizeof(tx_sync_type)) {
            printf(" SM3_sync_type=0x%04x", (unsigned int)tx_sync_type);
        } else {
            printf(" SM3_sync_type=unread");
        }
        size = (int)sizeof(tx_sync_cycle_ns);
        if (ec_SDOread((uint16)slave, 0x1c33u, 2u, FALSE, &size, &tx_sync_cycle_ns,
                       EC_TIMEOUTRXM) > 0 && size == (int)sizeof(tx_sync_cycle_ns)) {
            printf(" SM3_cycle_ns=%u", (unsigned int)tx_sync_cycle_ns);
        } else {
            printf(" SM3_cycle_ns=unread");
        }
        if (ec_FPRD(ec_slave[slave].configadr, 0x0981u, (uint16)sizeof(dc_activation),
                    &dc_activation, EC_TIMEOUTRET) > 0) {
            printf(" DC_activation=0x%02x", (unsigned int)dc_activation);
        } else {
            printf(" DC_activation=unread");
        }
        if (ec_FPRD(ec_slave[slave].configadr, 0x09a0u, (uint16)sizeof(dc_cycle_data),
                    dc_cycle_data, EC_TIMEOUTRET) > 0) {
            dc_cycle_ns = (uint32_t)dc_cycle_data[0] |
                          ((uint32_t)dc_cycle_data[1] << 8u) |
                          ((uint32_t)dc_cycle_data[2] << 16u) |
                          ((uint32_t)dc_cycle_data[3] << 24u);
            printf(" DC_cycle_ns=%u", (unsigned int)dc_cycle_ns);
        } else {
            printf(" DC_cycle_ns=unread");
        }
        if (ec_FPRD(ec_slave[slave].configadr, 0x0990u, (uint16)sizeof(dc_start_data),
                    dc_start_data, EC_TIMEOUTRET) > 0) {
            printf(" DC_start_raw=[");
            console_ethercat_print_bytes(dc_start_data, (unsigned int)sizeof(dc_start_data));
            printf("]");
        } else {
            printf(" DC_start_raw=unread");
        }
        printf("\n");
    }
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
        console_ethercat_wait_next_cycle();
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
        console_ethercat_wait_next_cycle();
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
                                   unsigned int hold_ms,
                                   const char *sync0_shift_text)
{
    unsigned int slave_id;
    bool all_slaves;
    int32_t sync0_shift_ns = 0;

    if (console_ethercat_validate_interface(interface) != 0 ||
        console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0 ||
        hold_ms > ETHERCAT_KAIXUAN_ENABLE_MAX_HOLD_MS ||
        (sync0_shift_text != NULL &&
         console_ethercat_parse_sync0_shift(sync0_shift_text, &sync0_shift_ns) != 0)) {
        printf("%s: ethercat_enable <network_interface> <slave_id|all> [hold_ms:0..%u] [sync0_shift_ns:-4000000..4000000]\n",
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
        printf("%s: interface=%s slave=%s hold=until-Ctrl-C sync0_cycle_ns=%u sync0_shift_ns=%d\n",
               chinese ? "ethercat_enable: 开始" : "ethercat_enable: start",
               interface, selection, ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS,
               (int)sync0_shift_ns);
    } else {
        printf("%s: interface=%s slave=%s hold_ms=%u sync0_cycle_ns=%u sync0_shift_ns=%d\n",
               chinese ? "ethercat_enable: 开始" : "ethercat_enable: start",
               interface, selection, hold_ms, ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS,
               (int)sync0_shift_ns);
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
    console_ethercat_reset_exchange_diagnostics();
    ethercat_expected_work_counter =
        ((int)ec_group[0].outputsWKC * 2) + (int)ec_group[0].inputsWKC;
    ec_configdc();
    if ((ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) & 0x0fu) != EC_STATE_SAFE_OP) {
        printf("%s\n", chinese ? "ethercat_enable: 从站未进入 SAFE-OP" :
               "ethercat_enable: slaves did not reach SAFE-OP");
        goto cleanup;
    }
    if (console_ethercat_enable_dc_sync(selected, sync0_shift_ns) != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 目标从站不支持 DC Sync0" :
               "ethercat_enable: selected slave does not support DC Sync0");
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
        console_ethercat_wait_next_cycle();
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
        console_ethercat_print_selected_status(selected, 0x0006u);
        goto cleanup;
    }
    console_ethercat_set_control_word(selected, 0x0007u);
    if (console_ethercat_wait_for_status(selected, 0x0023u) != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 0x0007 状态确认失败" :
               "ethercat_enable: 0x0007 state confirmation failed");
        console_ethercat_print_selected_status(selected, 0x0007u);
        goto cleanup;
    }
    console_ethercat_set_control_word(selected, 0x000fu);
    if (console_ethercat_wait_for_status(selected, 0x0027u) != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 0x000F 状态确认失败" :
               "ethercat_enable: 0x000F state confirmation failed");
        console_ethercat_print_selected_status(selected, 0x000fu);
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
        console_ethercat_wait_next_cycle();
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

static int console_ethercat_position(bool chinese,
                                     const char *interface,
                                     const char *selection,
                                     double position_rad,
                                     unsigned int hold_ms)
{
    unsigned int slave_id;
    bool all_slaves;

    if (console_ethercat_validate_interface(interface) != 0 ||
        console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0 ||
        !isfinite(position_rad) || hold_ms > ETHERCAT_KAIXUAN_ENABLE_MAX_HOLD_MS) {
        printf("%s: ethercat_position <network_interface> <slave_id|all> <target_rad> [hold_ms:1..%u]\n",
               chinese ? "用法" : "usage", ETHERCAT_KAIXUAN_ENABLE_MAX_HOLD_MS);
        return -1;
    }

#ifndef HAVE_SOEM
    printf("%s\n", chinese ?
           "当前程序未编译 SOEM，无法执行 EtherCAT 位控。从 SOEM 源码编译后，执行 make "
           "FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" 重新构建。" :
           "EtherCAT position control is unavailable because this build has no SOEM support. "
           "Build again with make FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" after building SOEM.");
    return -1;
#else
    uint8_t process_image[ETHERCAT_KAIXUAN_PROCESS_IMAGE_SIZE];
    uint8_t selected[EC_MAXSLAVE] = {0};
    int32_t target_positions[EC_MAXSLAVE];
    double position_count_double;
    uint64_t deadline;
    long long position_count_long;
    int slave;
    int32_t position_count;
    int result = -1;
    bool mapped = false;

    position_count_double = position_rad * ETHERCAT_KAIXUAN_COUNTS_PER_REV /
                            ETHERCAT_KAIXUAN_TWO_PI;
    if (position_count_double < (double)INT32_MIN || position_count_double > (double)INT32_MAX) {
        printf("%s\n", chinese ? "ethercat_position: 目标弧度超出 int32 位置范围" :
               "ethercat_position: target radians exceed int32 position range");
        return -1;
    }
    position_count_long = llround(position_count_double);
    position_count = (int32_t)position_count_long;
    for (slave = 0; slave < EC_MAXSLAVE; slave++) {
        target_positions[slave] = position_count;
    }
    if (hold_ms == 0u) {
        printf("%s: interface=%s slave=%s target_rad=%.6f target_count=%d hold=until-Ctrl-C\n",
               chinese ? "ethercat_position: 开始" : "ethercat_position: start",
               interface, selection, position_rad, position_count);
    } else {
        printf("%s: interface=%s slave=%s target_rad=%.6f target_count=%d hold_ms=%u\n",
               chinese ? "ethercat_position: 开始" : "ethercat_position: start",
               interface, selection, position_rad, position_count, hold_ms);
    }
    if (ec_init((char *)interface) == 0) {
        printf("%s: %s\n", chinese ? "ethercat_position: 打开网卡失败" :
               "ethercat_position: failed to open interface", interface);
        return -1;
    }
    if (ec_config_init(FALSE) <= 0) {
        printf("%s\n", chinese ? "ethercat_position: 未发现 EtherCAT 从站" :
               "ethercat_position: no EtherCAT slaves found");
        goto close_socket;
    }
    if (!all_slaves && slave_id > (unsigned int)ec_slavecount) {
        printf("%s: %u (1..%d)\n", chinese ? "ethercat_position: 从站序号不存在" :
               "ethercat_position: slave id is out of range", slave_id, ec_slavecount);
        goto close_socket;
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (all_slaves || (unsigned int)slave == slave_id) {
            selected[slave] = 1u;
        }
    }

    ec_config_map(process_image);
    mapped = true;
    console_ethercat_reset_exchange_diagnostics();
    ethercat_expected_work_counter =
        ((int)ec_group[0].outputsWKC * 2) + (int)ec_group[0].inputsWKC;
    ec_configdc();
    if ((ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) & 0x0fu) != EC_STATE_SAFE_OP) {
        printf("%s\n", chinese ? "ethercat_position: 从站未进入 SAFE-OP" :
               "ethercat_position: slaves did not reach SAFE-OP");
        goto cleanup;
    }
    if (console_ethercat_enable_dc_sync(selected, 0) != 0) {
        printf("%s\n", chinese ? "ethercat_position: 目标从站不支持 DC Sync0" :
               "ethercat_position: selected slave does not support DC Sync0");
        goto cleanup;
    }
    if (console_ethercat_selected_ready(selected, target_positions) != 0) {
        printf("%s\n", chinese ?
               "ethercat_position: PDO 映射不是当前开璇驱动器要求的 13B 输出/14B 输入，已拒绝控制" :
               "ethercat_position: PDO mapping is not the required Kaixuan 13B output/14B input layout; control refused");
        goto cleanup;
    }
    if (console_ethercat_exchange() != 0 || console_ethercat_exchange() != 0 ||
        console_ethercat_exchange() != 0) {
        printf("%s\n", chinese ? "ethercat_position: 初始 PDO 通信失败" :
               "ethercat_position: initial PDO exchange failed");
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
            break;
        }
        sleep_ms(ETHERCAT_KAIXUAN_CONTROL_PERIOD_MS);
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] != 0u &&
            (ec_statecheck((uint16)slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET) & 0x0fu) !=
            EC_STATE_OPERATIONAL) {
            printf("%s: %d\n", chinese ? "ethercat_position: 从站未进入 OP" :
                   "ethercat_position: slave did not reach OP", slave);
            goto cleanup;
        }
    }
    console_ethercat_set_control_word(selected, 0x0006u);
    if (console_ethercat_wait_for_status(selected, 0x0021u) != 0) {
        printf("%s\n", chinese ? "ethercat_position: 0x0006 状态确认失败" :
               "ethercat_position: 0x0006 state confirmation failed");
        console_ethercat_print_selected_status(selected, 0x0006u);
        goto cleanup;
    }
    console_ethercat_set_control_word(selected, 0x0007u);
    if (console_ethercat_wait_for_status(selected, 0x0023u) != 0) {
        printf("%s\n", chinese ? "ethercat_position: 0x0007 状态确认失败" :
               "ethercat_position: 0x0007 state confirmation failed");
        console_ethercat_print_selected_status(selected, 0x0007u);
        goto cleanup;
    }
    console_ethercat_set_control_word(selected, 0x000fu);
    if (console_ethercat_wait_for_status(selected, 0x0027u) != 0) {
        printf("%s\n", chinese ? "ethercat_position: 0x000F 状态确认失败" :
               "ethercat_position: 0x000F state confirmation failed");
        console_ethercat_print_selected_status(selected, 0x000fu);
        goto cleanup;
    }
    printf("%s\n", chinese ?
           "ethercat_position: 已进入 CSP 位控并持续发送目标位置；Ctrl-C 时失能。" :
           "ethercat_position: CSP position control is active and target positions are being sent; Ctrl-C disables.");
    deadline = hold_ms == 0u ? UINT64_MAX : time_us() + (uint64_t)hold_ms * 1000u;
    while (!stop_requested && time_us() < deadline) {
        if (console_ethercat_exchange() != 0) {
            printf("%s\n", chinese ? "ethercat_position: PDO 通信中断" :
                   "ethercat_position: PDO communication lost");
            goto cleanup;
        }
        sleep_ms(ETHERCAT_KAIXUAN_CONTROL_PERIOD_MS);
    }
    result = stop_requested ? -1 : 0;

cleanup:
    if (mapped) {
        console_ethercat_disable_selected(selected);
    }
    printf("%s\n", chinese ? "ethercat_position: 已发送失能并关闭 EtherCAT 主站" :
           "ethercat_position: disable sent and EtherCAT master closed");
close_socket:
    ec_close();
    return result;
#endif
}

static int console_ethercat_zero(bool chinese, const char *interface, const char *selection)
{
    unsigned int slave_id;
    bool all_slaves;

    if (console_ethercat_validate_interface(interface) != 0 ||
        console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0) {
        printf("%s: ethercat_zero <network_interface> <slave_id|all>\n",
               chinese ? "用法" : "usage");
        return -1;
    }

#ifndef HAVE_SOEM
    printf("%s\n", chinese ?
           "当前程序未编译 SOEM，无法设置 EtherCAT 零位。从 SOEM 源码编译后，执行 make "
           "FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" 重新构建。" :
           "EtherCAT zero setting is unavailable because this build has no SOEM support. "
           "Build again with make FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" after building SOEM.");
    return -1;
#else
    uint8_t process_image[ETHERCAT_KAIXUAN_PROCESS_IMAGE_SIZE];
    uint8_t selected[EC_MAXSLAVE] = {0};
    int32_t zero_positions[EC_MAXSLAVE] = {0};
    int16_t parameter_value;
    int slave;
    int failed = 0;
    bool mapped = false;
    bool operational = false;

    printf("%s: interface=%s slave=%s\n",
           chinese ? "ethercat_zero: 开始" : "ethercat_zero: start", interface, selection);
    if (ec_init((char *)interface) == 0) {
        printf("%s: %s\n", chinese ? "ethercat_zero: 打开网卡失败" :
               "ethercat_zero: failed to open interface", interface);
        return -1;
    }
    if (ec_config_init(FALSE) <= 0) {
        printf("%s\n", chinese ? "ethercat_zero: 未发现 EtherCAT 从站" :
               "ethercat_zero: no EtherCAT slaves found");
        goto close_socket;
    }
    if (!all_slaves && slave_id > (unsigned int)ec_slavecount) {
        printf("%s: %u (1..%d)\n", chinese ? "ethercat_zero: 从站序号不存在" :
               "ethercat_zero: slave id is out of range", slave_id, ec_slavecount);
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
    if ((ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) & 0x0fu) != EC_STATE_SAFE_OP ||
        console_ethercat_selected_ready(selected, zero_positions) != 0 ||
        console_ethercat_exchange() != 0) {
        printf("%s\n", chinese ? "ethercat_zero: PDO 映射或 SAFE-OP 初始化失败" :
               "ethercat_zero: PDO mapping or SAFE-OP initialization failed");
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
        printf("%s\n", chinese ? "ethercat_zero: 从站未进入 OP，已拒绝写入 Pn101" :
               "ethercat_zero: slave did not reach OP; Pn101 write refused");
        goto cleanup;
    }
    console_ethercat_set_control_word(selected, 0u);
    if (console_ethercat_wait_for_status(selected, 0x0040u) != 0) {
        printf("%s\n", chinese ? "ethercat_zero: 伺服失能状态确认失败" :
               "ethercat_zero: servo-disable state confirmation failed");
        goto cleanup;
    }
    console_ethercat_disable_selected(selected);
    operational = false;
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] == 0u) {
            continue;
        }
        parameter_value = 0;
        if (ec_SDOwrite((uint16)slave, 0x2101u, 0u, FALSE, (int)sizeof(parameter_value),
                        &parameter_value, EC_TIMEOUTRXM) <= 0) {
            failed++;
            continue;
        }
        parameter_value = 1;
        if (ec_SDOwrite((uint16)slave, 0x2101u, 0u, FALSE, (int)sizeof(parameter_value),
                        &parameter_value, EC_TIMEOUTRXM) <= 0) {
            failed++;
            continue;
        }
        parameter_value = 0;
        if (ec_SDOwrite((uint16)slave, 0x2101u, 0u, FALSE, (int)sizeof(parameter_value),
                        &parameter_value, EC_TIMEOUTRXM) <= 0) {
            failed++;
            continue;
        }
        printf("[slave%d]: Pn101 0->1->0 sent\n", slave);
    }
    if (failed == 0) {
        printf("%s\n", chinese ?
               "ethercat_zero: Pn101 已写入。请同时重启执行器主电和 USB 电源，零位才会生效。" :
               "ethercat_zero: Pn101 was written. Restart both actuator main power and USB power for zero to take effect.");
    } else {
        printf("%s: failed=%d\n", chinese ? "ethercat_zero: Pn101 写入失败" :
               "ethercat_zero: Pn101 write failed", failed);
    }

cleanup:
    if (mapped && operational) {
        console_ethercat_disable_selected(selected);
    }
close_socket:
    ec_close();
    return failed == 0 && !stop_requested ? 0 : -1;
#endif
}

static int console_ethercat_info(bool chinese, const char *interface, const char *selection)
{
    unsigned int slave_id;
    bool all_slaves;

    if (console_ethercat_validate_interface(interface) != 0 ||
        console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0) {
        printf("%s: ethercat_info <network_interface> <slave_id|all>\n",
               chinese ? "用法" : "usage");
        return -1;
    }

#ifndef HAVE_SOEM
    printf("%s\n", chinese ?
           "当前程序未编译 SOEM，无法读取 EtherCAT 电机信息。从 SOEM 源码编译后，执行 make "
           "FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" 重新构建。" :
           "EtherCAT info is unavailable because this build has no SOEM support. Build again "
           "with make FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" after building SOEM.");
    return -1;
#else
    int slave;
    int failed = 0;

    printf("%s: interface=%s slave=%s\n",
           chinese ? "ethercat_info: 开始读取" : "ethercat_info: read start", interface, selection);
    if (ec_init((char *)interface) == 0) {
        printf("%s: %s\n", chinese ? "ethercat_info: 打开网卡失败" :
               "ethercat_info: failed to open interface", interface);
        return -1;
    }
    if (ec_config_init(FALSE) <= 0) {
        printf("%s\n", chinese ? "ethercat_info: 未发现 EtherCAT 从站" :
               "ethercat_info: no EtherCAT slaves found");
        goto close_socket;
    }
    if (!all_slaves && slave_id > (unsigned int)ec_slavecount) {
        printf("%s: %u (1..%d)\n", chinese ? "ethercat_info: 从站序号不存在" :
               "ethercat_info: slave id is out of range", slave_id, ec_slavecount);
        goto close_socket;
    }
    ec_readstate();
    for (slave = 1; slave <= ec_slavecount; slave++) {
        uint16_t control_word;
        uint16_t status_word;
        uint16_t error_code;
        uint16_t sm2_sync_type;
        uint16_t sm3_sync_type;
        int8_t mode_command;
        int8_t mode_display;
        int16_t torque_actual;
        int16_t current_actual;
        int16_t target_torque;
        int16_t pn001;
        int16_t pn002;
        int16_t pn070;
        int16_t pn075;
        int16_t pn077;
        int16_t pn079;
        int16_t pn085;
        int16_t pn088;
        int32_t position_actual;
        int32_t velocity_actual;
        int32_t target_position;
        int32_t target_velocity;
        int32_t monitor_value;
        uint32_t sm2_cycle_ns;
        uint32_t sm3_cycle_ns;
        uint32_t digital_inputs;
        uint8_t dc_activation;
        uint8_t dc_cycle_data[4];
        bool control_word_ok;
        bool status_word_ok;
        bool error_code_ok;
        bool mode_command_ok;
        bool mode_display_ok;
        bool position_actual_ok;
        bool velocity_actual_ok;
        bool torque_actual_ok;
        bool current_actual_ok;
        bool target_position_ok;
        bool target_velocity_ok;
        bool target_torque_ok;
        bool digital_inputs_ok;
        bool pn001_ok;
        bool pn002_ok;
        bool pn070_ok;
        bool pn075_ok;
        bool pn077_ok;
        bool pn079_ok;
        bool pn085_ok;
        bool pn088_ok;
        bool monitor_value_ok;
        bool sm2_sync_type_ok;
        bool sm2_cycle_ns_ok;
        bool sm3_sync_type_ok;
        bool sm3_cycle_ns_ok;
        bool dc_activation_ok;
        bool dc_cycle_ok;
        int size;

        if (!all_slaves && (unsigned int)slave != slave_id) {
            continue;
        }
        printf("[slave%d] %s state=0x%02x(%s) addr=0x%04x identity=%08x:%08x rev=%08x\n",
               slave, ec_slave[slave].name, (unsigned int)ec_slave[slave].state,
               console_ethercat_state_name(ec_slave[slave].state),
               (unsigned int)ec_slave[slave].configadr,
               (unsigned int)ec_slave[slave].eep_man, (unsigned int)ec_slave[slave].eep_id,
               (unsigned int)ec_slave[slave].eep_rev);

#define ETHERCAT_INFO_READ(index, value) \
        (size = (int)sizeof(value), \
         ec_SDOread((uint16)slave, (index), 0u, FALSE, &size, &(value), EC_TIMEOUTRXM) > 0 && \
         size == (int)sizeof(value))

        control_word_ok = ETHERCAT_INFO_READ(0x6040u, control_word);
        status_word_ok = ETHERCAT_INFO_READ(0x6041u, status_word);
        error_code_ok = ETHERCAT_INFO_READ(0x603fu, error_code);
        mode_command_ok = ETHERCAT_INFO_READ(0x6060u, mode_command);
        mode_display_ok = ETHERCAT_INFO_READ(0x6061u, mode_display);
        position_actual_ok = ETHERCAT_INFO_READ(0x6064u, position_actual);
        velocity_actual_ok = ETHERCAT_INFO_READ(0x606cu, velocity_actual);
        torque_actual_ok = ETHERCAT_INFO_READ(0x6077u, torque_actual);
        current_actual_ok = ETHERCAT_INFO_READ(0x6078u, current_actual);
        target_position_ok = ETHERCAT_INFO_READ(0x607au, target_position);
        target_velocity_ok = ETHERCAT_INFO_READ(0x60ffu, target_velocity);
        target_torque_ok = ETHERCAT_INFO_READ(0x6071u, target_torque);
        digital_inputs_ok = ETHERCAT_INFO_READ(0x60fdu, digital_inputs);
        pn001_ok = ETHERCAT_INFO_READ(0x2001u, pn001);
        pn002_ok = ETHERCAT_INFO_READ(0x2002u, pn002);
        pn070_ok = ETHERCAT_INFO_READ(0x2070u, pn070);
        pn075_ok = ETHERCAT_INFO_READ(0x2075u, pn075);
        pn077_ok = ETHERCAT_INFO_READ(0x2077u, pn077);
        pn079_ok = ETHERCAT_INFO_READ(0x2079u, pn079);
        pn085_ok = ETHERCAT_INFO_READ(0x2085u, pn085);
        pn088_ok = ETHERCAT_INFO_READ(0x2088u, pn088);
        monitor_value_ok = ETHERCAT_INFO_READ(0x3000u, monitor_value);
        sm2_sync_type_ok = (size = (int)sizeof(sm2_sync_type),
            ec_SDOread((uint16)slave, 0x1c32u, 1u, FALSE, &size, &sm2_sync_type,
                       EC_TIMEOUTRXM) > 0 && size == (int)sizeof(sm2_sync_type));
        sm2_cycle_ns_ok = (size = (int)sizeof(sm2_cycle_ns),
            ec_SDOread((uint16)slave, 0x1c32u, 2u, FALSE, &size, &sm2_cycle_ns,
                       EC_TIMEOUTRXM) > 0 && size == (int)sizeof(sm2_cycle_ns));
        sm3_sync_type_ok = (size = (int)sizeof(sm3_sync_type),
            ec_SDOread((uint16)slave, 0x1c33u, 1u, FALSE, &size, &sm3_sync_type,
                       EC_TIMEOUTRXM) > 0 && size == (int)sizeof(sm3_sync_type));
        sm3_cycle_ns_ok = (size = (int)sizeof(sm3_cycle_ns),
            ec_SDOread((uint16)slave, 0x1c33u, 2u, FALSE, &size, &sm3_cycle_ns,
                       EC_TIMEOUTRXM) > 0 && size == (int)sizeof(sm3_cycle_ns));
        dc_activation_ok = ec_FPRD(ec_slave[slave].configadr, 0x0981u,
                                   (uint16)sizeof(dc_activation),
                                   &dc_activation, EC_TIMEOUTRET) > 0;
        dc_cycle_ok = ec_FPRD(ec_slave[slave].configadr, 0x09a0u,
                              (uint16)sizeof(dc_cycle_data),
                              dc_cycle_data, EC_TIMEOUTRET) > 0;

        printf("  drive: Pn001=");
        if (pn001_ok) printf("%d", (int)pn001); else printf("?");
        printf(" Pn002=");
        if (pn002_ok) printf("%d", (int)pn002); else printf("?");
        printf(" Pn070=");
        if (pn070_ok) printf("%d", (int)pn070); else printf("?");
        printf(" Pn075=");
        if (pn075_ok) printf("%d", (int)pn075); else printf("?");
        printf(" Pn077=");
        if (pn077_ok) printf("%d", (int)pn077); else printf("?");
        printf("\n  params: Pn079=");
        if (pn079_ok) printf("%d", (int)pn079); else printf("?");
        printf(" Pn085=");
        if (pn085_ok) printf("%d (0.01A)", (int)pn085); else printf("?");
        printf(" Pn088=");
        if (pn088_ok) printf("%d(%s)", (int)pn088,
                             pn088 == 0 ? "rpm" : (pn088 == 1 ? "count/s" : "unit?"));
        else printf("?");
        printf("\n  bus: PDO out=%uB/%ubit in=%uB/%ubit DC=%s active=%u actreg=",
               (unsigned int)ec_slave[slave].Obytes, (unsigned int)ec_slave[slave].Obits,
               (unsigned int)ec_slave[slave].Ibytes, (unsigned int)ec_slave[slave].Ibits,
               ec_slave[slave].hasdc ? "yes" : "no",
               (unsigned int)ec_slave[slave].DCactive);
        if (dc_activation_ok) printf("0x%02x", (unsigned int)dc_activation);
        else printf("?");
        printf(" cycle=");
        if (dc_cycle_ok) {
            uint32_t dc_cycle_ns = (uint32_t)dc_cycle_data[0] |
                                   ((uint32_t)dc_cycle_data[1] << 8u) |
                                   ((uint32_t)dc_cycle_data[2] << 16u) |
                                   ((uint32_t)dc_cycle_data[3] << 24u);
            printf("%uns", (unsigned int)dc_cycle_ns);
        } else {
            printf("?");
        }
        printf(" shift=%d\n  sync: SM2 type=", (int)ec_slave[slave].DCshift);
        if (sm2_sync_type_ok) printf("0x%04x", (unsigned int)sm2_sync_type);
        else printf("?");
        printf(" cycle=");
        if (sm2_cycle_ns_ok) printf("%uns", (unsigned int)sm2_cycle_ns);
        else printf("?");
        printf(" | SM3 type=");
        if (sm3_sync_type_ok) printf("0x%04x", (unsigned int)sm3_sync_type);
        else printf("?");
        printf(" cycle=");
        if (sm3_cycle_ns_ok) printf("%uns\n", (unsigned int)sm3_cycle_ns);
        else printf("?\n");

        printf("  state: CW=");
        if (control_word_ok) printf("0x%04x", (unsigned int)control_word); else printf("?");
        printf(" SW=");
        if (status_word_ok) printf("0x%04x", (unsigned int)status_word); else printf("?");
        printf(" error=");
        if (error_code_ok) printf("0x%04x", (unsigned int)error_code); else printf("?");
        printf(" mode=");
        if (mode_command_ok) printf("%d", (int)mode_command); else printf("?");
        printf("/");
        if (mode_display_ok) printf("%d\n", (int)mode_display); else printf("?\n");

        printf("  actual: pos=");
        if (position_actual_ok) {
            printf("%.6frad (%d count)",
                   (double)position_actual * ETHERCAT_KAIXUAN_TWO_PI /
                   ETHERCAT_KAIXUAN_COUNTS_PER_REV, position_actual);
        } else printf("?");
        printf(" vel=");
        if (velocity_actual_ok) {
            if (pn088_ok && pn088 == 0) {
                printf("%.6frad/s (%d rpm)",
                       (double)velocity_actual * ETHERCAT_KAIXUAN_TWO_PI / 60.0,
                       velocity_actual);
            } else if (pn088_ok && pn088 == 1) {
                printf("%.6frad/s (%d count/s)",
                       (double)velocity_actual * ETHERCAT_KAIXUAN_TWO_PI /
                       ETHERCAT_KAIXUAN_COUNTS_PER_REV, velocity_actual);
            } else {
                printf("%d (unit unknown)", velocity_actual);
            }
        } else printf("?");
        printf(" torque=");
        if (torque_actual_ok) printf("%d (0.01A)", (int)torque_actual); else printf("?");
        printf(" current=");
        if (current_actual_ok) printf("%d (0.01A)\n", (int)current_actual);
        else printf("?\n");
        printf("  target: pos=");
        if (target_position_ok) {
            printf("%.6frad (%d count)",
                   (double)target_position * ETHERCAT_KAIXUAN_TWO_PI /
                   ETHERCAT_KAIXUAN_COUNTS_PER_REV, target_position);
        } else printf("?");
        printf(" vel=");
        if (target_velocity_ok) printf("%d raw", target_velocity); else printf("?");
        printf(" torque=");
        if (target_torque_ok) printf("%d (0.01A)", (int)target_torque); else printf("?");
        printf(" inputs=");
        if (digital_inputs_ok) printf("0x%08x", (unsigned int)digital_inputs); else printf("?");
        printf(" monitor=");
        if (monitor_value_ok) printf("%d(Pn002)", monitor_value); else printf("?");
        printf("\n");
#undef ETHERCAT_INFO_READ
    }
    printf("%s\n", chinese ?
           "ethercat_info: 只读 SDO 查询完成；未配置 PDO、未请求 OP、未发送使能或运动命令。" :
           "ethercat_info: read-only SDO query complete; did not configure PDOs, request OP, or send enable/motion commands.");
    ec_close();
    return failed == 0 ? 0 : -1;

close_socket:
    ec_close();
    return -1;
#endif
}

static int console_ethercat_pn077(bool chinese,
                                  const char *interface,
                                  const char *selection,
                                  unsigned int value)
{
    unsigned int slave_id;
    bool all_slaves;

    if (console_ethercat_validate_interface(interface) != 0 ||
        console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0 ||
        value > 1u) {
        printf("%s: ethercat_pn077 <network_interface> <slave_id|all> <0|1>\n",
               chinese ? "用法" : "usage");
        return -1;
    }

#ifndef HAVE_SOEM
    printf("%s\n", chinese ?
           "当前程序未编译 SOEM，无法写入 Pn077。从 SOEM 源码编译后，执行 make FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" 重新构建。" :
           "Pn077 write is unavailable because this build has no SOEM support. Rebuild with make FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\".");
    return -1;
#else
    uint16_t selected_status[EC_MAXSLAVE] = {0};
    uint8_t selected[EC_MAXSLAVE] = {0};
    int16_t current_value[EC_MAXSLAVE] = {0};
    int16_t requested_value = (int16_t)value;
    int16_t readback_value;
    int slave;
    int size;

    printf("%s: interface=%s slave=%s value=%u\n",
           chinese ? "ethercat_pn077: 开始" : "ethercat_pn077: start",
           interface, selection, value);
    if (ec_init((char *)interface) == 0) {
        printf("%s: %s\n", chinese ? "ethercat_pn077: 打开网卡失败" :
               "ethercat_pn077: failed to open interface", interface);
        return -1;
    }
    if (ec_config_init(FALSE) <= 0) {
        printf("%s\n", chinese ? "ethercat_pn077: 未发现 EtherCAT 从站" :
               "ethercat_pn077: no EtherCAT slaves found");
        goto close_socket;
    }
    if (!all_slaves && slave_id > (unsigned int)ec_slavecount) {
        printf("%s: %u (1..%d)\n", chinese ? "ethercat_pn077: 从站序号不存在" :
               "ethercat_pn077: slave id is out of range", slave_id, ec_slavecount);
        goto close_socket;
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (all_slaves || (unsigned int)slave == slave_id) {
            selected[slave] = 1u;
        }
    }

    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] == 0u) {
            continue;
        }
        if (ec_slave[slave].eep_man != ETHERCAT_KAIXUAN_VENDOR_ID ||
            ec_slave[slave].eep_id != ETHERCAT_KAIXUAN_PRODUCT_CODE) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "不是受支持的开璇驱动器，拒绝写入" :
                   "not a supported Kaixuan drive; refusing write");
            goto close_socket;
        }
        size = (int)sizeof(selected_status[slave]);
        if (ec_SDOread((uint16)slave, 0x6041u, 0u, FALSE, &size,
                       &selected_status[slave], EC_TIMEOUTRXM) <= 0 ||
            size != (int)sizeof(selected_status[slave])) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "读取 0x6041 状态字失败，拒绝写入" :
                   "failed to read 0x6041 statusword; refusing write");
            goto close_socket;
        }
        if ((selected_status[slave] & 0x006fu) == 0x0021u ||
            (selected_status[slave] & 0x006fu) == 0x0023u ||
            (selected_status[slave] & 0x006fu) == 0x0027u ||
            (selected_status[slave] & 0x006fu) == 0x0007u) {
            printf("[slave%d]: status_word=0x%04x %s\n", slave,
                   (unsigned int)selected_status[slave], chinese ?
                   "驱动器未处于失能状态，拒绝写入 Pn077" :
                   "drive is not disabled; refusing Pn077 write");
            goto close_socket;
        }
        size = (int)sizeof(current_value[slave]);
        if (ec_SDOread((uint16)slave, 0x2077u, 0u, FALSE, &size,
                       &current_value[slave], EC_TIMEOUTRXM) <= 0 ||
            size != (int)sizeof(current_value[slave])) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "读取当前 Pn077 失败，拒绝写入" :
                   "failed to read current Pn077; refusing write");
            goto close_socket;
        }
    }

    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] == 0u) {
            continue;
        }
        size = (int)sizeof(requested_value);
        if (ec_SDOwrite((uint16)slave, 0x2077u, 0u, FALSE, size,
                        &requested_value, EC_TIMEOUTRXM) <= 0) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "写入 Pn077 失败" : "Pn077 write failed");
            goto close_socket;
        }
        size = (int)sizeof(readback_value);
        if (ec_SDOread((uint16)slave, 0x2077u, 0u, FALSE, &size,
                       &readback_value, EC_TIMEOUTRXM) <= 0 ||
            size != (int)sizeof(readback_value) || readback_value != requested_value) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "Pn077 写入后回读不匹配" : "Pn077 readback mismatch");
            goto close_socket;
        }
        printf("[slave%d]: Pn077 %d -> %d (readback=%d)\n", slave,
               (int)current_value[slave], (int)requested_value,
               (int)readback_value);
    }
    printf("%s\n", chinese ?
           "Pn077 已写入并回读。根据手册，需重启执行器后生效；该参数不是主站 Sync0 shift。" :
           "Pn077 was written and verified. Restart the actuator for it to take effect; this is not the master Sync0 shift.");
    ec_close();
    return 0;

close_socket:
    ec_close();
    return -1;
#endif
}

static int console_ethercat_save(bool chinese,
                                 const char *interface,
                                 const char *selection)
{
    unsigned int slave_id;
    bool all_slaves;

    if (console_ethercat_validate_interface(interface) != 0 ||
        console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0) {
        printf("%s: ethercat_save <network_interface> <slave_id|all>\n",
               chinese ? "用法" : "usage");
        return -1;
    }

#ifndef HAVE_SOEM
    printf("%s\n", chinese ?
           "当前程序未编译 SOEM，无法保存 EtherCAT 参数。从 SOEM 源码编译后，执行 make FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\" 重新构建。" :
           "EtherCAT parameter save is unavailable because this build has no SOEM support. Rebuild with make FLAGS_USER=\"-DSOEM_ROOT=$HOME/SOEM-v1.4.0\".");
    return -1;
#else
    uint16_t selected_status[EC_MAXSLAVE] = {0};
    uint8_t selected[EC_MAXSLAVE] = {0};
    int16_t save_parameter[EC_MAXSLAVE] = {0};
    int16_t save_on = 1;
    int16_t save_off = 0;
    int slave;
    int size;

    printf("%s: interface=%s slave=%s object=0x2097:00 sequence=1->0\n",
           chinese ? "ethercat_save: 开始" : "ethercat_save: start",
           interface, selection);
    if (ec_init((char *)interface) == 0) {
        printf("%s: %s\n", chinese ? "ethercat_save: 打开网卡失败" :
               "ethercat_save: failed to open interface", interface);
        return -1;
    }
    if (ec_config_init(FALSE) <= 0) {
        printf("%s\n", chinese ? "ethercat_save: 未发现 EtherCAT 从站" :
               "ethercat_save: no EtherCAT slaves found");
        goto close_socket;
    }
    if (!all_slaves && slave_id > (unsigned int)ec_slavecount) {
        printf("%s: %u (1..%d)\n", chinese ? "ethercat_save: 从站序号不存在" :
               "ethercat_save: slave id is out of range", slave_id, ec_slavecount);
        goto close_socket;
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (all_slaves || (unsigned int)slave == slave_id) {
            selected[slave] = 1u;
        }
    }

    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] == 0u) {
            continue;
        }
        if (ec_slave[slave].eep_man != ETHERCAT_KAIXUAN_VENDOR_ID ||
            ec_slave[slave].eep_id != ETHERCAT_KAIXUAN_PRODUCT_CODE) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "不是受支持的开璇驱动器，拒绝保存参数" :
                   "not a supported Kaixuan drive; refusing parameter save");
            goto close_socket;
        }
        size = (int)sizeof(selected_status[slave]);
        if (ec_SDOread((uint16)slave, 0x6041u, 0u, FALSE, &size,
                       &selected_status[slave], EC_TIMEOUTRXM) <= 0 ||
            size != (int)sizeof(selected_status[slave])) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "读取 0x6041 状态字失败，拒绝保存参数" :
                   "failed to read 0x6041 statusword; refusing parameter save");
            goto close_socket;
        }
        if ((selected_status[slave] & 0x006fu) == 0x0021u ||
            (selected_status[slave] & 0x006fu) == 0x0023u ||
            (selected_status[slave] & 0x006fu) == 0x0027u ||
            (selected_status[slave] & 0x006fu) == 0x0007u) {
            printf("[slave%d]: status_word=0x%04x %s\n", slave,
                   (unsigned int)selected_status[slave], chinese ?
                   "驱动器未处于失能状态，拒绝保存参数" :
                   "drive is not disabled; refusing parameter save");
            goto close_socket;
        }
        size = (int)sizeof(save_parameter[slave]);
        if (ec_SDOread((uint16)slave, 0x2097u, 0u, FALSE, &size,
                       &save_parameter[slave], EC_TIMEOUTRXM) <= 0 ||
            size != (int)sizeof(save_parameter[slave])) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "读取 Pn097 (0x2097:00) 失败，拒绝保存" :
                   "failed to read Pn097 (0x2097:00); refusing save");
            goto close_socket;
        }
    }

    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] == 0u) {
            continue;
        }
        size = (int)sizeof(save_on);
        if (ec_SDOwrite((uint16)slave, 0x2097u, 0u, FALSE, size,
                        &save_on, EC_TIMEOUTRXM) <= 0) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "写入 Pn097=1 失败" : "failed to write Pn097=1");
            goto close_socket;
        }
        size = (int)sizeof(save_parameter[slave]);
        if (ec_SDOread((uint16)slave, 0x2097u, 0u, FALSE, &size,
                       &save_parameter[slave], EC_TIMEOUTRXM) <= 0 ||
            size != (int)sizeof(save_parameter[slave]) ||
            save_parameter[slave] != 1) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "Pn097=1 未能回读确认；尝试复位为 0" :
                   "Pn097=1 readback failed; attempting reset to 0");
            size = (int)sizeof(save_off);
            ec_SDOwrite((uint16)slave, 0x2097u, 0u, FALSE, size,
                        &save_off, EC_TIMEOUTRXM);
            goto close_socket;
        }
        printf("[slave%d]: %s\n", slave, chinese ?
               "Pn097=1 已回读确认，保持 100ms 供驱动器处理保存请求" :
               "Pn097=1 readback confirmed; holding for 100ms so the drive can process the save request");
        sleep_ms(100u);
        size = (int)sizeof(save_off);
        if (ec_SDOwrite((uint16)slave, 0x2097u, 0u, FALSE, size,
                        &save_off, EC_TIMEOUTRXM) <= 0) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "写入 Pn097=0 失败" : "failed to write Pn097=0 after save trigger");
            ec_SDOwrite((uint16)slave, 0x2097u, 0u, FALSE, size,
                        &save_off, EC_TIMEOUTRXM);
            goto close_socket;
        }
        size = (int)sizeof(save_parameter[slave]);
        if (ec_SDOread((uint16)slave, 0x2097u, 0u, FALSE, &size,
                       &save_parameter[slave], EC_TIMEOUTRXM) <= 0 ||
            size != (int)sizeof(save_parameter[slave]) ||
            save_parameter[slave] != 0) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "Pn097 保存脉冲后未回到 0" :
                   "Pn097 did not return to 0 after save pulse");
            goto close_socket;
        }
        printf("[slave%d]: Pn097 1->0 已发送并回读为 0\n", slave);
    }
    printf("%s\n", chinese ?
           "参数保存触发已发送；请按厂商流程重启执行器并回读参数确认持久化。" :
           "Parameter save trigger sent; restart the actuator and read back parameters to verify persistence.");
    ec_close();
    return 0;

close_socket:
    ec_close();
    return -1;
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
