#include <net/if.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>

typedef struct {
    bool chinese;
    unsigned int hold_ms;
    char interface[IFNAMSIZ];
    char selection[32];
    char sync0_shift[32];
    char sync0_cycle_ms[32];
    bool has_sync0_shift;
    bool has_sync0_cycle_ms;
    bool disable_task;
} ethercat_background_args;

static pthread_mutex_t ethercat_background_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t ethercat_background_condition = PTHREAD_COND_INITIALIZER;
static pthread_t ethercat_background_thread;
static ethercat_background_args ethercat_background_arguments;
static bool ethercat_background_started;
static bool ethercat_background_running;
static bool ethercat_background_ready;
static bool ethercat_background_stop_requested;
static int ethercat_background_result;

static bool console_ethercat_background_should_stop(void)
{
    bool should_stop;

    pthread_mutex_lock(&ethercat_background_mutex);
    should_stop = ethercat_background_stop_requested;
    pthread_mutex_unlock(&ethercat_background_mutex);
    return should_stop;
}

static void console_ethercat_background_mark_ready(void)
{
    pthread_mutex_lock(&ethercat_background_mutex);
    if (ethercat_background_started && ethercat_background_running) {
        ethercat_background_ready = true;
        pthread_cond_broadcast(&ethercat_background_condition);
    }
    pthread_mutex_unlock(&ethercat_background_mutex);
}

#ifdef HAVE_ETHERLAB
#include "etherlab_compat.h"
#include "ethercat_etherlab_compat.c"

typedef struct {
    int kernel_log_fd;
    int temporary_log_fd;
    bool active;
    bool debug_enabled;
    char temporary_path[PATH_MAX];
} ethercat_failure_log;

static int console_ethercat_set_kernel_debug(unsigned int level)
{
    const char *configured_cli = getenv("ETHERLAB_CLI");
    const char *cli_path = NULL;
    char level_text[8];
    char *arguments[4];
    pid_t child;
    pid_t waited;
    int status;

    snprintf(level_text, sizeof(level_text), "%u", level);
    arguments[0] = (char *)"ethercat";
    arguments[1] = (char *)"debug";
    arguments[2] = level_text;
    arguments[3] = NULL;
    if (configured_cli != NULL && configured_cli[0] != '\0' &&
        configured_cli[0] == '/' && access(configured_cli, X_OK) == 0) {
        cli_path = configured_cli;
    } else if (access("/usr/local/etherlab/bin/ethercat", X_OK) == 0) {
        cli_path = "/usr/local/etherlab/bin/ethercat";
    } else if (access("/usr/bin/ethercat", X_OK) == 0) {
        cli_path = "/usr/bin/ethercat";
    }
    if (cli_path == NULL) return -1;
    child = fork();
    if (child < 0) return -1;
    if (child == 0) {
        int null_fd = open("/dev/null", O_WRONLY);
        if (null_fd >= 0) {
            dup2(null_fd, STDOUT_FILENO);
            dup2(null_fd, STDERR_FILENO);
            close(null_fd);
        }
        execv(cli_path, arguments);
        _exit(127);
    }
    do {
        waited = waitpid(child, &status, 0);
    } while (waited < 0 && errno == EINTR);
    if (waited != child) return -1;
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) return -1;
    return 0;
}

static void console_ethercat_failure_log_start(ethercat_failure_log *capture,
                                                bool chinese)
{
    memset(capture, 0, sizeof(*capture));
    capture->kernel_log_fd = -1;
    capture->temporary_log_fd = -1;
    snprintf(capture->temporary_path, sizeof(capture->temporary_path),
             "/tmp/ethercat-enable-XXXXXX");
    capture->temporary_log_fd = mkstemp(capture->temporary_path);
    if (capture->temporary_log_fd < 0) {
        printf("%s\n", chinese ?
               "ethercat_enable: 无法创建内核日志临时文件；仍继续使能" :
               "ethercat_enable: cannot create temporary kernel log; continuing enable");
        return;
    }
    fchmod(capture->temporary_log_fd, S_IRUSR | S_IWUSR);
    capture->kernel_log_fd = open("/dev/kmsg", O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (capture->kernel_log_fd >= 0 && lseek(capture->kernel_log_fd, 0, SEEK_END) < 0) {
        close(capture->kernel_log_fd);
        capture->kernel_log_fd = -1;
    }
    capture->active = true;
    if (console_ethercat_set_kernel_debug(1u) == 0) {
        capture->debug_enabled = true;
    } else {
        printf("%s\n", chinese ?
               "ethercat_enable: 无法自动设置 EtherLab debug=1；将尽量保存现有内核日志" :
               "ethercat_enable: could not set EtherLab debug=1; will save available kernel logs");
    }
}

static int console_ethercat_write_all(int fd, const char *data, size_t length)
{
    size_t written = 0u;

    while (written < length) {
        ssize_t result = write(fd, data + written, length - written);
        if (result < 0 && errno == EINTR) continue;
        if (result <= 0) return -1;
        written += (size_t)result;
    }
    return 0;
}

static void console_ethercat_failure_log_finish(ethercat_failure_log *capture,
                                                 bool failed,
                                                 bool chinese,
                                                 const char *interface,
                                                 const char *selection,
                                                 uint32_t sync0_cycle_ns,
                                                 int32_t sync0_shift_ns,
                                                 const char *failure_stage)
{
    char final_path[PATH_MAX];
    char header[1024];
    char kernel_message[4096];
    struct timespec now;
    struct tm local_time;
    char timestamp[32] = "unknown-time";
    char date[24];
    int header_length;
    bool saved = false;
    bool kernel_log_overrun = false;

    if (!capture->active) return;
    if (failed && capture->temporary_log_fd >= 0) {
        if (clock_gettime(CLOCK_REALTIME, &now) == 0 &&
            localtime_r(&now.tv_sec, &local_time) != NULL &&
            strftime(date, sizeof(date), "%Y%m%d-%H%M%S", &local_time) > 0u) {
            snprintf(timestamp, sizeof(timestamp), "%s-%09ld", date, now.tv_nsec);
        }
        header_length = snprintf(header, sizeof(header),
                                 "EtherCAT enable failure diagnostics\n"
                                 "timestamp=%s\ninterface=%s\nslave=%s\n"
                                 "sync0_cycle_ns=%u\nsync0_shift_ns=%d\n"
                                 "failure_stage=%s\n"
                                 "kernel_log_source=/dev/kmsg (messages since enable attempt)\n\n",
                                 timestamp, interface, selection, sync0_cycle_ns,
                                 (int)sync0_shift_ns, failure_stage);
        if (header_length > 0 && (size_t)header_length < sizeof(header)) {
            console_ethercat_write_all(capture->temporary_log_fd, header,
                                       (size_t)header_length);
        }
        if (capture->kernel_log_fd >= 0) {
            for (;;) {
                ssize_t result = read(capture->kernel_log_fd, kernel_message,
                                      sizeof(kernel_message));
                if (result > 0) {
                    console_ethercat_write_all(capture->temporary_log_fd,
                                               kernel_message, (size_t)result);
                    continue;
                }
                if (result < 0 && errno == EINTR) continue;
                if (result < 0 && errno == EPIPE) {
                    static const char overrun[] =
                        "\n[warning: kernel log buffer overrun; some messages may be missing]\n";
                    if (kernel_log_overrun) break;
                    console_ethercat_write_all(capture->temporary_log_fd,
                                               overrun, sizeof(overrun) - 1u);
                    kernel_log_overrun = true;
                    continue;
                }
                break;
            }
        } else {
            static const char unavailable[] =
                "Unable to read /dev/kmsg; check root privileges and kernel.dmesg_restrict.\n";
            console_ethercat_write_all(capture->temporary_log_fd,
                                       unavailable, sizeof(unavailable) - 1u);
        }
        fsync(capture->temporary_log_fd);
        close(capture->temporary_log_fd);
        capture->temporary_log_fd = -1;
        snprintf(final_path, sizeof(final_path),
                 "/tmp/ethercat-enable-failure-%s-%ld.log", timestamp, (long)getpid());
        saved = rename(capture->temporary_path, final_path) == 0;
        if (saved) {
            printf("%s: %s\n", chinese ? "EtherCAT 内核日志已保存" :
                   "EtherCAT kernel log saved", final_path);
        } else {
            printf("%s: %s\n", chinese ? "EtherCAT 内核日志保存失败" :
                   "Failed to save EtherCAT kernel log", capture->temporary_path);
            unlink(capture->temporary_path);
        }
    } else if (capture->temporary_log_fd >= 0) {
        close(capture->temporary_log_fd);
        capture->temporary_log_fd = -1;
        unlink(capture->temporary_path);
    }
    if (capture->kernel_log_fd >= 0) {
        close(capture->kernel_log_fd);
        capture->kernel_log_fd = -1;
    }
    if (capture->debug_enabled && console_ethercat_set_kernel_debug(0u) != 0) {
        printf("%s\n", chinese ?
               "ethercat_enable: 未能恢复 EtherLab debug=0，请手动执行 sudo /usr/local/etherlab/bin/ethercat debug 0" :
               "ethercat_enable: failed to restore EtherLab debug=0; run sudo /usr/local/etherlab/bin/ethercat debug 0 manually");
    }
    capture->active = false;
}

typedef struct {
    bool valid;
    bool live_valid;
    bool live_error_valid;
    bool runtime_fault_reported;
    char name[80];
    uint32_t vendor;
    uint32_t product;
    uint32_t revision;
    uint8_t state;
    uint16_t obytes;
    uint16_t ibytes;
    uint16_t obits;
    uint16_t ibits;
    int16_t pn001;
    int16_t pn002;
    int16_t pn070;
    int16_t pn075;
    int16_t pn077;
    int16_t pn079;
    int16_t pn085;
    int16_t pn088;
    int16_t pn051;
    int16_t pn150;
    int8_t mode_display;
    uint16_t error_code;
    int32_t monitor_value;
    uint16_t sm2_sync_type;
    uint16_t sm3_sync_type;
    uint32_t sm2_cycle_ns;
    uint32_t sm3_cycle_ns;
    bool dc_supported;
    uint8_t dc_activation;
    uint32_t dc_cycle_ns;
    uint32_t sync0_cycle_ns;
    int32_t sync0_shift_ns;
    int64_t dc_sync_phase_error_ns;
    int64_t dc_sync_adjustment_ns;
    uint32_t config_valid_mask;
    uint16_t live_error_code;
    uint8_t outputs[13];
    uint8_t inputs[14];
    uint64_t live_timestamp_us;
} ethercat_cached_slave;

static ethercat_cached_slave ethercat_cached_slaves[EC_MAXSLAVE];

static const char *console_ethercat_cia402_state_name(uint16_t status_word);

enum {
    ETHERCAT_CACHE_PN001 = 1u << 0,
    ETHERCAT_CACHE_PN002 = 1u << 1,
    ETHERCAT_CACHE_PN070 = 1u << 2,
    ETHERCAT_CACHE_PN075 = 1u << 3,
    ETHERCAT_CACHE_PN077 = 1u << 4,
    ETHERCAT_CACHE_PN079 = 1u << 5,
    ETHERCAT_CACHE_PN085 = 1u << 6,
    ETHERCAT_CACHE_PN088 = 1u << 7,
    ETHERCAT_CACHE_MODE_DISPLAY = 1u << 8,
    ETHERCAT_CACHE_ERROR_CODE = 1u << 9,
    ETHERCAT_CACHE_MONITOR = 1u << 10,
    ETHERCAT_CACHE_SM2_TYPE = 1u << 11,
    ETHERCAT_CACHE_SM2_CYCLE = 1u << 12,
    ETHERCAT_CACHE_SM3_TYPE = 1u << 13,
    ETHERCAT_CACHE_SM3_CYCLE = 1u << 14,
    ETHERCAT_CACHE_DC_ACTIVATION = 1u << 15,
    ETHERCAT_CACHE_DC_CYCLE = 1u << 16,
    ETHERCAT_CACHE_PN150 = 1u << 17,
    ETHERCAT_CACHE_PN051 = 1u << 18,
};
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

static int console_ethercat_parse_pn_number(const char *text, uint16_t *pn_number)
{
    char *end;
    unsigned long parsed;

    if (text == NULL || pn_number == NULL) {
        return -1;
    }
    if ((text[0] == 'P' || text[0] == 'p') &&
        (text[1] == 'N' || text[1] == 'n')) {
        text += 2;
    }
    if (text[0] == '\0') {
        return -1;
    }
    errno = 0;
    parsed = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed > 0xdfffu) {
        return -1;
    }
    *pn_number = (uint16_t)parsed;
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

static int console_ethercat_parse_cycle_ms(const char *text, uint32_t *cycle_ns)
{
    static const char prefix[] = "sync0_cycle_ms=";

    if (text == NULL || cycle_ns == NULL) {
        return -1;
    }
    if (strncmp(text, prefix, sizeof(prefix) - 1u) == 0) {
        text += sizeof(prefix) - 1u;
    }
    if (strcmp(text, "0.5") == 0) {
        *cycle_ns = 500000u;
    } else if (strcmp(text, "1") == 0) {
        *cycle_ns = 1000000u;
    } else if (strcmp(text, "2") == 0) {
        *cycle_ns = 2000000u;
    } else if (strcmp(text, "4") == 0) {
        *cycle_ns = 4000000u;
    } else if (strcmp(text, "5") == 0) {
        *cycle_ns = 5000000u;
    } else if (strcmp(text, "8") == 0) {
        *cycle_ns = 8000000u;
    } else {
        return -1;
    }
    return 0;
}

enum {
    ETHERCAT_KAIXUAN_VENDOR_ID = 0x00010203u,
    ETHERCAT_KAIXUAN_PRODUCT_CODE = 0x00000402u,
    ETHERCAT_KAIXUAN_RXPDO_BITS = 104u,
    ETHERCAT_KAIXUAN_TXPDO_MIN_BITS = 112u,
    ETHERCAT_KAIXUAN_CONTROL_PERIOD_MS = 1u,
    ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS = 1000000u,
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

static const char *const ethercat_sync0_cycle_words[] = {
    "sync0_cycle_ms=0.5", "sync0_cycle_ms=1", "sync0_cycle_ms=2",
    "sync0_cycle_ms=4", "sync0_cycle_ms=5", "sync0_cycle_ms=8",
};

static const char *const ethercat_enable_initial_words[] = {
    "1000", "2000", "5000", "10000", "30000", "60000",
    "sync0_cycle_ms=0.5", "sync0_cycle_ms=1", "sync0_cycle_ms=2",
    "sync0_cycle_ms=4", "sync0_cycle_ms=5", "sync0_cycle_ms=8",
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

static const char *const ethercat_pn_number_words[] = {
    "Pn001", "Pn002", "Pn070", "Pn075", "Pn077", "Pn079", "Pn085",
    "Pn088", "Pn097", "Pn101", "Pn150",
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
         strcmp(first, "ethercat_pnread") == 0 ||
         strcmp(first, "ethercat_save") == 0 ||
         strcmp(first, "ethercat_pn077") == 0) && tokens_before == 1u) {
        return console_ethercat_slave_completion_words(count);
    }
    if (strcmp(first, "ethercat_pnread") == 0 && tokens_before == 2u) {
        *count = sizeof(ethercat_pn_number_words) / sizeof(ethercat_pn_number_words[0]);
        return ethercat_pn_number_words;
    }
    if (strcmp(first, "ethercat_disable") == 0 ||
        strcmp(first, "ethercat_zero") == 0 ||
        strcmp(first, "ethercat_info") == 0 ||
        strcmp(first, "ethercat_save") == 0) {
        return console_ethercat_interface_completion_words(count);
    }
    if (strcmp(first, "ethercat_pnread") == 0 && tokens_before >= 3u) {
        return console_ethercat_interface_completion_words(count);
    }
    if (strcmp(first, "ethercat_enable") == 0 && tokens_before == 2u) {
        *count = sizeof(ethercat_enable_initial_words) /
                 sizeof(ethercat_enable_initial_words[0]);
        return ethercat_enable_initial_words;
    }
    if (strcmp(first, "ethercat_enable") == 0 && tokens_before == 3u) {
        *count = sizeof(ethercat_sync0_shift_words) /
                 sizeof(ethercat_sync0_shift_words[0]);
        return ethercat_sync0_shift_words;
    }
    if (strcmp(first, "ethercat_enable") == 0 && tokens_before == 4u) {
        *count = sizeof(ethercat_sync0_cycle_words) /
                 sizeof(ethercat_sync0_cycle_words[0]);
        return ethercat_sync0_cycle_words;
    }
    if (strcmp(first, "ethercat_enable") == 0 && tokens_before >= 5u) {
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

#ifdef HAVE_ETHERLAB
static int ethercat_last_work_counter;
static int ethercat_expected_work_counter;
static int ethercat_min_work_counter;
static unsigned int ethercat_incomplete_work_counter_count;
static uint64_t ethercat_last_exchange_us;
static uint64_t ethercat_last_exchange_interval_us;
static uint64_t ethercat_max_exchange_interval_us;
static struct timespec ethercat_next_cycle;
static bool ethercat_cycle_initialized;
static uint32_t ethercat_control_period_ns = ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS;
static bool ethercat_dc_sync_enabled;
static int64_t ethercat_dc_sync_target_ns;
static int64_t ethercat_dc_sync_integral_ns;
static int64_t ethercat_dc_sync_phase_error_ns;
static int64_t ethercat_dc_sync_adjustment_ns;

static void console_ethercat_update_dc_sync(void)
{
    int64_t cycle_ns = (int64_t)ethercat_control_period_ns;
    int64_t integral_limit_ns;
    int64_t phase_ns;

    if (!ethercat_dc_sync_enabled || cycle_ns <= 0) {
        return;
    }
    phase_ns = ((int64_t)ec_DCtime - ethercat_dc_sync_target_ns) % cycle_ns;
    if (phase_ns > cycle_ns / 2) {
        phase_ns -= cycle_ns;
    } else if (phase_ns < -(cycle_ns / 2)) {
        phase_ns += cycle_ns;
    }
    ethercat_dc_sync_phase_error_ns = -phase_ns;
    ethercat_dc_sync_integral_ns += ethercat_dc_sync_phase_error_ns;
    integral_limit_ns = cycle_ns * 10000;
    if (ethercat_dc_sync_integral_ns > integral_limit_ns) {
        ethercat_dc_sync_integral_ns = integral_limit_ns;
    } else if (ethercat_dc_sync_integral_ns < -integral_limit_ns) {
        ethercat_dc_sync_integral_ns = -integral_limit_ns;
    }
    ethercat_dc_sync_adjustment_ns =
        (int64_t)((double)ethercat_dc_sync_phase_error_ns * 0.01 +
                  (double)ethercat_dc_sync_integral_ns * 0.00002);
}

static void console_ethercat_wait_next_cycle(void)
{
    struct timespec now;
    int64_t cycle_increment_ns = (int64_t)ethercat_control_period_ns;
    int wait_result;

    if (ethercat_dc_sync_enabled) {
        cycle_increment_ns += ethercat_dc_sync_adjustment_ns;
    }
    if (cycle_increment_ns <= 0) {
        cycle_increment_ns = (int64_t)ethercat_control_period_ns;
    }
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        sleep_ms((unsigned int)((cycle_increment_ns + 999999) / 1000000));
        return;
    }
    if (!ethercat_cycle_initialized) {
        ethercat_next_cycle = now;
        ethercat_cycle_initialized = true;
    }
    ethercat_next_cycle.tv_nsec += (long)cycle_increment_ns;
    while (ethercat_next_cycle.tv_nsec >= 1000000000L) {
        ethercat_next_cycle.tv_sec++;
        ethercat_next_cycle.tv_nsec -= 1000000000L;
    }
    while (ethercat_next_cycle.tv_nsec < 0L) {
        ethercat_next_cycle.tv_sec--;
        ethercat_next_cycle.tv_nsec += 1000000000L;
    }
    while ((ethercat_next_cycle.tv_sec < now.tv_sec) ||
           (ethercat_next_cycle.tv_sec == now.tv_sec &&
            ethercat_next_cycle.tv_nsec <= now.tv_nsec)) {
        ethercat_next_cycle.tv_nsec += (long)cycle_increment_ns;
        while (ethercat_next_cycle.tv_nsec >= 1000000000L) {
            ethercat_next_cycle.tv_sec++;
            ethercat_next_cycle.tv_nsec -= 1000000000L;
        }
        while (ethercat_next_cycle.tv_nsec < 0L) {
            ethercat_next_cycle.tv_sec--;
            ethercat_next_cycle.tv_nsec += 1000000000L;
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

#ifdef HAVE_ETHERLAB
static bool console_ethercat_read_sdo_value(int slave,
                                            uint16_t index,
                                            uint8_t subindex,
                                            void *value,
                                            int value_size)
{
    int actual_size = value_size;

    return ec_SDOread((uint16)slave, index, subindex, FALSE,
                      &actual_size, value, EC_TIMEOUTRXM) > 0 &&
           actual_size == value_size;
}

static int console_ethercat_write_pn101_verified(bool chinese,
                                                 int slave,
                                                 int16_t value,
                                                 const char *step)
{
    int16_t readback = 0;

    printf("[slave%d]: Pn101 %s: write %d\n", slave, step, (int)value);
    if (ec_SDOwrite((uint16)slave, 0x2101u, 0u, FALSE, (int)sizeof(value),
                    &value, EC_TIMEOUTRXM) <= 0) {
        printf("[slave%d]: Pn101 %s: %s\n", slave, step,
               chinese ? "SDO 写入未确认" : "SDO write was not acknowledged");
        return -1;
    }
    printf("[slave%d]: Pn101 %s: %s\n", slave, step,
           chinese ? "SDO 写入已确认，正在回读" : "SDO write acknowledged; reading back");
    if (!console_ethercat_read_sdo_value(slave, 0x2101u, 0u,
                                         &readback, (int)sizeof(readback))) {
        printf("[slave%d]: Pn101 %s: %s\n", slave, step,
               chinese ? "回读失败" : "readback failed");
        return -1;
    }
    printf("[slave%d]: Pn101 %s: readback=%d\n", slave, step, (int)readback);
    if (readback != value) {
        printf("[slave%d]: Pn101 %s: %s (expected=%d actual=%d)\n",
               slave, step,
               chinese ? "回读值不匹配" : "readback mismatch",
               (int)value, (int)readback);
        return -1;
    }
    return 0;
}

static void console_ethercat_cache_configuration(const uint8_t selected[EC_MAXSLAVE],
                                                 uint32_t sync0_cycle_ns,
                                                 int32_t sync0_shift_ns)
{
    int slave;

    for (slave = 1; slave <= ec_slavecount; slave++) {
        ethercat_cached_slave snapshot;
        uint8_t dc_cycle_data[4] = {0};

        if (selected[slave] == 0u) {
            continue;
        }
        memset(&snapshot, 0, sizeof(snapshot));
        snprintf(snapshot.name, sizeof(snapshot.name), "%s", ec_slave[slave].name);
        snapshot.vendor = ec_slave[slave].eep_man;
        snapshot.product = ec_slave[slave].eep_id;
        snapshot.revision = ec_slave[slave].eep_rev;
        snapshot.state = (uint8_t)ec_slave[slave].state;
        snapshot.dc_supported = ec_slave[slave].hasdc != 0u;
        snapshot.obytes = ec_slave[slave].Obytes;
        snapshot.ibytes = ec_slave[slave].Ibytes;
        snapshot.obits = ec_slave[slave].Obits;
        snapshot.ibits = ec_slave[slave].Ibits;
        snapshot.sync0_cycle_ns = sync0_cycle_ns;
        snapshot.sync0_shift_ns = sync0_shift_ns;
        snapshot.dc_sync_phase_error_ns = ethercat_dc_sync_phase_error_ns;
        snapshot.dc_sync_adjustment_ns = ethercat_dc_sync_adjustment_ns;

#define ETHERCAT_CACHE_SDO(index, subindex, field, mask) \
        if (console_ethercat_read_sdo_value(slave, (index), (subindex), \
                                            &snapshot.field, (int)sizeof(snapshot.field))) { \
            snapshot.config_valid_mask |= (mask); \
        }
        ETHERCAT_CACHE_SDO(0x2001u, 0u, pn001, ETHERCAT_CACHE_PN001);
        ETHERCAT_CACHE_SDO(0x2002u, 0u, pn002, ETHERCAT_CACHE_PN002);
        ETHERCAT_CACHE_SDO(0x2070u, 0u, pn070, ETHERCAT_CACHE_PN070);
        ETHERCAT_CACHE_SDO(0x2075u, 0u, pn075, ETHERCAT_CACHE_PN075);
        ETHERCAT_CACHE_SDO(0x2077u, 0u, pn077, ETHERCAT_CACHE_PN077);
        ETHERCAT_CACHE_SDO(0x2079u, 0u, pn079, ETHERCAT_CACHE_PN079);
        ETHERCAT_CACHE_SDO(0x2085u, 0u, pn085, ETHERCAT_CACHE_PN085);
        ETHERCAT_CACHE_SDO(0x2088u, 0u, pn088, ETHERCAT_CACHE_PN088);
        ETHERCAT_CACHE_SDO(0x2051u, 0u, pn051, ETHERCAT_CACHE_PN051);
        ETHERCAT_CACHE_SDO(0x2150u, 0u, pn150, ETHERCAT_CACHE_PN150);
        ETHERCAT_CACHE_SDO(0x6061u, 0u, mode_display, ETHERCAT_CACHE_MODE_DISPLAY);
        ETHERCAT_CACHE_SDO(0x603fu, 0u, error_code, ETHERCAT_CACHE_ERROR_CODE);
        ETHERCAT_CACHE_SDO(0x3000u, 0u, monitor_value, ETHERCAT_CACHE_MONITOR);
        ETHERCAT_CACHE_SDO(0x1c32u, 1u, sm2_sync_type, ETHERCAT_CACHE_SM2_TYPE);
        ETHERCAT_CACHE_SDO(0x1c32u, 2u, sm2_cycle_ns, ETHERCAT_CACHE_SM2_CYCLE);
        ETHERCAT_CACHE_SDO(0x1c33u, 1u, sm3_sync_type, ETHERCAT_CACHE_SM3_TYPE);
        ETHERCAT_CACHE_SDO(0x1c33u, 2u, sm3_cycle_ns, ETHERCAT_CACHE_SM3_CYCLE);
#undef ETHERCAT_CACHE_SDO

        if (ec_FPRD(ec_slave[slave].configadr, 0x0981u,
                    (uint16)sizeof(snapshot.dc_activation),
                    &snapshot.dc_activation, EC_TIMEOUTRET) > 0) {
            snapshot.config_valid_mask |= ETHERCAT_CACHE_DC_ACTIVATION;
        }
        if (ec_FPRD(ec_slave[slave].configadr, 0x09a0u,
                    (uint16)sizeof(dc_cycle_data), dc_cycle_data, EC_TIMEOUTRET) > 0) {
            snapshot.dc_cycle_ns = (uint32_t)dc_cycle_data[0] |
                                   ((uint32_t)dc_cycle_data[1] << 8u) |
                                   ((uint32_t)dc_cycle_data[2] << 16u) |
                                   ((uint32_t)dc_cycle_data[3] << 24u);
            snapshot.config_valid_mask |= ETHERCAT_CACHE_DC_CYCLE;
        }
        snapshot.valid = true;
        pthread_mutex_lock(&ethercat_background_mutex);
        ethercat_cached_slaves[slave] = snapshot;
        pthread_mutex_unlock(&ethercat_background_mutex);
    }
}

static void console_ethercat_capture_live_pdo(const uint8_t selected[EC_MAXSLAVE])
{
    bool report_fault[EC_MAXSLAVE] = {false};
    uint16_t status_words[EC_MAXSLAVE] = {0};
    uint16_t error_codes[EC_MAXSLAVE] = {0};
    int slave;
    uint64_t timestamp_us = time_us();

    pthread_mutex_lock(&ethercat_background_mutex);
    for (slave = 1; slave <= ec_slavecount; slave++) {
        ethercat_cached_slave *snapshot = &ethercat_cached_slaves[slave];

        if (selected[slave] == 0u || !snapshot->valid ||
            ec_slave[slave].outputs == NULL || ec_slave[slave].inputs == NULL ||
            ec_slave[slave].Obytes < sizeof(snapshot->outputs) ||
            ec_slave[slave].Ibytes < sizeof(snapshot->inputs)) {
            continue;
        }
        memcpy(snapshot->outputs, ec_slave[slave].outputs, sizeof(snapshot->outputs));
        memcpy(snapshot->inputs, ec_slave[slave].inputs, sizeof(snapshot->inputs));
        status_words[slave] = console_ethercat_read_u16(snapshot->inputs);
        error_codes[slave] = console_ethercat_read_u16(snapshot->inputs + 12u);
        snapshot->live_error_code = error_codes[slave];
        snapshot->live_error_valid = true;
        if ((status_words[slave] & 0x006fu) != 0x0027u) {
            report_fault[slave] = !snapshot->runtime_fault_reported;
            snapshot->runtime_fault_reported = true;
        } else {
            snapshot->runtime_fault_reported = false;
        }
        snapshot->state = (uint8_t)ec_slave[slave].state;
        snapshot->live_timestamp_us = timestamp_us;
        snapshot->live_valid = true;
        snapshot->dc_sync_phase_error_ns = ethercat_dc_sync_phase_error_ns;
        snapshot->dc_sync_adjustment_ns = ethercat_dc_sync_adjustment_ns;
    }
    pthread_mutex_unlock(&ethercat_background_mutex);
    for (slave = 1; slave < EC_MAXSLAVE; slave++) {
        if (report_fault[slave]) {
            printf("[slave%d] %s\n", slave,
                   ethercat_background_arguments.chinese ?
                   "实时状态首次离开 operation-enabled" :
                   "first live state transition out of operation-enabled");
            printf("  drive: SW=0x%04x CIA402=%s PDO_error=0x%04x\n",
                   status_words[slave],
                   console_ethercat_cia402_state_name(status_words[slave]),
                   error_codes[slave]);
            printf("  exchange: WKC last=%d expected=%d min=%d incomplete=%u\n",
                   ethercat_last_work_counter, ethercat_expected_work_counter,
                   ethercat_min_work_counter, ethercat_incomplete_work_counter_count);
            printf("  timing: target=%uus last_interval=%lluus max_interval=%lluus\n",
                   ethercat_control_period_ns / 1000u,
                   (unsigned long long)ethercat_last_exchange_interval_us,
                   (unsigned long long)ethercat_max_exchange_interval_us);
            printf("  DC: error=%lldns adjustment=%lldns\n",
                   (long long)ethercat_dc_sync_phase_error_ns,
                   (long long)ethercat_dc_sync_adjustment_ns);
        }
    }
}

static int console_ethercat_print_background_info(bool chinese,
                                                  const char *interface,
                                                  const char *selection)
{
    ethercat_cached_slave snapshots[EC_MAXSLAVE];
    bool print_slave[EC_MAXSLAVE] = {false};
    unsigned int slave_id;
    bool all_slaves;
    bool active;
    size_t count = 0u;
    int slave;

    if (console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0) {
        return -1;
    }
    pthread_mutex_lock(&ethercat_background_mutex);
    active = ethercat_background_running;
    if (!active) {
        pthread_mutex_unlock(&ethercat_background_mutex);
        return 0;
    }
    if (ethercat_background_arguments.disable_task ||
        strcmp(ethercat_background_arguments.interface, interface) != 0) {
        pthread_mutex_unlock(&ethercat_background_mutex);
        return -1;
    }
    memset(snapshots, 0, sizeof(snapshots));
    for (slave = 1; slave < EC_MAXSLAVE; slave++) {
        if (!ethercat_cached_slaves[slave].valid ||
            (!all_slaves && (unsigned int)slave != slave_id)) {
            continue;
        }
        snapshots[slave] = ethercat_cached_slaves[slave];
        print_slave[slave] = true;
        count++;
    }
    pthread_mutex_unlock(&ethercat_background_mutex);
    if (count == 0u) {
        return -1;
    }

    printf("%s: interface=%s slave=%s (%s)\n",
           chinese ? "ethercat_info: 后台快照" : "ethercat_info: background snapshot",
           interface, selection,
           chinese ? "SDO 为使能前缓存，PDO 为实时采样" :
           "SDO cached before enable; PDO sampled live");
    for (slave = 1; slave < EC_MAXSLAVE; slave++) {
        const ethercat_cached_slave *snapshot = &snapshots[slave];
        uint16_t status_word;
        uint16_t control_word;
        int32_t position_actual;
        int32_t velocity_actual;
        int16_t torque_actual;
        int32_t target_position;
        int32_t target_velocity;
        int16_t target_torque;
        int8_t mode_command;
        uint64_t age_us;
        uint64_t now_us;

        if (!print_slave[slave]) {
            continue;
        }
        if (!snapshot->live_valid) {
            printf("[slave%d] %s\n", slave,
                   chinese ? "尚无有效 PDO 实时样本" : "no valid live PDO sample yet");
            continue;
        }
        status_word = console_ethercat_read_u16(snapshot->inputs);
        control_word = console_ethercat_read_u16(snapshot->outputs);
        position_actual = console_ethercat_read_i32(snapshot->inputs + 2u);
        velocity_actual = console_ethercat_read_i32(snapshot->inputs + 6u);
        torque_actual = (int16_t)console_ethercat_read_u16(snapshot->inputs + 10u);
        target_position = console_ethercat_read_i32(snapshot->outputs + 2u);
        target_velocity = console_ethercat_read_i32(snapshot->outputs + 6u);
        target_torque = (int16_t)console_ethercat_read_u16(snapshot->outputs + 10u);
        mode_command = (int8_t)snapshot->outputs[12];
        now_us = time_us();
        age_us = now_us >= snapshot->live_timestamp_us ?
                 now_us - snapshot->live_timestamp_us : 0u;

        printf("[slave%d] %s state=0x%02x(%s) identity=%08x:%08x rev=%08x\n",
               slave, snapshot->name, snapshot->state,
               console_ethercat_state_name(snapshot->state), snapshot->vendor,
               snapshot->product, snapshot->revision);
        printf("  config snapshot: Pn001=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_PN001) printf("%d", snapshot->pn001); else printf("?");
        printf(" Pn002=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_PN002) printf("%d", snapshot->pn002); else printf("?");
        printf(" Pn070=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_PN070) printf("%d", snapshot->pn070); else printf("?");
        printf(" Pn075=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_PN075) printf("%d", snapshot->pn075); else printf("?");
        printf(" Pn077=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_PN077) printf("%d", snapshot->pn077); else printf("?");
        printf(" Pn150=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_PN150) {
            printf("%d(%s)\n", snapshot->pn150,
                   snapshot->pn150 == 0 ?
                   (chinese ? "双编码器" : "dual encoder") :
                   (snapshot->pn150 == 1 ?
                    (chinese ? "单编码器/多圈" : "single multi-turn encoder") :
                    (chinese ? "未知配置值" : "unknown value")));
        } else {
            printf("?\n");
        }
        printf("  mechanics snapshot: Pn051=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_PN051) {
            printf("%d (reduction ratio)\n", snapshot->pn051);
        } else {
            printf("?\n");
        }
        printf("  params snapshot: Pn079=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_PN079) printf("%d", snapshot->pn079); else printf("?");
        printf(" Pn085=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_PN085) printf("%d (0.01A)", snapshot->pn085); else printf("?");
        printf(" Pn088=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_PN088) printf("%d", snapshot->pn088); else printf("?");
        printf(" mode_display(snapshot)=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_MODE_DISPLAY) printf("%d", snapshot->mode_display); else printf("?");
        printf(" error_before_enable=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_ERROR_CODE) printf("0x%04x", snapshot->error_code); else printf("?");
        printf(" monitor_before_enable=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_MONITOR) printf("%d(Pn002)\n", snapshot->monitor_value); else printf("?\n");
        printf("  bus snapshot: PDO out=%uB/%ubit in=%uB/%ubit DC_supported=%s active-reg=",
               snapshot->obytes, snapshot->obits, snapshot->ibytes, snapshot->ibits,
               snapshot->dc_supported ? "yes" : "no");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_DC_ACTIVATION) printf("0x%02x", snapshot->dc_activation); else printf("?");
        printf(" cycle=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_DC_CYCLE) printf("%uns", snapshot->dc_cycle_ns); else printf("?");
        printf(" shift=%dns Sync0=%uns\n", snapshot->sync0_shift_ns, snapshot->sync0_cycle_ns);
        printf("  sync snapshot: SM2 type=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_SM2_TYPE) printf("0x%04x", snapshot->sm2_sync_type); else printf("?");
        printf(" cycle=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_SM2_CYCLE) printf("%uns", snapshot->sm2_cycle_ns); else printf("?");
        printf(" | SM3 type=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_SM3_TYPE) printf("0x%04x", snapshot->sm3_sync_type); else printf("?");
        printf(" cycle=");
        if (snapshot->config_valid_mask & ETHERCAT_CACHE_SM3_CYCLE) printf("%uns\n", snapshot->sm3_cycle_ns); else printf("?\n");
        printf("  live PDO: age=%lluus CW=0x%04x SW=0x%04x mode_command=%d DC_error=%lldns DC_adjust=%lldns error_code=",
               (unsigned long long)age_us, control_word, status_word, (int)mode_command,
               (long long)snapshot->dc_sync_phase_error_ns,
               (long long)snapshot->dc_sync_adjustment_ns);
        if (snapshot->live_error_valid) printf("0x%04x\n", snapshot->live_error_code); else printf("?\n");
        printf("  actual: motor_pos=%.6frad (%d count) output_pos=",
               (double)position_actual * ETHERCAT_KAIXUAN_TWO_PI /
               ETHERCAT_KAIXUAN_COUNTS_PER_REV, position_actual);
        if ((snapshot->config_valid_mask & ETHERCAT_CACHE_PN051) && snapshot->pn051 > 0) {
            printf("%.6frad",
                   (double)position_actual * ETHERCAT_KAIXUAN_TWO_PI /
                   ETHERCAT_KAIXUAN_COUNTS_PER_REV / (double)snapshot->pn051);
        } else {
            printf("?");
        }
        printf(" vel=");
        if ((snapshot->config_valid_mask & ETHERCAT_CACHE_PN088) && snapshot->pn088 == 0) {
            printf("%.6frad/s (%d rpm)",
                   (double)velocity_actual * ETHERCAT_KAIXUAN_TWO_PI / 60.0,
                   velocity_actual);
        } else if ((snapshot->config_valid_mask & ETHERCAT_CACHE_PN088) && snapshot->pn088 == 1) {
            printf("%.6frad/s (%d count/s)",
                   (double)velocity_actual * ETHERCAT_KAIXUAN_TWO_PI /
                   ETHERCAT_KAIXUAN_COUNTS_PER_REV, velocity_actual);
        } else {
            printf("%d (unit unknown)", velocity_actual);
        }
        printf(" torque=%d (0.01A) current=not-mapped\n", (int)torque_actual);
        printf("  target PDO: pos=%.6frad (%d count) vel=%d count/s torque=%d (0.01A); digital_inputs=not-mapped\n",
               (double)target_position * ETHERCAT_KAIXUAN_TWO_PI /
               ETHERCAT_KAIXUAN_COUNTS_PER_REV,
               target_position, target_velocity, (int)target_torque);
    }
    return 1;
}
#endif

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

        ethercat_last_exchange_interval_us = interval_us;
        if (interval_us > ethercat_max_exchange_interval_us) {
            ethercat_max_exchange_interval_us = interval_us;
        }
    } else {
        ethercat_last_exchange_interval_us = 0u;
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
    if (work_counter > 0) {
        console_ethercat_update_dc_sync();
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
    ethercat_last_exchange_interval_us = 0u;
    ethercat_max_exchange_interval_us = 0u;
    ethercat_cycle_initialized = false;
    ethercat_dc_sync_enabled = false;
    ethercat_dc_sync_target_ns = 0;
    ethercat_dc_sync_integral_ns = 0;
    ethercat_dc_sync_phase_error_ns = 0;
    ethercat_dc_sync_adjustment_ns = 0;
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
                                           int32_t shift_ns,
                                           uint32_t cycle_ns)
{
    int slave;

    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] == 0u) {
            continue;
        }
        if (ec_slave[slave].hasdc == 0u) {
            return -1;
        }
        if (ec_dcsync0((uint16)slave, TRUE, cycle_ns, shift_ns) != 0) {
            return -1;
        }
    }
    ethercat_dc_sync_target_ns = ((int64_t)cycle_ns / 2) + (int64_t)shift_ns;
    ethercat_dc_sync_target_ns %= (int64_t)cycle_ns;
    if (ethercat_dc_sync_target_ns < 0) {
        ethercat_dc_sync_target_ns += (int64_t)cycle_ns;
    }
    ethercat_dc_sync_integral_ns = 0;
    ethercat_dc_sync_phase_error_ns = 0;
    ethercat_dc_sync_adjustment_ns = 0;
    ethercat_dc_sync_enabled = true;
    return 0;
}

static void console_ethercat_print_selected_status(const uint8_t selected[EC_MAXSLAVE],
                                                   uint16_t requested_control_word,
                                                   bool read_sdo_snapshot)
{
    int slave;

    ec_readstate();
    printf("[EtherCAT diag]\n"
           "  request: control_word=0x%04x slaves=%d\n"
           "  exchange: WKC expected=%d last=%d",
           (unsigned int)requested_control_word, ec_slavecount,
           ethercat_expected_work_counter, ethercat_last_work_counter);
    if (ethercat_min_work_counter == INT_MAX) {
        printf(" min=n/a");
    } else {
        printf(" min=%d", ethercat_min_work_counter);
    }
    printf(" incomplete=%u\n"
           "  timing: target=%uus max_interval=%lluus DC_error=%lldns DC_adjust=%lldns\n",
           ethercat_incomplete_work_counter_count,
           ethercat_control_period_ns / 1000u,
           (unsigned long long)ethercat_max_exchange_interval_us,
           (long long)ethercat_dc_sync_phase_error_ns,
           (long long)ethercat_dc_sync_adjustment_ns);
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
        printf("[slave%d] EtherCAT=%s(0x%02x) AL=0x%04x ALcode=0x%04x CIA402=%s\n"
               "  identity: vendor=0x%08x product=0x%08x revision=0x%08x\n"
               "  PDO map: out=%uB/%ubit in=%uB/%ubit\n"
               "  DC: supported=%s active=%s cycle=%uns shift=%dns\n",
               slave, console_ethercat_state_name(ec_slave[slave].state),
               (unsigned int)ec_slave[slave].state,
               (unsigned int)al_status,
               (unsigned int)ec_slave[slave].ALstatuscode,
               console_ethercat_cia402_state_name(status_word),
               (unsigned int)ec_slave[slave].eep_man, (unsigned int)ec_slave[slave].eep_id,
               (unsigned int)ec_slave[slave].eep_rev,
               (unsigned int)ec_slave[slave].Obytes, (unsigned int)ec_slave[slave].Obits,
               (unsigned int)ec_slave[slave].Ibytes, (unsigned int)ec_slave[slave].Ibits,
               ec_slave[slave].hasdc ? "yes" : "no",
               ec_slave[slave].DCactive ? "yes" : "no",
               (unsigned int)ec_slave[slave].DCcycle, (int)ec_slave[slave].DCshift);

        if (read_sdo_snapshot) {
            size = (int)sizeof(mode_display);
            if (ec_SDOread((uint16)slave, 0x6061u, 0u, FALSE, &size, &mode_display,
                           EC_TIMEOUTRXM) > 0 && size == (int)sizeof(mode_display)) {
                printf("  drive: mode_display=%d", (int)mode_display);
            } else {
                printf("  drive: mode_display=unread");
            }
            size = (int)sizeof(error_code);
            if (ec_SDOread((uint16)slave, 0x603fu, 0u, FALSE, &size, &error_code,
                           EC_TIMEOUTRXM) > 0 && size == (int)sizeof(error_code)) {
                printf(" error_code=0x%04x (%u)", (unsigned int)error_code,
                       (unsigned int)error_code);
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
                printf("\n  sync: SM2 type=0x%04x", (unsigned int)sync_type);
            } else {
                printf("\n  sync: SM2 type=unread");
            }
            size = (int)sizeof(sync_cycle_ns);
            if (ec_SDOread((uint16)slave, 0x1c32u, 2u, FALSE, &size, &sync_cycle_ns,
                           EC_TIMEOUTRXM) > 0 && size == (int)sizeof(sync_cycle_ns)) {
                printf(" cycle=%uns", (unsigned int)sync_cycle_ns);
            } else {
                printf(" cycle=unread");
            }
            size = (int)sizeof(tx_sync_type);
            if (ec_SDOread((uint16)slave, 0x1c33u, 1u, FALSE, &size, &tx_sync_type,
                           EC_TIMEOUTRXM) > 0 && size == (int)sizeof(tx_sync_type)) {
                printf(" | SM3 type=0x%04x", (unsigned int)tx_sync_type);
            } else {
                printf(" | SM3 type=unread");
            }
            size = (int)sizeof(tx_sync_cycle_ns);
            if (ec_SDOread((uint16)slave, 0x1c33u, 2u, FALSE, &size, &tx_sync_cycle_ns,
                           EC_TIMEOUTRXM) > 0 && size == (int)sizeof(tx_sync_cycle_ns)) {
                printf(" cycle=%uns", (unsigned int)tx_sync_cycle_ns);
            } else {
                printf(" cycle=unread");
            }
        } else {
            uint16_t pdo_error_code = 0xffffu;
            if (ec_slave[slave].inputs != NULL && ec_slave[slave].Ibits >= 112u) {
                pdo_error_code = console_ethercat_read_u16(
                    (const uint8_t *)ec_slave[slave].inputs + 12u);
            }
            printf("  drive: mode_display=not-read error_code=0x%04x(PDO)"
                   " Pn077=not-read Pn078=not-read\n"
                   "  sync: SM2/SM3 SDO values not-read\n",
                   (unsigned int)pdo_error_code);
        }
        printf("\n  DC registers: activation=");
        if (ec_FPRD(ec_slave[slave].configadr, 0x0981u, (uint16)sizeof(dc_activation),
                    &dc_activation, EC_TIMEOUTRET) > 0) {
            printf("0x%02x", (unsigned int)dc_activation);
        } else {
            printf("unread");
        }
        if (ec_FPRD(ec_slave[slave].configadr, 0x09a0u, (uint16)sizeof(dc_cycle_data),
                    dc_cycle_data, EC_TIMEOUTRET) > 0) {
            dc_cycle_ns = (uint32_t)dc_cycle_data[0] |
                          ((uint32_t)dc_cycle_data[1] << 8u) |
                          ((uint32_t)dc_cycle_data[2] << 16u) |
                          ((uint32_t)dc_cycle_data[3] << 24u);
            printf(" cycle=%uns", (unsigned int)dc_cycle_ns);
        } else {
            printf(" cycle=unread");
        }
        if (ec_FPRD(ec_slave[slave].configadr, 0x0990u, (uint16)sizeof(dc_start_data),
                    dc_start_data, EC_TIMEOUTRET) > 0) {
            printf(" start=[");
            console_ethercat_print_bytes(dc_start_data, (unsigned int)sizeof(dc_start_data));
            printf("]");
        } else {
            printf(" start=unread");
        }
        printf("\n");
        if (ec_slave[slave].outputs != NULL) {
            printf("  PDO out raw: [");
            console_ethercat_print_bytes((const uint8_t *)ec_slave[slave].outputs,
                                         ec_slave[slave].Obytes);
            printf("]\n");
        }
        if (ec_slave[slave].inputs != NULL) {
            printf("  PDO in raw:  [");
            console_ethercat_print_bytes((const uint8_t *)ec_slave[slave].inputs,
                                         ec_slave[slave].Ibytes);
            printf("]\n");
        }
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
    unsigned int max_attempts = (400000000u + ethercat_control_period_ns - 1u) /
                                ethercat_control_period_ns;

    for (attempt = 0u; attempt < max_attempts && !stop_requested &&
         !console_ethercat_background_should_stop(); attempt++) {
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
    unsigned int disable_cycles = (40000000u + ethercat_control_period_ns - 1u) /
                                  ethercat_control_period_ns;

    if (disable_cycles < 10u) {
        disable_cycles = 10u;
    }

    console_ethercat_set_control_word(selected, 0u);
    for (cycle = 0u; cycle < disable_cycles; cycle++) {
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

#ifndef HAVE_ETHERLAB
    printf("%s\n", chinese ?
           "当前程序未编译 EtherLab，无法访问 EtherCAT。请安装 EtherLab 主站开发库后重新构建。" :
           "EtherCAT scan is unavailable because EtherLab is not installed; install the EtherLab master development library and rebuild.");
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
                                   const char *sync0_shift_text,
                                   const char *sync0_cycle_ms_text)
{
    unsigned int slave_id;
    bool all_slaves;
    int32_t sync0_shift_ns = 0;
    uint32_t sync0_cycle_ns = ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS;

    if (console_ethercat_validate_interface(interface) != 0 ||
        console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0 ||
        hold_ms > ETHERCAT_KAIXUAN_ENABLE_MAX_HOLD_MS ||
        (sync0_shift_text != NULL &&
         console_ethercat_parse_sync0_shift(sync0_shift_text, &sync0_shift_ns) != 0) ||
        (sync0_cycle_ms_text != NULL &&
         console_ethercat_parse_cycle_ms(sync0_cycle_ms_text, &sync0_cycle_ns) != 0)) {
        printf("%s: ethercat_enable <network_interface> <slave_id|all> [hold_ms:0..%u] [sync0_shift_ns:-4000000..4000000 [sync0_cycle_ms:0.5|1|2|4|5|8]]\n",
               chinese ? "用法" : "usage", ETHERCAT_KAIXUAN_ENABLE_MAX_HOLD_MS);
        return -1;
    }

#ifndef HAVE_ETHERLAB
    printf("%s\n", chinese ?
           "当前程序未编译 EtherLab，无法使能 EtherCAT 电机。请安装 EtherLab 主站开发库后重新构建。" :
           "EtherCAT enable is unavailable because EtherLab is not installed; install the EtherLab master development library and rebuild.");
    return -1;
#else
    uint8_t process_image[ETHERCAT_KAIXUAN_PROCESS_IMAGE_SIZE];
    uint8_t selected[EC_MAXSLAVE] = {0};
    int32_t target_positions[EC_MAXSLAVE] = {0};
    uint64_t deadline;
    int slave;
    uint16_t diagnostic_control_word = 0u;
    unsigned int op_attempt;
    unsigned int max_op_attempts;
    int result = -1;
    bool mapped = false;
    ethercat_failure_log failure_capture;
    const char *failure_stage = "opening-master";

    if (hold_ms == 0u) {
        printf("%s: interface=%s slave=%s hold=until-Ctrl-C sync0_cycle_ns=%u sync0_shift_ns=%d\n",
               chinese ? "ethercat_enable: 开始" : "ethercat_enable: start",
               interface, selection, sync0_cycle_ns,
               (int)sync0_shift_ns);
    } else {
        printf("%s: interface=%s slave=%s hold_ms=%u sync0_cycle_ns=%u sync0_shift_ns=%d\n",
               chinese ? "ethercat_enable: 开始" : "ethercat_enable: start",
               interface, selection, hold_ms, sync0_cycle_ns,
               (int)sync0_shift_ns);
    }
    console_ethercat_failure_log_start(&failure_capture, chinese);
    if (ec_init((char *)interface) == 0) {
        printf("%s: %s\n", chinese ? "ethercat_enable: 打开网卡失败" :
               "ethercat_enable: failed to open interface", interface);
        console_ethercat_failure_log_finish(&failure_capture, true, chinese,
                                            interface, selection, sync0_cycle_ns,
                                            sync0_shift_ns, failure_stage);
        return -1;
    }
    ethercat_control_period_ns = sync0_cycle_ns;
    failure_stage = "slave-discovery";
    if (ec_config_init(FALSE) <= 0) {
        printf("%s\n", chinese ? "ethercat_enable: 未发现 EtherCAT 从站" :
               "ethercat_enable: no EtherCAT slaves found");
        goto close_socket;
    }
    if (!all_slaves && slave_id > (unsigned int)ec_slavecount) {
        failure_stage = "slave-selection";
        printf("%s: %u (1..%d)\n", chinese ? "ethercat_enable: 从站序号不存在" :
               "ethercat_enable: slave id is out of range", slave_id, ec_slavecount);
        goto close_socket;
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (all_slaves || (unsigned int)slave == slave_id) {
            selected[slave] = 1u;
        }
    }

    failure_stage = "pdo-configuration";
    ec_config_map(process_image);
    mapped = true;
    console_ethercat_reset_exchange_diagnostics();
    ethercat_expected_work_counter =
        ((int)ec_group[0].outputsWKC * 2) + (int)ec_group[0].inputsWKC;
    ec_configdc();
    failure_stage = "sync0-configuration";
    if (console_ethercat_enable_dc_sync(selected, sync0_shift_ns, sync0_cycle_ns) != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 目标从站不支持 DC Sync0" :
               "ethercat_enable: selected slave does not support DC Sync0");
        goto cleanup;
    }
    failure_stage = "pre-op-to-safe-op";
    if ((ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) & 0x0fu) != EC_STATE_SAFE_OP) {
        printf("%s\n", chinese ? "ethercat_enable: 从站未进入 SAFE-OP" :
               "ethercat_enable: slaves did not reach SAFE-OP");
        goto cleanup;
    }
    console_ethercat_cache_configuration(selected, sync0_cycle_ns, sync0_shift_ns);
    failure_stage = "initial-pdo-exchange";
    if (console_ethercat_exchange() != 0 || console_ethercat_exchange() != 0 ||
        console_ethercat_exchange() != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 初始 PDO 通信失败" :
               "ethercat_enable: initial PDO exchange failed");
        goto cleanup;
    }
    failure_stage = "pdo-identity-validation";
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
    failure_stage = "pdo-layout-validation";
    if (console_ethercat_selected_ready(selected, target_positions) != 0) {
        printf("%s\n", chinese ?
               "ethercat_enable: PDO 映射不是当前开璇驱动器要求的 13B 输出/14B 输入，已拒绝使能" :
               "ethercat_enable: PDO mapping is not the required Kaixuan 13B output/14B input layout; enable refused");
        goto cleanup;
    }
    failure_stage = "initial-position-write";
    if (console_ethercat_exchange() != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 写入初始位置失败" :
               "ethercat_enable: failed to write initial positions");
        goto cleanup;
    }
    failure_stage = "request-operational";
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] != 0u) {
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate((uint16)slave);
        }
    }
    max_op_attempts = (400000000u + sync0_cycle_ns - 1u) / sync0_cycle_ns;
    for (op_attempt = 0u; op_attempt < max_op_attempts && !stop_requested &&
         !console_ethercat_background_should_stop(); op_attempt++) {
        if (console_ethercat_exchange() != 0) {
            break;
        }
        if (ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTRET) == EC_STATE_OPERATIONAL) {
            break;
        }
        console_ethercat_wait_next_cycle();
    }
    failure_stage = "wait-operational";
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (selected[slave] != 0u &&
            (ec_statecheck((uint16)slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET) & 0x0fu) !=
            EC_STATE_OPERATIONAL) {
            printf("%s: %d\n", chinese ? "ethercat_enable: 从站未进入 OP" :
                   "ethercat_enable: slave did not reach OP", slave);
            goto cleanup;
        }
    }
    diagnostic_control_word = 0x0006u;
    failure_stage = "cia402-shutdown-0x0006";
    console_ethercat_set_control_word(selected, diagnostic_control_word);
    if (console_ethercat_wait_for_status(selected, 0x0021u) != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 0x0006 状态确认失败" :
               "ethercat_enable: 0x0006 state confirmation failed");
        goto cleanup;
    }
    diagnostic_control_word = 0x0007u;
    failure_stage = "cia402-switch-on-0x0007";
    console_ethercat_set_control_word(selected, diagnostic_control_word);
    if (console_ethercat_wait_for_status(selected, 0x0023u) != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 0x0007 状态确认失败" :
               "ethercat_enable: 0x0007 state confirmation failed");
        goto cleanup;
    }
    diagnostic_control_word = 0x000fu;
    failure_stage = "cia402-enable-0x000f";
    console_ethercat_set_control_word(selected, diagnostic_control_word);
    if (console_ethercat_wait_for_status(selected, 0x0027u) != 0) {
        printf("%s\n", chinese ? "ethercat_enable: 0x000F 状态确认失败" :
               "ethercat_enable: 0x000F state confirmation failed");
        goto cleanup;
    }
    console_ethercat_failure_log_finish(&failure_capture, false, chinese,
                                        interface, selection, sync0_cycle_ns,
                                        sync0_shift_ns, failure_stage);
    console_ethercat_capture_live_pdo(selected);
    printf("%s\n", chinese ?
           "ethercat_enable: 已使能并保持当前位置；可用 ethercat_disable 停止，Ctrl-C 也会自动失能。" :
           "ethercat_enable: enabled and holding current positions; use ethercat_disable to stop, or Ctrl-C to disable on exit.");
    console_ethercat_background_mark_ready();
    deadline = hold_ms == 0u ? UINT64_MAX : time_us() + (uint64_t)hold_ms * 1000u;
    failure_stage = "enabled-pdo-hold";
    while (!stop_requested && !console_ethercat_background_should_stop() &&
           time_us() < deadline) {
        if (console_ethercat_exchange() != 0) {
            printf("%s\n", chinese ? "ethercat_enable: PDO 通信中断" :
                   "ethercat_enable: PDO communication lost");
            goto cleanup;
        }
        console_ethercat_capture_live_pdo(selected);
        console_ethercat_wait_next_cycle();
    }
    result = stop_requested || console_ethercat_background_should_stop() ? -1 : 0;

cleanup:
    if (result != 0 && !stop_requested &&
        !console_ethercat_background_should_stop()) {
        printf("%s\n", chinese ?
               "ethercat_enable: 使能失败，自动采集现场诊断信息（失能前）" :
               "ethercat_enable: enable failed; collecting diagnostics before disabling");
        console_ethercat_print_selected_status(selected, diagnostic_control_word, false);
    }
    if (mapped) {
        console_ethercat_disable_selected(selected);
    }
    printf("%s\n", chinese ? "ethercat_enable: 已发送失能并关闭 EtherCAT 主站" :
           "ethercat_enable: disable sent and EtherCAT master closed");
close_socket:
    ethercat_control_period_ns = ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS;
    ec_close();
    console_ethercat_failure_log_finish(
        &failure_capture,
        result != 0 && !stop_requested && !console_ethercat_background_should_stop(),
        chinese, interface, selection, sync0_cycle_ns, sync0_shift_ns,
        failure_stage);
    return result;
#endif
}

static int console_ethercat_disable(bool chinese,
                                    const char *interface,
                                    const char *selection);

static void *console_ethercat_background_worker(void *argument)
{
    ethercat_background_args *args = argument;
    int result;

    if (args->disable_task) {
        result = console_ethercat_disable(args->chinese,
                                          args->interface,
                                          args->selection);
    } else {
        result = console_ethercat_enable(args->chinese,
                                         args->interface,
                                         args->selection,
                                         args->hold_ms,
                                         args->has_sync0_shift ? args->sync0_shift : NULL,
                                         args->has_sync0_cycle_ms ? args->sync0_cycle_ms : NULL);
    }

    pthread_mutex_lock(&ethercat_background_mutex);
    ethercat_background_result = result;
    ethercat_background_running = false;
    pthread_cond_broadcast(&ethercat_background_condition);
    pthread_mutex_unlock(&ethercat_background_mutex);
    return NULL;
}

static int console_ethercat_background_stop(void);

static void console_ethercat_background_reap(void)
{
    pthread_t thread;
    bool should_join;

    pthread_mutex_lock(&ethercat_background_mutex);
    should_join = ethercat_background_started && !ethercat_background_running;
    thread = ethercat_background_thread;
    if (should_join) {
        ethercat_background_started = false;
    }
    pthread_mutex_unlock(&ethercat_background_mutex);
    if (should_join) {
        pthread_join(thread, NULL);
    }
}

static bool console_ethercat_background_is_running(void)
{
    bool running;

    pthread_mutex_lock(&ethercat_background_mutex);
    running = ethercat_background_running;
    pthread_mutex_unlock(&ethercat_background_mutex);
    return running;
}

static int console_ethercat_background_stop(void)
{
    pthread_t thread;
    bool should_join;
    bool was_running;

    pthread_mutex_lock(&ethercat_background_mutex);
    was_running = ethercat_background_running;
    if (was_running) {
        ethercat_background_stop_requested = true;
    }
    should_join = ethercat_background_started;
    thread = ethercat_background_thread;
    pthread_mutex_unlock(&ethercat_background_mutex);

    if (should_join) {
        pthread_join(thread, NULL);
        pthread_mutex_lock(&ethercat_background_mutex);
        ethercat_background_started = false;
        ethercat_background_running = false;
        ethercat_background_ready = false;
        ethercat_background_stop_requested = false;
        pthread_mutex_unlock(&ethercat_background_mutex);
    }
    return was_running ? 1 : 0;
}

static int console_ethercat_background_disable(const char *interface,
                                              const char *selection)
{
    bool active;
    bool selection_matches;

    pthread_mutex_lock(&ethercat_background_mutex);
    active = ethercat_background_running;
    selection_matches = strcmp(ethercat_background_arguments.selection, "all") == 0 ?
                        strcmp(selection, "all") == 0 :
                        (strcmp(ethercat_background_arguments.selection, selection) == 0 ||
                         strcmp(selection, "all") == 0);
    if (!active) {
        pthread_mutex_unlock(&ethercat_background_mutex);
        return 0;
    }
    if (strcmp(ethercat_background_arguments.interface, interface) != 0 ||
        !selection_matches) {
        pthread_mutex_unlock(&ethercat_background_mutex);
        return -1;
    }
    ethercat_background_stop_requested = true;
    pthread_mutex_unlock(&ethercat_background_mutex);
    console_ethercat_background_stop();
    return 1;
}

static int console_ethercat_background_start(bool chinese,
                                             const char *interface,
                                             const char *selection,
                                             unsigned int hold_ms,
                                             const char *sync0_shift,
                                             const char *sync0_cycle_ms)
{
    int wait_result = 0;
    bool ready;
    bool running;
    unsigned int slave_id;
    bool all_slaves;
    int32_t parsed_shift_ns;
    uint32_t parsed_cycle_ns;

    if (console_ethercat_validate_interface(interface) != 0 ||
        console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0 ||
        hold_ms > ETHERCAT_KAIXUAN_ENABLE_MAX_HOLD_MS ||
        (sync0_shift != NULL &&
         console_ethercat_parse_sync0_shift(sync0_shift, &parsed_shift_ns) != 0) ||
        (sync0_cycle_ms != NULL &&
         console_ethercat_parse_cycle_ms(sync0_cycle_ms, &parsed_cycle_ns) != 0)) {
        printf("%s\n", chinese ?
               "ethercat_enable 参数无效；检查从站、保持时间、shift 和 Sync0 周期。" :
               "Invalid ethercat_enable arguments; check slave, hold time, shift, and Sync0 cycle.");
        return -1;
    }

    console_ethercat_background_reap();
    pthread_mutex_lock(&ethercat_background_mutex);
    if (ethercat_background_running) {
        pthread_mutex_unlock(&ethercat_background_mutex);
        printf("%s\n", chinese ? "EtherCAT 后台主站已在运行；先使用 ethercat_disable 停止。" :
               "EtherCAT background master is already running; stop it with ethercat_disable first.");
        return -1;
    }
    memset(&ethercat_background_arguments, 0, sizeof(ethercat_background_arguments));
#ifdef HAVE_ETHERLAB
    memset(ethercat_cached_slaves, 0, sizeof(ethercat_cached_slaves));
#endif
    ethercat_background_arguments.chinese = chinese;
    ethercat_background_arguments.hold_ms = hold_ms;
    snprintf(ethercat_background_arguments.interface,
             sizeof(ethercat_background_arguments.interface), "%s", interface);
    snprintf(ethercat_background_arguments.selection,
             sizeof(ethercat_background_arguments.selection), "%s", selection);
    if (sync0_shift != NULL) {
        snprintf(ethercat_background_arguments.sync0_shift,
                 sizeof(ethercat_background_arguments.sync0_shift), "%s", sync0_shift);
        ethercat_background_arguments.has_sync0_shift = true;
    }
    if (sync0_cycle_ms != NULL) {
        snprintf(ethercat_background_arguments.sync0_cycle_ms,
                 sizeof(ethercat_background_arguments.sync0_cycle_ms), "%s", sync0_cycle_ms);
        ethercat_background_arguments.has_sync0_cycle_ms = true;
    }
    ethercat_background_stop_requested = false;
    ethercat_background_ready = false;
    ethercat_background_running = true;
    ethercat_background_started = true;
    if (pthread_create(&ethercat_background_thread, NULL,
                       console_ethercat_background_worker,
                       &ethercat_background_arguments) != 0) {
        ethercat_background_running = false;
        ethercat_background_started = false;
        pthread_mutex_unlock(&ethercat_background_mutex);
        printf("%s\n", chinese ? "无法创建 EtherCAT 后台任务。" :
               "Failed to create EtherCAT background task.");
        return -1;
    }
    while (ethercat_background_running && !ethercat_background_ready && wait_result == 0) {
        wait_result = pthread_cond_wait(&ethercat_background_condition,
                                        &ethercat_background_mutex);
    }
    ready = ethercat_background_ready;
    running = ethercat_background_running;
    pthread_mutex_unlock(&ethercat_background_mutex);
    if (wait_result != 0) {
        console_ethercat_background_stop();
        return -1;
    }
    if (!ready || !running) {
        console_ethercat_background_reap();
        return -1;
    }
    printf("%s\n", chinese ? "EtherCAT 后台循环已启动；可继续输入命令。" :
           "EtherCAT background cycle is running; you can continue entering commands.");
    return 0;
}

static int console_ethercat_background_disable_start(bool chinese,
                                                     const char *interface,
                                                     const char *selection)
{
    unsigned int slave_id;
    bool all_slaves;

    if (console_ethercat_validate_interface(interface) != 0 ||
        console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0) {
        printf("%s\n", chinese ?
               "ethercat_disable 参数无效；请检查网卡和从站编号。" :
               "Invalid ethercat_disable arguments; check the interface and slave selection.");
        return -1;
    }
#ifndef HAVE_ETHERLAB
    return console_ethercat_disable(chinese, interface, selection);
#else
    console_ethercat_background_reap();
    pthread_mutex_lock(&ethercat_background_mutex);
    if (ethercat_background_running) {
        pthread_mutex_unlock(&ethercat_background_mutex);
        printf("%s\n", chinese ?
               "EtherCAT 后台主站仍在运行；请等待当前操作完成或先停止它。" :
               "The EtherCAT background master is busy; wait for it to finish or stop it first.");
        return -1;
    }
    memset(&ethercat_background_arguments, 0, sizeof(ethercat_background_arguments));
    ethercat_background_arguments.chinese = chinese;
    ethercat_background_arguments.disable_task = true;
    snprintf(ethercat_background_arguments.interface,
             sizeof(ethercat_background_arguments.interface), "%s", interface);
    snprintf(ethercat_background_arguments.selection,
             sizeof(ethercat_background_arguments.selection), "%s", selection);
    ethercat_background_stop_requested = false;
    ethercat_background_ready = false;
    ethercat_background_running = true;
    ethercat_background_started = true;
    printf("%s: interface=%s slave=%s %s\n",
           chinese ? "ethercat_disable: 开始" : "ethercat_disable: start",
           interface, selection,
           chinese ? "正在后台失能并释放 EtherCAT 主站。" :
           "disabling in background and releasing the EtherCAT master.");
    if (pthread_create(&ethercat_background_thread, NULL,
                       console_ethercat_background_worker,
                       &ethercat_background_arguments) != 0) {
        ethercat_background_running = false;
        ethercat_background_started = false;
        pthread_mutex_unlock(&ethercat_background_mutex);
        printf("%s\n", chinese ? "无法创建 EtherCAT 后台任务。" :
               "Failed to create EtherCAT background task.");
        return -1;
    }
    pthread_mutex_unlock(&ethercat_background_mutex);
    return 0;
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

#ifndef HAVE_ETHERLAB
    printf("%s\n", chinese ?
           "当前程序未编译 EtherLab，无法执行 EtherCAT 位控。请安装 EtherLab 主站开发库后重新构建。" :
           "EtherCAT position control is unavailable because EtherLab is not installed; install the EtherLab master development library and rebuild.");
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
    if (console_ethercat_enable_dc_sync(selected, 0,
                                        ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS) != 0) {
        printf("%s\n", chinese ? "ethercat_position: 目标从站不支持 DC Sync0" :
               "ethercat_position: selected slave does not support DC Sync0");
        goto cleanup;
    }
    if ((ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) & 0x0fu) != EC_STATE_SAFE_OP) {
        printf("%s\n", chinese ? "ethercat_position: 从站未进入 SAFE-OP" :
               "ethercat_position: slaves did not reach SAFE-OP");
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
        console_ethercat_print_selected_status(selected, 0x0006u, true);
        goto cleanup;
    }
    console_ethercat_set_control_word(selected, 0x0007u);
    if (console_ethercat_wait_for_status(selected, 0x0023u) != 0) {
        printf("%s\n", chinese ? "ethercat_position: 0x0007 状态确认失败" :
               "ethercat_position: 0x0007 state confirmation failed");
        console_ethercat_print_selected_status(selected, 0x0007u, true);
        goto cleanup;
    }
    console_ethercat_set_control_word(selected, 0x000fu);
    if (console_ethercat_wait_for_status(selected, 0x0027u) != 0) {
        printf("%s\n", chinese ? "ethercat_position: 0x000F 状态确认失败" :
               "ethercat_position: 0x000F state confirmation failed");
        console_ethercat_print_selected_status(selected, 0x000fu, true);
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

#ifndef HAVE_ETHERLAB
    printf("%s\n", chinese ?
           "当前程序未编译 EtherLab，无法设置 EtherCAT 零位。请安装 EtherLab 主站开发库后重新构建。" :
           "EtherCAT zero setting is unavailable because EtherLab is not installed; install the EtherLab master development library and rebuild.");
    return -1;
#else
    uint8_t process_image[ETHERCAT_KAIXUAN_PROCESS_IMAGE_SIZE];
    uint8_t selected[EC_MAXSLAVE] = {0};
    int32_t zero_positions[EC_MAXSLAVE] = {0};
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
    if (console_ethercat_enable_dc_sync(selected, 0,
                                        ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS) != 0) {
        printf("%s\n", chinese ? "ethercat_zero: 目标从站不支持 DC Sync0" :
               "ethercat_zero: selected slave does not support DC Sync0");
        goto cleanup;
    }
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
        int sequence_result = 0;

        if (selected[slave] == 0u) {
            continue;
        }
        if (console_ethercat_write_pn101_verified(
                chinese, slave, 0, chinese ? "步骤 1/3 复位为 0" : "step 1/3 reset to 0") != 0) {
            sequence_result = -1;
        }
        if (sequence_result == 0 &&
            console_ethercat_write_pn101_verified(
                chinese, slave, 1, chinese ? "步骤 2/3 触发上升沿" : "step 2/3 rising-edge trigger") != 0) {
            sequence_result = -1;
        }
        if (sequence_result == 0 &&
            console_ethercat_write_pn101_verified(
                chinese, slave, 0, chinese ? "步骤 3/3 恢复为 0" : "step 3/3 restore to 0") != 0) {
            sequence_result = -1;
        }
        if (sequence_result != 0) {
            printf("[slave%d]: Pn101 %s\n", slave,
                   chinese ? "序列未完整确认，尝试恢复为 0" :
                   "sequence was not fully verified; attempting recovery to 0");
            if (console_ethercat_write_pn101_verified(
                    chinese, slave, 0, chinese ? "故障恢复" : "recovery") != 0) {
                printf("[slave%d]: Pn101 %s\n", slave,
                       chinese ? "恢复为 0 也未能确认，请勿断言零位请求状态" :
                       "recovery to 0 was not verified; zero-request state is unknown");
            }
            failed++;
            continue;
        }
        printf("[slave%d]: %s\n", slave, chinese ?
               "Pn101 0->1->0 每步写入并回读确认；未增加固定延时" :
               "Pn101 0->1->0 write/readback verified at each step; no fixed delay added");
    }
    if (failed == 0) {
        printf("%s\n", chinese ?
               "ethercat_zero: Pn101 三步写入均已回读确认；按手册同时重启执行器主电和 USB 电源后，再读取 0x6064 验证零位。" :
               "ethercat_zero: all three Pn101 writes were verified by readback; restart actuator main and USB power together per the manual, then verify zero via 0x6064.");
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

#ifndef HAVE_ETHERLAB
    printf("%s\n", chinese ?
           "当前程序未编译 EtherLab，无法读取 EtherCAT 电机信息。请安装 EtherLab 主站开发库后重新构建。" :
           "EtherCAT info is unavailable because EtherLab is not installed; install the EtherLab master development library and rebuild.");
    return -1;
#else
    int slave;
    int failed = 0;
    int background_info_result;

    background_info_result = console_ethercat_print_background_info(chinese, interface, selection);
    if (background_info_result > 0) {
        return 0;
    }
    if (background_info_result < 0) {
        console_ethercat_background_stop();
        printf("%s\n", chinese ?
               "请求节点不在后台缓存中；已先安全停止后台循环，再执行独立 SDO 查询。" :
               "Requested slave is not in the background snapshot; stopped the cycle before opening a read-only SDO session.");
    }

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
        int16_t pn051;
        int16_t pn150;
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
        bool pn051_ok;
        bool pn150_ok;
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
        pn051_ok = ETHERCAT_INFO_READ(0x2051u, pn051);
        pn150_ok = ETHERCAT_INFO_READ(0x2150u, pn150);
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
        printf("\n  mechanics: Pn051=");
        if (pn051_ok) printf("%d (reduction ratio)", (int)pn051);
        else printf("?");
        printf("\n  encoder: Pn150=");
        if (pn150_ok) {
            printf("%d(%s)", (int)pn150,
                   pn150 == 0 ?
                   (chinese ? "双编码器" : "dual encoder") :
                   (pn150 == 1 ?
                    (chinese ? "单编码器/多圈" : "single multi-turn encoder") :
                    (chinese ? "未知配置值" : "unknown value")));
        } else {
            printf("?");
        }
        printf("\n  bus: PDO=");
        if (ec_slave[slave].Obytes == 0u && ec_slave[slave].Ibytes == 0u) {
            printf("unmapped in read-only session");
        } else {
            printf("out=%uB/%ubit in=%uB/%ubit",
                   (unsigned int)ec_slave[slave].Obytes, (unsigned int)ec_slave[slave].Obits,
                   (unsigned int)ec_slave[slave].Ibytes, (unsigned int)ec_slave[slave].Ibits);
        }
        printf(" DC_supported=%s active=%u actreg=",
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

        printf("  actual: motor_pos=");
        if (position_actual_ok) {
            printf("%.6frad (%d count)",
                   (double)position_actual * ETHERCAT_KAIXUAN_TWO_PI /
                   ETHERCAT_KAIXUAN_COUNTS_PER_REV, position_actual);
        } else printf("?");
        printf(" output_pos=");
        if (position_actual_ok && pn051_ok && pn051 > 0) {
            printf("%.6frad",
                   (double)position_actual * ETHERCAT_KAIXUAN_TWO_PI /
                   ETHERCAT_KAIXUAN_COUNTS_PER_REV / (double)pn051);
        } else {
            printf("?");
        }
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

static int console_ethercat_pnread(bool chinese,
                                   const char *interface,
                                   const char *selection,
                                   uint16_t pn_number)
{
    unsigned int slave_id;
    bool all_slaves;

    if (console_ethercat_validate_interface(interface) != 0 ||
        console_ethercat_parse_slave_selection(selection, &slave_id, &all_slaves) != 0 ||
        pn_number > 0xdfffu) {
        printf("%s: ethercat_pnread <slave_id|all> <Pn编号> [network_interface]\n",
               chinese ? "用法" : "usage");
        return -1;
    }

#ifndef HAVE_ETHERLAB
    printf("%s\n", chinese ?
           "当前程序未编译 EtherLab，无法读取 Pn 参数。请安装 EtherLab 主站开发库后重新构建。" :
           "Pn parameter reads are unavailable because EtherLab is not installed.");
    return -1;
#else
    uint8_t selected[EC_MAXSLAVE] = {0};
    uint16_t index = (uint16_t)(0x2000u + pn_number);
    int slave;
    int failed = 0;

    printf("%s: interface=%s slave=%s Pn%u index=0x%04x:00\n",
           chinese ? "ethercat_pnread: 开始读取" : "ethercat_pnread: reading",
           interface, selection, (unsigned int)pn_number, (unsigned int)index);
    if (ec_init((char *)interface) == 0) {
        printf("%s: %s\n", chinese ? "ethercat_pnread: 打开网卡失败" :
               "ethercat_pnread: failed to open interface", interface);
        return -1;
    }
    if (ec_config_init(FALSE) <= 0) {
        printf("%s\n", chinese ? "ethercat_pnread: 未发现 EtherCAT 从站" :
               "ethercat_pnread: no EtherCAT slaves found");
        goto close_socket;
    }
    if (!all_slaves && slave_id > (unsigned int)ec_slavecount) {
        printf("%s: %u (1..%d)\n", chinese ? "ethercat_pnread: 从站序号不存在" :
               "ethercat_pnread: slave id is out of range", slave_id, ec_slavecount);
        goto close_socket;
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        if (all_slaves || (unsigned int)slave == slave_id) {
            selected[slave] = 1u;
        }
    }
    for (slave = 1; slave <= ec_slavecount; slave++) {
        uint8_t data[256] = {0};
        uint64_t raw_value = 0u;
        int actual_size = (int)sizeof(data);
        int result;
        int byte_index;

        if (selected[slave] == 0u) {
            continue;
        }
        if (ec_slave[slave].eep_man != ETHERCAT_KAIXUAN_VENDOR_ID ||
            ec_slave[slave].eep_id != ETHERCAT_KAIXUAN_PRODUCT_CODE) {
            printf("[slave%d]: %s\n", slave, chinese ?
                   "不是当前 Pn 编号映射所支持的开璇驱动器，跳过读取" :
                   "not a supported Kaixuan drive for this Pn mapping; skipping");
            failed++;
            continue;
        }
        result = ec_SDOread((uint16)slave, index, 0u, FALSE,
                            &actual_size, data, EC_TIMEOUTRXM);
        if (result <= 0 || actual_size <= 0 || actual_size > (int)sizeof(data)) {
            printf("[slave%d]: Pn%u index=0x%04x:00 %s\n", slave,
                   (unsigned int)pn_number, (unsigned int)index,
                   chinese ? "读取失败或数据长度无效" : "read failed or invalid data length");
            failed++;
            continue;
        }
        printf("[slave%d]: Pn%u index=0x%04x:00 size=%d raw=[",
               slave, (unsigned int)pn_number, (unsigned int)index, actual_size);
        for (byte_index = 0; byte_index < actual_size; byte_index++) {
            printf("%s%02x", byte_index == 0 ? "" : " ", data[byte_index]);
            if (byte_index < 8) {
                raw_value |= (uint64_t)data[byte_index] << (8u * (unsigned int)byte_index);
            }
        }
        printf("]");
        if (actual_size <= 8) {
            printf(" little_endian=0x%llx (%llu)",
                   (unsigned long long)raw_value,
                   (unsigned long long)raw_value);
            if (actual_size <= 4) {
                uint64_t sign_bit = 1ull << (8u * (unsigned int)actual_size - 1u);
                int64_t signed_value = (int64_t)raw_value;

                if ((raw_value & sign_bit) != 0u) {
                    signed_value -= (int64_t)(1ull << (8u * (unsigned int)actual_size));
                }
                printf(" signed=%lld", (long long)signed_value);
            }
        }
        printf("\n");
    }
    printf("%s\n", chinese ?
           "读取完成；未配置 PDO 或请求 OP。输出为 SDO 原始字节及小端数值，类型/单位请以驱动器手册为准。" :
           "Read complete; no PDO configuration or OP request. Raw bytes and little-endian values shown; consult the manual for type and units.");
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

#ifndef HAVE_ETHERLAB
    printf("%s\n", chinese ?
           "当前程序未编译 EtherLab，无法写入 Pn077。请安装 EtherLab 主站开发库后重新构建。" :
           "Pn077 write is unavailable because EtherLab is not installed.");
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

#ifndef HAVE_ETHERLAB
    printf("%s\n", chinese ?
           "当前程序未编译 EtherLab，无法保存 EtherCAT 参数。请安装 EtherLab 主站开发库后重新构建。" :
           "EtherCAT parameter save is unavailable because EtherLab is not installed.");
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

#ifndef HAVE_ETHERLAB
    printf("%s\n", chinese ?
           "当前程序未编译 EtherLab，无法失能 EtherCAT 电机。请安装 EtherLab 主站开发库后重新构建。" :
           "EtherCAT disable is unavailable because EtherLab is not installed; install the EtherLab master development library and rebuild.");
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
    if (console_ethercat_enable_dc_sync(selected, 0,
                                        ETHERCAT_KAIXUAN_CONTROL_PERIOD_NS) != 0) {
        printf("%s\n", chinese ? "ethercat_disable: 目标从站不支持 DC Sync0" :
               "ethercat_disable: selected slave does not support DC Sync0");
        goto cleanup;
    }
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
    printf("%s\n", chinese ? "ethercat_disable: 已发送 0x0000；关闭主站并释放 EtherLab master" :
           "ethercat_disable: sent 0x0000; deactivating and releasing the EtherLab master");
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
