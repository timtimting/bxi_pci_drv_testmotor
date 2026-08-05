#define _DEFAULT_SOURCE

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <dirent.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include "bxi_motor_comm.h"
#include "bxi_pci_drv.h"

#define DEFAULT_FIRMWARE_DIR "firmware"
#define DEFAULT_FIRMWARE_MAP "config/motor_console.yaml"
#define DEFAULT_BOOT_ENTER_DELAY_MS 100u
#define DEFAULT_BOOT_UPDATE_DELAY_MS 200u
#define DEFAULT_BOOT_TX_GAP_MS 1u
#define DEFAULT_BOOT_TX_RETRY_COUNT 5u
#define DEFAULT_BOOT_TX_RETRY_DELAY_MS 2u

enum {
    CAN_CMD_REG_WRITE = 0x11,
    CAN_CMD_REG_READ = 0x12,
    CAN_CMD_REG_SAVE = 0x13,
    CAN_CMD_REG_RESET_ALL = 0x14,
    CAN_CMD_REG_FW_VERSION = 0x15,
};

enum {
    YMODEM_SOH = 0x01,
    YMODEM_STX = 0x02,
    YMODEM_EOT = 0x04,
    YMODEM_ACK = 0x06,
    YMODEM_NAK = 0x15,
    YMODEM_CAN = 0x18,
    YMODEM_C = 0x43,
    YMODEM_PACKET_128 = 128,
    YMODEM_PACKET_1K = 1024,
    YMODEM_FRAME_PREFIX = 3,
    YMODEM_FRAME_SUFFIX = 2,
    YMODEM_MAX_PACKET = YMODEM_FRAME_PREFIX + YMODEM_PACKET_1K + YMODEM_FRAME_SUFFIX,
    RX_RING_SIZE = 16384,
    FRAME_RING_SIZE = 256,
    HISTORY_DEPTH = 32,
    LINE_LEN = 512,
    MOTOR_TYPE_LEN = 32,
    MOTOR_NAME_LEN = 64,
    MOTOR_STATE_LEN = 32,
    STATE_PATTERN_LEN = 128,
    STATE_PATTERN_MAX = 32,
    STATE_CODE_MAX = 32,
    PATH_LEN = 512,
    MOTOR_MAP_MAX = 256,
    FIRMWARE_FILE_MAX = 256,
    FIRMWARE_NAME_MAX = 128,
    FIRMWARE_MAPPING_MAX = 16,
    MOTOR_TYPE_CONFIG_MAX = 16,
};

typedef enum
{
    TOOL_MODE_FLASH = 0,
    TOOL_MODE_DEBUG,
} tool_mode;

typedef struct
{
    uint8_t data[RX_RING_SIZE];
    size_t head;
    size_t tail;
    size_t used;
    pthread_mutex_t lock;
    pthread_cond_t cond;
} rx_ring;

typedef struct
{
    unsigned int bus;
    unsigned int can_id;
    unsigned int len;
    unsigned int flags;
    uint8_t data[CANFD_MAX_DLEN];
    uint64_t time_us;
} rx_can_frame;

typedef struct
{
    rx_can_frame data[FRAME_RING_SIZE];
    size_t head;
    size_t tail;
    size_t used;
    pthread_mutex_t lock;
    pthread_cond_t cond;
} frame_ring;

typedef struct
{
    unsigned int index;
    unsigned int bus;
    unsigned int id;
    unsigned int line_no;
    char type[MOTOR_TYPE_LEN];
    char name[MOTOR_NAME_LEN];
} motor_map_entry;

typedef struct
{
    char type[MOTOR_TYPE_LEN];
    char filename[FIRMWARE_NAME_MAX];
} firmware_mapping;

typedef struct
{
    char type[MOTOR_TYPE_LEN];
    char firmware[FIRMWARE_NAME_MAX];
    bxi_motor_limits limits;
} motor_type_config;

typedef struct
{
    char label[MOTOR_STATE_LEN];
    char pattern[STATE_PATTERN_LEN];
} state_pattern_rule;

typedef struct
{
    unsigned int code;
    char label[MOTOR_STATE_LEN];
} state_code_rule;

typedef struct
{
    char firmware_dir[PATH_LEN];
    char firmware_base_url[PATH_LEN];
    char firmware_cache_dir[PATH_LEN];
    bool firmware_sync_on_start;
    float home_kp;
    float home_kd;
    float home_kp_by_index[MOTOR_MAP_MAX];
    float home_kd_by_index[MOTOR_MAP_MAX];
    bool home_kp_by_index_set[MOTOR_MAP_MAX];
    bool home_kd_by_index_set[MOTOR_MAP_MAX];
    unsigned int home_soft_start_ms;
    unsigned int scan_timeout_ms;
    unsigned int power_on_wait_ms;
    unsigned int can_warmup_count;
    unsigned int can_warmup_period_ms;
    char state_boot_pattern[STATE_PATTERN_LEN];
    char state_motor_pattern[STATE_PATTERN_LEN];
    char state_no_app_pattern[STATE_PATTERN_LEN];
    char state_boot_menu_pattern[STATE_PATTERN_LEN];
    char state_motor_menu_pattern[STATE_PATTERN_LEN];
    char state_menu_pattern[STATE_PATTERN_LEN];
    char state_code_prefix[STATE_PATTERN_LEN];
    state_pattern_rule state_patterns[STATE_PATTERN_MAX];
    size_t state_pattern_count;
    state_code_rule state_codes[STATE_CODE_MAX];
    size_t state_code_count;
    bxi_motor_limits mit_limits;
    bool mit_canfd;
    bool live_output;
    bool chinese_ui;
    firmware_mapping firmware_mappings[FIRMWARE_MAPPING_MAX];
    size_t firmware_mapping_count;
    motor_type_config type_configs[MOTOR_TYPE_CONFIG_MAX];
    size_t type_config_count;
    motor_map_entry entries[MOTOR_MAP_MAX];
    size_t entry_count;
} motor_map_config;

typedef struct
{
    char type[MOTOR_TYPE_LEN];
    char path[PATH_LEN];
} firmware_file;

typedef struct
{
    firmware_file files[FIRMWARE_FILE_MAX];
    size_t count;
} firmware_inventory;

typedef struct
{
    bool online;
    bool enabled;
    uint64_t last_reply_us;
    bxi_motor_reply last_reply;
    bool aux_valid[16];
    float aux_value[16];
    uint16_t aux_payload[16];
    unsigned int rx_count;
    unsigned int timeout_count;
    char state_label[MOTOR_STATE_LEN];
} motor_runtime;

typedef struct
{
    uint64_t tx_packets;
    uint64_t tx_failed;
    uint64_t rx_packets;
    uint64_t expected_replies;
    uint64_t received_replies;
    uint64_t decoded_motor_replies;
    uint64_t outstanding_replies;
    uint64_t reply_timeouts;
} can_bus_stats;

typedef struct
{
    unsigned int bus;
    unsigned int boot_id;
    unsigned int tx_id;
    unsigned int rx_id;
    unsigned int master_id;
    tool_mode mode;
    bool auto_power_cycle;
    bool debug_use_canfd;
    bool quiet_tx;
    bool pci_started;
    rx_ring rx;
    frame_ring frames;
    bxi_motor_limits limits;
    motor_map_config config;
    motor_runtime motors[MOTOR_MAP_MAX];
    can_bus_stats can_stats[CANFD_DEVICE_NUM];
    bool config_loaded;
    bool motor_power_on;
    bool show_can_output;
    bool show_motor_input;
    bool suppress_boot_text;
    bool brief_flash_output;
    bool brief_register_output;
    bool firmware_prefetch_ready;
    char active_firmware_dir[PATH_LEN];
    unsigned int brief_flash_index;
    size_t brief_flash_ordinal;
    size_t brief_flash_total;
    bool language_override_active;
    bool chinese_override;
    char config_path[PATH_LEN];
} flash_state;

static const char *state_label_for_code(const motor_map_config *config,
                                        unsigned int code);

static volatile sig_atomic_t stop_requested = 0;
static char command_history[HISTORY_DEPTH][LINE_LEN];
static size_t command_history_count = 0u;

static int set_target_id(flash_state *state, unsigned int id);
static const bxi_motor_limits *limits_for_entry(const flash_state *state,
                                                const motor_map_entry *entry);

static uint64_t time_us(void)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static void sleep_ms(unsigned int ms)
{
    while (ms > 0u && !stop_requested) {
        unsigned int step = ms > 100u ? 100u : ms;
        usleep((useconds_t)step * 1000u);
        ms -= step;
    }
}

static void on_signal(int sig)
{
    (void)sig;
    stop_requested = 1;
}

static unsigned int clean_can_id(canid_t can_id)
{
    if ((can_id & CAN_EFF_FLAG) != 0u) {
        return can_id & CAN_EFF_MASK;
    }
    return can_id & CAN_SFF_MASK;
}

static bool is_hardware_error_frame(canid_t can_id)
{
    return (can_id & CAN_ERR_FLAG) != 0u;
}

static void update_boot_ids(flash_state *state)
{
    unsigned int id = state->boot_id & 0x0fu;

    state->tx_id = 0x7e0u | id;     /* Host -> motor boot/debug terminal. */
    state->rx_id = 0x7f0u | id;     /* Motor -> host boot/debug terminal. */
    state->master_id = 0x10u | id;  /* Motor -> host MIT feedback. */
}

static int parse_uint_arg(const char *text, unsigned int *value)
{
    char *end = NULL;
    unsigned long parsed;

    errno = 0;
    parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed > 0xfffffffful) {
        return -1;
    }

    *value = (unsigned int)parsed;
    return 0;
}

static int parse_on_off(const char *text, bool *value)
{
    if (strcmp(text, "on") == 0 || strcmp(text, "1") == 0 || strcmp(text, "true") == 0) {
        *value = true;
        return 0;
    }
    if (strcmp(text, "off") == 0 || strcmp(text, "0") == 0 || strcmp(text, "false") == 0) {
        *value = false;
        return 0;
    }
    return -1;
}

static int parse_float_arg(const char *text, float *value)
{
    char *end = NULL;
    float parsed;

    errno = 0;
    parsed = strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0') {
        return -1;
    }

    *value = parsed;
    return 0;
}

static void u32_to_data(uint32_t value, uint8_t *data)
{
    data[0] = (uint8_t)((value >> 0) & 0xffu);
    data[1] = (uint8_t)((value >> 8) & 0xffu);
    data[2] = (uint8_t)((value >> 16) & 0xffu);
    data[3] = (uint8_t)((value >> 24) & 0xffu);
}

static uint32_t data_to_u32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 0) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static float data_to_float(const uint8_t *data)
{
    uint32_t raw = data_to_u32(data);
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static void rx_ring_init(rx_ring *ring)
{
    memset(ring, 0, sizeof(*ring));
    pthread_mutex_init(&ring->lock, NULL);
    pthread_cond_init(&ring->cond, NULL);
}

static void rx_ring_clear(rx_ring *ring)
{
    pthread_mutex_lock(&ring->lock);
    ring->head = 0u;
    ring->tail = 0u;
    ring->used = 0u;
    pthread_mutex_unlock(&ring->lock);
}

static void rx_ring_push(rx_ring *ring, const uint8_t *data, size_t len)
{
    size_t i;

    pthread_mutex_lock(&ring->lock);
    for (i = 0u; i < len; i++) {
        if (ring->used == RX_RING_SIZE) {
            ring->tail = (ring->tail + 1u) % RX_RING_SIZE;
            ring->used--;
        }
        ring->data[ring->head] = data[i];
        ring->head = (ring->head + 1u) % RX_RING_SIZE;
        ring->used++;
    }
    pthread_cond_broadcast(&ring->cond);
    pthread_mutex_unlock(&ring->lock);
}

static int rx_ring_pop(rx_ring *ring, uint8_t *byte, unsigned int timeout_ms)
{
    int ret = 0;

    pthread_mutex_lock(&ring->lock);
    if (timeout_ms == 0u) {
        if (ring->used == 0u) {
            pthread_mutex_unlock(&ring->lock);
            return 0;
        }
    } else {
        struct timespec ts;

        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += (time_t)(timeout_ms / 1000u);
        ts.tv_nsec += (long)(timeout_ms % 1000u) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000L;
        }

        while (ring->used == 0u && !stop_requested) {
            int wait_ret = pthread_cond_timedwait(&ring->cond, &ring->lock, &ts);
            if (wait_ret == ETIMEDOUT) {
                break;
            }
        }
    }

    if (ring->used > 0u) {
        *byte = ring->data[ring->tail];
        ring->tail = (ring->tail + 1u) % RX_RING_SIZE;
        ring->used--;
        ret = 1;
    }
    pthread_mutex_unlock(&ring->lock);
    return ret;
}

static void frame_ring_init(frame_ring *ring)
{
    memset(ring, 0, sizeof(*ring));
    pthread_mutex_init(&ring->lock, NULL);
    pthread_cond_init(&ring->cond, NULL);
}

static void frame_ring_clear(frame_ring *ring)
{
    pthread_mutex_lock(&ring->lock);
    ring->head = 0u;
    ring->tail = 0u;
    ring->used = 0u;
    pthread_mutex_unlock(&ring->lock);
}

static void frame_ring_push(frame_ring *ring, const rx_can_frame *frame)
{
    pthread_mutex_lock(&ring->lock);
    if (ring->used == FRAME_RING_SIZE) {
        ring->tail = (ring->tail + 1u) % FRAME_RING_SIZE;
        ring->used--;
    }
    ring->data[ring->head] = *frame;
    ring->head = (ring->head + 1u) % FRAME_RING_SIZE;
    ring->used++;
    pthread_cond_broadcast(&ring->cond);
    pthread_mutex_unlock(&ring->lock);
}

static int frame_ring_pop(frame_ring *ring, rx_can_frame *frame, unsigned int timeout_ms)
{
    int ret = 0;

    pthread_mutex_lock(&ring->lock);
    if (timeout_ms == 0u) {
        if (ring->used == 0u) {
            pthread_mutex_unlock(&ring->lock);
            return 0;
        }
    } else {
        struct timespec ts;

        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += (time_t)(timeout_ms / 1000u);
        ts.tv_nsec += (long)(timeout_ms % 1000u) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000L;
        }

        while (ring->used == 0u && !stop_requested) {
            int wait_ret = pthread_cond_timedwait(&ring->cond, &ring->lock, &ts);
            if (wait_ret == ETIMEDOUT) {
                break;
            }
        }
    }

    if (ring->used > 0u) {
        *frame = ring->data[ring->tail];
        ring->tail = (ring->tail + 1u) % FRAME_RING_SIZE;
        ring->used--;
        ret = 1;
    }
    pthread_mutex_unlock(&ring->lock);
    return ret;
}

static bool boot_output_id_matches(unsigned int can_id, unsigned int motor_id)
{
    return motor_id > 0u && motor_id <= 0x0fu &&
           can_id == (0x7f0u | (motor_id & 0x0fu));
}

static int is_reg_cmd_id(unsigned int can_id);

static bool mit_reply_frame_matches(const rx_can_frame *frame, unsigned int motor_id)
{
    unsigned int master_id = 0x10u | (motor_id & 0x0fu);

    if (frame == NULL || motor_id == 0u || motor_id > 0x0fu ||
        frame->len < BXI_MOTOR_MIT_LEN) {
        return false;
    }
    if (is_reg_cmd_id(frame->can_id) || boot_output_id_matches(frame->can_id, motor_id)) {
        return false;
    }
    return frame->can_id == master_id && frame->data[0] == master_id;
}

static char boot_log_lines[MOTOR_MAP_MAX][LINE_LEN];
static size_t boot_log_line_lens[MOTOR_MAP_MAX];
static unsigned char boot_log_last_newline[MOTOR_MAP_MAX];

static void motor_runtime_set_state(motor_runtime *runtime, const char *state)
{
    if (runtime == NULL || state == NULL || state[0] == '\0') {
        return;
    }
    snprintf(runtime->state_label, sizeof(runtime->state_label), "%s", state);
}

static const char *motor_runtime_state(const motor_runtime *runtime)
{
    if (runtime == NULL || runtime->state_label[0] == '\0') {
        return "unknown";
    }
    return runtime->state_label;
}

static void motor_runtime_update_aux(const motor_map_config *config,
                                     motor_runtime *runtime,
                                     const bxi_motor_reply *reply)
{
    unsigned int fsm_state;

    if (runtime == NULL || reply == NULL || !reply->aux_valid ||
        reply->aux_id >= 16u || !reply->aux_payload_valid) {
        return;
    }

    runtime->aux_valid[reply->aux_id] = true;
    runtime->aux_value[reply->aux_id] = reply->aux_value;
    runtime->aux_payload[reply->aux_id] = reply->aux_payload;

    if (reply->aux_id == 0x0u) {
        runtime->last_reply.mos_temperature = reply->aux_value;
    } else if (reply->aux_id == 0x1u) {
        runtime->last_reply.motor_temperature = reply->aux_value;
    } else if (reply->aux_id == 0x7u) {
        fsm_state = reply->aux_payload & 0x0fu;
        motor_runtime_set_state(runtime, state_label_for_code(config, fsm_state));
        runtime->enabled = ((reply->aux_payload & (1u << 4)) != 0u);
    }
}

static void print_mit_aux_decode(const motor_map_config *config,
                                 const bxi_motor_reply *reply)
{
    unsigned int fsm_state;
    unsigned int control_mode;

    if (reply == NULL || !reply->aux_valid) {
        return;
    }
    printf(" aux=0x%x:%s", reply->aux_id, bxi_motor_aux_name(reply->aux_id));
    if (!reply->aux_payload_valid) {
        printf(" invalid");
        return;
    }
    if (reply->aux_id == 0x7u) {
        fsm_state = reply->aux_payload & 0x0fu;
        control_mode = (reply->aux_payload >> 9) & 0x07u;
        printf(" state=%s armed=%u enci=%u enco=%u fw=%u mit=%u ctrl=%s raw=0x%03x",
               state_label_for_code(config, fsm_state),
               (reply->aux_payload >> 4) & 0x01u,
               (reply->aux_payload >> 5) & 0x01u,
               (reply->aux_payload >> 6) & 0x01u,
               (reply->aux_payload >> 7) & 0x01u,
               (reply->aux_payload >> 8) & 0x01u,
               bxi_motor_control_mode_name(control_mode),
               reply->aux_payload);
        return;
    }
    printf(" value=% .3f%s raw=%u",
           reply->aux_value,
           bxi_motor_aux_unit(reply->aux_id),
           reply->aux_payload);
}

static bool motor_state_marker_from_line(const motor_map_config *config,
                                         const char *line,
                                         const char **state)
{
    const char *p;
    unsigned int fsm_state;

    if (config == NULL || line == NULL || state == NULL ||
        config->state_code_prefix[0] == '\0') {
        return false;
    }
    p = strstr(line, config->state_code_prefix);
    if (p == NULL) {
        return false;
    }
    p += strlen(config->state_code_prefix);
    while (*p != '\0' && isspace((unsigned char)*p)) {
        p++;
    }
    if (!isdigit((unsigned char)*p)) {
        return false;
    }
    fsm_state = (unsigned int)strtoul(p, NULL, 10);
    *state = state_label_for_code(config, fsm_state);
    return true;
}

static void update_motor_state_from_boot_line(flash_state *state,
                                              motor_runtime *runtime,
                                              const char *line)
{
    const motor_map_config *config;
    const char *state_from_marker = NULL;

    if (state == NULL || runtime == NULL || line == NULL) {
        return;
    }
    config = &state->config;
    if (motor_state_marker_from_line(config, line, &state_from_marker)) {
        motor_runtime_set_state(runtime, state_from_marker);
    }
    {
        size_t i;

        for (i = 0u; i < config->state_pattern_count; i++) {
            if (strstr(line, config->state_patterns[i].pattern) != NULL) {
                motor_runtime_set_state(runtime, config->state_patterns[i].label);
            }
        }
    }
}

static bool boot_menu_text_seen(const flash_state *state, const char *text)
{
    const motor_map_config *config;
    size_t i;

    if (state == NULL || text == NULL) {
        return false;
    }
    config = &state->config;
    for (i = 0u; i < config->state_pattern_count; i++) {
        if (strcmp(config->state_patterns[i].label, "boot_menu") == 0 &&
            strstr(text, config->state_patterns[i].pattern) != NULL) {
            return true;
        }
    }
    return false;
}

static void print_boot_output_prefix(const motor_map_entry *entry,
                                     const motor_runtime *runtime)
{
    printf("[motor%02u,%s]:", entry->index, motor_runtime_state(runtime));
}

static void print_boot_log_line(flash_state *state,
                                const motor_map_entry *entry,
                                motor_runtime *runtime,
                                size_t idx,
                                bool force)
{
    if (idx >= MOTOR_MAP_MAX) {
        idx = 0u;
    }
    if (boot_log_line_lens[idx] == 0u && !force) {
        return;
    }

    boot_log_lines[idx][boot_log_line_lens[idx]] = '\0';
    update_motor_state_from_boot_line(state, runtime, boot_log_lines[idx]);
    print_boot_output_prefix(entry, runtime);
    printf("%s\n", boot_log_lines[idx]);
    boot_log_line_lens[idx] = 0u;
    boot_log_lines[idx][0] = '\0';
}

static void print_boot_terminal_output(flash_state *state,
                                       const motor_map_entry *entry,
                                       motor_runtime *runtime,
                                       unsigned int can_id,
                                       const uint8_t *data,
                                       unsigned int len)
{
    unsigned int i;

    for (i = 0u; i < len; i++) {
        unsigned char ch = data[i];
        size_t idx = entry->index;

        if (idx >= MOTOR_MAP_MAX) {
            idx = 0u;
        }
        if (ch == '\n' || ch == '\r') {
            if (boot_log_last_newline[idx] != 0u && ch != boot_log_last_newline[idx]) {
                boot_log_last_newline[idx] = 0u;
                continue;
            }
            print_boot_log_line(state, entry, runtime, idx, false);
            boot_log_last_newline[idx] = ch;
            continue;
        }
        boot_log_last_newline[idx] = 0u;

        if (ch == '\t' || ch >= 0x20u) {
            if (boot_log_line_lens[idx] + 1u >= LINE_LEN) {
                print_boot_log_line(state, entry, runtime, idx, true);
            }
            boot_log_lines[idx][boot_log_line_lens[idx]++] = (char)ch;
        } else {
            char escaped[5];

            snprintf(escaped, sizeof(escaped), "\\x%02x", ch);
            if (boot_log_line_lens[idx] + strlen(escaped) >= LINE_LEN) {
                print_boot_log_line(state, entry, runtime, idx, true);
            }
            snprintf(&boot_log_lines[idx][boot_log_line_lens[idx]],
                     LINE_LEN - boot_log_line_lens[idx], "%s", escaped);
            boot_log_line_lens[idx] += strlen(escaped);
        }
    }
}

static const motor_map_entry *find_motor_by_bus_id(const flash_state *state,
                                                   unsigned int bus,
                                                   unsigned int id)
{
    size_t i;

    if (state == NULL || !state->config_loaded) {
        return NULL;
    }
    for (i = 0u; i < state->config.entry_count; i++) {
        const motor_map_entry *entry = &state->config.entries[i];

        if (entry->bus == bus && entry->id == id) {
            return entry;
        }
    }
    return NULL;
}

static void print_motor_input_prefix(const flash_state *state,
                                     unsigned int bus,
                                     unsigned int id)
{
    const motor_map_entry *entry = find_motor_by_bus_id(state, bus, id);

    if (entry != NULL) {
        printf("[motor%02u]:", entry->index);
    } else {
        printf("[motor?? bus=%u id=%u]:", bus, id);
    }
}

static void print_motor_input_data(const uint8_t *data, unsigned int len)
{
    unsigned int i;

    printf(" data:");
    for (i = 0u; i < len; i++) {
        printf(" %02x", data[i]);
    }
}

static void print_motor_input_byte(flash_state *state, uint8_t byte)
{
    if (!state->show_motor_input) {
        return;
    }

    flockfile(stdout);
    printf("\n");
    print_motor_input_prefix(state, state->bus, state->boot_id);
    printf("boot/debug tx can_id=0x%03x", state->tx_id);
    if (byte == '\r') {
        printf(" char='\\r'");
    } else if (byte == '\n') {
        printf(" char='\\n'");
    } else if (byte == '\t') {
        printf(" char='\\t'");
    } else if (byte >= 0x20u && byte <= 0x7eu) {
        printf(" char='%c'", byte);
    }
    printf(" hex=0x%02x\n", byte);
    fflush(stdout);
    funlockfile(stdout);
}

static void print_motor_input_frame(flash_state *state,
                                    const char *kind,
                                    unsigned int can_id,
                                    const uint8_t *data,
                                    unsigned int len,
                                    unsigned int flags)
{
    unsigned int id = can_id & 0x0fu;

    if (!state->show_motor_input) {
        return;
    }

    flockfile(stdout);
    printf("\n");
    print_motor_input_prefix(state, state->bus, id);
    printf("%s tx can_id=0x%03x len=%u flags=0x%02x",
           kind, can_id, len, flags);
    print_motor_input_data(data, len);
    printf("\n");
    fflush(stdout);
    funlockfile(stdout);
}

static int can_rx_callback(void *arg, canfd_packet *msg)
{
    flash_state *state = (flash_state *)arg;
    unsigned int can_id;
    rx_can_frame frame;
    bool decoded_motor = false;
    bool decoded_boot = false;

    if (state == NULL || msg == NULL) {
        return 0;
    }
    if (msg->bus >= CANFD_DEVICE_NUM) {
        return 0;
    }

    state->can_stats[msg->bus].rx_packets++;

    if (is_hardware_error_frame(msg->frame.can_id)) {
        unsigned int i;

        if (state->show_can_output) {
            flockfile(stdout);
            printf("\n【hardware】 bus=%u raw_id=0x%08x err=0x%08x len=%u flags=0x%02x data:",
                   msg->bus,
                   msg->frame.can_id,
                   msg->frame.can_id & CAN_ERR_MASK,
                   msg->frame.len,
                   msg->frame.flags);
            for (i = 0u; i < msg->frame.len && i < CANFD_MAX_DLEN; i++) {
                printf(" %02x", msg->frame.data[i]);
            }
            printf("\n");
            fflush(stdout);
            funlockfile(stdout);
        }
        return 0;
    }

    can_id = clean_can_id(msg->frame.can_id);
    memset(&frame, 0, sizeof(frame));
    frame.bus = msg->bus;
    frame.can_id = can_id;
    frame.len = msg->frame.len;
    frame.flags = msg->frame.flags;
    frame.time_us = time_us();
    if (frame.len > CANFD_MAX_DLEN) {
        frame.len = CANFD_MAX_DLEN;
    }
    memcpy(frame.data, msg->frame.data, frame.len);
    frame_ring_push(&state->frames, &frame);

    if (msg->bus == state->bus && can_id == state->rx_id) {
        rx_ring_push(&state->rx, msg->frame.data, msg->frame.len);
    }

    if (state->config_loaded) {
        size_t i;

        for (i = 0u; i < state->config.entry_count; i++) {
            const motor_map_entry *entry = &state->config.entries[i];
            motor_runtime *runtime = &state->motors[i];

            if (entry->bus == msg->bus && boot_output_id_matches(frame.can_id, entry->id)) {
                runtime->online = true;
                runtime->last_reply_us = frame.time_us;
                runtime->rx_count++;
                decoded_boot = true;
                if (state->can_stats[msg->bus].outstanding_replies > 0u) {
                    state->can_stats[msg->bus].outstanding_replies--;
                    state->can_stats[msg->bus].received_replies++;
                }
                if (state->show_can_output) {
                    flockfile(stdout);
                    print_boot_terminal_output(state, entry, runtime, frame.can_id,
                                               frame.data, frame.len);
                    fflush(stdout);
                    funlockfile(stdout);
                }
                break;
            }
            if (entry->bus == msg->bus &&
                mit_reply_frame_matches(&frame, entry->id)) {
                bxi_motor_reply reply;

                const bxi_motor_limits *limits = limits_for_entry(state, entry);

                if (bxi_motor_unpack_reply(frame.data, frame.len, limits, &reply) == 0) {
                    float old_mos = runtime->last_reply.mos_temperature;
                    float old_motor = runtime->last_reply.motor_temperature;

                    runtime->online = true;
                    runtime->last_reply_us = frame.time_us;
                    runtime->last_reply = reply;
                    runtime->last_reply.mos_temperature = old_mos;
                    runtime->last_reply.motor_temperature = old_motor;
                    if (strcmp(motor_runtime_state(runtime), "unknown") == 0 ||
                        strcmp(motor_runtime_state(runtime), "boot") == 0 ||
                        strcmp(motor_runtime_state(runtime), "boot_menu") == 0) {
                        motor_runtime_set_state(runtime, "motor");
                    }
                    motor_runtime_update_aux(&state->config, runtime, &reply);
                    runtime->rx_count++;
                    decoded_motor = true;
                    state->can_stats[msg->bus].decoded_motor_replies++;
                    if (state->can_stats[msg->bus].outstanding_replies > 0u) {
                        state->can_stats[msg->bus].outstanding_replies--;
                        state->can_stats[msg->bus].received_replies++;
                    }
                    if (state->show_can_output) {
                        flockfile(stdout);
                        if (state->config.chinese_ui) {
                            printf("\n[电机输出] index=%02u bus=%u id=%u "
                                   "位置=% .5f 速度=% .5f 转矩=% .5f MOS温度=% .1fC 电机温度=% .1fC",
                                   entry->index, entry->bus, entry->id,
                                   reply.position, reply.velocity, reply.torque,
                                   runtime->last_reply.mos_temperature,
                                   runtime->last_reply.motor_temperature);
                        } else {
                            printf("\n[MOTOR OUTPUT] index=%02u bus=%u id=%u "
                                   "pos=% .5f vel=% .5f torque=% .5f mos=% .1fC motor=% .1fC",
                                   entry->index, entry->bus, entry->id,
                                   reply.position, reply.velocity, reply.torque,
                                   runtime->last_reply.mos_temperature,
                                   runtime->last_reply.motor_temperature);
                        }
                        print_mit_aux_decode(&state->config, &reply);
                        printf("\n");
                        fflush(stdout);
                        funlockfile(stdout);
                    }
                }
                break;
            }
        }
    }
    if (state->show_can_output && !decoded_motor && !decoded_boot &&
        !is_reg_cmd_id(frame.can_id)) {
        unsigned int i;

        flockfile(stdout);
        printf("\n%s bus=%u id=0x%03x len=%u flags=0x%02x data:",
               state->config.chinese_ui ? "[CAN 输出]" : "[CAN OUTPUT]",
               frame.bus, frame.can_id, frame.len, frame.flags);
        for (i = 0u; i < frame.len; i++) {
            printf(" %02x", frame.data[i]);
        }
        printf("\n");
        fflush(stdout);
        funlockfile(stdout);
    }
    return 0;
}

static void print_data(const uint8_t *data, unsigned int len)
{
    unsigned int i;

    for (i = 0u; i < len; i++) {
        printf(" %02x", data[i]);
    }
}

static void console_print_motor_tx_frame(const motor_map_entry *motor,
                                         const char *kind,
                                         unsigned int bus,
                                         unsigned int can_id,
                                         unsigned int len,
                                         unsigned int flags,
                                         const uint8_t *data)
{
    if (motor != NULL) {
        printf("[motor%02u]: tx kind=%s bus=%u id=%u can_id=0x%03x len=%u flags=0x%02x data:",
               motor->index,
               kind,
               bus,
               motor->id,
               clean_can_id(can_id),
               len,
               flags);
    } else {
        printf("[motor??]: tx kind=%s bus=%u id=%u can_id=0x%03x len=%u flags=0x%02x data:",
               kind,
               bus,
               can_id & 0x0fu,
               clean_can_id(can_id),
               len,
               flags);
    }
    print_data(data, len);
    printf("\n");
    fflush(stdout);
}

static int send_can_bytes(flash_state *state, unsigned int can_id, const uint8_t *data, unsigned int len)
{
    canfd_packet packet;
    int ret;
    unsigned int attempt;
    unsigned int retry_count = DEFAULT_BOOT_TX_RETRY_COUNT;
    unsigned int retry_delay_ms = DEFAULT_BOOT_TX_RETRY_DELAY_MS;

    if (len > 8u) {
        fprintf(stderr, "classic boot CAN frame length %u is too large\n", len);
        return -1;
    }

    memset(&packet, 0, sizeof(packet));
    packet.bus = state->bus;
    packet.frame.can_id = can_id;
    packet.frame.len = (uint8_t)len;
    packet.frame.flags = 0u;
    memcpy(packet.frame.data, data, len);

    if (!state->quiet_tx) {
        printf("tx can[%u] id:0x%03x len:%u data:",
               packet.bus,
               clean_can_id(packet.frame.can_id),
               packet.frame.len);
        print_data(packet.frame.data, packet.frame.len);
        printf("\n");
        fflush(stdout);
    }

    for (attempt = 0u; attempt <= retry_count && !stop_requested; attempt++) {
        ret = canfd_send_packet(&packet, 1u);
        state->can_stats[packet.bus].tx_packets++;
        if (ret >= 0) {
            return 0;
        }

        state->can_stats[packet.bus].tx_failed++;
        if (attempt < retry_count) {
            if (retry_delay_ms != 0u) {
                sleep_ms(retry_delay_ms);
            }
            continue;
        }

        fprintf(stderr, "canfd_send_packet failed after %u retries: %d\n",
                retry_count,
                ret);
        return -1;
    }

    return -1;
}

static int send_debug_packet(flash_state *state, unsigned int can_id, const uint8_t *data, unsigned int len)
{
    canfd_packet packet;
    int ret;

    if (len > CANFD_MAX_DLEN) {
        fprintf(stderr, "debug frame length %u is too large\n", len);
        return -1;
    }
    if (!state->debug_use_canfd && len > 8u) {
        fprintf(stderr, "classic debug frame length %u is too large\n", len);
        return -1;
    }

    memset(&packet, 0, sizeof(packet));
    packet.bus = state->bus;
    packet.frame.can_id = can_id;
    packet.frame.len = (uint8_t)len;
    packet.frame.flags = state->debug_use_canfd ? (CANFD_BRS | CANFD_FDF) : 0u;
    memcpy(packet.frame.data, data, len);

    if (can_id != state->boot_id) {
        print_motor_input_frame(state, "debug", can_id, data, len, packet.frame.flags);
    }

    if (!state->quiet_tx) {
        printf("tx can[%u] id:0x%03x len:%u flags:0x%02x data:",
               packet.bus,
               clean_can_id(packet.frame.can_id),
               packet.frame.len,
               packet.frame.flags);
        print_data(packet.frame.data, packet.frame.len);
        printf("\n");
        fflush(stdout);
    }

    ret = canfd_send_packet(&packet, 1u);
    state->can_stats[packet.bus].tx_packets++;
    if (ret < 0) {
        state->can_stats[packet.bus].tx_failed++;
        fprintf(stderr, "canfd_send_packet failed: %d\n", ret);
        return -1;
    }

    return 0;
}

static int is_reg_cmd_id(unsigned int can_id)
{
    unsigned int cmd = can_id >> 4;

    return cmd == CAN_CMD_REG_READ ||
           cmd == CAN_CMD_REG_WRITE ||
           cmd == CAN_CMD_REG_SAVE ||
           cmd == CAN_CMD_REG_RESET_ALL ||
           cmd == CAN_CMD_REG_FW_VERSION;
}

typedef enum
{
    REG_VALUE_INT = 0,
    REG_VALUE_UINT,
    REG_VALUE_FLOAT,
    REG_VALUE_BOOL,
    REG_VALUE_UNKNOWN,
} reg_value_type;

typedef struct
{
    uint32_t addr;
    const char *name;
    reg_value_type type;
} reg_config_meta;

static const reg_config_meta reg_config_table[] = {
    {0x01u, "motor_pole_pairs", REG_VALUE_INT},
    {0x02u, "motor_phase_resistance", REG_VALUE_FLOAT},
    {0x03u, "motor_phase_inductance", REG_VALUE_FLOAT},
    {0x04u, "inertia", REG_VALUE_FLOAT},
    {0x05u, "encoder_dir_rev", REG_VALUE_INT},
    {0x06u, "encoder_offset", REG_VALUE_INT},
    {0x07u, "calib_valid", REG_VALUE_INT},
    {0x08u, "calib_current", REG_VALUE_FLOAT},
    {0x09u, "calib_max_voltage", REG_VALUE_FLOAT},
    {0x0au, "control_mode", REG_VALUE_INT},
    {0x0bu, "current_ramp_rate", REG_VALUE_FLOAT},
    {0x0cu, "vel_ramp_rate", REG_VALUE_FLOAT},
    {0x0du, "traj_vel", REG_VALUE_FLOAT},
    {0x0eu, "traj_accel", REG_VALUE_FLOAT},
    {0x0fu, "traj_decel", REG_VALUE_FLOAT},
    {0x10u, "pos_gain", REG_VALUE_FLOAT},
    {0x11u, "vel_gain", REG_VALUE_FLOAT},
    {0x12u, "vel_integrator_gain", REG_VALUE_FLOAT},
    {0x13u, "vel_limit", REG_VALUE_FLOAT},
    {0x14u, "current_limit", REG_VALUE_FLOAT},
    {0x15u, "current_ctrl_p_gain", REG_VALUE_FLOAT},
    {0x16u, "current_ctrl_i_gain", REG_VALUE_FLOAT},
    {0x17u, "current_ctrl_bandwidth", REG_VALUE_INT},
    {0x18u, "protect_under_voltage", REG_VALUE_FLOAT},
    {0x19u, "protect_over_voltage", REG_VALUE_FLOAT},
    {0x1au, "protect_over_speed", REG_VALUE_FLOAT},
    {0x1bu, "can_id", REG_VALUE_INT},
    {0x1cu, "can_timeout_ms", REG_VALUE_INT},
    {0x1du, "can_sync_target_enable", REG_VALUE_BOOL},
    {0x1eu, "torquet_limit", REG_VALUE_FLOAT},
    {0x1fu, "protect_temperature_low", REG_VALUE_FLOAT},
    {0x20u, "protect_temperature_high", REG_VALUE_FLOAT},
    {0x21u, "protect_i_abc_error", REG_VALUE_FLOAT},
    {0x22u, "can_mode_switch", REG_VALUE_BOOL},
    {0x23u, "master_id", REG_VALUE_INT},
    {0x24u, "field_weaken_mode", REG_VALUE_BOOL},
    {0x25u, "sync_boot_id_flag", REG_VALUE_BOOL},
    {0x26u, "current_test_enable", REG_VALUE_BOOL},
    {0x27u, "mit_mode", REG_VALUE_BOOL},
    {0x28u, "max_pos", REG_VALUE_FLOAT},
    {0x29u, "max_vel", REG_VALUE_FLOAT},
    {0x2au, "max_tor", REG_VALUE_FLOAT},
    {0x2bu, "kp_max", REG_VALUE_FLOAT},
    {0x2cu, "kd_max", REG_VALUE_FLOAT},
    {0x2du, "usr_enc1_offset", REG_VALUE_INT},
    {0x2eu, "usr_enc2_offset", REG_VALUE_FLOAT},
    {0x2fu, "enco_calib_valid", REG_VALUE_INT},
    {0x30u, "config_version", REG_VALUE_UINT},
    {0x31u, "reduction_ratio", REG_VALUE_FLOAT},
    {0x7cu, "config_version", REG_VALUE_UINT},
    {0xf0u, "enc2_closed_loop_enable", REG_VALUE_BOOL},
};

static const reg_config_meta *reg_config_meta_for_addr(uint32_t addr)
{
    size_t i;

    for (i = 0u; i < sizeof(reg_config_table) / sizeof(reg_config_table[0]); i++) {
        if (reg_config_table[i].addr == addr) {
            return &reg_config_table[i];
        }
    }
    return NULL;
}

static const char *reg_cmd_name(unsigned int cmd)
{
    if (cmd == CAN_CMD_REG_WRITE) {
        return "write";
    }
    if (cmd == CAN_CMD_REG_READ) {
        return "read";
    }
    if (cmd == CAN_CMD_REG_SAVE) {
        return "save";
    }
    if (cmd == CAN_CMD_REG_RESET_ALL) {
        return "reset_all";
    }
    if (cmd == CAN_CMD_REG_FW_VERSION) {
        return "fw_version";
    }
    return "unknown";
}

static void print_register_value(uint32_t value, reg_value_type type)
{
    if (type == REG_VALUE_FLOAT) {
        printf("% .6g", data_to_float((const uint8_t *)&value));
    } else if (type == REG_VALUE_UINT) {
        printf("%u", value);
    } else if (type == REG_VALUE_BOOL) {
        printf("%u", value != 0u ? 1u : 0u);
    } else if (type == REG_VALUE_INT) {
        printf("%d", (int32_t)value);
    } else {
        printf("raw=0x%08x", value);
    }
}

static int print_register_debug_frame(flash_state *state, const rx_can_frame *frame)
{
    unsigned int cmd = frame->can_id >> 4;
    unsigned int node_id = frame->can_id & 0x0fu;
    const motor_map_entry *entry = find_motor_by_bus_id(state, frame->bus, node_id);

    if ((cmd == CAN_CMD_REG_READ || cmd == CAN_CMD_REG_WRITE) && frame->len >= 8u) {
        uint32_t reg = data_to_u32(&frame->data[0]);
        uint32_t value = data_to_u32(&frame->data[4]);
        const reg_config_meta *meta = reg_config_meta_for_addr(reg);

        if (state->brief_register_output) {
            if (entry != NULL) {
                printf("[motor%02u]: rx kind=reg bus=%u id=%u can_id=0x%03x len=%u flags=0x%02x data:",
                       entry->index,
                       frame->bus,
                       node_id,
                       frame->can_id,
                       frame->len,
                       frame->flags);
            } else {
                printf("[motor??]: rx kind=reg bus=%u id=%u can_id=0x%03x len=%u flags=0x%02x data:",
                       frame->bus,
                       node_id,
                       frame->can_id,
                       frame->len,
                       frame->flags);
            }
            print_data(frame->data, frame->len);
            printf(" reg=0x%02x", reg);
            if (meta != NULL) {
                printf(" %s", meta->name);
            }
            printf(" value=");
            print_register_value(value, meta != NULL ? meta->type : REG_VALUE_UNKNOWN);
            printf("\n");
            fflush(stdout);
            return 1;
        }
        if (entry != NULL) {
            printf("\n%s index=%02u reg=0x%02x",
                   state->config.chinese_ui ? "[寄存器回复]" : "[REGISTER]",
                   entry->index,
                   reg);
        } else {
            printf("\n%s index=?? reg=0x%02x",
                   state->config.chinese_ui ? "[寄存器回复]" : "[REGISTER]",
                   reg);
        }
        if (meta != NULL) {
            printf(" %s", meta->name);
        }
        printf(" value=");
        print_register_value(value, meta != NULL ? meta->type : REG_VALUE_UNKNOWN);
        printf("\n  bus=%u id=%u can_id=0x%03x cmd=%s\n",
               frame->bus, node_id, frame->can_id, reg_cmd_name(cmd));
        fflush(stdout);
        return 1;
    }
    if (cmd == CAN_CMD_REG_SAVE && frame->len >= 4u) {
        int32_t status = (int32_t)data_to_u32(&frame->data[0]);

        if (state->brief_register_output) {
            if (entry != NULL) {
                printf("[motor%02u]: rx kind=reg bus=%u id=%u can_id=0x%03x len=%u flags=0x%02x data:",
                       entry->index,
                       frame->bus,
                       node_id,
                       frame->can_id,
                       frame->len,
                       frame->flags);
            } else {
                printf("[motor??]: rx kind=reg bus=%u id=%u can_id=0x%03x len=%u flags=0x%02x data:",
                       frame->bus,
                       node_id,
                       frame->can_id,
                       frame->len,
                       frame->flags);
            }
            print_data(frame->data, frame->len);
            printf(" reg_save status=%d%s\n", status, status == 0 ? " OK" : "");
            fflush(stdout);
            return 1;
        }
        if (entry != NULL) {
            printf("\n%s index=%02u status=%d%s\n",
                   state->config.chinese_ui ? "[寄存器保存]" : "[REGISTER SAVE]",
                   entry->index,
                   status,
                   status == 0 ? " OK" : "");
        } else {
            printf("\n%s index=?? status=%d%s\n",
                   state->config.chinese_ui ? "[寄存器保存]" : "[REGISTER SAVE]",
                   status,
                   status == 0 ? " OK" : "");
        }
        printf("  bus=%u id=%u can_id=0x%03x cmd=%s\n",
               frame->bus, node_id, frame->can_id, reg_cmd_name(cmd));
        fflush(stdout);
        return 1;
    }
    return 0;
}

static void print_debug_frame(flash_state *state, const rx_can_frame *frame)
{
    const char *label = "CAN REPLY";

    if (is_reg_cmd_id(frame->can_id) &&
        print_register_debug_frame(state, frame)) {
        return;
    }

    if (is_reg_cmd_id(frame->can_id)) {
        label = "REGISTER REPLY";
    } else if (frame->len >= BXI_MOTOR_MIT_LEN && frame->data[0] == state->boot_id) {
        label = "MOTOR REPLY";
    }
    printf("\n==================== [%s] ====================\n", label);
    printf("bus:%u  can_id:0x%03x  len:%u  flags:0x%02x\nraw:",
           frame->bus,
           frame->can_id,
           frame->len,
           frame->flags);
    print_data(frame->data, frame->len);

    if (is_reg_cmd_id(frame->can_id)) {
        unsigned int cmd = frame->can_id >> 4;

        if ((cmd == CAN_CMD_REG_READ || cmd == CAN_CMD_REG_WRITE) &&
            frame->len >= 8u) {
            uint32_t reg = data_to_u32(&frame->data[0]);
            uint32_t value = data_to_u32(&frame->data[4]);

            printf("\nregister: addr=0x%08x value_raw=0x%08x uint=%u int=%d float=% .6g",
                   reg,
                   value,
                   value,
                   (int32_t)value,
                   data_to_float(&frame->data[4]));
        } else if (cmd == CAN_CMD_REG_SAVE && frame->len >= 4u) {
            int32_t status = (int32_t)data_to_u32(&frame->data[0]);

            printf("\nregister save: status=%d%s",
                   status,
                   status == 0 ? " OK" : "");
        } else if (cmd == CAN_CMD_REG_RESET_ALL && frame->len >= 4u) {
            int32_t status = (int32_t)data_to_u32(&frame->data[0]);

            printf("\nregister reset-all: status=%d%s",
                   status,
                   status == 0 ? " OK" : "");
        } else if (cmd == CAN_CMD_REG_FW_VERSION && frame->len >= 8u) {
            printf("\nfirmware version: major=%u minor=%u",
                   data_to_u32(&frame->data[0]),
                   data_to_u32(&frame->data[4]));
        }
    } else if (mit_reply_frame_matches(frame, state->boot_id)) {
        bxi_motor_reply reply;

        if (bxi_motor_unpack_reply(frame->data, frame->len, &state->limits, &reply) == 0) {
            printf("\nmotor:%u\nposition:% .5f  velocity:% .5f  torque:% .5f",
                   reply.motor_id,
                   reply.position,
                   reply.velocity,
                   reply.torque);
            print_mit_aux_decode(&state->config, &reply);
        }
    }
    printf("\n=========================================================\n");
    fflush(stdout);
}

static int debug_frame_matches(flash_state *state,
                               const rx_can_frame *frame,
                               unsigned int expected_id,
                               bool all)
{
    unsigned int node_id = frame->can_id & 0x0fu;

    if (all) {
        return 1;
    }
    if (is_reg_cmd_id(expected_id)) {
        return frame->can_id == expected_id && node_id == state->boot_id;
    }
    if (expected_id != 0u && frame->can_id == expected_id) {
        return 1;
    }
    if (is_reg_cmd_id(frame->can_id) && node_id == state->boot_id) {
        return 1;
    }
    if (frame->can_id == state->master_id) {
        return 1;
    }
    if (mit_reply_frame_matches(frame, state->boot_id)) {
        return 1;
    }
    return 0;
}

static int wait_debug_reply(flash_state *state, unsigned int expected_id, unsigned int timeout_ms)
{
    uint64_t deadline = time_us() + (uint64_t)timeout_ms * 1000ULL;

    while (!stop_requested && time_us() < deadline) {
        rx_can_frame frame;
        uint64_t now = time_us();
        unsigned int step_ms = 20u;

        if (deadline > now) {
            uint64_t remain_ms = (deadline - now) / 1000ULL;
            if (remain_ms < step_ms) {
                step_ms = (unsigned int)remain_ms;
            }
        }
        if (step_ms == 0u) {
            step_ms = 1u;
        }

        if (!frame_ring_pop(&state->frames, &frame, step_ms)) {
            continue;
        }
        if (!debug_frame_matches(state, &frame, expected_id, false)) {
            continue;
        }
        print_debug_frame(state, &frame);
        return 0;
    }

    if (!state->brief_register_output) {
        printf("no debug reply received in %u ms\n", timeout_ms);
    }
    return 1;
}

static int send_debug_mit(flash_state *state,
                          float position,
                          float velocity,
                          float kp,
                          float kd,
                          float torque)
{
    uint8_t data[BXI_MOTOR_MIT_LEN];

    bxi_motor_pack_mit(data, &state->limits, position, velocity, kp, kd, torque);
    return send_debug_packet(state, state->boot_id, data, BXI_MOTOR_MIT_LEN);
}

static int send_debug_special(flash_state *state, uint8_t command)
{
    uint8_t data[BXI_MOTOR_MIT_LEN];

    bxi_motor_pack_special(data, command);
    return send_debug_packet(state, state->boot_id, data, BXI_MOTOR_MIT_LEN);
}

static int send_boot_data(flash_state *state, const uint8_t *data, size_t len)
{
    size_t offset = 0u;
    unsigned int tx_gap_ms = DEFAULT_BOOT_TX_GAP_MS;

    while (offset < len && !stop_requested) {
        unsigned int chunk = (len - offset) > 8u ? 8u : (unsigned int)(len - offset);

        if (send_can_bytes(state, state->tx_id, &data[offset], chunk) != 0) {
            return -1;
        }
        offset += chunk;
        if (tx_gap_ms != 0u && offset < len) {
            sleep_ms(tx_gap_ms);
        }
    }

    return stop_requested ? -1 : 0;
}

static int send_boot_byte(flash_state *state, uint8_t byte)
{
    print_motor_input_byte(state, byte);
    return send_boot_data(state, &byte, 1u);
}

static void append_rx_text(char *buf, size_t *len, size_t cap, uint8_t byte, bool echo)
{
    if (*len + 1u < cap) {
        buf[*len] = (char)byte;
        (*len)++;
        buf[*len] = '\0';
    } else if (cap > 1u) {
        memmove(buf, buf + 1, cap - 2u);
        buf[cap - 2u] = (char)byte;
        buf[cap - 1u] = '\0';
        *len = cap - 1u;
    }

    if (echo) {
        if (byte == '\r') {
            return;
        }
        if (byte == '\n' || byte == '\t' || byte >= 0x20u) {
            fwrite(&byte, 1u, 1u, stdout);
        } else {
            printf("\\x%02x", byte);
        }
        fflush(stdout);
    }
}

static int wait_for_byte(flash_state *state, uint8_t wanted, unsigned int timeout_ms, bool echo)
{
    uint64_t deadline = time_us() + (uint64_t)timeout_ms * 1000ULL;
    char text[256];
    size_t text_len = 0u;

    text[0] = '\0';
    while (!stop_requested && time_us() < deadline) {
        uint64_t now = time_us();
        unsigned int step_ms = 20u;
        uint8_t byte;

        if (deadline > now) {
            uint64_t remain_ms = (deadline - now) / 1000ULL;
            if (remain_ms < step_ms) {
                step_ms = (unsigned int)remain_ms;
            }
        }
        if (step_ms == 0u) {
            step_ms = 1u;
        }

        if (!rx_ring_pop(&state->rx, &byte, step_ms)) {
            continue;
        }
        if (byte == wanted) {
            return 0;
        }
        append_rx_text(text, &text_len, sizeof(text), byte, echo);
    }

    return -1;
}

static int wait_for_pattern(flash_state *state,
                            const char *pattern,
                            unsigned int timeout_ms,
                            bool echo)
{
    uint64_t deadline = time_us() + (uint64_t)timeout_ms * 1000ULL;
    char text[2048];
    size_t text_len = 0u;

    text[0] = '\0';
    while (!stop_requested && time_us() < deadline) {
        uint64_t now = time_us();
        unsigned int step_ms = 20u;
        uint8_t byte;

        if (deadline > now) {
            uint64_t remain_ms = (deadline - now) / 1000ULL;
            if (remain_ms < step_ms) {
                step_ms = (unsigned int)remain_ms;
            }
        }
        if (step_ms == 0u) {
            step_ms = 1u;
        }

        if (!rx_ring_pop(&state->rx, &byte, step_ms)) {
            continue;
        }
        append_rx_text(text, &text_len, sizeof(text), byte, echo);
        if (strstr(text, pattern) != NULL) {
            return 0;
        }
    }

    return -1;
}

static void collect_text(flash_state *state, unsigned int quiet_ms, unsigned int max_ms)
{
    uint64_t start = time_us();
    uint64_t last = start;
    bool printed = false;

    while (!stop_requested && time_us() - start < (uint64_t)max_ms * 1000ULL) {
        uint8_t byte;

        if (rx_ring_pop(&state->rx, &byte, 20u)) {
            if (byte != '\r' && !state->suppress_boot_text) {
                if (byte == '\n' || byte == '\t' || byte >= 0x20u) {
                    fwrite(&byte, 1u, 1u, stdout);
                } else {
                    printf("\\x%02x", byte);
                }
                printed = true;
            }
            last = time_us();
            continue;
        }
        if (time_us() - last >= (uint64_t)quiet_ms * 1000ULL) {
            break;
        }
    }
    if (printed) {
        printf("\n");
    }
}

static int power_cycle_motor(void)
{
    if (motor_pwr_set(0u) < 0) {
        fprintf(stderr, "motor_pwr_set(0) failed\n");
        return -1;
    }
    sleep_ms(250u);
    if (motor_pwr_set(1u) < 0) {
        fprintf(stderr, "motor_pwr_set(1) failed\n");
        return -1;
    }
    return 0;
}

static int enter_boot_menu(flash_state *state, bool power_cycle)
{
    uint64_t deadline;
    char text[2048];
    size_t text_len = 0u;
    unsigned int m_delay_ms = DEFAULT_BOOT_ENTER_DELAY_MS;

    if (power_cycle) {
        if (!state->brief_flash_output) {
            printf("power cycling motor, then sending one 'm' after %u ms\n", m_delay_ms);
        }
        if (power_cycle_motor() != 0) {
            return -1;
        }
    } else {
        if (!state->brief_flash_output) {
            printf("sending app reset command 'r', then one 'm' after %u ms\n", m_delay_ms);
        }
        rx_ring_clear(&state->rx);
        if (send_boot_byte(state, 'r') != 0) {
            return -1;
        }
    }

    rx_ring_clear(&state->rx);
    text[0] = '\0';
    sleep_ms(m_delay_ms);
    if (send_boot_byte(state, 'm') != 0) {
        return -1;
    }
    deadline = time_us() + 1800ULL * 1000ULL;
    while (!stop_requested && time_us() < deadline) {
        uint8_t byte;

        while (rx_ring_pop(&state->rx, &byte, 0u)) {
            append_rx_text(text, &text_len, sizeof(text), byte, !state->brief_flash_output);
            if (boot_menu_text_seen(state, text)) {
                if (!state->brief_flash_output) {
                    printf("\nentered boot menu on tx=0x%03x rx=0x%03x\n", state->tx_id, state->rx_id);
                }
                return 0;
            }
        }
        sleep_ms(2u);
    }

    fprintf(stderr, "failed to enter boot menu on tx=0x%03x rx=0x%03x\n",
            state->tx_id,
            state->rx_id);
    if (text_len != 0u) {
        fprintf(stderr, "boot text before timeout: %s\n", text);
    } else {
        fprintf(stderr, "boot text before timeout: (none)\n");
    }
    if (!power_cycle) {
        fprintf(stderr, "hint: soft reset did not reach boot; try `enter cycle` or `--power-cycle`\n");
    }
    return -1;
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0u;
    size_t i;

    for (i = 0u; i < len; i++) {
        unsigned int bit;

        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0u; bit < 8u; bit++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

static size_t make_ymodem_packet(uint8_t *out,
                                 uint8_t seq,
                                 const uint8_t *payload,
                                 size_t payload_len,
                                 size_t packet_size,
                                 uint8_t pad)
{
    uint16_t crc;

    out[0] = packet_size == YMODEM_PACKET_128 ? YMODEM_SOH : YMODEM_STX;
    out[1] = seq;
    out[2] = (uint8_t)(0xffu - seq);
    memset(&out[3], pad, packet_size);
    if (payload_len > packet_size) {
        payload_len = packet_size;
    }
    if (payload_len > 0u && payload != NULL) {
        memcpy(&out[3], payload, payload_len);
    }
    crc = crc16_ccitt(&out[3], packet_size);
    out[3 + packet_size] = (uint8_t)(crc >> 8);
    out[3 + packet_size + 1u] = (uint8_t)(crc & 0xffu);
    return YMODEM_FRAME_PREFIX + packet_size + YMODEM_FRAME_SUFFIX;
}

static const char *path_basename(const char *path)
{
    const char *slash = strrchr(path, '/');

    return slash != NULL ? slash + 1 : path;
}

static char *trim_space(char *text)
{
    char *end;

    while (*text != '\0' && isspace((unsigned char)*text)) {
        text++;
    }
    if (*text == '\0') {
        return text;
    }

    end = text + strlen(text) - 1u;
    while (end > text && isspace((unsigned char)*end)) {
        *end-- = '\0';
    }
    return text;
}

static int parse_float_list_arg(const char *text,
                                float *values,
                                bool *value_set,
                                size_t max_count,
                                size_t *parsed_count)
{
    char buffer[LINE_LEN * 4u];
    char *p;
    char *end;
    size_t count = 0u;

    if (text == NULL || values == NULL || value_set == NULL || parsed_count == NULL ||
        strlen(text) >= sizeof(buffer)) {
        return -1;
    }
    snprintf(buffer, sizeof(buffer), "%s", text);
    p = trim_space(buffer);
    if (*p == '[') {
        p++;
    }
    p = trim_space(p);
    end = p + strlen(p);
    while (end > p && isspace((unsigned char)end[-1])) {
        *--end = '\0';
    }
    if (end > p && end[-1] == ']') {
        *--end = '\0';
    }

    while (*p != '\0') {
        char *next;
        float value;

        while (*p != '\0' &&
               (isspace((unsigned char)*p) || *p == ',')) {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        if (count >= max_count) {
            return -1;
        }
        errno = 0;
        value = strtof(p, &next);
        if (errno != 0 || next == p || !isfinite(value)) {
            return -1;
        }
        values[count] = value;
        value_set[count] = true;
        count++;
        p = next;
        while (*p != '\0' && isspace((unsigned char)*p)) {
            p++;
        }
        if (*p == ',') {
            p++;
        } else if (*p != '\0') {
            return -1;
        }
    }
    *parsed_count = count;
    return count == 0u ? -1 : 0;
}

static int firmware_type_is_safe(const char *type)
{
    size_t i;

    if (type == NULL || type[0] == '\0') {
        return 0;
    }
    for (i = 0u; type[i] != '\0'; i++) {
        unsigned char ch = (unsigned char)type[i];

        if (!isalnum(ch) && ch != '_' && ch != '-' && ch != '.') {
            return 0;
        }
    }
    if (strstr(type, "..") != NULL) {
        return 0;
    }
    return 1;
}

static int has_suffix(const char *text, const char *suffix)
{
    size_t text_len;
    size_t suffix_len;

    if (text == NULL || suffix == NULL) {
        return 0;
    }
    text_len = strlen(text);
    suffix_len = strlen(suffix);
    return text_len >= suffix_len &&
           strcmp(&text[text_len - suffix_len], suffix) == 0;
}

static int normalize_firmware_type(const char *type, char *out, size_t out_len)
{
    size_t len;

    if (!firmware_type_is_safe(type)) {
        return -1;
    }
    len = strlen(type);
    if (len >= 4u && strcmp(&type[len - 4u], ".bin") == 0) {
        len -= 4u;
    }
    if (len == 0u || len >= out_len) {
        return -1;
    }
    memcpy(out, type, len);
    out[len] = '\0';
    return 0;
}

static int type_to_firmware_path(const char *firmware_dir,
                                 const char *type,
                                 char *path,
                                 size_t path_len)
{
    char normalized[MOTOR_TYPE_LEN];
    int written;

    if (normalize_firmware_type(type, normalized, sizeof(normalized)) != 0) {
        fprintf(stderr, "invalid firmware type: %s\n", type != NULL ? type : "(null)");
        return -1;
    }

    written = snprintf(path, path_len, "%s/bxi_motor_%s.bin", firmware_dir, normalized);
    if (written < 0 || (size_t)written >= path_len) {
        fprintf(stderr, "firmware path is too long for type: %s\n", type);
        return -1;
    }
    return 0;
}

static int firmware_filename_is_safe(const char *filename)
{
    return filename != NULL && filename[0] != '\0' &&
           strchr(filename, '/') == NULL && strstr(filename, "..") == NULL;
}

static int firmware_dir_is_safe(const char *dir)
{
    size_t i;
    size_t component_start = 0u;
    size_t component_len;

    if (dir == NULL || dir[0] == '\0' || dir[0] == '/') {
        return 0;
    }
    if (strstr(dir, "..") != NULL) {
        return 0;
    }
    for (i = 0u; dir[i] != '\0'; i++) {
        unsigned char ch = (unsigned char)dir[i];

        if (ch == '/') {
            component_len = i - component_start;
            if (component_len == 0u ||
                (component_len == 1u && dir[component_start] == '.')) {
                return 0;
            }
            component_start = i + 1u;
            continue;
        }
        if (!isalnum(ch) && ch != '_' && ch != '-' && ch != '.') {
            return 0;
        }
    }
    component_len = i - component_start;
    return component_len != 0u &&
           !(component_len == 1u && dir[component_start] == '.');
}

static int firmware_url_is_safe(const char *url)
{
    size_t i;

    if (url == NULL || url[0] == '\0') {
        return 1;
    }
    if (strncmp(url, "http://", 7u) != 0 &&
        strncmp(url, "https://", 8u) != 0) {
        return 0;
    }
    for (i = 0u; url[i] != '\0'; i++) {
        unsigned char ch = (unsigned char)url[i];

        if (isspace(ch) || ch == '"' || ch == '\'' || ch == '\\') {
            return 0;
        }
    }
    return 1;
}

static const motor_type_config *find_type_config(const motor_map_config *config,
                                                 const char *type)
{
    char normalized[MOTOR_TYPE_LEN];
    size_t i;

    if (normalize_firmware_type(type, normalized, sizeof(normalized)) != 0) {
        return NULL;
    }
    for (i = 0u; i < config->type_config_count; i++) {
        if (strcmp(config->type_configs[i].type, normalized) == 0) {
            return &config->type_configs[i];
        }
    }
    return NULL;
}

static const bxi_motor_limits *limits_for_type(const motor_map_config *config,
                                               const char *type)
{
    const motor_type_config *type_config = find_type_config(config, type);

    if (type_config != NULL) {
        return &type_config->limits;
    }
    return &config->mit_limits;
}

static const bxi_motor_limits *limits_for_entry(const flash_state *state,
                                                const motor_map_entry *entry)
{
    if (entry == NULL) {
        return &state->limits;
    }
    return limits_for_type(&state->config, entry->type);
}

static int motor_limits_are_valid(const bxi_motor_limits *limits)
{
    return limits != NULL &&
           isfinite(limits->p_min) && isfinite(limits->p_max) &&
           isfinite(limits->v_min) && isfinite(limits->v_max) &&
           isfinite(limits->t_min) && isfinite(limits->t_max) &&
           limits->p_min < limits->p_max &&
           limits->v_min < limits->v_max &&
           limits->t_min < limits->t_max &&
           limits->kp_min >= 0.0f &&
           limits->kd_min >= 0.0f &&
           limits->kp_max > limits->kp_min &&
           limits->kd_max > limits->kd_min;
}

static float *motor_limit_field(bxi_motor_limits *limits, const char *key)
{
    if (strcmp(key, "mit_p_min") == 0) return &limits->p_min;
    if (strcmp(key, "mit_p_max") == 0) return &limits->p_max;
    if (strcmp(key, "mit_v_min") == 0) return &limits->v_min;
    if (strcmp(key, "mit_v_max") == 0) return &limits->v_max;
    if (strcmp(key, "mit_t_min") == 0) return &limits->t_min;
    if (strcmp(key, "mit_t_max") == 0) return &limits->t_max;
    if (strcmp(key, "mit_kp_min") == 0) return &limits->kp_min;
    if (strcmp(key, "mit_kp_max") == 0) return &limits->kp_max;
    if (strcmp(key, "mit_kd_min") == 0) return &limits->kd_min;
    if (strcmp(key, "mit_kd_max") == 0) return &limits->kd_max;
    if (strcmp(key, "mit_temp_min") == 0) return &limits->temp_min;
    if (strcmp(key, "mit_temp_max") == 0) return &limits->temp_max;
    return NULL;
}

static int add_motor_type_config(motor_map_config *config,
                                 const char *type,
                                 const char *firmware,
                                 const bxi_motor_limits *limits,
                                 const char *map_path,
                                 unsigned int line_no)
{
    char normalized[MOTOR_TYPE_LEN];
    size_t i;

    if (normalize_firmware_type(type, normalized, sizeof(normalized)) != 0 ||
        (firmware != NULL && firmware[0] != '\0' &&
         (!firmware_filename_is_safe(firmware) || strlen(firmware) >= FIRMWARE_NAME_MAX)) ||
        !motor_limits_are_valid(limits)) {
        fprintf(stderr, "%s:%u: invalid motor type config: %s\n",
                map_path, line_no, type != NULL ? type : "");
        return -1;
    }
    for (i = 0u; i < config->type_config_count; i++) {
        if (strcmp(config->type_configs[i].type, normalized) == 0) {
            fprintf(stderr, "%s:%u: duplicate motor type: %s\n",
                    map_path, line_no, normalized);
            return -1;
        }
    }
    if (config->type_config_count >= MOTOR_TYPE_CONFIG_MAX) {
        fprintf(stderr, "%s:%u: only %u motor type configs are supported\n",
                map_path, line_no, (unsigned int)MOTOR_TYPE_CONFIG_MAX);
        return -1;
    }
    snprintf(config->type_configs[config->type_config_count].type,
             MOTOR_TYPE_LEN, "%s", normalized);
    snprintf(config->type_configs[config->type_config_count].firmware,
             FIRMWARE_NAME_MAX, "%s", firmware != NULL ? firmware : "");
    config->type_configs[config->type_config_count].limits = *limits;
    config->type_config_count++;
    return 0;
}

static int state_label_is_valid(const char *label)
{
    size_t i;

    if (label == NULL || label[0] == '\0' || strlen(label) >= MOTOR_STATE_LEN) {
        return 0;
    }
    for (i = 0u; label[i] != '\0'; i++) {
        unsigned char c = (unsigned char)label[i];

        if (!isalnum(c) && c != '_' && c != '-') {
            return 0;
        }
    }
    return 1;
}

static int add_state_pattern_rule(motor_map_config *config,
                                  const char *label,
                                  const char *pattern,
                                  const char *map_path,
                                  unsigned int line_no)
{
    if (!state_label_is_valid(label) ||
        pattern == NULL || pattern[0] == '\0' ||
        strlen(pattern) >= STATE_PATTERN_LEN) {
        fprintf(stderr, "%s:%u: invalid state pattern rule\n", map_path, line_no);
        return -1;
    }
    if (config->state_pattern_count >= STATE_PATTERN_MAX) {
        fprintf(stderr, "%s:%u: only %u state patterns are supported\n",
                map_path, line_no, (unsigned int)STATE_PATTERN_MAX);
        return -1;
    }
    snprintf(config->state_patterns[config->state_pattern_count].label,
             MOTOR_STATE_LEN, "%s", label);
    snprintf(config->state_patterns[config->state_pattern_count].pattern,
             STATE_PATTERN_LEN, "%s", pattern);
    config->state_pattern_count++;
    return 0;
}

static int add_state_code_rule(motor_map_config *config,
                               unsigned int code,
                               const char *label,
                               const char *map_path,
                               unsigned int line_no)
{
    size_t i;

    if (!state_label_is_valid(label)) {
        fprintf(stderr, "%s:%u: invalid state code label\n", map_path, line_no);
        return -1;
    }
    if (config->state_code_count >= STATE_CODE_MAX) {
        fprintf(stderr, "%s:%u: only %u state codes are supported\n",
                map_path, line_no, (unsigned int)STATE_CODE_MAX);
        return -1;
    }
    for (i = 0u; i < config->state_code_count; i++) {
        if (config->state_codes[i].code == code) {
            fprintf(stderr, "%s:%u: duplicate state code: %u\n",
                    map_path, line_no, code);
            return -1;
        }
    }
    config->state_codes[config->state_code_count].code = code;
    snprintf(config->state_codes[config->state_code_count].label,
             MOTOR_STATE_LEN, "%s", label);
    config->state_code_count++;
    return 0;
}

static const char *state_label_for_code(const motor_map_config *config,
                                        unsigned int code)
{
    size_t i;

    if (config != NULL) {
        for (i = 0u; i < config->state_code_count; i++) {
            if (config->state_codes[i].code == code) {
                return config->state_codes[i].label;
            }
        }
    }
    return bxi_motor_fsm_state_name(code);
}

static void add_default_state_rules_if_missing(motor_map_config *config)
{
    static const struct {
        const char *label;
        const char *pattern;
    } default_patterns[] = {
        {"boot", "boot..."},
        {"boot", "no useful app"},
        {"motor", "========== BOOT INFO =========="},
        {"motor", "APP   :"},
        {"motor", "READY"},
        {"boot_menu", "Boot menu:"},
        {"motor_menu", "Motor main menu:"},
        {"motor_menu", "Menu:"},
    };
    static const struct {
        unsigned int code;
        const char *label;
    } default_codes[] = {
        {0u, "startup"},
        {1u, "motor_menu"},
        {2u, "enable"},
        {3u, "calib"},
        {4u, "ktm"},
        {5u, "enco"},
        {6u, "ktm_auto"},
        {7u, "setup"},
        {8u, "error"},
        {9u, "openloop"},
        {10u, "sls"},
    };
    size_t i;

    if (config == NULL) {
        return;
    }
    if (config->state_code_prefix[0] == '\0') {
        snprintf(config->state_code_prefix, sizeof(config->state_code_prefix), "STATE :");
    }
    if (config->state_pattern_count == 0u) {
        for (i = 0u; i < sizeof(default_patterns) / sizeof(default_patterns[0]); i++) {
            (void)add_state_pattern_rule(config,
                                         default_patterns[i].label,
                                         default_patterns[i].pattern,
                                         "default",
                                         0u);
        }
    }
    if (config->state_code_count == 0u) {
        for (i = 0u; i < sizeof(default_codes) / sizeof(default_codes[0]); i++) {
            (void)add_state_code_rule(config,
                                      default_codes[i].code,
                                      default_codes[i].label,
                                      "default",
                                      0u);
        }
    }
}

static int add_firmware_mapping(motor_map_config *config,
                                const char *type,
                                const char *filename,
                                const char *map_path,
                                unsigned int line_no)
{
    char normalized[MOTOR_TYPE_LEN];
    size_t i;

    if (normalize_firmware_type(type, normalized, sizeof(normalized)) != 0 ||
        !firmware_filename_is_safe(filename) || strlen(filename) >= FIRMWARE_NAME_MAX) {
        fprintf(stderr, "%s:%u: invalid firmware mapping %s: %s\n",
                map_path, line_no, type, filename);
        return -1;
    }
    for (i = 0u; i < config->firmware_mapping_count; i++) {
        if (strcmp(config->firmware_mappings[i].type, normalized) == 0) {
            fprintf(stderr, "%s:%u: duplicate firmware type: %s\n",
                    map_path, line_no, normalized);
            return -1;
        }
    }
    if (config->firmware_mapping_count >= FIRMWARE_MAPPING_MAX) {
        fprintf(stderr, "%s:%u: only %u firmware file mappings are supported\n",
                map_path, line_no, (unsigned int)FIRMWARE_MAPPING_MAX);
        return -1;
    }
    snprintf(config->firmware_mappings[config->firmware_mapping_count].type,
             MOTOR_TYPE_LEN, "%s", normalized);
    snprintf(config->firmware_mappings[config->firmware_mapping_count].filename,
             FIRMWARE_NAME_MAX, "%s", filename);
    config->firmware_mapping_count++;
    return 0;
}

static int configured_firmware_name(const motor_map_config *config,
                                    const char *type,
                                    char *filename,
                                    size_t filename_len);

static int configured_firmware_path(const motor_map_config *config,
                                    const char *type,
                                    char *path,
                                    size_t path_len)
{
    int written;
    char filename[FIRMWARE_NAME_MAX];

    if (configured_firmware_name(config, type, filename, sizeof(filename)) != 0) {
        return -1;
    }
    written = snprintf(path, path_len, "%s/%s", config->firmware_dir, filename);
    if (written < 0 || (size_t)written >= path_len) {
        fprintf(stderr, "firmware path is too long: %s/%s\n", config->firmware_dir, filename);
        return -1;
    }
    return 0;
}

static int configured_firmware_name(const motor_map_config *config,
                                    const char *type,
                                    char *filename,
                                    size_t filename_len)
{
    char normalized[MOTOR_TYPE_LEN];
    const char *configured = NULL;
    size_t i;
    int written;

    if (normalize_firmware_type(type, normalized, sizeof(normalized)) != 0 ||
        filename == NULL || filename_len == 0u) {
        return -1;
    }
    {
        const motor_type_config *type_config = find_type_config(config, normalized);

        if (type_config != NULL && type_config->firmware[0] != '\0') {
            configured = type_config->firmware;
        }
    }
    if (configured == NULL) {
        for (i = 0u; i < config->firmware_mapping_count; i++) {
            if (strcmp(config->firmware_mappings[i].type, normalized) == 0) {
                configured = config->firmware_mappings[i].filename;
                break;
            }
        }
    }
    if (configured == NULL) {
        if (config->firmware_mapping_count != 0u) {
            fprintf(stderr, "no firmware_files mapping for motor type %s\n", normalized);
            return -1;
        }
        written = snprintf(filename, filename_len, "bxi_motor_%s.bin", normalized);
    } else {
        written = snprintf(filename, filename_len, "%s", configured);
    }
    if (written < 0 || (size_t)written >= filename_len ||
        !firmware_filename_is_safe(filename)) {
        fprintf(stderr, "invalid firmware filename for type %s\n", normalized);
        return -1;
    }
    return 0;
}

static int firmware_inventory_has_type(const firmware_inventory *inventory, const char *type)
{
    char normalized[MOTOR_TYPE_LEN];
    size_t i;

    if (normalize_firmware_type(type, normalized, sizeof(normalized)) != 0) {
        return 0;
    }
    for (i = 0u; i < inventory->count; i++) {
        if (strcmp(inventory->files[i].type, normalized) == 0) {
            return 1;
        }
    }
    return 0;
}

static int scan_firmware_dir(const char *firmware_dir, firmware_inventory *inventory)
{
    DIR *dir;
    struct dirent *entry;

    memset(inventory, 0, sizeof(*inventory));
    dir = opendir(firmware_dir);
    if (dir == NULL) {
        perror(firmware_dir);
        return -1;
    }

    while ((entry = readdir(dir)) != NULL) {
        char type[MOTOR_TYPE_LEN];
        char path[PATH_LEN];
        size_t len;
        int written;

        if (!has_suffix(entry->d_name, ".bin")) {
            continue;
        }
        if (inventory->count >= FIRMWARE_FILE_MAX) {
            fprintf(stderr, "%s: too many firmware files, only first %u are tracked\n",
                    firmware_dir,
                    (unsigned int)FIRMWARE_FILE_MAX);
            break;
        }
        len = strlen(entry->d_name) - 4u;
        if (len == 0u || len >= sizeof(type)) {
            fprintf(stderr, "%s/%s: firmware file name is too long or empty\n",
                    firmware_dir,
                    entry->d_name);
            continue;
        }
        memcpy(type, entry->d_name, len);
        type[len] = '\0';
        if (!firmware_type_is_safe(type)) {
            fprintf(stderr, "%s/%s: invalid firmware type name\n", firmware_dir, entry->d_name);
            continue;
        }
        written = snprintf(path, sizeof(path), "%s/%s", firmware_dir, entry->d_name);
        if (written < 0 || (size_t)written >= sizeof(path)) {
            fprintf(stderr, "%s/%s: firmware path is too long\n", firmware_dir, entry->d_name);
            continue;
        }
        snprintf(inventory->files[inventory->count].type,
                 sizeof(inventory->files[inventory->count].type),
                 "%s",
                 type);
        snprintf(inventory->files[inventory->count].path,
                 sizeof(inventory->files[inventory->count].path),
                 "%s",
                 path);
        inventory->count++;
    }

    closedir(dir);
    return 0;
}

static int motor_map_add_entry(motor_map_config *config,
                               unsigned int index,
                               unsigned int bus,
                               unsigned int id,
                               const char *type,
                               const char *name,
                               unsigned int line_no)
{
    motor_map_entry *entry;
    char normalized[MOTOR_TYPE_LEN];

    if (config->entry_count >= MOTOR_MAP_MAX) {
        fprintf(stderr, "motor map has too many entries, max is %u\n",
                (unsigned int)MOTOR_MAP_MAX);
        return -1;
    }
    if (bus >= CANFD_DEVICE_NUM || id == 0u || id > 8u) {
        fprintf(stderr, "motor map line %u requires bus 0-%u and id 1-8\n",
                line_no, CANFD_DEVICE_NUM - 1u);
        return -1;
    }
    for (size_t i = 0u; i < config->entry_count; i++) {
        if (config->entries[i].index == index ||
            (config->entries[i].bus == bus && config->entries[i].id == id)) {
            fprintf(stderr, "motor map line %u duplicates index or bus/id\n", line_no);
            return -1;
        }
    }
    if (normalize_firmware_type(type, normalized, sizeof(normalized)) != 0) {
        fprintf(stderr, "motor map line %u has invalid firmware type: %s\n", line_no, type);
        return -1;
    }

    entry = &config->entries[config->entry_count++];
    entry->index = index;
    entry->bus = bus;
    entry->id = id;
    entry->line_no = line_no;
    snprintf(entry->type, sizeof(entry->type), "%s", normalized);
    if (name != NULL) {
        snprintf(entry->name, sizeof(entry->name), "%s", name);
    }
    return 0;
}

static int parse_csv_motor_map(const char *map_path, motor_map_config *config)
{
    FILE *fp;
    char line[256];
    unsigned int line_no = 0u;

    fp = fopen(map_path, "r");
    if (fp == NULL) {
        perror(map_path);
        return -1;
    }

    while (fgets(line, sizeof(line), fp) != NULL) {
        char *p;
        char *comment;
        char bus_text[32];
        char id_text[32];
        char type_text[MOTOR_TYPE_LEN];
        unsigned int parsed_bus;
        unsigned int parsed_id;
        size_t i;

        line_no++;
        comment = strchr(line, '#');
        if (comment != NULL) {
            *comment = '\0';
        }

        for (i = 0u; line[i] != '\0'; i++) {
            if (line[i] == ',' || line[i] == ';' || line[i] == '\t') {
                line[i] = ' ';
            }
        }

        p = trim_space(line);
        if (p[0] == '\0') {
            continue;
        }
        if (sscanf(p, "%31s %31s %31s", bus_text, id_text, type_text) != 3) {
            fprintf(stderr, "%s:%u: expected bus,id,type\n", map_path, line_no);
            fclose(fp);
            return -1;
        }
        if (strcmp(bus_text, "bus") == 0 && strcmp(id_text, "id") == 0) {
            continue;
        }
        if (parse_uint_arg(bus_text, &parsed_bus) != 0 ||
            parse_uint_arg(id_text, &parsed_id) != 0) {
            fprintf(stderr, "%s:%u: invalid bus or id\n", map_path, line_no);
            fclose(fp);
            return -1;
        }
        if (motor_map_add_entry(config, (unsigned int)config->entry_count + 1u,
                                parsed_bus, parsed_id, type_text, "", line_no) != 0) {
            fclose(fp);
            return -1;
        }
    }

    fclose(fp);
    return 0;
}

static int strip_optional_quotes(char *value)
{
    size_t len = strlen(value);

    if (len >= 2u &&
        ((value[0] == '"' && value[len - 1u] == '"') ||
         (value[0] == '\'' && value[len - 1u] == '\''))) {
        value[len - 1u] = '\0';
        memmove(value, value + 1, len - 1u);
    }
    return 0;
}

static int parse_yaml_key_value(char *line, char **key, char **value)
{
    char *colon;

    line = trim_space(line);
    if (line[0] == '-' && isspace((unsigned char)line[1])) {
        line = trim_space(line + 1);
    }
    colon = strchr(line, ':');
    if (colon == NULL) {
        return -1;
    }
    *colon = '\0';
    *key = trim_space(line);
    *value = trim_space(colon + 1);
    strip_optional_quotes(*value);
    return 0;
}

static int yaml_finalize_entry(const char *map_path,
                               motor_map_config *config,
                               unsigned int line_no,
                               int fields,
                               unsigned int index,
                               unsigned int bus,
                               unsigned int id,
                               const char *type,
                               const char *name)
{
    if (fields == 0) {
        return 0;
    }
    if ((fields & 0x07) != 0x07) {
        fprintf(stderr, "%s:%u: motor entry requires bus, id, and type\n", map_path, line_no);
        return -1;
    }
    if ((fields & 0x08) == 0) {
        index = (unsigned int)config->entry_count + 1u;
    }
    return motor_map_add_entry(config, index, bus, id, type, name, line_no);
}

static int yaml_finalize_type_config(const char *map_path,
                                     motor_map_config *config,
                                     unsigned int line_no,
                                     int fields,
                                     const char *type,
                                     const char *firmware,
                                     const bxi_motor_limits *limits)
{
    if (fields == 0) {
        return 0;
    }
    if ((fields & 0x01) != 0x01) {
        fprintf(stderr, "%s:%u: motor type entry requires type\n",
                map_path, line_no);
        return -1;
    }
    return add_motor_type_config(config, type, firmware, limits, map_path, line_no);
}

static int yaml_finalize_state_pattern(const char *map_path,
                                       motor_map_config *config,
                                       unsigned int line_no,
                                       int fields,
                                       const char *label,
                                       const char *pattern)
{
    if (fields == 0) {
        return 0;
    }
    if ((fields & 0x03) != 0x03) {
        fprintf(stderr, "%s:%u: state pattern entry requires label and pattern\n",
                map_path, line_no);
        return -1;
    }
    return add_state_pattern_rule(config, label, pattern, map_path, line_no);
}

static int yaml_finalize_state_code(const char *map_path,
                                    motor_map_config *config,
                                    unsigned int line_no,
                                    int fields,
                                    unsigned int code,
                                    const char *label)
{
    if (fields == 0) {
        return 0;
    }
    if ((fields & 0x03) != 0x03) {
        fprintf(stderr, "%s:%u: state code entry requires code and label\n",
                map_path, line_no);
        return -1;
    }
    return add_state_code_rule(config, code, label, map_path, line_no);
}

typedef enum
{
    YAML_SECTION_GLOBAL = 0,
    YAML_SECTION_FIRMWARE_FILES,
    YAML_SECTION_MOTOR_TYPES,
    YAML_SECTION_STATE_PATTERNS,
    YAML_SECTION_STATE_CODES,
    YAML_SECTION_MOTORS,
} yaml_section;

static int parse_yaml_motor_map(const char *map_path, motor_map_config *config)
{
    FILE *fp;
    char line[256];
    unsigned int line_no = 0u;
    unsigned int entry_line = 0u;
    unsigned int bus = 0u;
    unsigned int id = 0u;
    unsigned int index = 0u;
    char type[MOTOR_TYPE_LEN] = "";
    char name[MOTOR_NAME_LEN] = "";
    int fields = 0;
    unsigned int type_entry_line = 0u;
    char type_entry[MOTOR_TYPE_LEN] = "";
    char type_firmware[FIRMWARE_NAME_MAX] = "";
    bxi_motor_limits type_limits = config->mit_limits;
    int type_fields = 0;
    unsigned int state_pattern_line = 0u;
    char state_pattern_label[MOTOR_STATE_LEN] = "";
    char state_pattern_text[STATE_PATTERN_LEN] = "";
    int state_pattern_fields = 0;
    unsigned int state_code_line = 0u;
    unsigned int state_code = 0u;
    char state_code_label[MOTOR_STATE_LEN] = "";
    int state_code_fields = 0;
    yaml_section section = YAML_SECTION_GLOBAL;

    fp = fopen(map_path, "r");
    if (fp == NULL) {
        perror(map_path);
        return -1;
    }

    while (fgets(line, sizeof(line), fp) != NULL) {
        char *comment;
        char *p;
        char *key;
        char *value;

        line_no++;
        comment = strchr(line, '#');
        if (comment != NULL) {
            *comment = '\0';
        }
        p = trim_space(line);
        if (p[0] == '\0') {
            continue;
        }

        if (strncmp(p, "- ", 2u) == 0) {
            if (section == YAML_SECTION_MOTORS &&
                yaml_finalize_entry(map_path, config, entry_line, fields, index, bus, id,
                                    type, name) != 0) {
                fclose(fp);
                return -1;
            }
            if (section == YAML_SECTION_MOTOR_TYPES &&
                yaml_finalize_type_config(map_path, config, type_entry_line, type_fields,
                                          type_entry, type_firmware, &type_limits) != 0) {
                fclose(fp);
                return -1;
            }
            if (section == YAML_SECTION_STATE_PATTERNS &&
                yaml_finalize_state_pattern(map_path, config, state_pattern_line,
                                            state_pattern_fields, state_pattern_label,
                                            state_pattern_text) != 0) {
                fclose(fp);
                return -1;
            }
            if (section == YAML_SECTION_STATE_CODES &&
                yaml_finalize_state_code(map_path, config, state_code_line,
                                         state_code_fields, state_code,
                                         state_code_label) != 0) {
                fclose(fp);
                return -1;
            }
            if (section == YAML_SECTION_MOTORS) {
                entry_line = line_no;
                bus = 0u;
                id = 0u;
                index = 0u;
                type[0] = '\0';
                name[0] = '\0';
                fields = 0;
            } else if (section == YAML_SECTION_MOTOR_TYPES) {
                type_entry_line = line_no;
                type_entry[0] = '\0';
                type_firmware[0] = '\0';
                type_limits = config->mit_limits;
                type_fields = 0;
            } else if (section == YAML_SECTION_STATE_PATTERNS) {
                state_pattern_line = line_no;
                state_pattern_label[0] = '\0';
                state_pattern_text[0] = '\0';
                state_pattern_fields = 0;
            } else if (section == YAML_SECTION_STATE_CODES) {
                state_code_line = line_no;
                state_code = 0u;
                state_code_label[0] = '\0';
                state_code_fields = 0;
            }
        }

        if (parse_yaml_key_value(p, &key, &value) != 0) {
            fprintf(stderr, "%s:%u: expected key: value\n", map_path, line_no);
            fclose(fp);
            return -1;
        }
        if (strcmp(key, "firmware_dir") == 0) {
            if (!firmware_dir_is_safe(value) ||
                strlen(value) >= sizeof(config->firmware_dir)) {
                fprintf(stderr,
                        "%s:%u: invalid firmware_dir: %s "
                        "(use a relative directory without absolute paths or ..)\n",
                        map_path, line_no, value);
                fclose(fp);
                return -1;
            }
            snprintf(config->firmware_dir, sizeof(config->firmware_dir), "%s", value);
        } else if (strcmp(key, "firmware_base_url") == 0) {
            if (!firmware_url_is_safe(value) ||
                strlen(value) >= sizeof(config->firmware_base_url)) {
                fprintf(stderr,
                        "%s:%u: invalid firmware_base_url: %s "
                        "(use http:// or https:// without spaces)\n",
                        map_path, line_no, value);
                fclose(fp);
                return -1;
            }
            snprintf(config->firmware_base_url, sizeof(config->firmware_base_url), "%s", value);
        } else if (strcmp(key, "firmware_cache_dir") == 0) {
            if (!firmware_dir_is_safe(value) ||
                strlen(value) >= sizeof(config->firmware_cache_dir)) {
                fprintf(stderr,
                        "%s:%u: invalid firmware_cache_dir: %s "
                        "(use a relative directory without absolute paths or ..)\n",
                        map_path, line_no, value);
                fclose(fp);
                return -1;
            }
            snprintf(config->firmware_cache_dir, sizeof(config->firmware_cache_dir), "%s", value);
        } else if (strcmp(key, "firmware_sync_on_start") == 0) {
            if (parse_on_off(value, &config->firmware_sync_on_start) != 0) {
                fprintf(stderr, "%s:%u: firmware_sync_on_start must be on/off\n",
                        map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "home_kp") == 0) {
            if (parse_float_arg(value, &config->home_kp) != 0) {
                fprintf(stderr, "%s:%u: invalid home_kp\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "home_kd") == 0) {
            if (parse_float_arg(value, &config->home_kd) != 0) {
                fprintf(stderr, "%s:%u: invalid home_kd\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "home_kp_list") == 0) {
            size_t count = 0u;

            memset(config->home_kp_by_index_set, 0, sizeof(config->home_kp_by_index_set));
            if (parse_float_list_arg(value,
                                     config->home_kp_by_index,
                                     config->home_kp_by_index_set,
                                     MOTOR_MAP_MAX,
                                     &count) != 0) {
                fprintf(stderr, "%s:%u: invalid home_kp_list\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "home_kd_list") == 0) {
            size_t count = 0u;

            memset(config->home_kd_by_index_set, 0, sizeof(config->home_kd_by_index_set));
            if (parse_float_list_arg(value,
                                     config->home_kd_by_index,
                                     config->home_kd_by_index_set,
                                     MOTOR_MAP_MAX,
                                     &count) != 0) {
                fprintf(stderr, "%s:%u: invalid home_kd_list\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "home_soft_start_ms") == 0) {
            if (parse_uint_arg(value, &config->home_soft_start_ms) != 0) {
                fprintf(stderr, "%s:%u: invalid home_soft_start_ms\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "scan_timeout_ms") == 0) {
            if (parse_uint_arg(value, &config->scan_timeout_ms) != 0) {
                fprintf(stderr, "%s:%u: invalid scan_timeout_ms\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "power_on_wait_ms") == 0) {
            if (parse_uint_arg(value, &config->power_on_wait_ms) != 0) {
                fprintf(stderr, "%s:%u: invalid power_on_wait_ms\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "can_warmup_count") == 0) {
            if (parse_uint_arg(value, &config->can_warmup_count) != 0) {
                fprintf(stderr, "%s:%u: invalid can_warmup_count\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "can_warmup_period_ms") == 0) {
            if (parse_uint_arg(value, &config->can_warmup_period_ms) != 0) {
                fprintf(stderr, "%s:%u: invalid can_warmup_period_ms\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "state_boot_pattern") == 0) {
            snprintf(config->state_boot_pattern, sizeof(config->state_boot_pattern), "%s", value);
        } else if (strcmp(key, "state_motor_pattern") == 0) {
            snprintf(config->state_motor_pattern, sizeof(config->state_motor_pattern), "%s", value);
        } else if (strcmp(key, "state_no_app_pattern") == 0) {
            snprintf(config->state_no_app_pattern, sizeof(config->state_no_app_pattern), "%s", value);
        } else if (strcmp(key, "state_boot_menu_pattern") == 0) {
            snprintf(config->state_boot_menu_pattern, sizeof(config->state_boot_menu_pattern), "%s", value);
        } else if (strcmp(key, "state_motor_menu_pattern") == 0) {
            snprintf(config->state_motor_menu_pattern, sizeof(config->state_motor_menu_pattern), "%s", value);
        } else if (strcmp(key, "state_menu_pattern") == 0) {
            snprintf(config->state_menu_pattern, sizeof(config->state_menu_pattern), "%s", value);
        } else if (strcmp(key, "state_code_prefix") == 0) {
            snprintf(config->state_code_prefix, sizeof(config->state_code_prefix), "%s", value);
        } else if (strncmp(key, "mit_", 4u) == 0 &&
                   strcmp(key, "mit_canfd") != 0) {
            bxi_motor_limits *limits = section == YAML_SECTION_MOTOR_TYPES ?
                                       &type_limits : &config->mit_limits;
            float *target = motor_limit_field(limits, key);

            if (target == NULL || parse_float_arg(value, target) != 0) {
                fprintf(stderr, "%s:%u: unknown or invalid MIT limit: %s\n",
                        map_path, line_no, key);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "mit_canfd") == 0) {
            if (parse_on_off(value, &config->mit_canfd) != 0) {
                fprintf(stderr, "%s:%u: mit_canfd must be on/off\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "live_output") == 0) {
            if (parse_on_off(value, &config->live_output) != 0) {
                fprintf(stderr, "%s:%u: live_output must be on/off\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "language") == 0) {
            if (strcmp(value, "zh") == 0 || strcmp(value, "chinese") == 0) {
                config->chinese_ui = true;
            } else if (strcmp(value, "en") == 0 || strcmp(value, "english") == 0) {
                config->chinese_ui = false;
            } else {
                fprintf(stderr, "%s:%u: language must be zh or en\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
        } else if (strcmp(key, "firmware_files") == 0) {
            section = YAML_SECTION_FIRMWARE_FILES;
        } else if (strcmp(key, "motor_types") == 0) {
            if (section == YAML_SECTION_MOTOR_TYPES &&
                yaml_finalize_type_config(map_path, config, type_entry_line, type_fields,
                                          type_entry, type_firmware, &type_limits) != 0) {
                fclose(fp);
                return -1;
            }
            if (section == YAML_SECTION_STATE_PATTERNS &&
                yaml_finalize_state_pattern(map_path, config, state_pattern_line,
                                            state_pattern_fields, state_pattern_label,
                                            state_pattern_text) != 0) {
                fclose(fp);
                return -1;
            }
            if (section == YAML_SECTION_STATE_CODES &&
                yaml_finalize_state_code(map_path, config, state_code_line,
                                         state_code_fields, state_code,
                                         state_code_label) != 0) {
                fclose(fp);
                return -1;
            }
            section = YAML_SECTION_MOTOR_TYPES;
            type_entry_line = 0u;
            type_fields = 0;
        } else if (strcmp(key, "state_patterns") == 0) {
            if (section == YAML_SECTION_MOTOR_TYPES &&
                yaml_finalize_type_config(map_path, config, type_entry_line, type_fields,
                                          type_entry, type_firmware, &type_limits) != 0) {
                fclose(fp);
                return -1;
            }
            if (section == YAML_SECTION_STATE_CODES &&
                yaml_finalize_state_code(map_path, config, state_code_line,
                                         state_code_fields, state_code,
                                         state_code_label) != 0) {
                fclose(fp);
                return -1;
            }
            section = YAML_SECTION_STATE_PATTERNS;
            state_pattern_line = 0u;
            state_pattern_fields = 0;
        } else if (strcmp(key, "state_codes") == 0) {
            if (section == YAML_SECTION_MOTOR_TYPES &&
                yaml_finalize_type_config(map_path, config, type_entry_line, type_fields,
                                          type_entry, type_firmware, &type_limits) != 0) {
                fclose(fp);
                return -1;
            }
            if (section == YAML_SECTION_STATE_PATTERNS &&
                yaml_finalize_state_pattern(map_path, config, state_pattern_line,
                                            state_pattern_fields, state_pattern_label,
                                            state_pattern_text) != 0) {
                fclose(fp);
                return -1;
            }
            section = YAML_SECTION_STATE_CODES;
            state_code_line = 0u;
            state_code_fields = 0;
        } else if (strcmp(key, "motors") == 0) {
            if (section == YAML_SECTION_MOTOR_TYPES &&
                yaml_finalize_type_config(map_path, config, type_entry_line, type_fields,
                                          type_entry, type_firmware, &type_limits) != 0) {
                fclose(fp);
                return -1;
            }
            if (section == YAML_SECTION_STATE_PATTERNS &&
                yaml_finalize_state_pattern(map_path, config, state_pattern_line,
                                            state_pattern_fields, state_pattern_label,
                                            state_pattern_text) != 0) {
                fclose(fp);
                return -1;
            }
            if (section == YAML_SECTION_STATE_CODES &&
                yaml_finalize_state_code(map_path, config, state_code_line,
                                         state_code_fields, state_code,
                                         state_code_label) != 0) {
                fclose(fp);
                return -1;
            }
            section = YAML_SECTION_MOTORS;
            entry_line = 0u;
            fields = 0;
        } else if (section == YAML_SECTION_FIRMWARE_FILES) {
            if (add_firmware_mapping(config, key, value, map_path, line_no) != 0) {
                fclose(fp);
                return -1;
            }
        } else if (section == YAML_SECTION_MOTOR_TYPES &&
                   strcmp(key, "firmware") == 0) {
            if (!firmware_filename_is_safe(value) || strlen(value) >= sizeof(type_firmware)) {
                fprintf(stderr, "%s:%u: invalid firmware filename: %s\n",
                        map_path, line_no, value);
                fclose(fp);
                return -1;
            }
            snprintf(type_firmware, sizeof(type_firmware), "%s", value);
            if (type_entry_line == 0u) {
                type_entry_line = line_no;
            }
            type_fields |= 0x02;
        } else if (strcmp(key, "index") == 0) {
            if (parse_uint_arg(value, &index) != 0) {
                fprintf(stderr, "%s:%u: invalid motor index\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
            fields |= 0x08;
        } else if (strcmp(key, "bus") == 0) {
            if (parse_uint_arg(value, &bus) != 0) {
                fprintf(stderr, "%s:%u: invalid bus: %s\n", map_path, line_no, value);
                fclose(fp);
                return -1;
            }
            if (entry_line == 0u) {
                entry_line = line_no;
            }
            fields |= 0x01;
        } else if (strcmp(key, "id") == 0) {
            if (parse_uint_arg(value, &id) != 0) {
                fprintf(stderr, "%s:%u: invalid id: %s\n", map_path, line_no, value);
                fclose(fp);
                return -1;
            }
            if (entry_line == 0u) {
                entry_line = line_no;
            }
            fields |= 0x02;
        } else if (strcmp(key, "type") == 0) {
            char *target_type = section == YAML_SECTION_MOTOR_TYPES ? type_entry : type;

            if (normalize_firmware_type(value, target_type, MOTOR_TYPE_LEN) != 0) {
                fprintf(stderr, "%s:%u: invalid firmware type: %s\n", map_path, line_no, value);
                fclose(fp);
                return -1;
            }
            if (section == YAML_SECTION_MOTOR_TYPES) {
                if (type_entry_line == 0u) {
                    type_entry_line = line_no;
                }
                type_fields |= 0x01;
            } else if (entry_line == 0u) {
                entry_line = line_no;
                fields |= 0x04;
            } else {
                fields |= 0x04;
            }
        } else if (section == YAML_SECTION_STATE_PATTERNS &&
                   strcmp(key, "label") == 0) {
            snprintf(state_pattern_label, sizeof(state_pattern_label), "%s", value);
            if (state_pattern_line == 0u) {
                state_pattern_line = line_no;
            }
            state_pattern_fields |= 0x01;
        } else if (section == YAML_SECTION_STATE_PATTERNS &&
                   strcmp(key, "pattern") == 0) {
            snprintf(state_pattern_text, sizeof(state_pattern_text), "%s", value);
            if (state_pattern_line == 0u) {
                state_pattern_line = line_no;
            }
            state_pattern_fields |= 0x02;
        } else if (section == YAML_SECTION_STATE_CODES &&
                   strcmp(key, "code") == 0) {
            if (parse_uint_arg(value, &state_code) != 0) {
                fprintf(stderr, "%s:%u: invalid state code: %s\n",
                        map_path, line_no, value);
                fclose(fp);
                return -1;
            }
            if (state_code_line == 0u) {
                state_code_line = line_no;
            }
            state_code_fields |= 0x01;
        } else if (section == YAML_SECTION_STATE_CODES &&
                   strcmp(key, "label") == 0) {
            snprintf(state_code_label, sizeof(state_code_label), "%s", value);
            if (state_code_line == 0u) {
                state_code_line = line_no;
            }
            state_code_fields |= 0x02;
        } else if (section == YAML_SECTION_MOTORS && strcmp(key, "name") == 0) {
            if (strlen(value) >= sizeof(name)) {
                fprintf(stderr, "%s:%u: motor name is too long\n", map_path, line_no);
                fclose(fp);
                return -1;
            }
            snprintf(name, sizeof(name), "%s", value);
        }
    }

    if (section == YAML_SECTION_MOTOR_TYPES &&
        yaml_finalize_type_config(map_path, config, type_entry_line, type_fields,
                                  type_entry, type_firmware, &type_limits) != 0) {
        fclose(fp);
        return -1;
    }
    if (section == YAML_SECTION_STATE_PATTERNS &&
        yaml_finalize_state_pattern(map_path, config, state_pattern_line,
                                    state_pattern_fields, state_pattern_label,
                                    state_pattern_text) != 0) {
        fclose(fp);
        return -1;
    }
    if (section == YAML_SECTION_STATE_CODES &&
        yaml_finalize_state_code(map_path, config, state_code_line,
                                 state_code_fields, state_code,
                                 state_code_label) != 0) {
        fclose(fp);
        return -1;
    }
    if (section == YAML_SECTION_MOTORS &&
        yaml_finalize_entry(map_path, config, entry_line, fields, index, bus, id,
                            type, name) != 0) {
        fclose(fp);
        return -1;
    }

    fclose(fp);
    return 0;
}

static int load_motor_map_config(const char *map_path, motor_map_config *config)
{
    int ret;

    memset(config, 0, sizeof(*config));
    snprintf(config->firmware_dir, sizeof(config->firmware_dir), "%s", DEFAULT_FIRMWARE_DIR);
    snprintf(config->firmware_cache_dir, sizeof(config->firmware_cache_dir), "%s", "firmware_cache");
    config->home_kp = 20.0f;
    config->home_kd = 1.0f;
    config->home_soft_start_ms = 2000u;
    config->scan_timeout_ms = 1000u;
    config->power_on_wait_ms = 2000u;
    config->can_warmup_count = 260u;
    config->can_warmup_period_ms = 10u;
    snprintf(config->state_boot_pattern, sizeof(config->state_boot_pattern), "boot...");
    snprintf(config->state_motor_pattern, sizeof(config->state_motor_pattern), "APP   :");
    snprintf(config->state_no_app_pattern, sizeof(config->state_no_app_pattern), "no useful app");
    snprintf(config->state_boot_menu_pattern, sizeof(config->state_boot_menu_pattern), "Boot menu:");
    snprintf(config->state_motor_menu_pattern, sizeof(config->state_motor_menu_pattern), "Motor main menu:");
    snprintf(config->state_code_prefix, sizeof(config->state_code_prefix), "STATE :");
    config->mit_limits = bxi_motor_default_limits;
    config->mit_canfd = true;
    config->live_output = true;
    config->chinese_ui = true;

    if (has_suffix(map_path, ".csv")) {
        ret = parse_csv_motor_map(map_path, config);
        if (ret == 0) {
            add_default_state_rules_if_missing(config);
        }
        return ret;
    }
    ret = parse_yaml_motor_map(map_path, config);
    if (ret == 0) {
        add_default_state_rules_if_missing(config);
    }
    if (ret == 0 && (!isfinite(config->home_kp) || !isfinite(config->home_kd))) {
        fprintf(stderr, "%s: invalid home_kp/home_kd\n", map_path);
        return -1;
    }
    if (ret == 0 && !motor_limits_are_valid(&config->mit_limits)) {
        fprintf(stderr, "%s: invalid MIT ranges\n", map_path);
        return -1;
    }
    if (ret == 0 && config->type_config_count == 0u &&
        (config->home_kp < config->mit_limits.kp_min ||
         config->home_kp > config->mit_limits.kp_max ||
         config->home_kd < config->mit_limits.kd_min ||
         config->home_kd > config->mit_limits.kd_max)) {
        fprintf(stderr, "%s: home_kp/home_kd are outside the configured MIT limits\n",
                map_path);
        return -1;
    }
    if (ret == 0) {
        size_t i;

        for (i = 0u; i < config->type_config_count; i++) {
            const motor_type_config *type_config = &config->type_configs[i];

            if (config->home_kp < type_config->limits.kp_min ||
                config->home_kp > type_config->limits.kp_max ||
                config->home_kd < type_config->limits.kd_min ||
                config->home_kd > type_config->limits.kd_max) {
                fprintf(stderr,
                        "%s: home_kp/home_kd are outside MIT limits for motor type %s\n",
                        map_path,
                        type_config->type);
                return -1;
            }
        }
        for (i = 0u; i < config->entry_count; i++) {
            const motor_map_entry *entry = &config->entries[i];
            const motor_type_config *type_config = NULL;
            const bxi_motor_limits *limits = &config->mit_limits;
            float home_kp = entry->index < MOTOR_MAP_MAX &&
                            config->home_kp_by_index_set[entry->index] ?
                            config->home_kp_by_index[entry->index] : config->home_kp;
            float home_kd = entry->index < MOTOR_MAP_MAX &&
                            config->home_kd_by_index_set[entry->index] ?
                            config->home_kd_by_index[entry->index] : config->home_kd;

            if (config->type_config_count != 0u) {
                type_config = find_type_config(config, entry->type);
                if (type_config == NULL) {
                    fprintf(stderr, "%s: motor index %02u uses undefined motor type %s\n",
                            map_path, entry->index, entry->type);
                    return -1;
                }
                limits = &type_config->limits;
            }
            if (!isfinite(home_kp) || !isfinite(home_kd) ||
                home_kp < limits->kp_min || home_kp > limits->kp_max ||
                home_kd < limits->kd_min || home_kd > limits->kd_max) {
                fprintf(stderr,
                        "%s: stand_up kp/kd for motor index %02u are outside MIT limits\n",
                        map_path, entry->index);
                return -1;
            }
        }
    }
    if (ret == 0 && (config->scan_timeout_ms == 0u ||
                     config->scan_timeout_ms > 60000u ||
                     config->home_soft_start_ms > 60000u ||
                     config->power_on_wait_ms > 60000u ||
                     config->can_warmup_period_ms > 60000u)) {
        fprintf(stderr, "%s: timing values must be in the supported 0..60000 ms range\n",
                map_path);
        return -1;
    }
    return ret;
}

static void validate_motor_map_firmware(const motor_map_config *config,
                                        const firmware_inventory *inventory)
{
    size_t i;

    for (i = 0u; i < config->entry_count; i++) {
        const motor_map_entry *entry = &config->entries[i];

        if (!firmware_inventory_has_type(inventory, entry->type)) {
            fprintf(stderr,
                    "ERROR: missing firmware for bus=%u id=%u type=%s: expected %s/%s.bin\n",
                    entry->bus,
                    entry->id,
                    entry->type,
                    config->firmware_dir,
                    entry->type);
        }
    }
}

static int motor_is_selected(const motor_map_entry *entry,
                             const flash_state *state,
                             const char *selection)
{
    if (strcmp(selection, "all") == 0) {
        return 1;
    }
    if (strcmp(selection, "bus") == 0) {
        return entry->bus == state->bus;
    }
    return entry->bus == state->bus && entry->id == state->boot_id;
}

static int lookup_firmware_type(const motor_map_config *config,
                                unsigned int bus,
                                unsigned int id,
                                char *type,
                                size_t type_len)
{
    size_t i;

    for (i = 0u; i < config->entry_count; i++) {
        const motor_map_entry *entry = &config->entries[i];

        if (entry->bus == bus && entry->id == id) {
            snprintf(type, type_len, "%s", entry->type);
            return 0;
        }
    }

    fprintf(stderr, "no firmware type for bus=%u id=%u in map\n", bus, id);
    return -1;
}

static int send_ymodem_packet_with_retry(flash_state *state,
                                         const uint8_t *packet,
                                         size_t len,
                                         unsigned int ack_timeout_ms,
                                         unsigned int retry_limit)
{
    unsigned int attempt;

    for (attempt = 1u; attempt <= retry_limit && !stop_requested; attempt++) {
        uint64_t deadline;

        if (send_boot_data(state, packet, len) != 0) {
            return -1;
        }

        deadline = time_us() + (uint64_t)ack_timeout_ms * 1000ULL;
        while (!stop_requested && time_us() < deadline) {
            uint8_t byte;
            unsigned int step_ms = 20u;
            uint64_t now = time_us();

            if (deadline > now) {
                uint64_t remain_ms = (deadline - now) / 1000ULL;
                if (remain_ms < step_ms) {
                    step_ms = (unsigned int)remain_ms;
                }
            }
            if (step_ms == 0u) {
                step_ms = 1u;
            }

            if (!rx_ring_pop(&state->rx, &byte, step_ms)) {
                continue;
            }
            if (byte == YMODEM_ACK) {
                return 0;
            }
            if (byte == YMODEM_NAK || byte == YMODEM_CAN) {
                break;
            }
        }

        if (!state->brief_flash_output) {
            printf("packet retry %u/%u\n", attempt, retry_limit);
        }
    }

    return -1;
}

static int ymodem_send_file(flash_state *state, const char *path)
{
    struct stat st;
    FILE *fp = NULL;
    uint8_t packet[YMODEM_MAX_PACKET];
    uint8_t header[YMODEM_PACKET_128];
    uint8_t payload[YMODEM_PACKET_128];
    const char *name;
    uint32_t file_size;
    size_t packet_len;
    size_t sent = 0u;
    uint8_t seq = 1u;

    if (stat(path, &st) != 0) {
        perror(path);
        return -1;
    }
    if (!S_ISREG(st.st_mode) || st.st_size < 0 || st.st_size > UINT32_MAX) {
        fprintf(stderr, "invalid firmware file: %s\n", path);
        return -1;
    }

    fp = fopen(path, "rb");
    if (fp == NULL) {
        perror(path);
        return -1;
    }

    file_size = (uint32_t)st.st_size;
    name = path_basename(path);
    memset(header, 0, sizeof(header));
    if (snprintf((char *)header, sizeof(header), "%s%c%u", name, '\0', file_size) >=
        (int)sizeof(header)) {
        fprintf(stderr, "file name is too long for YMODEM header: %s\n", name);
        fclose(fp);
        return -1;
    }

    if (!state->brief_flash_output) {
        printf("waiting for YMODEM C request\n");
    }
    if (wait_for_byte(state, YMODEM_C, 5000u, !state->brief_flash_output) != 0) {
        fprintf(stderr, "bootloader did not request YMODEM transfer\n");
        fclose(fp);
        return -1;
    }

    if (!state->brief_flash_output) {
        printf("\nsending header: %s (%u bytes)\n", name, file_size);
    }
    packet_len = make_ymodem_packet(packet, 0u, header, sizeof(header), YMODEM_PACKET_128, 0u);
    if (send_ymodem_packet_with_retry(state, packet, packet_len, 15000u, 5u) != 0) {
        fprintf(stderr, "failed to send YMODEM header\n");
        fclose(fp);
        return -1;
    }
    if (wait_for_byte(state, YMODEM_C, 5000u, !state->brief_flash_output) != 0) {
        fprintf(stderr, "bootloader did not request first data packet\n");
        fclose(fp);
        return -1;
    }

    while (sent < (size_t)file_size && !stop_requested) {
        size_t n = fread(payload, 1u, sizeof(payload), fp);
        size_t progress;

        if (n == 0u && ferror(fp)) {
            perror(path);
            fclose(fp);
            return -1;
        }
        if (n == 0u) {
            break;
        }

        packet_len = make_ymodem_packet(packet, seq, payload, n, YMODEM_PACKET_128, 0x1au);
        if (send_ymodem_packet_with_retry(state, packet, packet_len, 5000u, 5u) != 0) {
            fprintf(stderr, "failed to send data packet seq=%u\n", seq);
            fclose(fp);
            return -1;
        }

        sent += n;
        seq = (uint8_t)(seq + 1u);
        progress = file_size == 0u ? 100u : sent * 100u / (size_t)file_size;
        if (state->brief_flash_output) {
            printf("\rFLASH PROGRESS %02zu/%02zu index=%02u %zu%%",
                   state->brief_flash_ordinal,
                   state->brief_flash_total,
                   state->brief_flash_index,
                   progress);
        } else {
            printf("\rflashing: %zu/%u bytes (%zu%%)", sent, file_size, progress);
        }
        fflush(stdout);
    }
    printf("\n");

    fclose(fp);

    if (send_boot_byte(state, YMODEM_EOT) != 0 ||
        wait_for_byte(state, YMODEM_NAK, 5000u, !state->brief_flash_output) != 0 ||
        send_boot_byte(state, YMODEM_EOT) != 0 ||
        wait_for_byte(state, YMODEM_ACK, 5000u, !state->brief_flash_output) != 0 ||
        wait_for_byte(state, YMODEM_C, 5000u, !state->brief_flash_output) != 0) {
        fprintf(stderr, "failed during YMODEM EOT handshake\n");
        return -1;
    }

    packet_len = make_ymodem_packet(packet, 0u, NULL, 0u, YMODEM_PACKET_128, 0u);
    if (send_ymodem_packet_with_retry(state, packet, packet_len, 5000u, 5u) != 0) {
        fprintf(stderr, "failed to send final empty YMODEM packet\n");
        return -1;
    }

    if (!state->brief_flash_output) {
        printf("firmware transfer complete\n");
    }
    collect_text(state, 250u, 2000u);
    return 0;
}

static int boot_flash_file(flash_state *state, const char *path, bool power_cycle)
{
    unsigned int update_delay_ms = DEFAULT_BOOT_UPDATE_DELAY_MS;
    bool already_in_boot_menu = false;
    int ret;
    size_t i;

    if (state->config_loaded) {
        for (i = 0u; i < state->config.entry_count; i++) {
            const motor_map_entry *entry = &state->config.entries[i];

            if (entry->bus == state->bus && entry->id == state->boot_id &&
                strcmp(motor_runtime_state(&state->motors[i]), "boot_menu") == 0) {
                already_in_boot_menu = true;
                break;
            }
        }
    }

    if (already_in_boot_menu) {
        if (!state->brief_flash_output) {
            printf("motor is already in boot menu; skip 'r'/'m'\n");
        }
    } else {
        if (enter_boot_menu(state, power_cycle) != 0) {
            return -1;
        }
    }
    rx_ring_clear(&state->rx);
    if (update_delay_ms != 0u) {
        if (!state->brief_flash_output) {
            printf("waiting %u ms before sending 'u'\n", update_delay_ms);
        }
        sleep_ms(update_delay_ms);
    }
    if (send_boot_byte(state, 'u') != 0) {
        return -1;
    }
    ret = ymodem_send_file(state, path);
    if (ret != 0) {
        return ret;
    }

    if (!state->brief_flash_output) {
        printf("sending jump-to-app command 'a'\n");
    }
    rx_ring_clear(&state->rx);
    if (send_boot_byte(state, 'a') != 0) {
        return -1;
    }
    collect_text(state, 250u, 2000u);
    return 0;
}

static int run_flash_auto(flash_state *state, int argc, char **argv)
{
    const char *map_path = DEFAULT_FIRMWARE_MAP;
    const char *selection = "one";
    bool cycle = state->auto_power_cycle;
    bool map_set = false;
    motor_map_config config;
    size_t i;
    size_t selected = 0u;
    size_t succeeded = 0u;
    size_t failed = 0u;
    unsigned int original_bus = state->bus;
    unsigned int original_id = state->boot_id;

    for (i = 0u; i < (size_t)argc; i++) {
        if (strcmp(argv[i], "cycle") == 0) {
            cycle = true;
        } else if (strcmp(argv[i], "one") == 0 ||
                   strcmp(argv[i], "bus") == 0 ||
                   strcmp(argv[i], "all") == 0) {
            selection = argv[i];
        } else if (!map_set) {
            map_path = argv[i];
            map_set = true;
        } else {
            printf("usage: flash-auto [map.yaml|map.csv] [one|bus|all] [cycle]\n");
            return -1;
        }
    }

    if (load_motor_map_config(map_path, &config) != 0) {
        return -1;
    }
    /* Preflight every selected target before changing any motor. */
    for (i = 0u; i < config.entry_count; i++) {
        char path[PATH_LEN] = "";
        const motor_map_entry *entry = &config.entries[i];

        if (!motor_is_selected(entry, state, selection)) {
            continue;
        }
        selected++;
        if (configured_firmware_path(&config, entry->type, path, sizeof(path)) != 0 ||
            access(path, R_OK) != 0) {
            if (path[0] != '\0') {
                fprintf(stderr, "cannot read firmware for bus=%u id=%u type=%s: %s\n",
                        entry->bus, entry->id, entry->type, path);
            }
            failed++;
        }
    }
    if (selected == 0u) {
        fprintf(stderr, "no motors selected by '%s' (bus=%u id=%u)\n",
                selection, state->bus, state->boot_id);
        return -1;
    }
    if (failed != 0u) {
        fprintf(stderr, "preflight failed: %zu/%zu selected firmware files unavailable; nothing flashed\n",
                failed, selected);
        return -1;
    }

    if (state->brief_flash_output) {
        printf("FLASH SUMMARY total=%zu start\n", selected);
    } else {
        printf("\nBATCH FLASH: selection=%s targets=%zu map=%s power must be ON\n",
               selection, selected, map_path);
    }
    failed = 0u;
    for (i = 0u; i < config.entry_count && !stop_requested; i++) {
        char path[PATH_LEN] = "";
        const motor_map_entry *entry = &config.entries[i];
        size_t ordinal;

        if (!motor_is_selected(entry, state, selection)) {
            continue;
        }
        state->bus = entry->bus;
        set_target_id(state, entry->id);
        configured_firmware_path(&config, entry->type, path, sizeof(path));
        ordinal = succeeded + failed + 1u;
        state->brief_flash_index = entry->index;
        state->brief_flash_ordinal = ordinal;
        state->brief_flash_total = selected;
        if (state->brief_flash_output) {
            printf("FLASH START %02zu/%02zu index=%02u bus=%u id=%u type=%s\n",
                   ordinal, selected, entry->index, entry->bus, entry->id, entry->type);
        } else {
            printf("\n######## FLASH %zu/%zu: bus=%u id=%u type=%s file=%s ########\n",
                   ordinal, selected, entry->bus, entry->id, entry->type, path);
        }
        if (boot_flash_file(state, path, cycle) == 0) {
            succeeded++;
            if (state->brief_flash_output) {
                printf("FLASH DONE  %02zu/%02zu index=%02u result=OK\n",
                       ordinal, selected, entry->index);
            } else {
                printf("######## SUCCESS: bus=%u id=%u ########\n", entry->bus, entry->id);
            }
        } else {
            failed++;
            if (state->brief_flash_output) {
                fprintf(stderr, "FLASH DONE  %02zu/%02zu index=%02u result=FAILED\n",
                        ordinal, selected, entry->index);
            } else {
                fprintf(stderr, "######## FAILED: bus=%u id=%u ########\n", entry->bus, entry->id);
            }
        }
    }
    state->bus = original_bus;
    set_target_id(state, original_id);
    if (state->brief_flash_output) {
        printf("FLASH SUMMARY total=%zu success=%zu failed=%zu%s\n",
               selected, succeeded, failed, stop_requested ? " interrupted" : "");
    } else {
        printf("\nBATCH SUMMARY: selected=%zu success=%zu failed=%zu%s\n",
               selected, succeeded, failed, stop_requested ? " interrupted" : "");
    }
    return failed == 0u && succeeded == selected ? 0 : -1;
}

static int set_target_id(flash_state *state, unsigned int id)
{
    if (id > 8u) {
        return -1;
    }

    state->boot_id = id;
    update_boot_ids(state);
    rx_ring_clear(&state->rx);
    frame_ring_clear(&state->frames);
    return 0;
}

static int reg_frame_id(const flash_state *state, unsigned int cmd)
{
    return (int)((cmd << 4) | (state->boot_id & 0x0fu));
}

static int run_debug_ping(flash_state *state, int argc, char **argv)
{
    unsigned int wait_ms = 1000u;

    if (argc > 0 && parse_uint_arg(argv[0], &wait_ms) != 0) {
        printf("usage: ping [wait_ms]\n");
        return -1;
    }

    frame_ring_clear(&state->frames);
    if (send_debug_mit(state, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f) != 0) {
        return -1;
    }
    return wait_debug_reply(state, state->master_id, wait_ms);
}

static int run_debug_special(flash_state *state, uint8_t command, int argc, char **argv)
{
    unsigned int wait_ms = 1000u;

    if (argc > 0 && parse_uint_arg(argv[0], &wait_ms) != 0) {
        printf("usage: enable|disable|zero [wait_ms]\n");
        return -1;
    }

    frame_ring_clear(&state->frames);
    if (send_debug_special(state, command) != 0) {
        return -1;
    }
    return wait_debug_reply(state, state->master_id, wait_ms);
}

static int run_debug_mit(flash_state *state, int argc, char **argv)
{
    float position;
    float velocity;
    float kp;
    float kd;
    float torque;
    unsigned int duration_ms = 0u;
    unsigned int period_ms = 10u;
    uint64_t end;

    if (argc < 5 ||
        parse_float_arg(argv[0], &position) != 0 ||
        parse_float_arg(argv[1], &velocity) != 0 ||
        parse_float_arg(argv[2], &kp) != 0 ||
        parse_float_arg(argv[3], &kd) != 0 ||
        parse_float_arg(argv[4], &torque) != 0) {
        printf("usage: mit <pos> <vel> <kp> <kd> <torque> [duration_ms] [period_ms]\n");
        return -1;
    }
    if (argc > 5 && parse_uint_arg(argv[5], &duration_ms) != 0) {
        printf("invalid duration_ms: %s\n", argv[5]);
        return -1;
    }
    if (argc > 6 && parse_uint_arg(argv[6], &period_ms) != 0) {
        printf("invalid period_ms: %s\n", argv[6]);
        return -1;
    }
    if (period_ms == 0u) {
        period_ms = 1u;
    }

    frame_ring_clear(&state->frames);
    if (duration_ms == 0u) {
        if (send_debug_mit(state, position, velocity, kp, kd, torque) != 0) {
            return -1;
        }
        return wait_debug_reply(state, state->master_id, 1000u);
    }

    end = time_us() + (uint64_t)duration_ms * 1000ULL;
    while (!stop_requested && time_us() < end) {
        if (send_debug_mit(state, position, velocity, kp, kd, torque) != 0) {
            return -1;
        }
        sleep_ms(period_ms);
    }
    printf("sent MIT command stream for %u ms\n", duration_ms);
    return 0;
}

static int run_debug_reg_read(flash_state *state, int argc, char **argv)
{
    unsigned int reg;
    unsigned int wait_ms = 1000u;
    uint8_t data[4];
    unsigned int frame_id;

    if (argc < 1 || parse_uint_arg(argv[0], &reg) != 0) {
        printf("usage: reg-read <index> [wait_ms]\n");
        return -1;
    }
    if (argc > 1 && parse_uint_arg(argv[1], &wait_ms) != 0) {
        printf("invalid wait_ms: %s\n", argv[1]);
        return -1;
    }

    frame_id = (unsigned int)reg_frame_id(state, CAN_CMD_REG_READ);
    u32_to_data(reg, data);
    frame_ring_clear(&state->frames);
    if (send_debug_packet(state, frame_id, data, sizeof(data)) != 0) {
        return -1;
    }
    return wait_debug_reply(state, frame_id, wait_ms);
}

static int run_debug_reg_write(flash_state *state, int argc, char **argv)
{
    unsigned int reg;
    unsigned int value;
    unsigned int wait_ms = 1000u;
    uint8_t data[8];
    unsigned int frame_id;

    if (argc < 2 ||
        parse_uint_arg(argv[0], &reg) != 0 ||
        parse_uint_arg(argv[1], &value) != 0) {
        printf("usage: reg-write <index> <u32_value> [wait_ms]\n");
        return -1;
    }
    if (argc > 2 && parse_uint_arg(argv[2], &wait_ms) != 0) {
        printf("invalid wait_ms: %s\n", argv[2]);
        return -1;
    }

    frame_id = (unsigned int)reg_frame_id(state, CAN_CMD_REG_WRITE);
    u32_to_data(reg, &data[0]);
    u32_to_data(value, &data[4]);
    frame_ring_clear(&state->frames);
    if (send_debug_packet(state, frame_id, data, sizeof(data)) != 0) {
        return -1;
    }
    return wait_debug_reply(state, frame_id, wait_ms);
}

static int run_debug_reg_save(flash_state *state, int argc, char **argv)
{
    unsigned int wait_ms = 1000u;
    uint8_t data[4] = {0u, 0u, 0u, 0u};
    unsigned int frame_id;

    if (argc > 0 && parse_uint_arg(argv[0], &wait_ms) != 0) {
        printf("usage: reg-save [wait_ms]\n");
        return -1;
    }

    frame_id = (unsigned int)reg_frame_id(state, CAN_CMD_REG_SAVE);
    frame_ring_clear(&state->frames);
    if (send_debug_packet(state, frame_id, data, sizeof(data)) != 0) {
        return -1;
    }
    return wait_debug_reply(state, frame_id, wait_ms);
}

static int run_debug_listen(flash_state *state, int argc, char **argv)
{
    unsigned int seconds = 5u;
    bool all = false;
    uint64_t deadline;

    if (argc > 0 && parse_uint_arg(argv[0], &seconds) != 0) {
        printf("usage: listen [seconds] [all]\n");
        return -1;
    }
    if (argc > 1 && strcmp(argv[1], "all") == 0) {
        all = true;
    }

    printf("debug listening for %u second(s)%s\n", seconds, all ? " (all bus frames)" : "");
    deadline = time_us() + (uint64_t)seconds * 1000000ULL;
    while (!stop_requested && time_us() < deadline) {
        rx_can_frame frame;
        uint64_t now = time_us();
        unsigned int step_ms = 50u;

        if (deadline > now) {
            uint64_t remain_ms = (deadline - now) / 1000ULL;
            if (remain_ms < step_ms) {
                step_ms = (unsigned int)remain_ms;
            }
        }
        if (step_ms == 0u) {
            step_ms = 1u;
        }
        if (!frame_ring_pop(&state->frames, &frame, step_ms)) {
            continue;
        }
        if (!debug_frame_matches(state, &frame, 0u, all)) {
            continue;
        }
        print_debug_frame(state, &frame);
    }
    return 0;
}

static void print_usage(const char *prog)
{
    printf("Usage:\n");
    printf("  sudo %s [options]\n", prog);
    printf("  sudo %s [options] debug\n", prog);
    printf("  sudo %s [options] flash <firmware.bin> [cycle]\n", prog);
    printf("  sudo %s [options] flash-auto [config.yaml] [one|bus|all] [cycle]\n", prog);
    printf("  sudo %s [options] enter [cycle]\n", prog);
    printf("\nOptions:\n");
    printf("  -b, --bus N          CANFD bus, default 0\n");
    printf("  -i, --id N           bootloader id in [1,8], default 1\n");
    printf("      --power-cycle    power cycle instead of soft reset for enter/flash\n");
    printf("      --power          turn motor power on and wait 2 seconds before command\n");
    printf("      --keep-power     leave motor power on when --power was used\n");
    printf("      --quiet-tx       do not print transmitted CAN frames\n");
    printf("  -h, --help           show this help\n");
}

static void print_config(const flash_state *state)
{
    printf("config: mode=%s bus=%u motor_id=%u boot_tx=0x%03x boot_rx=0x%03x master_id=0x%03x auto_power_cycle=%s debug_frame=%s\n",
           state->mode == TOOL_MODE_DEBUG ? "debug" : "flash",
           state->bus,
           state->boot_id,
           state->tx_id,
           state->rx_id,
           state->master_id,
           state->auto_power_cycle ? "on" : "off",
           state->debug_use_canfd ? "CANFD+BRS" : "classic CAN");
}

static void print_terminal_help(void)
{
    printf("Interactive commands:\n");
    printf("  help | ?                 show this help\n");
    printf("  config                   show current settings\n");
    printf("  mode flash|debug         switch terminal state\n");
    printf("  debug [command...]       enter debug state or run one debug command\n");
    printf("  flash-mode               return to bootloader flashing state\n");
    printf("  bus <0-4>                set CANFD bus\n");
    printf("  id <1-8>                 select target motor/bootloader id\n");
    printf("  master-id <id>           set expected MIT feedback id\n");
    printf("  canfd | classic          set debug frame format, boot flashing stays classic\n");
    printf("  auto-cycle on|off        use power cycle instead of soft reset by default\n");
    printf("  power on|off|cycle       motor power control\n");
    printf("  enter [cycle]            send 'r', then 'm' to enter boot; cycle uses power reset\n");
    printf("  version                  request bootloader version info\n");
    printf("  set-id <1-8>             set bootloader id in flash\n");
    printf("  flash <firmware.bin> [cycle] enter boot, send 'u', then YMODEM over CAN\n");
    printf("  flash-one [map] [cycle]  flash selected bus/id from the configuration\n");
    printf("  flash-bus [map] [cycle]  flash all configured motors on current bus\n");
    printf("  flash-all [map] [cycle]  flash all configured motors on all buses\n");
    printf("  flash-auto [map] [one|bus|all] [cycle] compatible generic form\n");
    printf("  jump                     jump from bootloader to app\n");
    printf("  ping [wait_ms]           debug: send zero MIT frame to selected motor\n");
    printf("  enable|disable|zero      debug: send MIT special command\n");
    printf("  mit <p> <v> <kp> <kd> <t> [duration_ms] [period_ms]\n");
    printf("  reg-read <index> [wait_ms]\n");
    printf("  reg-write <index> <u32_value> [wait_ms]\n");
    printf("  reg-save [wait_ms]\n");
    printf("  listen [seconds] [all]   flash: boot output, debug: selected motor frames\n");
    printf("  send <text>              send raw menu text bytes\n");
    printf("  q | quit | exit          exit terminal\n");
}

static int split_line(char *line, char **argv, int max_args)
{
    int argc = 0;
    char *p = line;

    while (*p != '\0' && argc < max_args) {
        while (*p != '\0' && isspace((unsigned char)*p)) {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        argv[argc++] = p;
        while (*p != '\0' && !isspace((unsigned char)*p)) {
            p++;
        }
        if (*p != '\0') {
            *p++ = '\0';
        }
    }

    return argc;
}

static const char *const command_words[] = {
    "q",
    "quit",
    "exit",
    "help",
    "?",
    "config",
    "mode",
    "debug",
    "flash-mode",
    "bus",
    "id",
    "motor",
    "master-id",
    "master",
    "canfd",
    "classic",
    "auto-cycle",
    "power",
    "enter",
    "version",
    "set-id",
    "flash",
    "flash-one",
    "flash-bus",
    "flash-all",
    "flash-auto",
    "jump",
    "ping",
    "enable",
    "disable",
    "zero",
    "mit",
    "reg-read",
    "reg-write",
    "reg-save",
    "listen",
    "send",
};

static const char *const on_off_words[] = {
    "on",
    "off",
};

static const char *const power_words[] = {
    "on",
    "off",
    "cycle",
};

static const char *const cycle_words[] = {
    "cycle",
};

static const char *const mode_words[] = {
    "flash",
    "debug",
};

static const char *const listen_words[] = {
    "all",
};

static const char *const reset_words[] = {
    "reset",
};

static const char *const language_words[] = {
    "zh",
    "en",
};

/* A program embedding this terminal editor may replace top-level completion. */
static const char *const *custom_command_words = NULL;
static size_t custom_command_word_count = 0u;

static int starts_with(const char *text, const char *prefix)
{
    return strncmp(text, prefix, strlen(prefix)) == 0;
}

static int current_token_start(const char *line, size_t len)
{
    int pos = (int)len;

    while (pos > 0 && !isspace((unsigned char)line[pos - 1])) {
        pos--;
    }

    return pos;
}

static unsigned int count_tokens_before(const char *line, int end)
{
    unsigned int count = 0u;
    int pos = 0;

    while (pos < end) {
        while (pos < end && isspace((unsigned char)line[pos])) {
            pos++;
        }
        if (pos >= end) {
            break;
        }
        count++;
        while (pos < end && !isspace((unsigned char)line[pos])) {
            pos++;
        }
    }

    return count;
}

static int copy_nth_token(const char *line, unsigned int target, char *out, size_t out_len)
{
    unsigned int count = 0u;
    size_t pos = 0u;

    if (out_len == 0u) {
        return -1;
    }

    while (line[pos] != '\0') {
        size_t out_pos = 0u;

        while (line[pos] != '\0' && isspace((unsigned char)line[pos])) {
            pos++;
        }
        if (line[pos] == '\0') {
            break;
        }
        if (count == target) {
            while (line[pos] != '\0' && !isspace((unsigned char)line[pos])) {
                if (out_pos + 1u < out_len) {
                    out[out_pos++] = line[pos];
                }
                pos++;
            }
            out[out_pos] = '\0';
            return 0;
        }
        count++;
        while (line[pos] != '\0' && !isspace((unsigned char)line[pos])) {
            pos++;
        }
    }

    out[0] = '\0';
    return -1;
}

static const char *const *completion_words_for_line(const char *line, size_t len, size_t *count)
{
    int start = current_token_start(line, len);
    unsigned int tokens_before = count_tokens_before(line, start);
    char first[64];

    if (tokens_before == 0u) {
        if (custom_command_words != NULL) {
            *count = custom_command_word_count;
            return custom_command_words;
        }
        *count = sizeof(command_words) / sizeof(command_words[0]);
        return command_words;
    }
    if (copy_nth_token(line, 0u, first, sizeof(first)) != 0) {
        *count = 0u;
        return NULL;
    }
    if (strcmp(first, "mode") == 0 && tokens_before == 1u) {
        *count = sizeof(mode_words) / sizeof(mode_words[0]);
        return mode_words;
    }
    if (strcmp(first, "auto-cycle") == 0 && tokens_before == 1u) {
        *count = sizeof(on_off_words) / sizeof(on_off_words[0]);
        return on_off_words;
    }
    if (strcmp(first, "power") == 0 && tokens_before == 1u) {
        *count = sizeof(power_words) / sizeof(power_words[0]);
        return power_words;
    }
    if (custom_command_words != NULL &&
        strcmp(first, "can_status") == 0 && tokens_before == 1u) {
        *count = sizeof(reset_words) / sizeof(reset_words[0]);
        return reset_words;
    }
    if (custom_command_words != NULL &&
        (strcmp(first, "language") == 0 || strcmp(first, "lang") == 0) &&
        tokens_before == 1u) {
        *count = sizeof(language_words) / sizeof(language_words[0]);
        return language_words;
    }
    if ((strcmp(first, "enter") == 0 ||
         strcmp(first, "flash") == 0 ||
         strcmp(first, "flash-one") == 0 ||
         strcmp(first, "flash-bus") == 0 ||
         strcmp(first, "flash-all") == 0 ||
         (custom_command_words != NULL && strcmp(first, "flash_single") == 0) ||
         (custom_command_words != NULL && strcmp(first, "flash_all") == 0) ||
         strcmp(first, "flash-auto") == 0) &&
        tokens_before == 1u) {
        *count = sizeof(cycle_words) / sizeof(cycle_words[0]);
        return cycle_words;
    }
    if (strcmp(first, "listen") == 0 && tokens_before == 2u) {
        *count = sizeof(listen_words) / sizeof(listen_words[0]);
        return listen_words;
    }

    *count = 0u;
    return NULL;
}

static void redraw_prompt_cursor(const char *prompt, const char *line, size_t len, size_t cursor)
{
    printf("\r\033[2K%s%s", prompt, line);
    if (cursor < len) {
        printf("\033[%zuD", len - cursor);
    }
    fflush(stdout);
}

static int line_has_text(const char *line)
{
    size_t i;

    for (i = 0u; line[i] != '\0'; i++) {
        if (!isspace((unsigned char)line[i])) {
            return 1;
        }
    }
    return 0;
}

static void history_push(const char *line)
{
    if (!line_has_text(line)) {
        return;
    }
    if (command_history_count > 0u &&
        strcmp(command_history[command_history_count - 1u], line) == 0) {
        return;
    }
    if (command_history_count == HISTORY_DEPTH) {
        memmove(command_history,
                command_history + 1,
                sizeof(command_history[0]) * (HISTORY_DEPTH - 1u));
        command_history_count--;
    }
    snprintf(command_history[command_history_count],
             sizeof(command_history[command_history_count]),
             "%s",
             line);
    command_history_count++;
}

static void copy_line(char *dst, size_t *dst_len, size_t max_len, const char *src)
{
    size_t len;

    if (max_len == 0u) {
        return;
    }
    len = strlen(src);
    if (len >= max_len) {
        len = max_len - 1u;
    }
    memcpy(dst, src, len);
    dst[len] = '\0';
    *dst_len = len;
}

static int insert_text_at_cursor(char *line,
                                 size_t *len,
                                 size_t *cursor,
                                 size_t max_len,
                                 const char *text,
                                 size_t text_len)
{
    if (*len + text_len >= max_len) {
        return -1;
    }
    memmove(&line[*cursor + text_len], &line[*cursor], *len - *cursor + 1u);
    memcpy(&line[*cursor], text, text_len);
    *cursor += text_len;
    *len += text_len;
    return 0;
}

static int delete_before_cursor(char *line, size_t *len, size_t *cursor)
{
    if (*cursor == 0u) {
        return -1;
    }
    memmove(&line[*cursor - 1u], &line[*cursor], *len - *cursor + 1u);
    (*cursor)--;
    (*len)--;
    return 0;
}

static int delete_at_cursor(char *line, size_t *len, size_t cursor)
{
    if (cursor >= *len) {
        return -1;
    }
    memmove(&line[cursor], &line[cursor + 1u], *len - cursor);
    (*len)--;
    return 0;
}

static size_t longest_common_prefix_len(const char *const *words, size_t count)
{
    size_t len;
    size_t i;

    if (count == 0u) {
        return 0u;
    }
    len = strlen(words[0]);
    for (i = 1u; i < count; i++) {
        size_t j = 0u;

        while (j < len && words[i][j] != '\0' && words[0][j] == words[i][j]) {
            j++;
        }
        len = j;
    }
    return len;
}

static void complete_line(char *line, size_t *len, size_t *cursor, size_t max_len, const char *prompt)
{
    const char *const *words;
    const char *matches[64];
    size_t word_count;
    size_t match_count = 0u;
    int token_start;
    const char *prefix;
    size_t prefix_len;
    size_t i;

    if (*cursor != *len) {
        putchar('\a');
        fflush(stdout);
        return;
    }

    token_start = current_token_start(line, *cursor);
    prefix = &line[token_start];
    prefix_len = *cursor - (size_t)token_start;
    words = completion_words_for_line(line, *len, &word_count);
    if (words == NULL || word_count == 0u) {
        putchar('\a');
        fflush(stdout);
        return;
    }

    for (i = 0u; i < word_count && match_count < sizeof(matches) / sizeof(matches[0]); i++) {
        if (starts_with(words[i], prefix)) {
            matches[match_count++] = words[i];
        }
    }
    if (match_count == 0u) {
        putchar('\a');
        fflush(stdout);
        return;
    }

    if (match_count == 1u) {
        const char *match = matches[0];
        size_t match_len = strlen(match);

        if (match_len > prefix_len &&
            insert_text_at_cursor(line,
                                  len,
                                  cursor,
                                  max_len,
                                  &match[prefix_len],
                                  match_len - prefix_len) != 0) {
            putchar('\a');
            fflush(stdout);
            return;
        }
        if (*len == 0u || line[*len - 1u] != ' ') {
            if (insert_text_at_cursor(line, len, cursor, max_len, " ", 1u) != 0) {
                putchar('\a');
                fflush(stdout);
                return;
            }
        }
        redraw_prompt_cursor(prompt, line, *len, *cursor);
        return;
    }

    {
        size_t common_len = longest_common_prefix_len(matches, match_count);

        if (common_len > prefix_len) {
            if (insert_text_at_cursor(line,
                                      len,
                                      cursor,
                                      max_len,
                                      &matches[0][prefix_len],
                                      common_len - prefix_len) != 0) {
                putchar('\a');
                fflush(stdout);
                return;
            }
            redraw_prompt_cursor(prompt, line, *len, *cursor);
            return;
        }
    }

    printf("\n");
    for (i = 0u; i < match_count; i++) {
        printf("%s%s", matches[i], (i + 1u == match_count) ? "\n" : "  ");
    }
    redraw_prompt_cursor(prompt, line, *len, *cursor);
}

static int read_line_with_completion(const char *prompt, char *line, size_t max_len)
{
    struct termios old_term;
    struct termios new_term;
    size_t len = 0u;
    size_t cursor = 0u;
    size_t history_pos = command_history_count;
    bool history_browsing = false;
    char saved_line[LINE_LEN];
    size_t saved_len = 0u;

    if (max_len == 0u) {
        return -1;
    }
    if (!isatty(STDIN_FILENO)) {
        if (fgets(line, (int)max_len, stdin) == NULL) {
            return -1;
        }
        return 0;
    }
    if (tcgetattr(STDIN_FILENO, &old_term) != 0) {
        return -1;
    }
    new_term = old_term;
    new_term.c_lflag &= (tcflag_t)~(ICANON | ECHO);
    new_term.c_cc[VMIN] = 1;
    new_term.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_term) != 0) {
        return -1;
    }

    line[0] = '\0';
    saved_line[0] = '\0';
    while (!stop_requested) {
        unsigned char ch;
        ssize_t n = read(STDIN_FILENO, &ch, 1u);

        if (n <= 0) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
            return -1;
        }
        if (ch == '\r' || ch == '\n') {
            putchar('\n');
            line[len] = '\0';
            history_push(line);
            tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
            return 0;
        }
        if (ch == 27u) {
            unsigned char seq[2];

            if (read(STDIN_FILENO, &seq[0], 1u) <= 0 ||
                read(STDIN_FILENO, &seq[1], 1u) <= 0 ||
                seq[0] != '[') {
                continue;
            }
            if (seq[1] == 'A') {
                if (command_history_count == 0u) {
                    putchar('\a');
                    fflush(stdout);
                    continue;
                }
                if (!history_browsing) {
                    copy_line(saved_line, &saved_len, sizeof(saved_line), line);
                    history_pos = command_history_count;
                    history_browsing = true;
                }
                if (history_pos == 0u) {
                    putchar('\a');
                    fflush(stdout);
                    continue;
                }
                history_pos--;
                copy_line(line, &len, max_len, command_history[history_pos]);
                cursor = len;
                redraw_prompt_cursor(prompt, line, len, cursor);
                continue;
            }
            if (seq[1] == 'B') {
                if (!history_browsing) {
                    putchar('\a');
                    fflush(stdout);
                    continue;
                }
                if (history_pos + 1u < command_history_count) {
                    history_pos++;
                    copy_line(line, &len, max_len, command_history[history_pos]);
                } else {
                    history_browsing = false;
                    history_pos = command_history_count;
                    copy_line(line, &len, max_len, saved_line);
                }
                cursor = len;
                redraw_prompt_cursor(prompt, line, len, cursor);
                continue;
            }
            if (seq[1] == 'C') {
                if (cursor < len) {
                    cursor++;
                    printf("\033[C");
                    fflush(stdout);
                } else {
                    putchar('\a');
                    fflush(stdout);
                }
                continue;
            }
            if (seq[1] == 'D') {
                if (cursor > 0u) {
                    cursor--;
                    printf("\033[D");
                    fflush(stdout);
                } else {
                    putchar('\a');
                    fflush(stdout);
                }
                continue;
            }
            if (seq[1] == '3') {
                unsigned char tail;

                if (read(STDIN_FILENO, &tail, 1u) <= 0 || tail != '~') {
                    continue;
                }
                if (delete_at_cursor(line, &len, cursor) != 0) {
                    putchar('\a');
                    fflush(stdout);
                    continue;
                }
                redraw_prompt_cursor(prompt, line, len, cursor);
                continue;
            }
            continue;
        }
        if (ch == '\t') {
            complete_line(line, &len, &cursor, max_len, prompt);
            history_browsing = false;
            continue;
        }
        if (ch == 4u) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
            return -1;
        }
        if (ch == 127u || ch == 8u) {
            if (delete_before_cursor(line, &len, &cursor) != 0) {
                putchar('\a');
                fflush(stdout);
                continue;
            }
            redraw_prompt_cursor(prompt, line, len, cursor);
            history_browsing = false;
            continue;
        }
        if (isprint(ch)) {
            if (insert_text_at_cursor(line, &len, &cursor, max_len, (const char *)&ch, 1u) != 0) {
                putchar('\a');
                fflush(stdout);
                continue;
            }
            redraw_prompt_cursor(prompt, line, len, cursor);
            history_browsing = false;
            continue;
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
    return -1;
}

static int run_command(flash_state *state, int argc, char **argv)
{
    const char *cmd;
    unsigned int value;
    bool enabled;

    if (argc <= 0) {
        return 0;
    }

    cmd = argv[0];
    if (strcmp(cmd, "q") == 0 || strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) {
        return 1;
    }
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        print_terminal_help();
        return 0;
    }
    if (strcmp(cmd, "config") == 0) {
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "mode") == 0) {
        if (argc < 2 ||
            (strcmp(argv[1], "flash") != 0 && strcmp(argv[1], "debug") != 0)) {
            printf("usage: mode flash|debug\n");
            return -1;
        }
        state->mode = strcmp(argv[1], "debug") == 0 ? TOOL_MODE_DEBUG : TOOL_MODE_FLASH;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "debug") == 0) {
        state->mode = TOOL_MODE_DEBUG;
        if (argc > 1) {
            return run_command(state, argc - 1, argv + 1);
        }
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "flash-mode") == 0) {
        state->mode = TOOL_MODE_FLASH;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "bus") == 0) {
        if (argc < 2 || parse_uint_arg(argv[1], &value) != 0 || value >= CANFD_DEVICE_NUM) {
            printf("usage: bus <0-%u>\n", CANFD_DEVICE_NUM - 1u);
            return -1;
        }
        state->bus = value;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "id") == 0 || strcmp(cmd, "motor") == 0) {
        if (argc < 2 || parse_uint_arg(argv[1], &value) != 0 || set_target_id(state, value) != 0) {
            printf("usage: id <1-8>\n");
            return -1;
        }
        printf("selected motor %u\n", value);
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "master-id") == 0 || strcmp(cmd, "master") == 0) {
        if (argc < 2 || parse_uint_arg(argv[1], &value) != 0 || value > CAN_SFF_MASK) {
            printf("usage: master-id <0-0x%x>\n", CAN_SFF_MASK);
            return -1;
        }
        state->master_id = value;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "canfd") == 0) {
        state->debug_use_canfd = true;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "classic") == 0) {
        state->debug_use_canfd = false;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "auto-cycle") == 0) {
        if (argc < 2 || parse_on_off(argv[1], &enabled) != 0) {
            printf("usage: auto-cycle on|off\n");
            return -1;
        }
        state->auto_power_cycle = enabled;
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "power") == 0) {
        if (argc < 2) {
            printf("usage: power on|off|cycle\n");
            return -1;
        }
        if (strcmp(argv[1], "cycle") == 0) {
            return power_cycle_motor();
        }
        if (parse_on_off(argv[1], &enabled) != 0) {
            printf("usage: power on|off|cycle\n");
            return -1;
        }
        if (motor_pwr_set(enabled ? 1u : 0u) < 0) {
            printf("motor_pwr_set(%u) failed\n", enabled ? 1u : 0u);
            return -1;
        }
        printf("motor power %s\n", enabled ? "on" : "off");
        if (enabled) {
            printf("waiting 2 seconds for motor power soft start\n");
            sleep_ms(2000u);
        }
        return 0;
    }
    if (strcmp(cmd, "ping") == 0) {
        return run_debug_ping(state, argc - 1, argv + 1);
    }
    if (strcmp(cmd, "enable") == 0) {
        return run_debug_special(state, BXI_MOTOR_CMD_ENABLE, argc - 1, argv + 1);
    }
    if (strcmp(cmd, "disable") == 0) {
        return run_debug_special(state, BXI_MOTOR_CMD_DISABLE, argc - 1, argv + 1);
    }
    if (strcmp(cmd, "zero") == 0) {
        return run_debug_special(state, BXI_MOTOR_CMD_ZERO, argc - 1, argv + 1);
    }
    if (strcmp(cmd, "mit") == 0) {
        return run_debug_mit(state, argc - 1, argv + 1);
    }
    if (strcmp(cmd, "reg-read") == 0) {
        return run_debug_reg_read(state, argc - 1, argv + 1);
    }
    if (strcmp(cmd, "reg-write") == 0) {
        return run_debug_reg_write(state, argc - 1, argv + 1);
    }
    if (strcmp(cmd, "reg-save") == 0) {
        return run_debug_reg_save(state, argc - 1, argv + 1);
    }
    if (strcmp(cmd, "enter") == 0) {
        bool cycle = state->auto_power_cycle || (argc > 1 && strcmp(argv[1], "cycle") == 0);

        return enter_boot_menu(state, cycle);
    }
    if (strcmp(cmd, "version") == 0) {
        rx_ring_clear(&state->rx);
        if (send_boot_byte(state, 'v') != 0) {
            return -1;
        }
        collect_text(state, 250u, 1500u);
        return 0;
    }
    if (strcmp(cmd, "set-id") == 0) {
        uint8_t ch;

        if (argc < 2 || parse_uint_arg(argv[1], &value) != 0 || value < 1u || value > 8u) {
            printf("usage: set-id <1-8>\n");
            return -1;
        }
        rx_ring_clear(&state->rx);
        if (send_boot_byte(state, 's') != 0) {
            return -1;
        }
        wait_for_pattern(state, "Please enter the id", 1000u, true);
        ch = (uint8_t)('0' + value);
        if (send_boot_byte(state, ch) != 0) {
            return -1;
        }
        collect_text(state, 250u, 1500u);
        state->boot_id = value;
        update_boot_ids(state);
        print_config(state);
        return 0;
    }
    if (strcmp(cmd, "flash") == 0) {
        bool cycle = state->auto_power_cycle || (argc > 2 && strcmp(argv[2], "cycle") == 0);

        if (argc < 2) {
            printf("usage: flash <firmware.bin> [cycle]\n");
            return -1;
        }
        return boot_flash_file(state, argv[1], cycle);
    }
    if (strcmp(cmd, "flash-auto") == 0) {
        return run_flash_auto(state, argc - 1, argv + 1);
    }
    if (strcmp(cmd, "flash-one") == 0 ||
        strcmp(cmd, "flash-bus") == 0 ||
        strcmp(cmd, "flash-all") == 0) {
        char *flash_argv[4];
        const char *selection = strcmp(cmd, "flash-one") == 0 ? "one" :
                                (strcmp(cmd, "flash-bus") == 0 ? "bus" : "all");
        int flash_argc = 0;
        int i;

        flash_argv[flash_argc++] = (char *)selection;
        for (i = 1; i < argc && flash_argc < (int)(sizeof(flash_argv) / sizeof(flash_argv[0])); i++) {
            flash_argv[flash_argc++] = argv[i];
        }
        if (i < argc) {
            printf("usage: %s [map.yaml] [cycle]\n", cmd);
            return -1;
        }
        return run_flash_auto(state, flash_argc, flash_argv);
    }
    if (strcmp(cmd, "jump") == 0) {
        rx_ring_clear(&state->rx);
        if (send_boot_byte(state, 'a') != 0) {
            return -1;
        }
        collect_text(state, 250u, 1000u);
        return 0;
    }
    if (strcmp(cmd, "listen") == 0) {
        unsigned int seconds = 5u;
        uint64_t deadline;

        if (state->mode == TOOL_MODE_DEBUG) {
            return run_debug_listen(state, argc - 1, argv + 1);
        }
        if (argc > 1 && parse_uint_arg(argv[1], &seconds) != 0) {
            printf("usage: listen [seconds]\n");
            return -1;
        }
        printf("listening for %u second(s), Ctrl-C to stop\n", seconds);
        deadline = time_us() + (uint64_t)seconds * 1000000ULL;
        while (!stop_requested && time_us() < deadline) {
            uint8_t byte;

            if (!rx_ring_pop(&state->rx, &byte, 50u)) {
                continue;
            }
            if (byte == '\r') {
                continue;
            }
            if (byte == '\n' || byte == '\t' || byte >= 0x20u) {
                fwrite(&byte, 1u, 1u, stdout);
            } else {
                printf("\\x%02x", byte);
            }
            fflush(stdout);
        }
        printf("\n");
        return 0;
    }
    if (strcmp(cmd, "send") == 0) {
        int i;

        if (argc < 2) {
            printf("usage: send <text>\n");
            return -1;
        }
        for (i = 1; i < argc; i++) {
            if (i > 1) {
                uint8_t space = ' ';
                if (send_boot_data(state, &space, 1u) != 0) {
                    return -1;
                }
            }
            if (send_boot_data(state, (const uint8_t *)argv[i], strlen(argv[i])) != 0) {
                return -1;
            }
        }
        return 0;
    }

    printf("unknown command: %s\n", cmd);
    printf("type help for commands, q to exit\n");
    return -1;
}

static int run_terminal(flash_state *state)
{
    char line[LINE_LEN];
    char *argv[16];
    char prompt[64];

    printf("Entering motor console runtime terminal. No motor power, reset, or flash action has been performed.\n");
    printf("Type help for commands, Tab to complete, arrows to edit/history.\n");
    print_config(state);
    while (!stop_requested) {
        int argc;
        int ret;

        snprintf(prompt,
                 sizeof(prompt),
                 "%s[%u:%u]> ",
                 state->mode == TOOL_MODE_DEBUG ? "debug" : "flash",
                 state->bus,
                 state->boot_id);
        printf("%s", prompt);
        fflush(stdout);
        if (read_line_with_completion(prompt, line, sizeof(line)) != 0) {
            printf("\n");
            break;
        }
        argc = split_line(line, argv, (int)(sizeof(argv) / sizeof(argv[0])));
        ret = run_command(state, argc, argv);
        if (ret > 0) {
            break;
        }
    }

    return 0;
}

int main(int argc, char **argv)
{
    enum {
        OPT_POWER_CYCLE = 1000,
        OPT_QUIET_TX,
        OPT_POWER,
        OPT_KEEP_POWER,
    };
    static const struct option long_options[] = {
        {"bus", required_argument, NULL, 'b'},
        {"id", required_argument, NULL, 'i'},
        {"power-cycle", no_argument, NULL, OPT_POWER_CYCLE},
        {"quiet-tx", no_argument, NULL, OPT_QUIET_TX},
        {"power", no_argument, NULL, OPT_POWER},
        {"keep-power", no_argument, NULL, OPT_KEEP_POWER},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0},
    };
    flash_state state;
    int opt;
    int ret;
    bool power_requested = false;
    bool keep_power = false;

    memset(&state, 0, sizeof(state));
    state.bus = 0u;
    state.boot_id = 1u;
    state.mode = TOOL_MODE_FLASH;
    state.debug_use_canfd = true;
    state.limits = bxi_motor_default_limits;
    update_boot_ids(&state);
    rx_ring_init(&state.rx);
    frame_ring_init(&state.frames);

    while ((opt = getopt_long(argc, argv, "+b:i:h", long_options, NULL)) != -1) {
        switch (opt) {
        case 'b':
            if (parse_uint_arg(optarg, &state.bus) != 0 || state.bus >= CANFD_DEVICE_NUM) {
                fprintf(stderr, "invalid bus: %s\n", optarg);
                return 1;
            }
            break;
        case 'i':
            if (parse_uint_arg(optarg, &state.boot_id) != 0 ||
                state.boot_id < 1u ||
                state.boot_id > 8u) {
                fprintf(stderr, "id must be in [1,8]: %s\n", optarg);
                return 1;
            }
            update_boot_ids(&state);
            break;
        case OPT_POWER_CYCLE:
            state.auto_power_cycle = true;
            break;
        case OPT_QUIET_TX:
            state.quiet_tx = true;
            break;
        case OPT_POWER:
            power_requested = true;
            break;
        case OPT_KEEP_POWER:
            keep_power = true;
            break;
        case 'h':
            print_usage(argv[0]);
            return 0;
        default:
            print_usage(argv[0]);
            return 1;
        }
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (bxi_pci_init(can_rx_callback, &state, -1) == -1) {
        fprintf(stderr, "bxi_pci_init failed\n");
        return 1;
    }
    state.pci_started = true;

    if (power_requested) {
        if (motor_pwr_set(1u) < 0) {
            fprintf(stderr, "motor_pwr_set(1) failed\n");
            if (state.pci_started) {
                bxi_pci_exit();
                state.pci_started = false;
            }
            return 1;
        }
        printf("motor power on; waiting 2 seconds for soft start\n");
        sleep_ms(2000u);
    }

    if (optind >= argc) {
        ret = run_terminal(&state);
    } else {
        ret = run_command(&state, argc - optind, &argv[optind]);
        if (ret > 0) {
            ret = 0;
        }
    }

    if (power_requested && !keep_power) {
        motor_pwr_set(0u);
        printf("motor power off\n");
    }
    if (state.pci_started) {
        bxi_pci_exit();
        state.pci_started = false;
    }

    return ret == 0 ? 0 : 1;
}
