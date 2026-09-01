/*
 * Unified interactive motor service console.
 *
 * Reuse the internal terminal editor, motor protocol, boot-menu and YMODEM
 * implementation from lib/runtime.c.
 */
#define main runtime_embedded_main
#include "lib/runtime.c"
#undef main

enum {
    FLASH_PLAN_MAX_TARGETS = 128u,
};

#define DEFAULT_FLASH_PLAN "config/flash_plan_default.yaml"

typedef struct
{
    bool has_index;
    bool has_bus;
    bool has_id;
    unsigned int index;
    unsigned int bus;
    unsigned int id;
    unsigned int line_no;
    char version[PATH_LEN];
    char name[MOTOR_NAME_LEN];
} flash_plan_target;

typedef struct
{
    char firmware_dir[PATH_LEN];
    char firmware_base_url[PATH_LEN];
    char firmware_cache_dir[PATH_LEN];
    flash_plan_target targets[FLASH_PLAN_MAX_TARGETS];
    size_t target_count;
} flash_plan_config;

static const char *const console_command_words[] = {
    "help", "-h", "?", "power_on", "power_off",
    "mit_set", "maita_mit",
    "maita_power_on", "maita_power_off", "maita_info", "maita_enable", "maita_disable",
    "maita_torque", "maita_zero", "maita_pos",
    "can_status",
    "language", "lang", "quit", "exit", "q", "qq",
};

#include "lib/core.c"
#include "lib/display.c"
#include "lib/control.c"
#include "lib/maita.c"
#include "lib/flash.c"
#include "lib/debug.c"
#include "lib/terminal.c"

int main(int argc, char **argv)
{
    char resolved_config_path[PATH_LEN];
    const char *config_path = NULL;
    flash_state state;
    int opt;
    int ret;
    bool check_config = false;
    bool show_help = false;
    bool config_explicit = false;
    const char *language_override = NULL;

    static const struct option options[] = {
        {"config", required_argument, NULL, 'c'},
        {"check-config", no_argument, NULL, 'C'},
        {"language", required_argument, NULL, 'l'},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0},
    };

    while ((opt = getopt_long(argc, argv, "c:Cl:h", options, NULL)) != -1) {
        if (opt == 'c') {
            config_path = optarg;
            config_explicit = true;
        } else if (opt == 'C') {
            check_config = true;
        } else if (opt == 'l') {
            if (strcmp(optarg, "zh") != 0 && strcmp(optarg, "en") != 0) {
                printf("用法：--language zh|en\n");
                return 1;
            }
            language_override = optarg;
        } else if (opt == 'h') {
            show_help = true;
        } else {
            printf("用法：%s [-c config.yaml] [--language zh|en] [--check-config]\n", argv[0]);
            return 1;
        }
    }
    if (show_help) {
        bool verbose_help = optind < argc && strcmp(argv[optind], "all") == 0;

        console_print_help(language_override != NULL && strcmp(language_override, "zh") == 0,
                           verbose_help);
        return 0;
    }
    if (optind != argc) {
        printf("用法：%s [-c config.yaml] [--language zh|en] [--check-config]\n", argv[0]);
        return 1;
    }
    if (!config_explicit) {
        config_path = console_default_config_path(argv[0], resolved_config_path,
                                                  sizeof(resolved_config_path));
    }

    memset(&state, 0, sizeof(state));
    state.bus = 0u;
    state.boot_id = 1u;
    state.mode = TOOL_MODE_FLASH;
    state.debug_use_canfd = true;
    state.quiet_tx = true;
    state.show_can_output = true;
    state.show_motor_input = true;
    state.limits = bxi_motor_default_limits;
    custom_command_words = console_command_words;
    custom_command_word_count = sizeof(console_command_words) /
                                sizeof(console_command_words[0]);
    update_boot_ids(&state);
    rx_ring_init(&state.rx);
    frame_ring_init(&state.frames);
    if (language_override != NULL) {
        state.language_override_active = true;
        state.chinese_override = strcmp(language_override, "zh") == 0;
    }
    if (console_reload_config(&state, config_path) != 0) {
        return 1;
    }
    if (check_config) {
        console_print_config(&state);
        console_print_motors(&state);
        printf("%s\n", console_text(&state,
               "配置检查通过，未初始化 PCI/CAN，也未操作电机",
               "configuration check passed; no PCI/CAN initialization performed"));
        return 0;
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    if (bxi_pci_init(can_rx_callback, &state, -1) == -1) {
        fprintf(stderr, "%s\n", console_text(&state,
                "PCI/CAN 初始化失败：bxi_pci_init",
                "bxi_pci_init failed"));
        return 1;
    }
    state.pci_started = true;
    ret = console_terminal(&state);
    if (state.motor_power_on) {
        fprintf(stderr, "%s\n", console_text(&state,
                "警告：终端停止时电机仍处于上电状态，正在执行安全下电",
                "warning: console stopped while motor power state is ON; powering off"));
        console_power_off(&state);
    }
    if (state.pci_started) {
        bxi_pci_exit();
        state.pci_started = false;
    }
    return ret == 0 ? 0 : 1;
}
