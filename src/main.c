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
#define DEFAULT_KAIXUAN_ESI "assets/ethercat/KaiserDrive_KDE_ECAT_V1.2.xml"

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
    "help", "-h", "?", "power_on", "power_off", "motor_probe", "motor_scan", "motor_list",
    "ethercat_scan", "ethercat_enable", "ethercat_disable", "ethercat_position", "ethercat_zero",
    "ethercat_info",
    "mit_zero_set_all", "mit_zero_set_single", "mit_enable_all", "mit_disable_all",
    "mit_enable_single", "mit_disable_single", "mit_set", "stand_up",
    "enable", "disable", "debug", "mit",
    "reg_read", "reg_write", "reg_save", "reg_info",
    "flash_single", "flash_all", "flash_debug",
    "motor_dbg", "can_dbg",
    "can_status",
    "language", "lang", "quit", "exit", "q", "qq",
};

#include "lib/core.c"
#include "lib/ethercat_esi.c"
#include "lib/ethercat_scan.c"
#include "lib/display.c"
#include "lib/control.c"
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
    const char *ethercat_interface = NULL;
    const char *ethercat_enable_interface = NULL;
    const char *ethercat_enable_selection = NULL;
    const char *ethercat_disable_interface = NULL;
    const char *ethercat_disable_selection = NULL;
    const char *ethercat_position_interface = NULL;
    const char *ethercat_position_selection = NULL;
    const char *ethercat_zero_interface = NULL;
    const char *ethercat_zero_selection = NULL;
    const char *ethercat_info_interface = NULL;
    const char *ethercat_info_selection = NULL;
    unsigned int ethercat_enable_hold_ms = 0u;
    unsigned int ethercat_position_hold_ms = 0u;
    double ethercat_position_rad;
    unsigned int parsed_ethercat_slave_id;
    bool parsed_ethercat_all_slaves;

    static const struct option options[] = {
        {"config", required_argument, NULL, 'c'},
        {"check-config", no_argument, NULL, 'C'},
        {"language", required_argument, NULL, 'l'},
        {"ethercat-scan", required_argument, NULL, 'E'},
        {"ethercat-enable", required_argument, NULL, 'M'},
        {"ethercat-disable", required_argument, NULL, 'D'},
        {"ethercat-position", required_argument, NULL, 'P'},
        {"ethercat-zero", required_argument, NULL, 'Z'},
        {"ethercat-info", required_argument, NULL, 'I'},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0},
    };

    while ((opt = getopt_long(argc, argv, "c:Cl:hE:M:D:P:Z:I:", options, NULL)) != -1) {
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
        } else if (opt == 'E') {
            ethercat_interface = optarg;
        } else if (opt == 'M') {
            ethercat_enable_interface = optarg;
        } else if (opt == 'D') {
            ethercat_disable_interface = optarg;
        } else if (opt == 'P') {
            ethercat_position_interface = optarg;
        } else if (opt == 'Z') {
            ethercat_zero_interface = optarg;
        } else if (opt == 'I') {
            ethercat_info_interface = optarg;
        } else {
            printf("用法：%s [-c config.yaml] [--language zh|en] [--check-config] [--ethercat-scan interface] [--ethercat-enable interface slave_id|all [hold_ms]] [--ethercat-disable interface slave_id|all] [--ethercat-position interface slave_id|all target_rad [hold_ms]]\n", argv[0]);
            return 1;
        }
    }
    if (show_help) {
        bool verbose_help = optind < argc && strcmp(argv[optind], "all") == 0;

        console_print_help(language_override != NULL && strcmp(language_override, "zh") == 0,
                           verbose_help);
        return 0;
    }
    if (ethercat_enable_interface != NULL) {
        if (ethercat_interface != NULL || ethercat_disable_interface != NULL ||
            ethercat_position_interface != NULL || ethercat_zero_interface != NULL ||
            ethercat_info_interface != NULL ||
            optind >= argc || optind + 2 < argc ||
            console_ethercat_parse_slave_selection(argv[optind],
                                                    &parsed_ethercat_slave_id,
                                                    &parsed_ethercat_all_slaves) != 0) {
            printf("用法：%s --ethercat-enable <interface> <slave_id|all> [hold_ms]\n", argv[0]);
            return 1;
        }
        ethercat_enable_selection = argv[optind++];
        if (optind < argc && parse_uint_arg(argv[optind++], &ethercat_enable_hold_ms) != 0) {
            printf("用法：%s --ethercat-enable <interface> <slave_id|all> [hold_ms]\n", argv[0]);
            return 1;
        }
    } else if (ethercat_disable_interface != NULL) {
        if (ethercat_interface != NULL || ethercat_position_interface != NULL ||
            ethercat_zero_interface != NULL || ethercat_info_interface != NULL ||
            optind >= argc || optind + 1 != argc ||
            console_ethercat_parse_slave_selection(argv[optind],
                                                    &parsed_ethercat_slave_id,
                                                    &parsed_ethercat_all_slaves) != 0) {
            printf("用法：%s --ethercat-disable <interface> <slave_id|all>\n", argv[0]);
            return 1;
        }
        ethercat_disable_selection = argv[optind++];
    } else if (ethercat_position_interface != NULL) {
        if (ethercat_interface != NULL || ethercat_zero_interface != NULL ||
            ethercat_info_interface != NULL ||
            optind + 2 > argc || optind + 3 < argc ||
            console_ethercat_parse_slave_selection(argv[optind],
                                                    &parsed_ethercat_slave_id,
                                                    &parsed_ethercat_all_slaves) != 0) {
            printf("用法：%s --ethercat-position <interface> <slave_id|all> <target_rad> [hold_ms]\n", argv[0]);
            return 1;
        }
        ethercat_position_selection = argv[optind++];
        if (console_ethercat_parse_position_rad(argv[optind++], &ethercat_position_rad) != 0 ||
            (optind < argc && parse_uint_arg(argv[optind++], &ethercat_position_hold_ms) != 0)) {
            printf("用法：%s --ethercat-position <interface> <slave_id|all> <target_rad> [hold_ms]\n", argv[0]);
            return 1;
        }
    } else if (ethercat_zero_interface != NULL) {
        if (ethercat_interface != NULL || ethercat_info_interface != NULL ||
            optind + 1 != argc ||
            console_ethercat_parse_slave_selection(argv[optind],
                                                    &parsed_ethercat_slave_id,
                                                    &parsed_ethercat_all_slaves) != 0) {
            printf("用法：%s --ethercat-zero <interface> <slave_id|all>\n", argv[0]);
            return 1;
        }
        ethercat_zero_selection = argv[optind++];
    } else if (ethercat_info_interface != NULL) {
        if (ethercat_interface != NULL || optind + 1 != argc ||
            console_ethercat_parse_slave_selection(argv[optind],
                                                    &parsed_ethercat_slave_id,
                                                    &parsed_ethercat_all_slaves) != 0) {
            printf("用法：%s --ethercat-info <interface> <slave_id|all>\n", argv[0]);
            return 1;
        }
        ethercat_info_selection = argv[optind++];
    }
    if (optind != argc) {
        printf("用法：%s [-c config.yaml] [--language zh|en] [--check-config] [--ethercat-scan interface]\n", argv[0]);
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
    set_completion_words_hook(console_ethercat_completion_words);
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
    console_load_kaixuan_esi(&state, argv[0]);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    if (ethercat_interface != NULL) {
        return console_ethercat_scan(state.config.chinese_ui, ethercat_interface) == 0 ? 0 : 1;
    }
    if (ethercat_enable_interface != NULL) {
        return console_ethercat_enable(state.config.chinese_ui,
                                       ethercat_enable_interface,
                                       ethercat_enable_selection,
                                       ethercat_enable_hold_ms) == 0 ? 0 : 1;
    }
    if (ethercat_disable_interface != NULL) {
        return console_ethercat_disable(state.config.chinese_ui,
                                        ethercat_disable_interface,
                                        ethercat_disable_selection) == 0 ? 0 : 1;
    }
    if (ethercat_position_interface != NULL) {
        return console_ethercat_position(state.config.chinese_ui,
                                         ethercat_position_interface,
                                         ethercat_position_selection,
                                         ethercat_position_rad,
                                         ethercat_position_hold_ms) == 0 ? 0 : 1;
    }
    if (ethercat_zero_interface != NULL) {
        return console_ethercat_zero(state.config.chinese_ui,
                                    ethercat_zero_interface,
                                    ethercat_zero_selection) == 0 ? 0 : 1;
    }
    if (ethercat_info_interface != NULL) {
        return console_ethercat_info(state.config.chinese_ui,
                                     ethercat_info_interface,
                                     ethercat_info_selection) == 0 ? 0 : 1;
    }
    if (check_config) {
        console_print_config(&state);
        console_print_motors(&state);
        printf("%s\n", console_text(&state,
               "配置检查通过，未初始化 PCI/CAN，也未操作电机",
               "configuration check passed; no PCI/CAN initialization performed"));
        return 0;
    }
    console_prefetch_firmware(&state);

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
