#define MOTOR_CONSOLE_TEST
#include "../src/main.c"
#include <assert.h>

static flash_state fixture;

static void send_boot_text(const char *line)
{
    size_t offset = 0u;
    size_t length = strlen(line);

    while (offset < length) {
        canfd_packet packet = {0};
        size_t chunk = length - offset;

        if (chunk > 8u) chunk = 8u;
        packet.bus = 0u;
        packet.frame.can_id = 0x7f1u;
        packet.frame.len = (uint8_t)chunk;
        memcpy(packet.frame.data, line + offset, chunk);
        assert(can_rx_callback(&fixture, &packet) == 0);
        offset += chunk;
    }
}

int main(void)
{
    static const struct {
        const char *type;
        const char *firmware_v0_2;
        const char *firmware_v1_4;
    } firmware_cases[] = {
        {"50", "bxi_motor_50_v0.2.2.bin", "bxi_motor_50_v1.4.3.bin"},
        {"50L", "bxi_motor_50L_v0.2.2.bin", "bxi_motor_50L_v1.4.3.bin"},
        {"70", "bxi_motor_70_v0.2.2.bin", "bxi_motor_70_v1.4.3.bin"},
        {"85", "bxi_motor_85_v0.2.2.bin", "bxi_motor_85_v1.4.3.bin"},
    };
    char filename[FIRMWARE_NAME_MAX];
    char path[PATH_LEN];
    char expected_path[PATH_LEN];
    char source[16];
    motor_runtime *runtime = &fixture.motors[0];
    size_t case_index;
    unsigned int major;
    unsigned int minor;

    frame_ring_init(&fixture.frames);
    fixture.config_loaded = true;
    assert(load_motor_map_config("config/motor_console.yaml", &fixture.config) == 0);
    fixture.config.entry_count = 1u;
    fixture.config.entries[0].bus = 0u;
    fixture.config.entries[0].id = 1u;

    fixture.show_can_output = false;
    send_boot_text("VER   : code=1.4.27 config=1.2\r\n");
    assert(runtime->fw_version_from_boot);
    assert(runtime->fw_version_major == 1u);
    assert(runtime->fw_version_minor == 4u);
    assert(runtime->fw_version_patch == 27u);
    assert(runtime->config_version == 0x0102u);
    for (case_index = 0u;
         case_index < sizeof(firmware_cases) / sizeof(firmware_cases[0]);
         case_index++) {
        assert(console_firmware_name_for_slot(&fixture, firmware_cases[case_index].type,
                                              0u, filename, sizeof(filename),
                                              source, sizeof(source)) == 0);
        assert(strcmp(filename, firmware_cases[case_index].firmware_v1_4) == 0);
        assert(strcmp(source, "v1.4") == 0);
        assert(console_resolve_firmware_path(&fixture, filename,
                                             path, sizeof(path)) == 0);
        snprintf(expected_path, sizeof(expected_path), "firmware/%s",
                 firmware_cases[case_index].firmware_v1_4);
        assert(strcmp(path, expected_path) == 0);
    }

    send_boot_text("VER : code=1.4.0 config=1.4\n");
    assert(runtime->fw_version_patch == 0u);
    assert(console_firmware_name_for_slot(&fixture, "50L", 0u, filename,
                                          sizeof(filename), source, sizeof(source)) == 0);
    assert(strcmp(filename, "bxi_motor_50L_v1.4.3.bin") == 0);

    runtime->fw_version_from_boot = false;
    runtime->config_version = 0x0002u;
    for (case_index = 0u;
         case_index < sizeof(firmware_cases) / sizeof(firmware_cases[0]);
         case_index++) {
        assert(console_firmware_name_for_slot(&fixture, firmware_cases[case_index].type,
                                              0u, filename, sizeof(filename),
                                              source, sizeof(source)) == 0);
        assert(strcmp(filename, firmware_cases[case_index].firmware_v0_2) == 0);
        assert(strcmp(source, "cfg-v0.2?") == 0);
        assert(console_resolve_firmware_path(&fixture, filename,
                                             path, sizeof(path)) == 0);
    }
    runtime->config_version = 0x0003u;
    assert(console_firmware_name_for_slot(&fixture, "50L", 0u, filename,
                                          sizeof(filename), source, sizeof(source)) != 0);
    runtime->fw_version_valid = false;
    runtime->config_version = 0x0002u;
    assert(console_firmware_name_for_slot(&fixture, "50L", 0u, filename,
                                          sizeof(filename), source, sizeof(source)) == 0);
    assert(strcmp(filename, "bxi_motor_50L_v0.2.2.bin") == 0);
    runtime->config_version_valid = false;
    assert(console_firmware_name_for_slot(&fixture, "50L", 0u, filename,
                                          sizeof(filename), source, sizeof(source)) != 0);
    runtime->fw_version_from_boot = true;
    runtime->fw_version_valid = true;
    runtime->config_version_valid = true;
    assert(console_firmware_name_for_slot(&fixture, "50_hand", 0u, filename,
                                          sizeof(filename), source, sizeof(source)) != 0);
    assert(console_firmware_name_for_slot(&fixture, "50L_hand", 0u, filename,
                                          sizeof(filename), source, sizeof(source)) != 0);

    send_boot_text("VER : code=1.3.9 config=1.4\n");
    assert(runtime->fw_version_patch == 9u);
    assert(console_firmware_name_for_slot(&fixture, "50L", 0u, filename,
                                          sizeof(filename), source, sizeof(source)) != 0);

    send_boot_text("VER : code=0.2.19 config=0.1\n");
    assert(runtime->fw_version_major == 0u);
    assert(runtime->fw_version_minor == 2u);
    assert(runtime->fw_version_patch == 19u);
    for (case_index = 0u;
         case_index < sizeof(firmware_cases) / sizeof(firmware_cases[0]);
         case_index++) {
        assert(console_firmware_name_for_slot(&fixture, firmware_cases[case_index].type,
                                              0u, filename, sizeof(filename),
                                              source, sizeof(source)) == 0);
        assert(strcmp(filename, firmware_cases[case_index].firmware_v0_2) == 0);
        assert(strcmp(source, "v0.2") == 0);
        assert(console_resolve_firmware_path(&fixture, filename,
                                             path, sizeof(path)) == 0);
        snprintf(expected_path, sizeof(expected_path), "firmware/%s",
                 firmware_cases[case_index].firmware_v0_2);
        assert(strcmp(path, expected_path) == 0);
    }
    assert(console_firmware_name_for_slot(&fixture, "50_hand", 0u, filename,
                                          sizeof(filename), source, sizeof(source)) != 0);

    send_boot_text("VER : code=0.1.8 config=0.1\n");
    assert(console_firmware_name_for_slot(&fixture, "70", 0u, filename,
                                          sizeof(filename), source, sizeof(source)) == 0);
    assert(strcmp(filename, "bxi_motor_70_v0.1.2.bin") == 0);
    send_boot_text("VER : code=1.1.8 config=1.1\n");
    assert(console_firmware_name_for_slot(&fixture, "70", 0u, filename,
                                          sizeof(filename), source, sizeof(source)) == 0);
    assert(strcmp(filename, "bxi_motor_70_v1.1.2_new.bin") == 0);
    send_boot_text("VER : code=0.3.8 config=0.2\n");
    assert(console_firmware_name_for_slot(&fixture, "70", 0u, filename,
                                          sizeof(filename), source, sizeof(source)) != 0);

    runtime->fw_version_valid = false;
    runtime->config_version_valid = false;
    assert(console_firmware_name_for_slot(&fixture, "70", 0u, filename,
                                          sizeof(filename), source, sizeof(source)) != 0);
    assert(parse_firmware_series_key("firmware_v2_7", &major, &minor) == 0);
    assert(major == 2u && minor == 7u);
    assert(parse_firmware_series_key("firmware_v2_7_extra", &major, &minor) != 0);

    puts("PASS: boot/config version parsing and major.minor firmware selection");
    return 0;
}
