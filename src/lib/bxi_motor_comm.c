#include "bxi_motor_comm.h"

const bxi_motor_limits bxi_motor_default_limits = {
    -12.5f, 12.5f,
    -45.0f, 45.0f,
    0.0f, 500.0f,
    0.0f, 5.0f,
    -18.0f, 18.0f,
    -30.0f, 150.0f,
};

enum {
    BXI_MOTOR_AUX_INVALID_PAYLOAD = 0x0fffu,
    BXI_MOTOR_AUX_NTC1 = 0x0u,
    BXI_MOTOR_AUX_NTC2 = 0x1u,
    BXI_MOTOR_AUX_WINDING_TEMP = 0x2u,
    BXI_MOTOR_AUX_VBUS = 0x3u,
    BXI_MOTOR_AUX_IBUS = 0x4u,
    BXI_MOTOR_AUX_IQ = 0x5u,
    BXI_MOTOR_AUX_ID = 0x6u,
    BXI_MOTOR_AUX_FSM_STATUS = 0x7u,
    BXI_MOTOR_AUX_THERMAL_COEFF = 0x8u,
    BXI_MOTOR_AUX_VOLTAGE_UTIL = 0x9u,
    BXI_MOTOR_AUX_HEARTBEAT = 0x0fu,
};

const char *bxi_motor_aux_name(uint8_t aux_id)
{
    switch (aux_id) {
    case BXI_MOTOR_AUX_NTC1:
        return "ntc1";
    case BXI_MOTOR_AUX_NTC2:
        return "ntc2";
    case BXI_MOTOR_AUX_WINDING_TEMP:
        return "winding_temp";
    case BXI_MOTOR_AUX_VBUS:
        return "vbus";
    case BXI_MOTOR_AUX_IBUS:
        return "i_bus";
    case BXI_MOTOR_AUX_IQ:
        return "i_q";
    case BXI_MOTOR_AUX_ID:
        return "i_d";
    case BXI_MOTOR_AUX_FSM_STATUS:
        return "fsm_status";
    case BXI_MOTOR_AUX_THERMAL_COEFF:
        return "thermal_coeff";
    case BXI_MOTOR_AUX_VOLTAGE_UTIL:
        return "voltage_util";
    case BXI_MOTOR_AUX_HEARTBEAT:
        return "heartbeat";
    default:
        return "reserved";
    }
}

const char *bxi_motor_aux_unit(uint8_t aux_id)
{
    switch (aux_id) {
    case BXI_MOTOR_AUX_NTC1:
    case BXI_MOTOR_AUX_NTC2:
    case BXI_MOTOR_AUX_WINDING_TEMP:
        return "C";
    case BXI_MOTOR_AUX_VBUS:
        return "V";
    case BXI_MOTOR_AUX_IBUS:
    case BXI_MOTOR_AUX_IQ:
    case BXI_MOTOR_AUX_ID:
        return "A";
    case BXI_MOTOR_AUX_THERMAL_COEFF:
    case BXI_MOTOR_AUX_VOLTAGE_UTIL:
    case BXI_MOTOR_AUX_HEARTBEAT:
    case BXI_MOTOR_AUX_FSM_STATUS:
    default:
        return "";
    }
}

const char *bxi_motor_fsm_state_name(unsigned int fsm_state)
{
    switch (fsm_state & 0x0fu) {
    case 0u:
        return "startup";
    case 1u:
        return "motor_menu";
    case 2u:
        return "enable";
    case 3u:
        return "calib";
    case 4u:
        return "ktm";
    case 5u:
        return "enco";
    case 6u:
        return "ktm_auto";
    case 7u:
        return "setup";
    case 8u:
        return "error";
    case 9u:
        return "openloop";
    case 10u:
        return "sls";
    default:
        return "unknown";
    }
}

const char *bxi_motor_control_mode_name(unsigned int control_mode)
{
    switch (control_mode & 0x07u) {
    case 0u:
        return "current";
    case 1u:
        return "current_ramp";
    case 2u:
        return "velocity";
    case 3u:
        return "velocity_ramp";
    case 4u:
        return "position";
    case 5u:
        return "position_trap";
    default:
        return "unknown";
    }
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint32_t float_to_uint(float value, float min_value, float max_value, unsigned int bits)
{
    float span;
    float scaled;
    uint32_t max_int;

    if (max_value <= min_value || bits == 0u || bits >= 31u) {
        return 0u;
    }

    value = clamp_float(value, min_value, max_value);
    max_int = (1u << bits) - 1u;
    span = max_value - min_value;
    scaled = (value - min_value) * (float)max_int / span;

    if (scaled <= 0.0f) {
        return 0u;
    }
    if (scaled >= (float)max_int) {
        return max_int;
    }

    return (uint32_t)(scaled + 0.5f);
}

static float uint_to_float(uint32_t value, float min_value, float max_value, unsigned int bits)
{
    uint32_t max_int;

    if (max_value <= min_value || bits == 0u || bits >= 31u) {
        return min_value;
    }

    max_int = (1u << bits) - 1u;
    if (value > max_int) {
        value = max_int;
    }

    return ((float)value) * (max_value - min_value) / (float)max_int + min_value;
}

void bxi_motor_pack_special(uint8_t data[BXI_MOTOR_MIT_LEN], uint8_t command)
{
    size_t i;

    for (i = 0u; i < BXI_MOTOR_MIT_LEN - 1u; i++) {
        data[i] = 0xffu;
    }
    data[7] = command;
}

void bxi_motor_pack_mit(uint8_t data[BXI_MOTOR_MIT_LEN],
                        const bxi_motor_limits *limits,
                        float position,
                        float velocity,
                        float kp,
                        float kd,
                        float torque)
{
    uint32_t p_int;
    uint32_t v_int;
    uint32_t kp_int;
    uint32_t kd_int;
    uint32_t t_int;

    if (limits == 0) {
        limits = &bxi_motor_default_limits;
    }

    p_int = float_to_uint(position, limits->p_min, limits->p_max, 16u);
    v_int = float_to_uint(velocity, limits->v_min, limits->v_max, 12u);
    kp_int = float_to_uint(kp, limits->kp_min, limits->kp_max, 12u);
    kd_int = float_to_uint(kd, limits->kd_min, limits->kd_max, 12u);
    t_int = float_to_uint(torque, limits->t_min, limits->t_max, 12u);

    data[0] = (uint8_t)(p_int >> 8);
    data[1] = (uint8_t)(p_int & 0xffu);
    data[2] = (uint8_t)(v_int >> 4);
    data[3] = (uint8_t)(((v_int & 0x0fu) << 4) | (kp_int >> 8));
    data[4] = (uint8_t)(kp_int & 0xffu);
    data[5] = (uint8_t)(kd_int >> 4);
    data[6] = (uint8_t)(((kd_int & 0x0fu) << 4) | (t_int >> 8));
    data[7] = (uint8_t)(t_int & 0xffu);
}

/*
 * 解析新版 MIT 回复帧。
 *
 * 新版固件的 8 字节回复格式：
 *   byte0      : 电机 ID
 *   byte1..2   : 16 位位置原始值
 *   byte3..4   : 12 位速度原始值，其中 byte4 高 4 位属于速度
 *   byte4..5   : 12 位力矩原始值，其中 byte4 低 4 位属于力矩
 *   byte6..7   : 16 位轮询 AUX 遥测，格式为 [aux_id:4bit][payload:12bit]
 *
 * 注意：旧版固件的 byte6..7 是两个 NTC 温度字节，不应使用本函数解析。
 */
int bxi_motor_unpack_reply(const uint8_t *data,
                           size_t len,
                           const bxi_motor_limits *limits,
                           bxi_motor_reply *reply)
{
    /* MIT 标准部分的整型原始值。 */
    uint32_t p_int;
    uint32_t v_int;
    uint32_t t_int;
    /* 新版扩展部分：AUX 轮询遥测原始 16 位字段。 */
    uint16_t aux;

    /* 参数检查：至少需要完整 8 字节 MIT 回复。 */
    if (data == 0 || reply == 0 || len < BXI_MOTOR_MIT_LEN) {
        return -1;
    }

    /* 未指定型号范围时，使用默认 MIT 通信范围。 */
    if (limits == 0) {
        limits = &bxi_motor_default_limits;
    }

    /* byte1..2 组合为 16 位位置原始值。 */
    p_int = ((uint32_t)data[1] << 8) | (uint32_t)data[2];
    /* byte3 和 byte4 高 4 位组合为 12 位速度原始值。 */
    v_int = ((uint32_t)data[3] << 4) | ((uint32_t)data[4] >> 4);
    /* byte4 低 4 位和 byte5 组合为 12 位力矩原始值。 */
    t_int = (((uint32_t)data[4] & 0x0fu) << 8) | (uint32_t)data[5];

    /* byte6..7 是新版轮询 AUX 遥测。 */
    aux = ((uint16_t)data[6] << 8) | (uint16_t)data[7];
    /* 新版解析得到的 AUX 字段有效存在。 */
    reply->aux_valid = true;
    /* 高 4 位表示这次轮询到的遥测类型。 */
    reply->aux_id = (uint8_t)((aux >> 12) & 0x0fu);
    /* 低 12 位是遥测数据原始 payload。 */
    reply->aux_payload = aux & 0x0fffu;
    /*
     * 最新固件把 payload 0xFFF 保留为数值类遥测的无效值；
     * 但 heartbeat 本身允许完整 0..4095 循环，所以 heartbeat 例外。
     */
    reply->aux_payload_valid = reply->aux_id == BXI_MOTOR_AUX_HEARTBEAT ||
                               reply->aux_payload != BXI_MOTOR_AUX_INVALID_PAYLOAD;
    /* 先清零工程单位值，后面根据 aux_id 转换。 */
    reply->aux_value = 0.0f;
    /* byte0 是电机返回的 ID。 */
    reply->motor_id = data[0];
    /* 按配置范围把原始值还原成人可读物理量。 */
    reply->position = uint_to_float(p_int, limits->p_min, limits->p_max, 16u);
    reply->velocity = uint_to_float(v_int, limits->v_min, limits->v_max, 12u);
    reply->torque = uint_to_float(t_int, limits->t_min, limits->t_max, 12u);
    /* 新版温度来自 AUX 轮询；当前帧不一定刚好带温度，所以先置 0。 */
    reply->mos_temperature = 0.0f;
    reply->motor_temperature = 0.0f;

    /* 只有 payload 有效时才做工程单位转换。 */
    if (reply->aux_payload_valid) {
        switch (reply->aux_id) {
        case BXI_MOTOR_AUX_NTC1:
        case BXI_MOTOR_AUX_NTC2:
        case BXI_MOTOR_AUX_WINDING_TEMP:
            /* 温度范围：-50.0 .. 200.0 C，分辨率 0.1 C。 */
            reply->aux_value = (float)reply->aux_payload / 10.0f - 50.0f;
            break;
        case BXI_MOTOR_AUX_VBUS:
            /* 母线电压范围：0.0 .. 100.0 V，分辨率 0.1 V。 */
            reply->aux_value = (float)reply->aux_payload / 10.0f;
            break;
        case BXI_MOTOR_AUX_IBUS:
        case BXI_MOTOR_AUX_IQ:
        case BXI_MOTOR_AUX_ID:
            /* 电流范围：-150.0 .. 150.0 A，分辨率 0.1 A。 */
            reply->aux_value = (float)reply->aux_payload / 10.0f - 150.0f;
            break;
        case BXI_MOTOR_AUX_THERMAL_COEFF:
        case BXI_MOTOR_AUX_VOLTAGE_UTIL:
            /* 系数/利用率范围按 0.001 分辨率传输。 */
            reply->aux_value = (float)reply->aux_payload / 1000.0f;
            break;
        case BXI_MOTOR_AUX_FSM_STATUS:
        case BXI_MOTOR_AUX_HEARTBEAT:
        default:
            /* 状态字、心跳或保留项保留原始 payload。 */
            reply->aux_value = (float)reply->aux_payload;
            break;
        }
    }
    /* 如果当前轮询项是 NTC1，同步填入兼容字段 mos_temperature。 */
    if (reply->aux_id == BXI_MOTOR_AUX_NTC1 && reply->aux_payload_valid) {
        reply->mos_temperature = reply->aux_value;
    /* 如果当前轮询项是 NTC2，同步填入兼容字段 motor_temperature。 */
    } else if (reply->aux_id == BXI_MOTOR_AUX_NTC2 && reply->aux_payload_valid) {
        reply->motor_temperature = reply->aux_value;
    }

    return 0;
}

int bxi_motor_unpack_reply_legacy_ntc(const uint8_t *data,
                                      size_t len,
                                      const bxi_motor_limits *limits,
                                      bxi_motor_reply *reply)
{
    uint32_t p_int;
    uint32_t v_int;
    uint32_t t_int;

    if (data == 0 || reply == 0 || len < BXI_MOTOR_MIT_LEN) {
        return -1;
    }

    if (limits == 0) {
        limits = &bxi_motor_default_limits;
    }

    p_int = ((uint32_t)data[1] << 8) | (uint32_t)data[2];
    v_int = ((uint32_t)data[3] << 4) | ((uint32_t)data[4] >> 4);
    t_int = (((uint32_t)data[4] & 0x0fu) << 8) | (uint32_t)data[5];

    reply->motor_id = data[0];
    reply->position = uint_to_float(p_int, limits->p_min, limits->p_max, 16u);
    reply->velocity = uint_to_float(v_int, limits->v_min, limits->v_max, 12u);
    reply->torque = uint_to_float(t_int, limits->t_min, limits->t_max, 12u);
    reply->mos_temperature = uint_to_float(data[6], limits->temp_min, limits->temp_max, 8u);
    reply->motor_temperature = uint_to_float(data[7], limits->temp_min, limits->temp_max, 8u);
    reply->aux_valid = false;
    reply->aux_payload_valid = false;
    reply->aux_id = 0u;
    reply->aux_payload = 0u;
    reply->aux_value = 0.0f;

    return 0;
}
