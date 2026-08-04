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

int bxi_motor_unpack_reply(const uint8_t *data,
                           size_t len,
                           const bxi_motor_limits *limits,
                           bxi_motor_reply *reply)
{
    uint32_t p_int;
    uint32_t v_int;
    uint32_t t_int;
    uint16_t aux;

    if (data == 0 || reply == 0 || len < BXI_MOTOR_MIT_LEN) {
        return -1;
    }

    if (limits == 0) {
        limits = &bxi_motor_default_limits;
    }

    p_int = ((uint32_t)data[1] << 8) | (uint32_t)data[2];
    v_int = ((uint32_t)data[3] << 4) | ((uint32_t)data[4] >> 4);
    t_int = (((uint32_t)data[4] & 0x0fu) << 8) | (uint32_t)data[5];

    aux = ((uint16_t)data[6] << 8) | (uint16_t)data[7];
    reply->aux_valid = true;
    reply->aux_id = (uint8_t)((aux >> 12) & 0x0fu);
    reply->aux_payload = aux & 0x0fffu;
    reply->aux_payload_valid = reply->aux_payload != BXI_MOTOR_AUX_INVALID_PAYLOAD;
    reply->aux_value = 0.0f;
    reply->motor_id = data[0];
    reply->position = uint_to_float(p_int, limits->p_min, limits->p_max, 16u);
    reply->velocity = uint_to_float(v_int, limits->v_min, limits->v_max, 12u);
    reply->torque = uint_to_float(t_int, limits->t_min, limits->t_max, 12u);
    reply->mos_temperature = 0.0f;
    reply->motor_temperature = 0.0f;

    if (reply->aux_payload_valid) {
        switch (reply->aux_id) {
        case BXI_MOTOR_AUX_NTC1:
        case BXI_MOTOR_AUX_NTC2:
        case BXI_MOTOR_AUX_WINDING_TEMP:
            reply->aux_value = (float)reply->aux_payload / 10.0f - 50.0f;
            break;
        case BXI_MOTOR_AUX_VBUS:
            reply->aux_value = (float)reply->aux_payload / 20.0f;
            break;
        case BXI_MOTOR_AUX_IBUS:
        case BXI_MOTOR_AUX_IQ:
        case BXI_MOTOR_AUX_ID:
            reply->aux_value = (float)reply->aux_payload / 10.0f - 160.0f;
            break;
        case BXI_MOTOR_AUX_THERMAL_COEFF:
        case BXI_MOTOR_AUX_VOLTAGE_UTIL:
            reply->aux_value = (float)reply->aux_payload / 1000.0f;
            break;
        case BXI_MOTOR_AUX_FSM_STATUS:
        case BXI_MOTOR_AUX_HEARTBEAT:
        default:
            reply->aux_value = (float)reply->aux_payload;
            break;
        }
    }
    if (reply->aux_id == BXI_MOTOR_AUX_NTC1 && reply->aux_payload_valid) {
        reply->mos_temperature = reply->aux_value;
    } else if (reply->aux_id == BXI_MOTOR_AUX_NTC2 && reply->aux_payload_valid) {
        reply->motor_temperature = reply->aux_value;
    }

    return 0;
}
