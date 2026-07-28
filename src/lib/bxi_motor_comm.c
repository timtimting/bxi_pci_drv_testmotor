#include "bxi_motor_comm.h"

const bxi_motor_limits bxi_motor_default_limits = {
    -12.5f, 12.5f,
    -45.0f, 45.0f,
    0.0f, 500.0f,
    0.0f, 5.0f,
    -18.0f, 18.0f,
    -30.0f, 150.0f,
};

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

    return 0;
}
