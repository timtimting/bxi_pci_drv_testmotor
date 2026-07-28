#ifndef BXI_MOTOR_COMM_H
#define BXI_MOTOR_COMM_H

#include <stddef.h>
#include <stdint.h>

#define BXI_MOTOR_MIT_LEN 8u
#define BXI_MOTOR_BROADCAST_ID 0x7ffu

#define BXI_MOTOR_CMD_ENABLE 0xfcu
#define BXI_MOTOR_CMD_DISABLE 0xfdu
#define BXI_MOTOR_CMD_ZERO 0xfeu
#define BXI_MOTOR_CMD_TERMINAL_ON 0xfbu
#define BXI_MOTOR_CMD_TERMINAL_OFF 0xfau

typedef struct
{
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float t_min;
    float t_max;
    float temp_min;
    float temp_max;
} bxi_motor_limits;

typedef struct
{
    uint8_t motor_id;
    float position;
    float velocity;
    float torque;
    float mos_temperature;
    float motor_temperature;
} bxi_motor_reply;

extern const bxi_motor_limits bxi_motor_default_limits;

void bxi_motor_pack_special(uint8_t data[BXI_MOTOR_MIT_LEN], uint8_t command);
void bxi_motor_pack_mit(uint8_t data[BXI_MOTOR_MIT_LEN],
                        const bxi_motor_limits *limits,
                        float position,
                        float velocity,
                        float kp,
                        float kd,
                        float torque);
int bxi_motor_unpack_reply(const uint8_t *data,
                           size_t len,
                           const bxi_motor_limits *limits,
                           bxi_motor_reply *reply);

#endif
