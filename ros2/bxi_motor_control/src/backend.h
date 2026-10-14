#ifndef BXI_ROS_BACKEND_H
#define BXI_ROS_BACKEND_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
struct bxi_ros_command {
    uint8_t action;
    uint16_t slave_id, pn082;
    float position, velocity, torque, kp, kd;
    uint32_t duration_ms;
};
struct bxi_ros_state {
    uint16_t slave_id, status_word, control_word, error_code;
    uint8_t mode;
    int valid, pn082_valid;
    int16_t pn082;
    double sample_age_ms;
    float position, velocity, torque, target_position, target_velocity, target_torque, kp, kd;
};
int bxi_ros_init(const char *config, const char *interface, int init_pci);
int bxi_ros_scan(void);
int bxi_ros_command(const struct bxi_ros_command *command);
int bxi_ros_batch(const struct bxi_ros_command *commands, size_t count, uint32_t duration_ms);
size_t bxi_ros_feedback(struct bxi_ros_state *out, size_t capacity);
void bxi_ros_request_stop(void);
void bxi_ros_shutdown(void);
#ifdef __cplusplus
}
#endif
#endif
