#include <chrono>
#include <cmath>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bxi_motor_msgs/msg/motor_command.hpp"
#include "bxi_motor_msgs/msg/motor_state_array.hpp"
#include "backend.h"

class MotorNode : public rclcpp::Node {
public:
    MotorNode() : Node("bxi_motor_control") {
        auto config = declare_parameter<std::string>("config_file", "config/motor_console.yaml");
        auto interface = declare_parameter<std::string>("interface", "enp86s0");
        bool pci = declare_parameter<bool>("init_pci", false);
        bool power = declare_parameter<bool>("auto_power_on", false);
        bool scan = declare_parameter<bool>("auto_scan", true);
        default_hold_ms_ = declare_parameter<int>("default_hold_ms", 1000);
        double feedback_hz = declare_parameter<double>("feedback_hz", 100.0);
        if (default_hold_ms_ < 1 || default_hold_ms_ > 60000 ||
            !std::isfinite(feedback_hz) || feedback_hz < 1 || feedback_hz > 1000 || (power && !pci))
            throw std::invalid_argument("Invalid feedback_hz/default_hold_ms or auto_power_on without init_pci");
        if (bxi_ros_init(config.c_str(), interface.c_str(), pci) != 0)
            throw std::runtime_error("Motor backend initialization failed");
        try {
            if (power) {
                struct bxi_ros_command command{}; command.action = 7;
                if (bxi_ros_command(&command) != 0) throw std::runtime_error("Power-on failed");
            }
            if (scan && bxi_ros_scan() != 0) throw std::runtime_error("MIT PDO warm-up failed; inspect backend diagnostics");
            states_ = create_publisher<bxi_motor_msgs::msg::MotorStateArray>("~/states", rclcpp::SensorDataQoS());
            status_ = create_publisher<std_msgs::msg::String>("~/command_status", 10);
            commands_ = create_subscription<bxi_motor_msgs::msg::MotorCommand>("~/command", rclcpp::QoS(1).reliable(),
                [this](bxi_motor_msgs::msg::MotorCommand::SharedPtr msg) { command(*msg); });
            timer_ = create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9 / feedback_hz)),
                [this]() { publish_states(); });
        } catch (...) { bxi_ros_shutdown(); throw; }
        RCLCPP_INFO(get_logger(), "Ready: PDO 1000Hz, ROS feedback %.1fHz; command acceptance is not execution confirmation", feedback_hz);
    }
    ~MotorNode() override { bxi_ros_shutdown(); }
private:
    void command(const bxi_motor_msgs::msg::MotorCommand &m) {
        struct bxi_ros_command c{};
        c.action = m.action; c.slave_id = m.slave_id; c.pn082 = m.pn082;
        c.position = m.position; c.velocity = m.velocity; c.torque = m.torque;
        c.kp = m.kp; c.kd = m.kd; c.duration_ms = m.duration_ms;
        if ((m.action == m.MIT || m.action == m.ENABLE) && c.duration_ms == 0)
            c.duration_ms = default_hold_ms_;
        if (m.action == m.QUALITY && c.duration_ms == 0) c.duration_ms = 10000;
        int result = bxi_ros_command(&c);
        std_msgs::msg::String status;
        status.data = "action=" + std::to_string(m.action) + " slave=" + std::to_string(m.slave_id) +
            (result == 0 ? " accepted; see feedback/backend logs for execution" : " rejected");
        status_->publish(status);
    }
    void publish_states() {
        struct bxi_ros_state raw[255];
        size_t count = bxi_ros_feedback(raw, 255);
        bxi_motor_msgs::msg::MotorStateArray msg; msg.header.stamp = now();
        for (size_t i = 0; i < count; ++i) {
            const auto &s = raw[i]; bxi_motor_msgs::msg::MotorState m;
            m.slave_id = s.slave_id; m.valid = s.valid; m.sample_age_ms = s.sample_age_ms;
            m.status_word = s.status_word; m.control_word = s.control_word;
            m.error_code = s.error_code; m.mode = s.mode;
            m.position = s.position; m.velocity = s.velocity; m.torque = s.torque;
            m.target_position = s.target_position; m.target_velocity = s.target_velocity;
            m.target_torque = s.target_torque; m.kp = s.kp; m.kd = s.kd;
            m.pn082_valid = s.pn082_valid; m.pn082 = s.pn082; msg.motors.push_back(m);
        }
        states_->publish(msg);
    }
    int default_hold_ms_;
    rclcpp::Publisher<bxi_motor_msgs::msg::MotorStateArray>::SharedPtr states_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
    rclcpp::Subscription<bxi_motor_msgs::msg::MotorCommand>::SharedPtr commands_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::on_shutdown([]() { bxi_ros_request_stop(); });
    int result = 0;
    try { auto node = std::make_shared<MotorNode>(); rclcpp::spin(node); }
    catch (const std::exception &e) { fprintf(stderr, "bxi_motor_control: %s\n", e.what()); result = 1; }
    rclcpp::shutdown(); return result;
}
