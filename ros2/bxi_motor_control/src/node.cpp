#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include "communication/msg/actuator_cmds.hpp"
#include "communication/msg/actuator_states.hpp"
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
        auto topic_prefix = declare_parameter<std::string>("topic_prefix", "/hardware/");
        joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", {"l_shoulder_y_joint", "l_shoulder_x_joint", "l_shoulder_z_joint", "l_elbow_y_joint", "l_wrist_x_joint", "l_wrist_y_joint", "l_wrist_z_joint", "r_shoulder_y_joint", "r_shoulder_x_joint", "r_shoulder_z_joint", "r_elbow_y_joint", "r_wrist_x_joint", "r_wrist_y_joint", "r_wrist_z_joint"});
        ignore_unmapped_joints_ = declare_parameter<bool>("ignore_unmapped_joints", true);
        auto ids = declare_parameter<std::vector<int64_t>>("slave_ids", {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14});
        if (joint_names_.empty() || joint_names_.size() != ids.size() || joint_names_.size() > 255)
            throw std::invalid_argument("joint_names and slave_ids must have equal nonzero length <=255");
        std::unordered_set<int64_t> unique_ids;
        for (size_t i = 0; i < ids.size(); ++i) {
            if (joint_names_[i].empty() || ids[i] < 1 || ids[i] > 255 ||
                !name_to_slave_.emplace(joint_names_[i], ids[i]).second || !unique_ids.insert(ids[i]).second)
                throw std::invalid_argument("Joint names and slave IDs must be unique; IDs must be 1..255");
        }
        if (topic_prefix.empty()) topic_prefix = "/hardware/";
        if (topic_prefix.back() != '/') topic_prefix += '/';

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
            auto stream_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
            actuator_states_ = create_publisher<communication::msg::ActuatorStates>(
                topic_prefix + "actuator_states", stream_qos);
            actuator_commands_ = create_subscription<communication::msg::ActuatorCmds>(
                topic_prefix + "actuators_cmds", stream_qos,
                [this](communication::msg::ActuatorCmds::SharedPtr msg) { named_command(*msg); });

            commands_ = create_subscription<bxi_motor_msgs::msg::MotorCommand>("~/command", rclcpp::QoS(1).reliable(),
                [this](bxi_motor_msgs::msg::MotorCommand::SharedPtr msg) { command(*msg); });
            timer_ = create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9 / feedback_hz)),
                [this]() { publish_states(); });
        } catch (...) { bxi_ros_shutdown(); throw; }
        RCLCPP_INFO(get_logger(), "Ready: PDO 1000Hz, ROS feedback %.1fHz; command acceptance is not execution confirmation", feedback_hz);
    }
    ~MotorNode() override { bxi_ros_shutdown(); }
private:
    void named_command(const communication::msg::ActuatorCmds &m) {
        const size_t n = m.actuators_name.size();
        if (!n || n > 255 || m.pos.size() != n || m.vel.size() != n || m.torque.size() != n ||
            m.kp.size() != n || m.kd.size() != n) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected actuator frame: all arrays must match names");
            return;
        }
        std::vector<struct bxi_ros_command> frame;
        frame.reserve(n);
        std::unordered_set<std::string> names;
        for (size_t i = 0; i < n; ++i) {
            auto mapping = name_to_slave_.find(m.actuators_name[i]);
            if (!names.insert(m.actuators_name[i]).second) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected actuator frame: duplicate name");
                return;
            }
            if (mapping == name_to_slave_.end() && ignore_unmapped_joints_) continue;
            if (mapping == name_to_slave_.end() ||
                !std::isfinite(m.pos[i]) || !std::isfinite(m.vel[i]) || !std::isfinite(m.torque[i]) ||
                !std::isfinite(m.kp[i]) || !std::isfinite(m.kd[i]) || m.kp[i] < 0 || m.kd[i] < 0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected actuator frame: unknown/duplicate name or invalid value");
                return;
            }
            frame.emplace_back();
            auto &c = frame.back(); c.action = 3; c.slave_id = mapping->second;
            c.position = m.pos[i]; c.velocity = m.vel[i]; c.torque = m.torque[i];
            c.kp = m.kp[i]; c.kd = m.kd[i];
        }
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected actuator frame: no configured arm joints");
            return;
        }
        if (bxi_ros_batch(frame.data(), frame.size(), default_hold_ms_) != 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected actuator frame: PDO not ready, stale or queue full");
            std_msgs::msg::String status; status.data = "actuators_cmds rejected"; status_->publish(status);
        }
    }
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
        communication::msg::ActuatorStates named;
        named.header = msg.header;
        for (const auto &name : joint_names_) {
            const auto id = name_to_slave_.at(name);
            for (size_t i = 0; i < count; ++i) {
                const auto &motor = raw[i];
                if (motor.slave_id != id || !motor.valid) continue;
                named.name.push_back(name);
                named.position.push_back(motor.position);
                named.velocity.push_back(motor.velocity);
                named.effort.push_back(motor.torque);
                // Current MIT PDO contains no temperature feedback.
                named.driver_temperature.push_back(std::numeric_limits<double>::quiet_NaN());
                named.motor_temperature.push_back(std::numeric_limits<double>::quiet_NaN());
                break;
            }
        }
        actuator_states_->publish(named);

    }
    bool ignore_unmapped_joints_;
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, uint16_t> name_to_slave_;
    rclcpp::Publisher<communication::msg::ActuatorStates>::SharedPtr actuator_states_;
    rclcpp::Subscription<communication::msg::ActuatorCmds>::SharedPtr actuator_commands_;
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
