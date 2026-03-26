#pragma once

#include "robot_interfaces/msg/arm.hpp"  // 假设消息名为 Arm.msg
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <string>
#include <fstream>
#include <chrono>
#include <vector>

namespace arm_controller {

class ArmController : public controller_interface::ControllerInterface {
public:
    ArmController();

    // 生命周期接口
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    // 核心控制循环
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    // 指令接口配置
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    // ROS2 发布与订阅
    rclcpp::Publisher<robot_interfaces::msg::Arm>::SharedPtr state_publisher;
    rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr target_subscriber;

    // 关节名字列表
    std::vector<std::string> joints_name_;

    // 参数回调句柄
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    // 关节状态与目标
    robot_interfaces::msg::Arm joints_target;
    robot_interfaces::msg::Arm joints_state;

    // PID 参数
    double joint_kp[6];
    double joint_kd[6];

    // 滤波参数
    double joint_torque_filter_gate{0.8};
    double joint_omega_filter_gate{0.8};

    // CSV 记录
    std::ofstream csv_file_;
    bool csv_initialized_{false};
    std::chrono::steady_clock::time_point start_time_;

    // Soft start
    std::chrono::duration<double> soft_start_duration_{5.0};
    std::chrono::steady_clock::time_point soft_start_start_time;

    // 调试计数
    int debug_cnt{0};
};

} // namespace arm_controller