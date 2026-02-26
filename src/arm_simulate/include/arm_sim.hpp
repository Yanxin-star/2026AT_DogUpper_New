#pragma once

#include <controller_interface/controller_interface.hpp>
#include <robot_interfaces/msg/arm.hpp>

#include <rclcpp/rclcpp.hpp>

namespace arm_sim
{

class Arm_sim : public controller_interface::ControllerInterface
{
public:

    Arm_sim();

    

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State&) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State&) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State&) override;

    controller_interface::return_type update(
        const rclcpp::Time&,
        const rclcpp::Duration&) override;

    controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

private:

    rclcpp::Publisher<robot_interfaces::msg::Arm>::SharedPtr
        state_publisher_;

    rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr
        target_subscriber_;

    robot_interfaces::msg::Arm joints_state_;
    robot_interfaces::msg::Arm joints_target_;

    std::vector<std::string> joints_name_;

    std::chrono::steady_clock::time_point
        soft_start_start_time_;

    std::chrono::milliseconds
        soft_start_duration_;
};

}