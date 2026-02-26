#include <arm_sim.hpp>
#include <robot_interfaces/msg/arm.hpp>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <algorithm>

namespace arm_sim
{

Arm_sim::Arm_sim()
: soft_start_duration_(std::chrono::milliseconds(2000))
{
}

controller_interface::CallbackReturn Arm_sim::on_init()
{
    auto node = get_node();

    /* publisher: 发布机械臂状态 */
    state_publisher_ =
        node->create_publisher<robot_interfaces::msg::Arm>(
            "arm_status", 10);

    /* subscriber: 接收目标位置 */
    target_subscriber_ =
        node->create_subscription<robot_interfaces::msg::Arm>(
            "arm_target",
            10,
            [this](const robot_interfaces::msg::Arm& msg)
            {
                joints_target_ = msg;
            });

    /* 定义4个关节 */
    joints_name_ =
    {
        "joint1",
        "joint2",
        "joint3",
        "joint4"
    };

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Arm_sim::on_configure(
    const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_node()->get_logger(),
                "Arm_sim position controller configured");

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Arm_sim::on_activate(
    const rclcpp_lifecycle::State&)
{
    soft_start_start_time_ =
        std::chrono::steady_clock::now();

    RCLCPP_INFO(get_node()->get_logger(),
                "Arm_sim position controller activated");

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Arm_sim::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_node()->get_logger(),
                "Arm_sim position controller deactivated");

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Arm_sim::update(
    const rclcpp::Time&,
    const rclcpp::Duration&)
{
    
/* ---------- 读取关节当前位置 ---------- */

robot_interfaces::msg::Arm msg;


    msg.servo2.up =state_interfaces_[0].get_value();
    msg.servo2.low =state_interfaces_[1].get_value();
    msg.rob01.rad =state_interfaces_[2].get_value();
    msg.rob02.rad =state_interfaces_[3].get_value();


/* ---------- 发布状态 ---------- */

state_publisher_->publish(msg);
    /* ---------- soft start ---------- */
    auto now = std::chrono::steady_clock::now();

    double elapsed =
        std::chrono::duration<double>(
            now - soft_start_start_time_).count();

    double ramp =
        std::clamp(
            elapsed /
            std::chrono::duration<double>(
                soft_start_duration_).count(),
            0.0,
            1.0);

    /* ---------- 写入目标位置 ---------- */

 std::array<double, 4> targets;

targets[0] = static_cast<double>(joints_target_.servo2.up);
targets[1] = static_cast<double>(joints_target_.servo2.low);
targets[2] = joints_target_.rob01.rad;
targets[3] = joints_target_.rob02.rad;

for (size_t i = 0; i < 4; i++)
{
    command_interfaces_[i].set_value(targets[i] * ramp);
}

    return controller_interface::return_type::OK;
}

/* command interface: position */
controller_interface::InterfaceConfiguration
Arm_sim::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;

    cfg.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name_)
    {
        cfg.names.push_back(
            name + "/position");
    }

    return cfg;
}

/* state interface: position */
controller_interface::InterfaceConfiguration
Arm_sim::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;

    cfg.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name_)
    {
        cfg.names.push_back(
            name + "/position");
    }

    return cfg;
}

} // namespace arm_sim


PLUGINLIB_EXPORT_CLASS(
    arm_sim::Arm_sim,
    controller_interface::ControllerInterface)