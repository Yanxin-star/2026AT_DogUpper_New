#include <arm_sim.hpp>  // 假设你的包名是 arm_controller
#include <algorithm>
#include <robot_interfaces/msg/arm.hpp>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <iomanip>

namespace arm_controller {

ArmController::ArmController() : csv_initialized_(false) {}

controller_interface::CallbackReturn ArmController::on_init() {
    state_publisher = get_node()->create_publisher<robot_interfaces::msg::Arm>("arm_state", 10);
    target_subscriber = get_node()->create_subscription<robot_interfaces::msg::Arm>(
        "arm_target", 10, [this](const robot_interfaces::msg::Arm& msg) { joints_target = msg; });

    joints_name_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    // 初始化 PID 参数
    for (int i = 0; i < 6; ++i) {
        joint_kp[i] = 50.0;
        joint_kd[i] = 3.0;
    }

    auto node = get_node();
    for (int i = 0; i < 6; ++i) {
        node->declare_parameter("joint" + std::to_string(i+1) + "_kp", joint_kp[i]);
        node->declare_parameter("joint" + std::to_string(i+1) + "_kd", joint_kd[i]);
    }

    node->declare_parameter("record_torque", false);
    node->declare_parameter("torque_filter_gate", 0.8);
    node->declare_parameter("omega_filter_gate", 0.8);

    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
            for (int i = 0; i < 6; ++i) {
                if (param.get_name() == "joint" + std::to_string(i+1) + "_kp")
                    joint_kp[i] = param.as_double();
                if (param.get_name() == "joint" + std::to_string(i+1) + "_kd")
                    joint_kd[i] = param.as_double();
            }
            if (param.get_name() == "record_torque") {
                csv_initialized_ = param.as_bool();
                if (csv_initialized_) {
                    start_time_ = std::chrono::steady_clock::now();
                    csv_file_.open("/tmp/arm_joint_torque.csv");
                    csv_file_ << "time_ms";
                    for (int i = 0; i < 6; ++i) csv_file_ << ",joint" << (i+1) << "_torque";
                    csv_file_ << "\n";
                } else {
                    csv_file_.close();
                }
            }
            if (param.get_name() == "torque_filter_gate")
                joint_torque_filter_gate = param.as_double();
            if (param.get_name() == "omega_filter_gate")
                joint_omega_filter_gate = param.as_double();
        }
        return result;
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    auto node = get_node();
    for (int i = 0; i < 6; ++i) {
        joint_kp[i] = node->get_parameter("joint" + std::to_string(i+1) + "_kp").as_double();
        joint_kd[i] = node->get_parameter("joint" + std::to_string(i+1) + "_kd").as_double();
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    soft_start_start_time = std::chrono::steady_clock::now();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ArmController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    // 读取关节状态并发布
    for (size_t i = 0; i < joints_name_.size(); ++i) {
        joints_state.motor[i].rad    = state_interfaces_[i*3+0].get_value();
        joints_state.motor[i].omega  = joint_omega_filter_gate * joints_state.motor[i].omega +
                                        (1.0 - joint_omega_filter_gate) * state_interfaces_[i*3+1].get_value();
        joints_state.motor[i].torque = joint_torque_filter_gate * joints_state.motor[i].torque +
                                        (1.0 - joint_torque_filter_gate) * state_interfaces_[i*3+2].get_value();
    }

    state_publisher->publish(joints_state);

    // CSV 记录
    if (csv_file_.is_open()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
        csv_file_ << elapsed;
        for (int i = 0; i < 6; ++i) csv_file_ << "," << joints_state.motor[i].torque;
        csv_file_ << "\n";
        csv_file_.flush();
    }

    // ramp soft start
    auto now = std::chrono::steady_clock::now();
    double elapsed_s = std::chrono::duration<double>(now - soft_start_start_time).count();
    double ramp = std::clamp(elapsed_s / soft_start_duration_.count(), 0.0, 1.0);

    // 写入 command_interfaces_
    for (size_t i = 0; i < joints_name_.size(); ++i) {
        double effort = joint_kp[i] * (joints_target.motor[i].rad - joints_state.motor[i].rad) +
                        joint_kd[i] * (joints_target.motor[i].omega - joints_state.motor[i].omega);
        effort = std::clamp(effort, -12.0, 12.0);
        double sum_effort = effort + joints_target.motor[i].torque;
        sum_effort = std::clamp(sum_effort, -20.0, 20.0);
        command_interfaces_[i].set_value(sum_effort * ramp);
    }

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration ArmController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& name : joints_name_)
        cfg.names.push_back(name + "/effort");
    return cfg;
}

controller_interface::InterfaceConfiguration ArmController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& name : joints_name_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

} // namespace arm_controller

PLUGINLIB_EXPORT_CLASS(arm_controller::ArmController, controller_interface::ControllerInterface)