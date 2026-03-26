#include "arm_calc/arm_joint_update.hpp"
#include "arm_calc/arm_ctrl.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/logger.hpp>

using namespace std::chrono_literals;
using Vector6d = Eigen::Matrix<double, 6, 1>;



Arm_joint::Arm_joint(const rclcpp::Node::SharedPtr node, KDL::Chain& chain) {
    armstep   = std::make_unique<ArmStep>();
    armcalc   = std::make_unique<ArmCalc>(chain);
    arm_node_ = node;
};

Arm_joint::~Arm_joint() {};



std::tuple<Vector6d, Vector6d, Vector6d>
    Arm_joint::targetUpdate(const Vector6d& exp_joint, const Vector6d& cur_joint, const Vector6d& cur_vel) {
    Vector6d rad;
    Vector6d vel;
    Vector6d acc;
    Vector6d torque;
    auto now = arm_node_->get_clock()->now();

    // 👉 初始化时间（只在第一次）
    if (!first_read || (exp_joint - last_target).norm() > 1e-3) {
        start_time  = now;
        first_read  = true;
        last_target = exp_joint;
        armstep->update_arm_trajectory(cur_joint, cur_vel, exp_joint, Vector6d::Zero(), 4.0);
    }
    bool success;
    // 👉 计算运行时间
    double t = (now - start_time).seconds();

    // 👉 获取轨迹输出
    std::tie(rad, vel, acc) = armstep->get_arm_target(t, success);

    torque = armcalc->joint_torque_dynamic2(rad, vel, acc);

    return std::make_tuple(rad, vel, torque);
}
