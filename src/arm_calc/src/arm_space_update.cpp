#include "arm_calc/arm_space_update.hpp"
#include "arm_calc/arm_ctrl.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/Dense>
#include <chrono>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/logger.hpp>

using namespace std::chrono_literals;
using Vector6d = Eigen::Matrix<double, 6, 1>;



Arm_space::Arm_space(const rclcpp::Node::SharedPtr node,KDL::Chain& chain){
    armstep = std::make_unique<ArmStep>();
    armcalc = std::make_unique<ArmCalc>(chain);
    arm_node_ = node;
};

Arm_space::~Arm_space(){};


 
std::tuple<Vector6d, Vector6d, Vector6d> Arm_space::armtargetUpdate(
       const Vector6d &exp_pos_rpy,const Vector6d &rad,const Vector6d &omega
        )
    {
       Vector6d Rad;
       Vector6d Vel;
       Vector6d Acc;
       Vector6d torque;
       int result;

       auto now = arm_node_->get_clock()->now();

    // 👉 初始化时间（只在第一次）
    if (!first_read || (exp_pos_rpy - last_target).norm() > 1e-3)
     { start_time = now; first_read = true; 
        last_target = exp_pos_rpy; 
        armstep->update_arm_trajectory
        (rad ,omega , armcalc->joint_pos(exp_pos_rpy.head(3),exp_pos_rpy.tail(3),&result), Vector6d::Zero(), 4.0 );

     }
    bool success;
    // 👉 计算运行时间
    double t = (now - start_time).seconds();

    // 👉 获取轨迹输出
    std::tie(Rad,Vel,Acc)=armstep->get_arm_target(t,success);

    torque = armcalc->joint_torque_dynamic2(Rad,Vel,Acc);

    return std::make_tuple(Rad, Vel, torque);
}


