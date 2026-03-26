#include "arm_calc/arm_view.hpp"
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



Arm_view::Arm_view(const rclcpp::Node::SharedPtr node,double dt_,KDL::Chain& chain){

    armstep = std::make_unique<ArmStep>();
    armcalc = std::make_unique<ArmCalc>(chain);
    arm_node_=node;
    dt = dt_;

};

Arm_view::~Arm_view(){};


    // 初始化当前期望位置

    

    // 每个控制周期调用
std::tuple<Vector6d, Vector6d, Vector6d> Arm_view::armtargetview
(const Eigen::Vector3d &final_pos,const Vector6d &now_joint_pos) 
{

    

    int result;
    curr_pos = armcalc->foot_pos(now_joint_pos);
    pre_pos.head(3) = final_pos;
    pre_pos.tail(3) = curr_pos.tail(3);
    

//     if (log_counter >= 100) {
//     RCLCPP_INFO(arm_node_->get_logger(), "进入视觉伺服解算态");
   
// }



     // 1️⃣ 计算当前期望速度（比例控制）
    Vector6d vel_des = (pre_pos - curr_pos) * kp;

    // 2️⃣ 计算期望加速度（限制加速度）
    Vector6d acc_des = (vel_des - prev_vel) / dt;
    acc_des = acc_des.array().max(-max_acc_).min(max_acc_).matrix();

    // 3️⃣ 更新当前期望速度
    Vector6d curr_vel = prev_vel + acc_des * dt;

    // 4️⃣ 更新当前期望位置
    curr_pos += curr_vel * dt;

    // 5️⃣ 保存本次速度作为下次上次速度
    prev_vel = curr_vel;

    // 6️⃣ 计算关节值
    Vector6d joint_pos  = armcalc->joint_pos(curr_pos.head(3),curr_pos.tail(3),&result);
    if(result != 0) {
        if (log_counter >= 100) 
    RCLCPP_WARN(arm_node_->get_logger(), "IK 失败! %d",result);
    Vector6d joint_vel  = armcalc->joint_vel(joint_pos,curr_vel);
    Vector6d joint_torque = armcalc->joint_torque_dynamic(joint_pos,joint_vel,acc_des);
//       if (log_counter >= 100) {
//     RCLCPP_INFO(arm_node_->get_logger(), 
//     "返回期望cart值：%f %f %f",curr_pos[0],curr_pos[1],curr_pos[2]);
   
// }

//     if (log_counter >= 100) {
//     RCLCPP_INFO(arm_node_->get_logger(), 
//     "返回期望omega值：%f %f %f %f %f %f",joint_vel[0],joint_vel[1],joint_vel[2],joint_vel[3],joint_vel[4],joint_vel[5]);
   
// }

//      if (log_counter >= 100) {
//     RCLCPP_INFO(arm_node_->get_logger(), 
//     "返回期望torque值：%f %f %f %f %f %f",joint_torque[0],joint_torque[1],joint_torque[2],joint_torque[3],joint_torque[4],joint_torque[5]);
//    log_counter = 0;
// }

//    log_counter++;

    return std::make_tuple(now_joint_pos, Vector6d::Zero(), Vector6d::Zero());
}
     if (log_counter >= 100) {
    RCLCPP_INFO(arm_node_->get_logger(), "完成视觉伺服解算");
   
}

 
    Vector6d joint_vel  = armcalc->joint_vel(joint_pos,curr_vel);
    Vector6d joint_torque = armcalc->joint_torque_dynamic(joint_pos,joint_vel,acc_des);

//      if (log_counter >= 100) {
//     RCLCPP_INFO(arm_node_->get_logger(), 
//     "返回期望rad值：%f %f %f %f %f %f",joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5]);
   
// }

//     if (log_counter >= 100) {
//     RCLCPP_INFO(arm_node_->get_logger(), 
//     "返回期望omega值：%f %f %f %f %f %f",joint_vel[0],joint_vel[1],joint_vel[2],joint_vel[3],joint_vel[4],joint_vel[5]);
   
// }

//      if (log_counter >= 100) {
//     RCLCPP_INFO(arm_node_->get_logger(), 
//     "返回期望torque值：%f %f %f %f %f %f",joint_torque[0],joint_torque[1],joint_torque[2],joint_torque[3],joint_torque[4],joint_torque[5]);
//    log_counter = 0;
// }

//    log_counter++;


    // 7️⃣ 返回当前期望位置 / 当前期望速度 / 当前期望加速度
    return std::make_tuple(joint_pos, joint_vel, joint_torque);
    }



