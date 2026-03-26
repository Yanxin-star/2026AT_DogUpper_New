#pragma once

#include "arm_step.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <ctime>
#include <geometry_msgs/msg/point.hpp>
#include <kdl/jacobian.hpp>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_interfaces/msg/robot.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>   // ← 你缺的就是它
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

using Vector6d = Eigen::Matrix<double, 6, 1>;

class ArmCalc{
public:
    ArmCalc(KDL::Chain &chain);
    ~ArmCalc();
    void set_leg_state(KDL::JntArray &rad, KDL::JntArray &omega, KDL::JntArray &torque);    //在一个控制周期内，应首先调用它

    //int joint_pos(KDL::JntArray &joint_rad, KDL::Vector &foot_pos,KDL::JntArray &result);
    Vector6d joint_pos(const Eigen::Vector3d &pos,const Eigen::Vector3d &rpy,int *result);      //稍后需要在线安装IK求解器（手推的解析求解器或者数值迭代器）

    Vector6d joint_vel(const Vector6d &joint_rad, const Vector6d &arm_exp_vel); 

    Vector6d joint_torque_dynamic(const Vector6d &joint_rad, const Vector6d &joint_omega, const Vector6d &foot_acc);

    Vector6d joint_torque_dynamic2(const Vector6d &joint_rad, const Vector6d &joint_omega, const Vector6d &foot_acc);

    Vector6d joint_torque_foot_force(const Vector6d &joint_rad,const Vector6d &foot_force);    //由足端期望力计算的关节力矩

    Vector6d foot_force(const Vector6d &joint_rad,const Vector6d &joint_torque,const Vector6d &forward_torque);

    Vector6d foot_vel(const Vector6d &joint_rad, const Vector6d &joint_omega);
    
    Vector6d foot_pos(const Vector6d& joint_rad);

  
private:

   Vector6d joint_acc(const Vector6d &joint_rad, const Vector6d &joint_vel,Vector6d arm_acc);

    Eigen::Matrix<double, 6, 6> get_6x6_jacobian_(const KDL::Jacobian &full_jacobian);    //从KDL库中求出我们感兴趣的3*3位置雅可比矩阵

    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive fk_solver;  //关节位置->足端位置
    KDL::ChainJntToJacSolver jacobain_solver;        //求解雅可比矩阵
    KDL::ChainJntToJacDotSolver jdot_solver;         //求解dJdq
    KDL::ChainIkSolverVel_pinv vel_solver;          //
    KDL::ChainIkSolverPos_LMA ik_pos_solver;    //计算期望关节位置
    KDL::ChainDynParam dynamin_solver;         //关节运动状态->关节力矩
    
    //计算数据缓存区
    KDL::JntSpaceInertiaMatrix M;
    KDL::JntArray C;
    KDL::JntArray G;
    KDL::Jacobian temp_jacobain;

    KDL::JntArray last_exp_joint_pos;

    KDL::JntArray _temp_joint3_array;
    KDL::JntArray _temp2_joint3_array;

    KDL::JntArrayVel _temp_joint3_vel_array;
    KDL::Twist _temp_jdot_qd;
};
