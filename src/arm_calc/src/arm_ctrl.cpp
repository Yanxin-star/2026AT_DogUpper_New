#include "arm_calc/arm_ctrl.hpp"
#include "arm_calc/arm_step.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <algorithm>
#include <boost/stacktrace.hpp>
#include <chrono>
#include <cstdlib>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/frames.hpp>
#include <rclcpp/duration.hpp>
#include <robot_interfaces/msg/arm.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

ArmCalcNode::ArmCalcNode(const rclcpp::Node::SharedPtr node)
     {

    node_    = node;
    
    rviz_joint_publisher = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    arm_target_pub = node_->create_publisher<robot_interfaces::msg::Arm>("arm_target", 10); // 创建期望位置发布者

    arm_state_sub =
        node_->create_subscription<robot_interfaces::msg::Arm>("arm_status", 10, [this](const robot_interfaces::msg::Arm& msg) {
           //这里最终应该是舵机的rad值
                arm_joint_pos[3] = (double)msg.servo2.up;
                arm_joint_pos[2] = (double)msg.servo2.low;
                arm_joint_pos[1] = (double)msg.rob01.rad;
                arm_joint_pos[0] = (double)msg.rob02.rad;
                
            if (!last_target_initialized)
            {
                last_target_joint_pos = arm_joint_pos;
                last_target_initialized = true;

                RCLCPP_INFO(
                    node_->get_logger(),
                    "last_target_joint_pos 初始化完成");
            }
            
            

            if (!arm_state_updated ) {
                arm_state_updated = true;
                if(count>=5000){
                    RCLCPP_INFO(node_->get_logger(), "机械臂状态首次更新");
                    count=0;
                }
                
            }
            count++;
        });

    // 订阅机器人的运动期望
   

    arm_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "/robot_state_publisher");

    auto params = arm_description_param_->get_parameters({"robot_description"});
    urdf_xml    = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return;
    }

    robot_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    kdl_parser::treeFromString(urdf_xml, tree); // 解析机械臂的KDL树结构
    tree.getChain("base_link", "link4", arm_chain);
    

    // 初始化机械臂解算器
    arm_calc = std::make_shared<ArmCalc>(arm_chain);


    joint_display_msg.name = {"joint1", "joint2", "joint3", "joint4"};
    joint_display_msg.position.resize(4);

    move_cmd_sub =
        node_->create_subscription<robot_interfaces::msg::Armcmd>("arm_move_cmd", 10, [this](const robot_interfaces::msg::Armcmd& msg) {
               
        arm_exp_cart_pos[0] = msg.x;
        arm_exp_cart_pos[1] = msg.y;
        arm_exp_cart_pos[2] = msg.z;
        arm_exp_yaw         = msg.yaw;
        RCLCPP_INFO(node_->get_logger(), "接受到新目标: x=%f, y=%f, z=%f, yaw=%f", msg.x, msg.y, msg.z, msg.yaw);
        target_change = true;
        target_received = true; 
        });

    ui_update_timer  = node_->create_wall_timer(50ms, std::bind(&ArmCalcNode::show_callback, this));
    arm_update_timer = node_->create_wall_timer(10ms, std::bind(&ArmCalcNode::arm_update, this));

    RCLCPP_INFO(node_->get_logger(), "初始化完成");
}

ArmCalcNode::~ArmCalcNode() {}


void ArmCalcNode::show_callback() {


    joint_display_msg.position[0] = arm_joint_pos[0];
    joint_display_msg.position[1] = arm_joint_pos[1];
    joint_display_msg.position[2] = arm_joint_pos[2];
    joint_display_msg.position[3] = arm_joint_pos[3];

    joint_display_msg.header.stamp = node_->get_clock()->now();
    rviz_joint_publisher->publish(joint_display_msg);

    // RCLCPP_INFO(node_->get_logger(), "roll:%lf,pitch=%lf", roll_offset_virtual_torque, pitch_offset_virtual_torque);

    // RCLCPP_INFO(node_->get_logger(), "kp=%lf,kd=%lf,mass=%lf", vmc->kp, vmc->kd, vmc->mass);
}


Eigen::Vector4d ArmCalcNode::signal_arm_calc(const Vector3D& exp_cart_pos,double exp_yaw,std::shared_ptr<ArmCalc> arm_calc)
{
    Vector4D joint_pos;

    int result;

    // 调用逆运动学
    joint_pos = arm_calc->joint_pos(
        exp_cart_pos,
        exp_yaw,
        &result);

    // 错误处理
    if (result != 0)
    {
        RCLCPP_ERROR(node_->get_logger(),
            "逆运动学失败:(%f,%f,%f,%f)",
            exp_cart_pos[0],
            exp_cart_pos[1],
            exp_cart_pos[2],
            exp_yaw);
    }

    return joint_pos;
}



void ArmCalcNode::arm_update()
{
    if (!arm_state_updated || !last_target_initialized)
    {
        RCLCPP_INFO(node_->get_logger(), "等待首次机械臂状态更新中...");
        return;
    }

     if (!target_received)
    {
        robot_interfaces::msg::Arm joints_target;

        joints_target.servo2.up  = arm_joint_pos[3];
        joints_target.servo2.low = arm_joint_pos[2];
        joints_target.rob01.rad  = arm_joint_pos[1];
        joints_target.rob02.rad  = arm_joint_pos[0];

        arm_target_pub->publish(joints_target);

        return;
    }

    auto now = node_->get_clock()->now();
    Eigen::Vector4d current_target;
    // 计算新的目标关节角
    Eigen::Vector4d new_target_joint_pos =
        signal_arm_calc(
            arm_exp_cart_pos,
            arm_exp_yaw,
            arm_calc);

    // 判断目标是否改变
    double diff =
        (new_target_joint_pos - last_target_joint_pos).norm();

    // 如果改变，重新规划轨迹
    if (target_change || !trajectory_active)
    {
        RCLCPP_INFO(node_->get_logger(),
            "新轨迹规划: diff=%f",
            diff);

        arm_step.update_arm_trajectory(
            arm_joint_pos,
            new_target_joint_pos,
            trajectory_duration);

        start_time = now;

        last_target_joint_pos = new_target_joint_pos;

        trajectory_active = true;
        target_change     = false;
    }

    // 计算轨迹时间
    double t = (now - start_time).seconds();

    if (t > trajectory_duration)
        t = trajectory_duration;

    if (t >= trajectory_duration)
{
    trajectory_active = false;
}

    // 获取轨迹点
    arm_step.get_arm_pose(t, current_target);

    // 发布目标
    robot_interfaces::msg::Arm joints_target;

    joints_target.servo2.up =
        current_target[3];

    joints_target.servo2.low =
        current_target[2];

    joints_target.rob01.rad =
        current_target[1];

    joints_target.rob02.rad =
        current_target[0];

    arm_target_pub->publish(joints_target);
}