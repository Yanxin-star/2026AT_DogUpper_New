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
   
    arm_state_sub = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        10,
        [this](const sensor_msgs::msg::JointState& msg)
        {
            arm_joint_pos[0] = msg.position[0];
            arm_joint_pos[1] = msg.position[1];
            arm_joint_pos[2] = msg.position[2];
            arm_joint_pos[3] = msg.position[3];

            if (!last_target_initialized)
            {
                last_target_joint_pos = arm_joint_pos;
                last_target_initialized = true;

                RCLCPP_INFO(node_->get_logger(),
                    "last_target_joint_pos 初始化完成");
            }

            arm_state_updated = true;
        });

    arm_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "/robot_state_publisher");

    auto params = arm_description_param_->get_parameters({"robot_description"});
    urdf_xml    = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return;
    }

    robot_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    kdl_parser::treeFromString(urdf_xml, tree); // 解析机械臂的KDL树结构
    tree.getChain("base_link", "link5", arm_chain);
    

    // 初始化机械臂解算器
    arm_calc = std::make_shared<ArmCalc>(arm_chain);

    //机械臂吸盘相对末端的偏移（需要根据实际机械臂设计调整）
    T_link4_link5.setIdentity();

    T_link4_link5.translate(
    Eigen::Vector3d(-0.033, -0.0904, 0.02425));

    R = Eigen::AngleAxisd(-1.5708, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitZ());

    T_link4_link5.rotate(R);



    joint_display_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};
    joint_display_msg.position.resize(5);

    move_cmd_sub =
        node_->create_subscription<robot_interfaces::msg::Armcmd>("arm_move_cmd", 10, [this](const robot_interfaces::msg::Armcmd& msg) {
               
        arm_exp_cart_pos[0] = msg.x;
        arm_exp_cart_pos[1] = msg.y;
        arm_exp_cart_pos[2] = msg.z;
        arm_exp_yaw         = msg.yaw;//朝向已写死,目前无法控制pitch
        adsorb_state            =msg.mode;
        RCLCPP_INFO(node_->get_logger(), "接受到新目标: x=%f, y=%f, z=%f, yaw=%f, mode=%u", msg.x, msg.y, msg.z, msg.yaw, msg.mode);
        target_change = true;
        target_received = true; 
        });

    ui_update_timer  = node_->create_wall_timer(50ms, std::bind(&ArmCalcNode::show_callback, this));
    //arm_update_timer = node_->create_wall_timer(10ms, std::bind(&ArmCalcNode::arm_update, this));
    arm_control_timer = node_->create_wall_timer(10ms, std::bind(&ArmCalcNode::arm_control, this));
    RCLCPP_INFO(node_->get_logger(), "初始化完成");
}

ArmCalcNode::~ArmCalcNode() {}


void ArmCalcNode::show_callback() {


    joint_display_msg.header.stamp =
        node_->get_clock()->now();

    joint_display_msg.header.frame_id =
        "base_link";

    joint_display_msg.name =
    {
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5"
    };



    joint_display_msg.position[0] = arm_joint_pos[0];
    joint_display_msg.position[1] = arm_joint_pos[1];
    joint_display_msg.position[2] = arm_joint_pos[2];
    joint_display_msg.position[3] = arm_joint_pos[3];


    joint_display_msg.header.stamp = node_->get_clock()->now();
    rviz_joint_publisher->publish(joint_display_msg);

    
}


Eigen::Vector4d ArmCalcNode::signal_arm_calc(
    const Vector3D& exp_cart_pos,
    double exp_yaw,
    std::shared_ptr<ArmCalc> arm_calc)
{
    Vector4D joint_pos;
    int result;

    // 计算末端在平面上的朝向角
    double base_yaw = atan2(
        exp_cart_pos[1],
        exp_cart_pos[0]);

    //计算目标垂直到平面的点到原点的距离
    double r = sqrt(
        exp_cart_pos[0]*exp_cart_pos[0] +
        exp_cart_pos[1]*exp_cart_pos[1]);

    Vector3D planar_pos;

    planar_pos[0] = r;
    planar_pos[1] = 0;
    planar_pos[2] = exp_cart_pos[2];

    joint_pos = arm_calc->joint_pos(
        planar_pos,
        exp_yaw,
        &result);

    joint_pos[0] = base_yaw;

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

    arm_joint_pos = current_target;

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

    static int print_count = 0;
    print_count++;

    if(print_count >= 10)
    {
        RCLCPP_INFO(node_->get_logger(),
        "Current rad: %f %f %f %f",
            arm_joint_pos[0],
            arm_joint_pos[1],
            arm_joint_pos[2],
            arm_joint_pos[3]);

        print_count = 0;
    }

    arm_target_pub->publish(joints_target);
}


void ArmCalcNode::arm_control()
{

   if(adsorb_state == 0){
        
    arm_update();
        
    }else if(adsorb_state == 1){
        
    Eigen::Vector4d current_pos = arm_joint_pos;
    Eigen::Vector4d target_pos(0.0, 0.0, 0.0, 0.9);

    RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    1000,
    "吸附到块，正在回归初始位置");
        
    robot_interfaces::msg::Arm joints_target;

   current_pos[3] = ramp_control(target_pos[3], current_pos[3], 0.004);
   current_pos[2] = ramp_control(target_pos[2], current_pos[2], 0.004);
   current_pos[1] = ramp_control(target_pos[1], current_pos[1], 0.004);
   current_pos[0] = ramp_control(target_pos[0], current_pos[0], 0.004);

    arm_joint_pos = current_pos;

    joints_target.servo2.up  = current_pos[3];
    joints_target.servo2.low = current_pos[2];
    joints_target.rob01.rad  = current_pos[1];
    joints_target.rob02.rad  = current_pos[0];
        
    arm_target_pub->publish(joints_target);
        

    }

 

}

double ArmCalcNode::ramp_control(double target, double current, double ramp)
{
    double diff = target - current;

    if (diff > ramp)
    {
        current += ramp;
    }
    else if (diff < -ramp)
    {
        current -= ramp;
    }
    else
    {
        current = target;
    }

    return current;
}
