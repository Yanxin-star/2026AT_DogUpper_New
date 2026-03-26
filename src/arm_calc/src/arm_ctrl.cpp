#include "arm_calc/arm_ctrl.hpp"
#include "arm_calc/arm_calc.hpp"
#include "arm_calc/arm_joint_update.hpp"
#include "arm_calc/arm_space_update.hpp"
#include "arm_calc/arm_step.h"
#include "arm_calc/arm_view.hpp"
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>




using namespace std::chrono_literals;
using Vector6d = Eigen::Matrix<double, 6, 1>;


ArmCalcNode::ArmCalcNode(const rclcpp::Node::SharedPtr node) {



    node_ = node;

    rviz_joint_publisher = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    arm_target_pub = node_->create_publisher<robot_interfaces::msg::Arm>("myjoints_target", 10); // 创建期望位置发布者

    // arm_state_sub =
    //     node_->create_subscription<robot_interfaces::msg::Arm>("myjoints_state", 10, [this](const robot_interfaces::msg::Arm& msg) {
    //         for (int i = 0; i < 6; i++) {
    //             arm_joint_pos[i]    = msg.motor[i].rad;
    //             arm_joint_omega[i]  = msg.motor[i].omega;
    //             arm_joint_torque[i] = msg.motor[i].torque;
    //         }


    //         if (!last_target_initialized) {
    //             last_target_joint_pos   = arm_joint_pos;
    //             last_target_initialized = true;

    //             RCLCPP_INFO(node_->get_logger(), "last_target_joint_pos 初始化完成");
    //         }

    //         arm_state_updated = true;
    //     });


    node_->declare_parameter("arm_state", 0);

    node_->declare_parameter("exp_x", 0.0);
    node_->declare_parameter("exp_y", 0.0);
    node_->declare_parameter("exp_z", 0.3);

    node_->declare_parameter("exp_roll", 0.0);
    node_->declare_parameter("exp_pitch", 0.0);
    node_->declare_parameter("exp_yaw", 0.0);

    param_cb_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {

            const std::string& name = param.get_name();



            if (name == "arm_state") {
                state_ = param.as_int();

            } else if (name == "exp_x") {
                arm_exp_cart_pos[0] = param.as_double();

            } else if (name == "exp_y") {
                arm_exp_cart_pos[1] = param.as_double();

            } else if (name == "exp_z") {
                arm_exp_cart_pos[2] = param.as_double();

            } else if (name == "exp_roll") {
                arm_exp_cart_pos[3] = param.as_double();

            } else if (name == "exp_pitch") {
                arm_exp_cart_pos[4] = param.as_double();

            } else if (name == "exp_yaw") {
                arm_exp_cart_pos[5] = param.as_double();
            }
        }

        return result;
    });


    node_->get_parameter("arm_state", state_);

    node_->get_parameter("exp_x", arm_exp_cart_pos[0]);
    node_->get_parameter("exp_y", arm_exp_cart_pos[1]);
    node_->get_parameter("exp_z", arm_exp_cart_pos[2]);

    node_->get_parameter("exp_roll", arm_exp_cart_pos[3]);
    node_->get_parameter("exp_pitch", arm_exp_cart_pos[4]);
    node_->get_parameter("exp_yaw", arm_exp_cart_pos[5]);

    arm_state_sub =
        node_->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, [this](const sensor_msgs::msg::JointState& msg) {
            for (int i = 0; i < 6; i++) {
                arm_joint_pos[i]    = joints_target_rad[i];
                arm_joint_omega[i]  = joints_target_omega[i];
                arm_joint_torque[i] = joints_target_torque[i];
            }

            if (!last_target_initialized) {
                last_target_joint_pos   = arm_joint_pos;
                last_target_initialized = true;

                RCLCPP_INFO(node_->get_logger(), "last_target_joint_pos 初始化完成");
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
    tree.getChain("base_link", "link6", arm_chain);



    // 初始化机械臂解算器
    arm_calc         = std::make_shared<ArmCalc>(arm_chain);
    arm_joint_update = std::make_shared<Arm_joint>(node_, arm_chain);
    arm_space_update = std::make_shared<Arm_space>(node_, arm_chain);
    arm_view_update  = std::make_shared<Arm_view>(node_, 0.01, arm_chain); // 0.01为函数循环周期


    joint_display_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    joint_display_msg.position.resize(6);



    // 相机坐标系转基坐标系
    //  tf_buffer   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    //  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);




    move_cmd_sub =
        node_->create_subscription<robot_interfaces::msg::Armcmd>("arm_move_cmd", 10, [this](const robot_interfaces::msg::Armcmd& msg) {
            arm_exp_pos_[0] = msg.x;
            arm_exp_pos_[1] = msg.y;
            arm_exp_pos_[2] = msg.z;
            arm_exp_pos_[3] = msg.roll;
            arm_exp_pos_[4] = msg.pitch;
            arm_exp_pos_[5] = msg.yaw;

            // 新增 6 个关节目标角度(rad)
            arm_exp_cart_rad[0] = msg.rad1;
            arm_exp_cart_rad[1] = msg.rad2;
            arm_exp_cart_rad[2] = msg.rad3;
            arm_exp_cart_rad[3] = msg.rad4;
            arm_exp_cart_rad[4] = msg.rad5;
            arm_exp_cart_rad[5] = msg.rad6;

            plan_state      = msg.mode; // 0:保持当前姿态      1:通过笛卡尔轨迹规划运动到目标位置
            // 2：通过视觉伺服控制运动到目标位置           吸取，保持当前位置不动                 4：通过纯关节控制运动到拿块位置
            // 5：通过控制运动到方块位置        6：释放块，保持当前位置不动
            //  Eigen::Vector3d cam_point = arm_exp_cart_pos.head(3);

            // geometry_msgs::msg::PointStamped cam_pt, base_pt;

            // cam_pt.header.frame_id = "camera_link";
            // cam_pt.header.stamp    = rclcpp::Time(0);

            // cam_pt.point.x = cam_point.x();
            // cam_pt.point.y = cam_point.y();
            // cam_pt.point.z = cam_point.z();

            // try {
            //     base_pt = tf_buffer->transform(cam_pt, "base_link");
            // } catch (tf2::TransformException &ex) {
            //     RCLCPP_WARN(node_->get_logger(), "TF失败: %s", ex.what());
            //     return;
            // }

            // Eigen::Vector3d base_pos;
            // base_pos <<  base_pt.point.x,
            //              base_pt.point.y,
            //              base_pt.point.z;

            // arm_exp_cart_pos.head(3) = base_pos;


            // RCLCPP_INFO(node_->get_logger(), "接受到新目标: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f, mode=%u",
            // msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.mode);
            target_change   = true;
            target_received = true;
        });

    ui_update_timer   = node_->create_wall_timer(50ms, std::bind(&ArmCalcNode::show_callback, this));
    arm_update_timer  = node_->create_wall_timer(10ms, std::bind(&ArmCalcNode::arm_control, this));
    //arm_control_timer = node_->create_wall_timer(10ms, std::bind(&ArmCalcNode::task_manage, this));
    RCLCPP_INFO(node_->get_logger(), "初始化完成");
}

ArmCalcNode::~ArmCalcNode() {}
void ArmCalcNode::task_manage() {

    if (!arm_state_updated) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "机械臂状态未更新");
        return;
    }

    else {
        if (plan_state == 0) { // 该状态为待机状态

            state_ = 0;
            arm_control();
        }

        if (plan_state == 1) { // 该状态为去取块

            if (step == 0) {   // 首先应该让机械臂转到合适的待抓块位置
                arm_exp_cart_rad = arm_initialization_rad;
                state_           = 2;
                arm_control();
                if (is_motion_reached()) {
                    step = 1;
                    // RCLCPP_INFO(node_->get_logger(), "✅ 已到达初始化位，开始笛卡尔接近");
                }
            } else if (step == 1) { // 切换到笛卡尔轨迹规划，去接近物块
                arm_exp_cart_pos = arm_exp_pos_;
                state_           = 1;
                arm_control();
                if (is_motion_reached()) {
                    step = 2;
                    // RCLCPP_INFO(node_->get_logger(), "✅ 已到达初始化位，开始笛卡尔接近");
                } else if (step == 2) { // 切换到视觉伺服模式，去抓取物块
                    arm_exp_cart_pos = arm_exp_pos_;
                    state_           = 2;
                    arm_control();
                    RCLCPP_INFO(node_->get_logger(), "去接近抓取物块");
                    if (is_motion_reached()) {
                        step = 3;
                    }
                } else if (step == 3) { // 抓取到物块，回到机械臂合适等待位置
                    RCLCPP_INFO(node_->get_logger(), "抓取到物块");
                    arm_exp_cart_rad = arm_initialization_rad;
                    state_           = 2;
                    arm_control();
                    if (is_motion_reached()) {
                        step = 4;
                    }
                } else if (step == 4) {
                    step   = 0;
                    state_ = 0;
                }
            }
            if (plan_state == 2) {      // 该状态为去放块

                state_ = 3;
            }
        }
    }
}
void ArmCalcNode::arm_control() {

    if (log_counter > 200) {
        Vector6d arm_now_pos = arm_calc->foot_pos(arm_joint_pos);
        RCLCPP_INFO(
            node_->get_logger(), "当前笛卡尔位置：%f %f %f %f %f %f", arm_now_pos[0], arm_now_pos[1], arm_now_pos[2], arm_now_pos[3],
            arm_now_pos[4], arm_now_pos[5]);

        RCLCPP_INFO(
            node_->get_logger(), "当前关节位置：%f %f %f %f %f %f", arm_joint_pos[0], arm_joint_pos[1], arm_joint_pos[2], arm_joint_pos[3],
            arm_joint_pos[4], arm_joint_pos[5]);
        log_counter = 0;
    }
    log_counter++;

    if (state_ == 0) {
        robot_interfaces::msg::Arm joints_target_;

        for (int i = 0; i < 6; i++) {

            joints_target_.motor[i].rad = joints_target_rad[i];
            // ✅ 角速度限幅
            joints_target_.motor[i].omega = std::clamp(joints_target_omega[i], -MAX_VEL, MAX_VEL);

            // ✅ 力矩限幅
            joints_target_.motor[i].torque = std::clamp(joints_target_torque[i], -MAX_TORQUE, MAX_TORQUE);

            arm_target_pub->publish(joints_target_);
        }
    } else if (state_ == 1) { // 笛卡尔空间轨迹规划状态

        robot_interfaces::msg::Arm joints_target_0;

        std::tie(joints_target_rad, joints_target_omega, joints_target_torque) =
            arm_space_update->armtargetUpdate(arm_exp_cart_pos, arm_joint_pos, arm_joint_omega);

        for (int i = 0; i < 6; i++) {

            joints_target_0.motor[i].rad = joints_target_rad[i];
            // ✅ 角速度限幅
            joints_target_0.motor[i].omega = std::clamp(joints_target_omega[i], -MAX_VEL, MAX_VEL);

            // ✅ 力矩限幅
            joints_target_0.motor[i].torque = std::clamp(joints_target_torque[i], -MAX_TORQUE, MAX_TORQUE);
        }


        arm_target_pub->publish(joints_target_0);


    } else if (state_ == 2) { // 纯关节轨迹规划状态

        robot_interfaces::msg::Arm joints_target_1;

        std::tie(joints_target_rad, joints_target_omega, joints_target_torque) =
            arm_joint_update->targetUpdate(arm_exp_cart_rad, arm_joint_pos, arm_joint_omega);

        for (int i = 0; i < 6; i++) {

            joints_target_1.motor[i].rad = joints_target_rad[i];
            // ✅ 角速度限幅
            joints_target_1.motor[i].omega = std::clamp(joints_target_omega[i], -MAX_VEL, MAX_VEL);

            // ✅ 力矩限幅
            joints_target_1.motor[i].torque = std::clamp(joints_target_torque[i], -MAX_TORQUE, MAX_TORQUE);
        }


        arm_target_pub->publish(joints_target_1);


    } else if (state_ == 3) { // 视觉伺服控制状态

        robot_interfaces::msg::Arm joints_target_2;



        Eigen::Vector3d final_pos = arm_exp_cart_pos.head(3);

        std::tie(joints_target_rad, joints_target_omega, joints_target_torque) = arm_view_update->armtargetview(final_pos, arm_joint_pos);


        for (int i = 0; i < 6; i++) {

            joints_target_2.motor[i].rad = joints_target_rad[i];
            // ✅ 角速度限幅
            joints_target_2.motor[i].omega = std::clamp(joints_target_omega[i], -MAX_VEL, MAX_VEL);

            // ✅ 力矩限幅
            joints_target_2.motor[i].torque = std::clamp(joints_target_torque[i], -MAX_TORQUE, MAX_TORQUE);
        }


        arm_target_pub->publish(joints_target_2);
    }
}

bool ArmCalcNode::is_motion_reached() {
    if (state_ == 2) {                                             // 关节模式
        double joint_err = (arm_joint_pos - arm_exp_cart_rad).norm();
        return joint_err < 0.015 && arm_joint_omega.norm() < 0.03; // 建议 0.015rad ≈ 0.86°
    } else {                                                       // Cartesian / 视觉
        Vector6d now    = arm_calc->foot_pos(arm_joint_pos);
        Vector6d target = arm_exp_cart_pos;
        double pos_err  = (now.head<3>() - target.head<3>()).norm();
        // double ori_err = ... 可选
        return pos_err < 0.008 && arm_joint_omega.norm() < 0.03;
    }
}

// 视觉伺服“是否抓完”（视觉只发一次指令，只能用这个最小判断）
bool ArmCalcNode::is_grasp_done() {
    static rclcpp::Time start_time = rclcpp::Time(0); // 第一次进入时初始化
    static Vector6d last_pose      = Vector6d::Zero();

    if (start_time.seconds() == 0) {                  // 第一次进入视觉伺服
        start_time = node_->now();
        last_pose  = arm_calc->foot_pos(arm_joint_pos);
        return false;
    }

    double elapsed = (node_->now() - start_time).seconds();
    Vector6d now   = arm_calc->foot_pos(arm_joint_pos);
    double move    = (now - last_pose).norm();

    // 两种情况之一就认为抓取结束：
    // 1. 超时6秒（防止卡死）
    // 2. 位置已经很稳定1.2秒（认为夹住了）
    if (elapsed > 6.0 || (elapsed > 1.2 && move < 0.003)) {
        start_time = rclcpp::Time(0); // 重置，供下次使用
        return true;
    }
    last_pose = now;
    return false;
}
void ArmCalcNode::show_callback() {


    joint_display_msg.header.stamp = node_->get_clock()->now();

    joint_display_msg.header.frame_id = "base_link";

    joint_display_msg.name = {
        "joint1", "joint2", "joint3", "joint4",
        "joint5", "joint6"

    };

    /*
    joint_display_msg.position[0] = arm_joint_pos[0];
    joint_display_msg.position[1] = arm_joint_pos[1];
    joint_display_msg.position[2] = arm_joint_pos[2];
    joint_display_msg.position[3] = arm_joint_pos[3];
    joint_display_msg.position[4] = arm_joint_pos[4];
    joint_display_msg.position[5] = arm_joint_pos[5];
    */

    for (int i = 0; i < 6; i++) {
        joint_display_msg.position[i] = joints_target_rad[i];
    }

    // RCLCPP_INFO(node_->get_logger(), "接受到新目标:%f   %f   %f   %f  %f   %f",
    // joints_target_rad[0],joints_target_rad[1],joints_target_rad[2],joints_target_rad[3],joints_target_rad[4],joints_target_rad[5]);

    joint_display_msg.header.stamp = node_->get_clock()->now();
    rviz_joint_publisher->publish(joint_display_msg);
}
