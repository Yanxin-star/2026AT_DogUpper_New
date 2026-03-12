#pragma once

#include "arm_calc.hpp"
#include "arm_step.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <ctime>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_interfaces/msg/arm.hpp>
#include <robot_interfaces/msg/armcmd.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>



class ArmCalcNode {
public:
    ArmCalcNode(const rclcpp::Node::SharedPtr node);
    ~ArmCalcNode();

    int count{0};
   

    static constexpr double WHEEL_RADIUS = 0.065;

private:

    void show_callback();
    void arm_update();
    void arm_control();
    static double ramp_control(double target, double current, double ramp);
    
   /*
    std::tuple<Vector3D, Vector3D, Vector3D> signal_leg_calc(
        const Vector3D& exp_cart_pos, const Vector3D& exp_cart_vel, const Vector3D& exp_cart_acc, const Vector3D& exp_cart_force,
        std::shared_ptr<LegCalc> leg_calc);
    */
    Eigen::Vector4d signal_arm_calc(const Vector3D& exp_cart_pos,double exp_yaw,std::shared_ptr<ArmCalc> arm_calc);
    

    static void quaternionLowPassFilter(double& w,  double& x,  double& y,  double& z,double  w1, double  x1, double  y1, double  z1,double alpha);
    Vector3D get_grivate_center_pose(const Vector3D &lf_joint_pos,const Vector3D &rf_joint_pos,const Vector3D &lb_joint_pos,const Vector3D &rb_joint_pos);
    
    

    rclcpp::Node::SharedPtr node_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;


    double direction_filter_gate{0.8};
    
   
    
    rclcpp::Publisher<robot_interfaces::msg::Arm>::SharedPtr arm_target_pub;
    //rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr arm_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_state_sub;
     rclcpp::Subscription<robot_interfaces::msg::Armcmd>::SharedPtr move_cmd_sub;
    
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_angular_vel_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_publisher;
    rclcpp::SyncParametersClient::SharedPtr arm_description_param_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> robot_tf_broadcaster;




    rclcpp::TimerBase::SharedPtr ui_update_timer;
    rclcpp::TimerBase::SharedPtr arm_update_timer;

    rclcpp::TimerBase::SharedPtr arm_control_timer;

    std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4"};

    // 解算部分
    KDL::Tree tree;
    std::string urdf_xml;
    KDL::Chain arm_chain;
    Eigen::Isometry3d T_link4_link5;
    Eigen::Matrix3d R;
   
    std::shared_ptr<ArmCalc> arm_calc;
    Eigen::Vector4d target_joint_pos;
    Eigen::Vector3d joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4;
    Eigen::Vector4d arm_joint_pos;
    Eigen::Vector3d arm_exp_cart_pos{0.0,0.0,0.0};
    double arm_exp_yaw{0.0};

    Eigen::Vector4d last_target_joint_pos;
    bool trajectory_active = false;
    bool target_change     = false;
    bool last_target_initialized = false;
    bool target_received = false;
    uint32_t adsorb_state = 0;  // 1: 吸附上  0: 不吸附   释放中
    double trajectory_duration = 4.0;

    rclcpp::Time start_time;

    int rviz2_update_cnt{0};




    
    
    // 步态规划部分
    Vector2D lf_exp_vel,rf_exp_vel,lb_exp_vel,rb_exp_vel;
    ArmStep arm_step;

    Eigen::Vector3d lf_joint_pos, lf_joint_vel;
    Eigen::Vector3d rf_joint_pos, rf_joint_vel;
    Eigen::Vector3d lb_joint_pos, lb_joint_vel;
    Eigen::Vector3d rb_joint_pos, rb_joint_vel;
    
    double roll_offset_virtual_torque{0.0};
    double pitch_offset_virtual_torque{0.0};
    double roll_balance_force_compen{0.0};
    double pitch_balance_force_compen{0.0};
    double roll_balance_step_compen{0.0};
    double pitch_balance_step_compen{0.0};

    


    sensor_msgs::msg::JointState joint_display_msg;

    tf2::Quaternion robot_rotation;                    //机器人姿态
    geometry_msgs::msg::Twist robot_velocity;          //机器人速度信息



    //启动过程
    bool arm_state_updated{false};
   
    int setup_stage{0};
    

    
    bool enable_posture_safe{true};
};
