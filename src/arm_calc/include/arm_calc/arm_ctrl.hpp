#pragma once
#include "arm_calc/arm_ctrl.hpp"
#include "arm_calc/arm_joint_update.hpp"
#include "arm_calc/arm_space_update.hpp"
#include "arm_calc/arm_view.hpp"
#include "arm_calc/arm_calc.hpp"
#include "arm_calc/arm_step.h"
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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>



using Vector6d = Eigen::Matrix<double, 6, 1>;

class Arm_space;
class Arm_view;
class Arm_joint;
class ArmStep;


class ArmCalcNode {
public:
    ArmCalcNode(const rclcpp::Node::SharedPtr node);
    ~ArmCalcNode();

    int count{0};

    const double MAX_VEL = 1.0;    // rad/s
    const double MAX_TORQUE = 30.0; // N·m
   
    rclcpp::Node::SharedPtr get_node() const { return node_; };

    static constexpr double WHEEL_RADIUS = 0.065;

private:

    void show_callback();
   
    void arm_control();

    void task_manage();

    bool is_motion_reached();

    bool is_grasp_done();
    

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
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_ ;




    rclcpp::TimerBase::SharedPtr ui_update_timer;
    rclcpp::TimerBase::SharedPtr arm_update_timer;

    rclcpp::TimerBase::SharedPtr arm_control_timer;

    std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4","joint5","joint6"};

    // 解算部分
    KDL::Tree tree;
    std::string urdf_xml;
    KDL::Chain arm_chain;
    
   
    std::shared_ptr<ArmCalc> arm_calc;
    std::shared_ptr<Arm_joint> arm_joint_update;
    std::shared_ptr<Arm_space> arm_space_update;
    std::shared_ptr<Arm_view> arm_view_update;
    
    //Vector6d arm_exp_vel;
    Vector6d arm_exp_cart_rad{0.0,0.0,0.0,0.0,0.0,0.0};
    Vector6d arm_exp_cart_pos{0.0,0.0,0.0,0.0,0.0,0.0};
    Vector6d arm_exp_pos_{0.0,0.0,0.0,0.0,0.0,0.0};
    Vector6d arm_joint_pos{0.0,0.0,0.0,0.0,0.0,0.0};
    Vector6d arm_joint_omega{0.0,0.0,0.0,0.0,0.0,0.0};
    Vector6d arm_joint_torque{0.0,0.0,0.0,0.0,0.0,0.0};
    Vector6d joints_target_rad{0.0,0.0,0.0,0.0,0.0,0.0};
    Vector6d joints_target_omega{0.0,0.0,0.0,0.0,0.0,0.0};
    Vector6d joints_target_torque{0.0,0.0,0.0,0.0,0.0,0.0};
    Vector6d arm_initialization_rad{0.0, -0.2, -0.4, -0.4, 0.0, 0.0};


    Vector6d last_target_joint_pos{0.0,0.0,0.0,0.0,0.0,0.0};
    bool trajectory_active = false;
    bool target_change     = false;
    bool last_target_initialized = false;
    bool target_received = false;
    bool start_battle{false};
    
    uint32_t state_ = 0;  // 1: 吸附后执行关节轨迹规划  0: 笛卡尔空间规划   2：视觉伺服控制
    uint32_t plan_state = 0;
    int step = 0;
    int log_counter;
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

    int omega_log_counter;
    int torque_log_counter;
    int rad_log_counter;


    sensor_msgs::msg::JointState joint_display_msg;

    tf2::Quaternion robot_rotation;                    //机器人姿态
    geometry_msgs::msg::Twist robot_velocity;          //机器人速度信息
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;



    //启动过程
    bool arm_state_updated{false};
   
    int setup_stage{0};
    

    
    bool enable_posture_safe{true};
};
