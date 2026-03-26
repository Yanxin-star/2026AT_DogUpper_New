#ifndef __ARM_MANAGE_HPP__
#define __ARM_MANAGE_HPP__

#include <memory>
#include <robot_interfaces/msg/arm.hpp>
#include <thread>
#include "sensor_msgs/msg/imu.hpp"
#include <robot_interfaces/msg/move_cmd.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <ctime>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_interfaces/msg/armcmd.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tuple>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>




using Vector6d = Eigen::Matrix<double, 6, 1>;

class ArmManage : public rclcpp::Node
{
public:
    ArmManage();
    ~ArmManage();
private:
   Vector6d arm_exp_cart_pos;
   rclcpp::Subscription<robot_interfaces::msg::Armcmd>::SharedPtr move_cmd_sub;
   rclcpp::Publisher<robot_interfaces::msg::Arm>::SharedPtr arm_target_pub;
   uint32_t plan_state;

};

#endif
