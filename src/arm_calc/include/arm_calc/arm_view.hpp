#pragma once

#include "arm_calc/arm_ctrl.hpp"
#include "arm_calc/arm_calc.hpp"
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

class Arm_view{

    public:

    explicit Arm_view(const rclcpp::Node::SharedPtr node,double dt_,KDL::Chain& chain);

    ~Arm_view();

    std::tuple<Vector6d, Vector6d, Vector6d> armtargetview(
      const Eigen::Vector3d &final_pos,const Vector6d &now_joint_pos);

    void initialize(const Vector6d &actual_pos);




    private:

    int log_counter = 0;
    std::unique_ptr<ArmStep> armstep;
    std::unique_ptr<ArmCalc> armcalc;
    rclcpp::Time start_time;
    rclcpp::Node::SharedPtr arm_node_;
    Vector6d last_target;
    bool first_read;
    double dt;
    double kp{9.0};
    double max_acc_{100.0};
    //Vector6d max_acc_{10.0,10.0,10.0,10.0,10.0,10.0};
    Eigen::Vector3d fix_rpy{0.0,0.0,0.0};
    Vector6d curr_pos;
    Vector6d prev_vel{0.0,0.0,0.0,0.0,0.0,0.0};
    Vector6d pre_pos{0.0,0.0,0.0,0.0,0.0,0.0};//需要对rpy写死，待调整
};