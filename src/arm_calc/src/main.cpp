#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "arm_calc/arm_ctrl.hpp"


int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("arm_calc_node");
    auto robot_calc=std::make_shared<ArmCalcNode>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
