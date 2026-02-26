#include "arm_sernode.hpp"

int main(int argc,char**argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}

