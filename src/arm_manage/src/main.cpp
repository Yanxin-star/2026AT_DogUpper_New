#include "arm_manage.hpp"

int main(int argc,char**argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ArmManage>());
    rclcpp::shutdown();
    return 0;
}

