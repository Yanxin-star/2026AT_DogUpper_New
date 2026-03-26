#include <arm_manage.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/logging.hpp>
#include <robot_interfaces/msg/arm.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <algorithm>
#include <boost/stacktrace.hpp>
#include <cstdlib>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/duration.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tuple>




using namespace std::chrono_literals;
using Vector6d = Eigen::Matrix<double, 6, 1>;


ArmManage::ArmManage()
    : Node("arm_manage") {   

        


        move_cmd_sub =
        this->create_subscription<robot_interfaces::msg::Armcmd>
        ("arm_move_cmd", 10, [this](const robot_interfaces::msg::Armcmd::SharedPtr msg) {
               
        arm_exp_cart_pos[0] = msg->x;
        arm_exp_cart_pos[1] = msg->y;
        arm_exp_cart_pos[2] = msg->z;
        arm_exp_cart_pos[3] = msg->roll;
        arm_exp_cart_pos[4] = msg->pitch;
        arm_exp_cart_pos[5] = msg->yaw;
        plan_state          = msg->mode;//0:保持当前姿态      1:通过笛卡尔轨迹规划运动到目标位置             
        //2：通过视觉伺服控制运动到目标位置           吸取，保持当前位置不动                 4：通过纯关节控制运动到拿块位置
        //5：通过控制运动到方块位置        6：释放块，保持当前位置不动
        });
}




ArmManage::~ArmManage() {
    
}

