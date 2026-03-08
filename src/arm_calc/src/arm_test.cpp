#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_interfaces/msg/armcmd.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class ArmMoveNode : public rclcpp::Node{
public:
    ArmMoveNode():  Node("arm_move_node")
    {
        // declare parameters so rqt param plugin can modify them
        auto x_descriptor = rcl_interfaces::msg::ParameterDescriptor();
        x_descriptor.description = "Position in X direction";
        x_descriptor.floating_point_range.resize(1);
        x_descriptor.floating_point_range[0].from_value = -10.0;
        x_descriptor.floating_point_range[0].to_value = 10.0;
        this->declare_parameter<double>("x", 0.0, x_descriptor);

        auto y_descriptor = rcl_interfaces::msg::ParameterDescriptor();
        y_descriptor.description = "Position in Y direction";
        y_descriptor.floating_point_range.resize(1);
        y_descriptor.floating_point_range[0].from_value = -10.0;
        y_descriptor.floating_point_range[0].to_value = 10.0;
        this->declare_parameter<double>("y", 0.0, y_descriptor);

        auto z_descriptor = rcl_interfaces::msg::ParameterDescriptor();
        z_descriptor.description = "Position in Z direction";
        z_descriptor.floating_point_range.resize(1);
        z_descriptor.floating_point_range[0].from_value = -10.0;
        z_descriptor.floating_point_range[0].to_value = 10.0;
        this->declare_parameter<double>("z", 0.0, z_descriptor);

        auto yaw_descriptor = rcl_interfaces::msg::ParameterDescriptor();
        yaw_descriptor.description = "Rotation around Z axis in radians";
        yaw_descriptor.floating_point_range.resize(1);
        yaw_descriptor.floating_point_range[0].from_value = -3.14159;
        yaw_descriptor.floating_point_range[0].to_value = 3.14159;
        this->declare_parameter<double>("yaw", 0.0, yaw_descriptor);

        this->declare_parameter<int>("step_type", 0);

        // create publisher
        pub_ = this->create_publisher<robot_interfaces::msg::Armcmd>("arm_move_cmd", 10);

        // set up parameter change callback: publish a new MoveCmd whenever parameters change
        param_cb_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params){
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto & p : params) {
                    if (p.get_name() == "x") {
                        if (p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            x_ = static_cast<float>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "x must be a number";
                            return result;
                        }
                    } else if (p.get_name() == "y") {
                        if (p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            y_ = static_cast<float>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "y must be a number";
                            return result;
                        }
                    } else if (p.get_name() == "z") {
                        if (p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            z_ = static_cast<float>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "z must be a number";
                            return result;
                        }
                    } else if (p.get_name() == "yaw") {
                        if (p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            yaw_ = static_cast<float>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "yaw must be a number";
                            return result;
                        }
                    }
                    else if (p.get_name() == "step_type") {
                        if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
                            step_type_ = static_cast<uint32_t>(p.as_int());
                        } else if (p.get_type() == rclcpp::PARAMETER_DOUBLE) {
                            // allow integers provided as double
                            step_type_ = static_cast<uint32_t>(p.as_double());
                        } else {
                            result.successful = false;
                            result.reason = "step_type must be an integer";
                            return result;
                        }
                    }
                    
                }
                return result;
            }
        );

        // publish initial message based on default parameters
        x_ = static_cast<float>(this->get_parameter("x").as_double());
        y_ = static_cast<float>(this->get_parameter("y").as_double());
        z_ = static_cast<float>(this->get_parameter("z").as_double());
        yaw_ = static_cast<float>(this->get_parameter("yaw").as_double());
        step_type_ = static_cast<uint32_t>(this->get_parameter("step_type").as_int()); 
        publish_move_cmd();

        update_timer=this->create_wall_timer(100ms ,[this](){
            publish_move_cmd();
        });
    }

private:
    void publish_move_cmd()
    {
        robot_interfaces::msg::Armcmd msg;
        msg.x = x_;
        msg.y = y_;
        msg.z = z_;
        msg.yaw = yaw_;
        msg.mode = step_type_;
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published Armcmd: x=%.2f y=%.2f z=%.2f yaw=%.3f step_type=%u",
                    x_, y_, z_, yaw_, step_type_);
    }

    rclcpp::Publisher<robot_interfaces::msg::Armcmd>::SharedPtr pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rclcpp::TimerBase::SharedPtr update_timer;

    float x_{0.0f}, y_{0.0f}, z_{0.0f},yaw_{0.0f};
     uint32_t step_type_{0};

};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ArmMoveNode>());
    rclcpp::shutdown();
    return 0;
}