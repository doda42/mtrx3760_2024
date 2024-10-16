#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose.hpp"

class MotionControlNode : public rclcpp::Node {
    public:
        MotionControlNode() 
            : Node("motion_control_node")
        {
            declare_parameter<int>("speed", 4);
            speed = get_parameter("speed").as_int();
            subscriber = create_subscription<geometry_msgs::msg::Pose>("/flight/waypoints", 10,
                         std::bind(&MotionControlNode::newPoseCallback, this, std::placeholders::_1));

            RCLCPP_INFO(get_logger(), "Motion Controller active");
            RCLCPP_INFO(this->get_logger(), "Initial speed: %d", speed);

            paramEventHandler = std::make_shared<rclcpp::ParameterEventHandler>(this);
            paramCallbackHandle = paramEventHandler->add_parameter_callback("speed",
                std::bind(&MotionControlNode::paramCallback, this, std::placeholders::_1));
        }

    private:
        void newPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            RCLCPP_INFO(get_logger(), "Received x: %f, y: %f", msg->position.x, msg->position.y);
            RCLCPP_INFO(get_logger(), "Flying at %dm/s", speed);
            // Some actual motion control code
        }

        void paramCallback(const rclcpp::Parameter& param)
        {
            speed = get_parameter("speed").as_int();
            RCLCPP_INFO(get_logger(), "Parameter '%s' changed to '%ld'", param.get_name().c_str(), param.as_int());
        }

        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber;
        std::shared_ptr<rclcpp::ParameterEventHandler> paramEventHandler;
        rclcpp::ParameterCallbackHandle::SharedPtr paramCallbackHandle;
        int speed;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
