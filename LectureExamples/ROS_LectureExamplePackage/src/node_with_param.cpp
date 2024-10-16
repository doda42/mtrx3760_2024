#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

class MotionControlNode : public rclcpp::Node
{
    public:
        MotionControlNode()
            : Node("motion_control_node")
        {
            declare_parameter<int>("speed", 4); // declare and set default value
            subscriber = create_subscription<geometry_msgs::msg::Pose>("/flight/waypoints", 10,
                         std::bind(&MotionControlNode::callback, this, std::placeholders::_1));

            RCLCPP_INFO(get_logger(), "Motion Controller with Parameter active");
        }

    private:
        void callback(const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            speed = get_parameter("speed").as_int();
            RCLCPP_INFO(get_logger(), "Received x: %f, y: %f", msg->position.x, msg->position.y);
            RCLCPP_INFO(get_logger(), "Flying at %dm/s", speed);
            // Some actual motion control code
        }

        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber;
        int speed;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MotionControlNode> node = std::make_shared<MotionControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

