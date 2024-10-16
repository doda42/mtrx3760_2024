#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

class MotionControlNode : public rclcpp::Node
{
    public:
        MotionControlNode()
            : Node("motion_control_node")
        {
            subscriber = create_subscription<geometry_msgs::msg::Pose>("/flight/waypoints", 10,
                         std::bind(&MotionControlNode::callback, this, std::placeholders::_1));

            RCLCPP_INFO(get_logger(), "Motion Controller active");
        }

    private:
        void callback(const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            RCLCPP_INFO(get_logger(), "Received x: %f, y: %f", msg->position.x, msg->position.y);
            // Some actual motion control code
        }

        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MotionControlNode> node = std::make_shared<MotionControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

