#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

class AutoPilotNode : public rclcpp::Node
{
    public:
        AutoPilotNode()
            : Node("autopilot_node")
        {
            publisher = create_publisher<geometry_msgs::msg::Pose>("/flight/waypoints", 10);
            timer = create_wall_timer(std::chrono::milliseconds(1000),
                    std::bind(&AutoPilotNode::PublishWaypoints, this));
            RCLCPP_INFO(get_logger(), "Autopilot has been started");
        }

    private:
        void PublishWaypoints()
        {
            geometry_msgs::msg::Pose msg = geometry_msgs::msg::Pose();
            msg.position.x = 1.0;
            msg.position.y = 2.0;
            msg.orientation.w = 1.0;
            RCLCPP_INFO(get_logger(), "Sending pose x: %f y: %f", msg.position.x, msg.position.y);
            publisher->publish(msg);
        }
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<AutoPilotNode> node = std::make_shared<AutoPilotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

