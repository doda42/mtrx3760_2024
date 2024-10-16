#include "rclcpp/rclcpp.hpp"
#include "mtrx3760/msg/custom_message.hpp"

class AutoPilotNode : public rclcpp::Node
{
    public:
        AutoPilotNode()
            : Node("autopilot_node")
        {
            publisher = create_publisher<mtrx3760::msg::CustomMessage>("/flight/waypoints", 10);
            timer = create_wall_timer(std::chrono::milliseconds(1000),
                    std::bind(&AutoPilotNode::PublishWaypoints, this));
            RCLCPP_INFO(get_logger(), "Autopilot has been started");
        }

    private:
        void PublishWaypoints()
        {
            mtrx3760::msg::CustomMessage msg = mtrx3760::msg::CustomMessage();
            msg.x = 1.0;
            msg.y = 2.0;
            msg.msg = "Hello there";
            RCLCPP_INFO(get_logger(), "Sending pose x: %f y: %f. msg: %s", msg.x, msg.y, msg.msg.c_str());
            publisher->publish(msg);
        }
        rclcpp::Publisher<mtrx3760::msg::CustomMessage>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<AutoPilotNode> node = std::make_shared<AutoPilotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

