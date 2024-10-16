#include "rclcpp/rclcpp.hpp"
#include "mtrx3760/msg/custom_message.hpp"

class MotionControlNode : public rclcpp::Node
{
    public:
        MotionControlNode()
            : Node("motion_control_node")
        {
            subscriber = create_subscription<mtrx3760::msg::CustomMessage>("/flight/waypoints", 10,
                         std::bind(&MotionControlNode::callback, this, std::placeholders::_1));

            RCLCPP_INFO(get_logger(), "Motion Controller active");
        }

    private:
        void callback(const mtrx3760::msg::CustomMessage::SharedPtr msg)
        {
            RCLCPP_INFO(get_logger(), "Received x: %f, y: %f msg: %s", msg->x, msg->y, msg->msg.c_str());
            // Some actual motion control code
        }

        rclcpp::Subscription<mtrx3760::msg::CustomMessage>::SharedPtr subscriber;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MotionControlNode> node = std::make_shared<MotionControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

