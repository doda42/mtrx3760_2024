#include "minimalNode.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto turtlebot_node = std::make_shared<minimalNode>();

    rclcpp::Rate loop_rate(30);

    std::vector<float> lin_vel(3, 0);
    std::vector<float> ang_vel(3, 0);
    bool go_forward = 1;

    
    while (rclcpp::ok())
    {
        lin_vel[0] = go_forward; // Go forwards

        turtlebot_node->publish_velocity(lin_vel, ang_vel);

        rclcpp::spin_some(turtlebot_node);
        loop_rate.sleep();
        go_forward = !go_forward;
    }
    rclcpp::shutdown();
    return 0;
}
