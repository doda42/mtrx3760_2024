#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Navigator : public rclcpp::Node
{
    public:
        Navigator()
            : Node("navigate_to_goal_node")
        {
            RCLCPP_INFO(get_logger(), "C++ Node for navigation started!");
            client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
            sendGoal();
        }

    private:
        void sendGoal()
        {
            if (!client->wait_for_action_server(std::chrono::seconds(5)))
            {
                RCLCPP_ERROR(get_logger(), "Action server not available.");
                return;
            }

            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.header.stamp = now();

            goal_msg.pose.pose.position.x = 1.5;
            goal_msg.pose.pose.position.y = 0.5;
            goal_msg.pose.pose.orientation.w = 1.0;

            RCLCPP_INFO(get_logger(), "Sending goal...");
            auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            options.result_callback = std::bind(&Navigator::goalResultCallback, this, std::placeholders::_1);
            client->async_send_goal(goal_msg, options);
        }

        void goalResultCallback(const NavigateToPoseGoalHandle::WrappedResult& result)
        {
            (void)result;
            RCLCPP_INFO(get_logger(), "Navigation Completed");
        }

        rclcpp_action::Client<NavigateToPose>::SharedPtr client;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Navigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
