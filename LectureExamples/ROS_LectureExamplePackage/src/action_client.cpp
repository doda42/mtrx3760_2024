#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mtrx3760/action/move_robot.hpp"

using MoveRobot = mtrx3760::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ClientGoalHandle<MoveRobot>;

class MoveRobotClient : public rclcpp::Node
{
    public:
        MoveRobotClient()
            : Node("count_until_server")
        {
            client = rclcpp_action::create_client<MoveRobot>(this, "move_robot");
            RCLCPP_INFO(get_logger(), "Action client started");
            sendGoal(50, 60);
        }

        void sendGoal(int targetX, int targetY)
        {
            client->wait_for_action_server();
            auto goal = MoveRobot::Goal();
            goal.x = targetX;
            goal.y = targetY;

            auto options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
            options.result_callback = std::bind(&MoveRobotClient::goalResultCallback, this, std::placeholders::_1);

            RCLCPP_INFO(get_logger(), "Sending a goal from client");
            client->async_send_goal(goal, options);
        }

    private:
        void goalResultCallback(const MoveRobotGoalHandle::WrappedResult& result)
        {
            bool success = result.result->success;
            RCLCPP_INFO(get_logger(), "Action completed. Result = %d\n", success);
        }

        rclcpp_action::Client<MoveRobot>::SharedPtr client;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}