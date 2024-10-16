#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mtrx3760/action/move_robot.hpp"

using MoveRobot = mtrx3760::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ServerGoalHandle<MoveRobot>;

class MoveRobotServer : public rclcpp::Node
{
    public:
        MoveRobotServer()
            : Node("move_robot_server")
        {
            server = rclcpp_action::create_server<MoveRobot>(
                this,
                "move_robot",
                std::bind(&MoveRobotServer::goalCallback, this,
                        std::placeholders::_1, std::placeholders::_2),
                std::bind(&MoveRobotServer::cancelCallback, this, std::placeholders::_1),
                std::bind(&MoveRobotServer::executeCallback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Action server has been started");
        }

    private:
        rclcpp_action::GoalResponse goalCallback(
            const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MoveRobot::Goal> goal)
        {
            (void)uuid;
            (void)goal;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancelCallback(
            const std::shared_ptr<MoveRobotGoalHandle> goalHandle)
        {
            (void)goalHandle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void executeCallback(const std::shared_ptr<MoveRobotGoalHandle> goalHandle)
        {
            RCLCPP_INFO(get_logger(), "Executing callback");

            int targetX = goalHandle->get_goal()->x;
            int targetY = goalHandle->get_goal()->y;

            auto feedback = std::make_shared<MoveRobot::Feedback>();
            auto result = std::make_shared<MoveRobot::Result>();

            // Simulate robot moving to the target
            int currentX = 0;
            int currentY = 0;
            while (rclcpp::ok() && (currentX != targetX || currentY != targetY))
            {
                if (goalHandle->is_canceling())
                {
                    result->success = false;
                    goalHandle->canceled(result);
                    RCLCPP_INFO(get_logger(), "Goal canceled");
                    return;
                }

                // Move one step towards the target
                if (currentX < targetX)
                {
                    currentX++;
                }
                if (currentX > targetX)
                {
                    currentX--;
                }
                if (currentY < targetY)
                {
                    currentY++;
                }
                if (currentY > targetY)
                {
                    currentY--;
                }

                // Provide feedback about the current position
                feedback->current_x = currentX;
                feedback->current_y = currentY;
                goalHandle->publish_feedback(feedback);

                RCLCPP_INFO(get_logger(), "Moving to (%d, %d), current position (%d, %d)", 
                            targetX, targetY, currentX, currentY);

                // Simulate time taken to move
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            if (rclcpp::ok())
            {
                result->success = true;
                goalHandle->succeed(result);
                RCLCPP_INFO(get_logger(), "Reached target (%d, %d)", targetX, targetY);
            }
        }
        rclcpp_action::Server<MoveRobot>::SharedPtr server;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}