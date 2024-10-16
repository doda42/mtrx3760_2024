#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "std_msgs/msg/int32.hpp"

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleDrone : public rclcpp_lifecycle::LifecycleNode
{
    public:
        LifecycleDrone()
            : LifecycleNode("lifecycle_drone"), counter(0)
        {
            RCLCPP_INFO(get_logger(), "Lifecycle drone created");
        }

        // Allocate resources, load parameters, validate hardware
        LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State& previousState)
        {
            (void)previousState;
            publisher = create_publisher<std_msgs::msg::Int32>("/drone_uptime", 10);
            timer = create_wall_timer(std::chrono::seconds(1), std::bind(&LifecycleDrone::publishState, this));
            RCLCPP_INFO(get_logger(), "on_configure called");

            return LifecycleCallbackReturn::SUCCESS;
        }

        // Start timers, activate pub / sub, engage with hardware
        LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State& previousState)
        {
            LifecycleNode::on_activate(previousState);
            RCLCPP_INFO(get_logger(), "on_activate called");
            std::this_thread::sleep_for(std::chrono::seconds(5)); // Simulate hardware engagement time

            return LifecycleCallbackReturn::SUCCESS;
        }

        // Deactivate pub / sub, stop timers, disengage with hardware
        LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State& previousState)
        {
            LifecycleNode::on_deactivate(previousState);
            RCLCPP_INFO(get_logger(), "on_deactivate called");
            std::this_thread::sleep_for(std::chrono::seconds(5)); // Simulate hardware disengagement time

            return LifecycleCallbackReturn::SUCCESS;
        }

        // Deallcoate resources, reset parameters, 
        LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State& previousState)
        {
            (void)previousState;
            timer.reset();
            publisher.reset();
            RCLCPP_INFO(get_logger(), "on_cleanup called");

            return LifecycleCallbackReturn::SUCCESS;
        }

        // Release all resources
        LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State& previousState)
        {
            (void)previousState;
            timer.reset();
            publisher.reset();
            RCLCPP_INFO(get_logger(), "on_shutdown called");
            return LifecycleCallbackReturn::SUCCESS;
        }

    private:
        void publishState()
        {
            rclcpp_lifecycle::State currentState = get_current_state();
            RCLCPP_INFO(get_logger(), "Current State = %s", currentState.label().c_str());

            auto msg = std_msgs::msg::Int32();
            msg.data = counter;
            counter++;
            publisher->publish(msg);
        }

        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;
        int counter;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleDrone>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}