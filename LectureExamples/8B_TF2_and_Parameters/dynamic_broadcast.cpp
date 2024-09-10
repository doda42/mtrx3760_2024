#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <random>

class FramePublisher : public rclcpp::Node 
{
    public:
        FramePublisher() : Node("frame_publisher"), x(0)
        {
            broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            timer = create_wall_timer(std::chrono::milliseconds(500),
                    std::bind(&FramePublisher::broadcastTransform, this));
        }

    private:
        void broadcastTransform() 
        {
            geometry_msgs::msg::TransformStamped transformStamped;

            // Set the timestamp and frame IDs
            transformStamped.header.stamp = this->get_clock()->now();
            transformStamped.header.frame_id = "parent_frame";
            transformStamped.child_frame_id = "child_frame";

            // Set translation
            transformStamped.transform.translation.x = x;
            transformStamped.transform.translation.y = 2.0;
            transformStamped.transform.translation.z = 0.0;

            // Set rotation
            transformStamped.transform.rotation.x = 0.0;
            transformStamped.transform.rotation.y = 0.0;
            transformStamped.transform.rotation.z = 0.0;
            transformStamped.transform.rotation.w = 1.0;

            // Broadcast the dynamic transform
            RCLCPP_INFO(get_logger(), "Broadcasting a dynamic transform");
            x++;
            broadcaster->sendTransform(transformStamped);
        }

        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
        rclcpp::TimerBase::SharedPtr timer;
        int x;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}
