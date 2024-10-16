#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

class FrameListener : public rclcpp::Node
{
    public:
        FrameListener() : Node("frame_listener")
        {
            buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
            listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
            timer = create_wall_timer(std::chrono::milliseconds(500), 
                    std::bind(&FrameListener::listen, this));
        }

    private:
        void listen()
        {
            try
            {
                geometry_msgs::msg::TransformStamped transformStamped = buffer->lookupTransform(
                    "child_frame", "parent_frame", tf2::TimePointZero);

                RCLCPP_INFO(get_logger(), "Transform: [%.2f, %.2f, %.2f]",
                            transformStamped.transform.translation.x,
                            transformStamped.transform.translation.y,
                            transformStamped.transform.translation.z);
                // Do something with the transform

            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
            }
        }

        std::shared_ptr<tf2_ros::Buffer> buffer;
        std::shared_ptr<tf2_ros::TransformListener> listener;
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameListener>());
    rclcpp::shutdown();
    return 0;
}

