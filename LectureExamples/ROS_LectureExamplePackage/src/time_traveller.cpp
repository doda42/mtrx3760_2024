#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

class TimeTraveller : public rclcpp::Node
{
    public:
        TimeTraveller()
            : Node("time_travelling_listener")
        {
            buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
            listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
            timer = create_wall_timer(std::chrono::seconds(1), 
                    std::bind(&TimeTraveller::advancedListen, this));
        }

    private:
        void advancedListen()
        {
            try {
                rclcpp::Time when = get_clock()->now() - rclcpp::Duration::from_seconds(5.0);
                rclcpp::Time now = get_clock()->now();

                if (buffer->canTransform("parent_frame", now, "child_frame", when, "parent_frame"))
                {
                    geometry_msgs::msg::TransformStamped transformStamped = buffer->lookupTransform(
                        "parent_frame", now, "child_frame", when, "parent_frame");

                    RCLCPP_INFO(get_logger(), "Transform at time %.2f: [%.2f, %.2f, %.2f]",
                                when.seconds(),
                                transformStamped.transform.translation.x,
                                transformStamped.transform.translation.y,
                                transformStamped.transform.translation.z);
                } 
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(get_logger(), "Warning: %s", ex.what());
            }
        }

        std::shared_ptr<tf2_ros::Buffer> buffer;
        std::shared_ptr<tf2_ros::TransformListener> listener;
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimeTraveller>());
    rclcpp::shutdown();
    return 0;
}
