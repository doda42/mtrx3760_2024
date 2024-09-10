#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

class StaticFramePublisher : public rclcpp::Node 
{
   public:
       StaticFramePublisher()
            : Node("static_frame_publisher") 
       {
           staticBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
           broadcastStaticTransform();
       }
    
   private:
       void broadcastStaticTransform() 
       {
           geometry_msgs::msg::TransformStamped transformStamped;

           transformStamped.header.stamp = get_clock()->now();
           transformStamped.header.frame_id = "parent_frame";
           transformStamped.child_frame_id = "child_frame";

           transformStamped.transform.translation.x = 1.0;
           transformStamped.transform.rotation.w = 1.0;

           staticBroadcaster->sendTransform(transformStamped);
       }
       std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticBroadcaster;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>());
    rclcpp::shutdown();
    return 0;
}