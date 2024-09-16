#include "minimalNode.hpp"

minimalNode::minimalNode() : Node("minimal_node")
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile()
,std::bind(&minimalNode::scan_callback, this, std::placeholders::_1));
}

void minimalNode::scan_callback(const sensor_msgs::msg::LaserScan& msg)
{
    float min_angle = msg.angle_min;
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", min_angle);
}

void minimalNode::publish_velocity(const std::vector<float>& lin_vel, const std::vector<float>& ang_vel)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = lin_vel[0];
    msg.linear.y = lin_vel[1];
    msg.linear.z = lin_vel[2];
    msg.angular.x = ang_vel[0];
    msg.angular.y = ang_vel[1];
    msg.angular.z = ang_vel[2];
    publisher_->publish(msg);
}

