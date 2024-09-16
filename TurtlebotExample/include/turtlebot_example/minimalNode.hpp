#ifndef MINIMALNODE_HPP
#define MINIMALNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <vector>


class minimalNode : public rclcpp::Node
{
public:
    minimalNode();
    void scan_callback(const sensor_msgs::msg::LaserScan& msg);
    void publish_velocity(const std::vector<float>& lin_vel, const std::vector<float>& ang_vel);
    

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

#endif // MINIMALNODE_HPP
