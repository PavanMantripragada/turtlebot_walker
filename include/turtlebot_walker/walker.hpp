#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Walker : public rclcpp::Node{
    public:
    Walker();

    void move();

    void detectObstacles(const sensor_msgs::msg::LaserScan& msg);

    private:
    bool isObstructed;
    geometry_msgs::msg::Twist commandVelocity;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
};