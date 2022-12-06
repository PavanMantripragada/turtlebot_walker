#include <functional>
#include <chrono>
#include <memory>
#include "turtlebot_walker/walker.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

Walker::Walker() : Node("walker"), isObstructed(false) {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&Walker::move, this));
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&Walker::detectObstacles, this, _1));
}

void Walker::move() {
    if (isObstructed) {
      commandVelocity.linear.x = 0.0;
      commandVelocity.angular.z = 0.1;
    } else {
      commandVelocity.linear.x = 0.1;
      commandVelocity.angular.z = 0.0;
    }
    publisher_->publish(commandVelocity);
}

void Walker::detectObstacles(const sensor_msgs::msg::LaserScan& msg){
    isObstructed = false;
    float range = 5.0;
    for(int i = 0; i < 50; i++){
        range = msg.ranges[i];
        if(range < 0.3){
            isObstructed = true;
            break;
        }
    }
    for(int i = 290; i < 359; i++){
        range = msg.ranges[i];
        if(range < 0.3){
            isObstructed = true;
            break;
        }
    }
}

