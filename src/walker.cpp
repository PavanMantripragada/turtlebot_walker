/**
 * MIT License
 *
 * Copyright (c) 2022 Pavan Mantripragada
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file walker.cpp
 * @author Pavan Mantripragada (mppavan@umd.edu)
 * @brief cpp file for walker class
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "turtlebot_walker/walker.hpp"

#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

Walker::Walker() : Node("walker"), isObstructed(false) {
  publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
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

void Walker::detectObstacles(const sensor_msgs::msg::LaserScan& msg) {
  isObstructed = false;
  float range;
  for (int i = 0; i < 50; i++) {
    range = msg.ranges[i];
    if (range < 0.3) {
      isObstructed = true;
      break;
    }
  }
  for (int i = 290; i < 359; i++) {
    range = msg.ranges[i];
    if (range < 0.3) {
      isObstructed = true;
      break;
    }
  }
}
