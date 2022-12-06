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
 * @file walker.hpp
 * @author Pavan Mantripragada (mppavan@umd.edu)
 * @brief Header file for walker class
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * @brief Walker class
 * for moving turtlebot3
 * in a map while avoiding
 * obstacles
 */
class Walker : public rclcpp::Node {
 public:
  /**
   * @brief Constructor
   * for Walker class
   */
  Walker();

  /**
   * @brief Call back function
   * to move the robot by publishing
   * twist messages
   */
  void move();

  /**
   * @brief Call back function
   * to detect obstacles
   * @param msg LaserScan message
   */
  void detectObstacles(const sensor_msgs::msg::LaserScan& msg);

 private:
  /** @brief flag for obstruction */
  bool isObstructed;
  /** @brief command velocity given to the turtlebot3 */
  geometry_msgs::msg::Twist commandVelocity;
  /** @brief timer pointer */
  rclcpp::TimerBase::SharedPtr timer_;
  /** @brief publisher pointer */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  /** @brief subscriber pointer */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
};
