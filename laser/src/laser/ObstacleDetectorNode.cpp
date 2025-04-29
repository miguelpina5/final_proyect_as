// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "laser/ObstacleDetectorNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace laser
{

using std::placeholders::_1;

ObstacleDetectorNode::ObstacleDetectorNode()
: Node("obstacle_detector_node")
{
  declare_parameter("min_distance", min_distance_);
  get_parameter("min_distance", min_distance_);

  RCLCPP_INFO(get_logger(), "ObstacleDetectorNode set to %f m", min_distance_);

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS().reliable(),
    std::bind(&ObstacleDetectorNode::laser_callback, this, _1));

  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>(
    "obstacle", 100);

  vector3_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
    "vector3_laser_topic", 100);
}

void
ObstacleDetectorNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  double distance_min = 1;
  double min_idx;
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    float distance = scan->ranges[i];
    if (distance >= 0.5 && distance <= 1.0 && distance < distance_min) {
      distance_min = distance;
      min_idx = i;
    }
  }

  auto obstacle_msg = std_msgs::msg::Bool();

  if (distance_min >= 0.5 && distance_min <= 1) {
    geometry_msgs::msg::Vector3 laser_msg;
    float angle = scan->angle_min + scan->angle_increment * min_idx;
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}

    laser_msg.x = distance_min * cos(angle);
    laser_msg.y = distance_min * sin(angle);
    laser_msg.z = 0.0;
    vector3_pub_->publish(laser_msg);

    // RCLCPP_INFO(get_logger(), "x: %f. y:%f", laser_msg.x, laser_msg.y);

    obstacle_msg.data = true;
  } else {
    obstacle_msg.data = false;
  }

  obstacle_pub_->publish(obstacle_msg);
}

}  // namespace laser
