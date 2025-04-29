// Copyright 2023 Intelligent Robotics Lab
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
#include <iostream>
#include <vector>
#include <string>

#include "bt_yolo/DetectPersonNode.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/macros.hpp"

#include "yolo_msgs/msg/detection_array.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace detectperson
{

DetectPersonBTNode::DetectPersonBTNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  person_detected_(false)
{
  node_ = config.blackboard->get<std::shared_ptr<rclcpp::Node>>("node");

  sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detection_2d", 10,
    [this](const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
      for (const auto & detection : msg->detections) {
        if (!detection.results.empty() &&
            detection.results[0].hypothesis.class_id == "person")
        {
          person_detected_ = true;
        }
      }
    });

  executor_.add_node(node_);
}

BT::NodeStatus
DetectPersonBTNode::tick()
{
  person_detected_ = false;

  executor_.spin_some();  // ejecuta los callbacks pendientes

  if (person_detected_) {
    RCLCPP_INFO(node_->get_logger(), "[BT] Persona detectada");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(node_->get_logger(), "[BT] No se detectó persona");
  return BT::NodeStatus::FAILURE;
}

}  // namespace detectperson
