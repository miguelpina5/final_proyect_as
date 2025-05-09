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

#include "final_proyect_as/DetectPersonNode.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/macros.hpp"

#include "yolo_msgs/msg/detection_array.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace detectperson
{

  DetectPersonNode::DetectPersonNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  person_detected_(false)
{
  node_ = config.blackboard->get<std::shared_ptr<rclcpp::Node>>("node");

  sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detection_2d", 10,
    std::bind(&DetectPersonNode::detection_callback, this, _1));

}

void DetectPersonNode::detection_callback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  encontrado_yolo = 0;
  for (const auto & detection : msg->detections) {
    if (!detection.results.empty() &&
        detection.results[0].hypothesis.class_id == "person")
    {
      encontrado_yolo ++;
      person_detected_ = true;
    } else {
      person_detected_ = false;
    }

  }
}

BT::PortsList DetectPersonNode::providedPorts()
{
  return {
    BT::OutputPort<int>("encontrados")
  };
}

BT::NodeStatus
DetectPersonNode::tick()
{
  executor_.spin_some();  // ejecuta los callbacks pendientes

  if (!person_detected_) {
    RCLCPP_INFO(node_->get_logger(), "[BT] No se detectó persona");
    return BT::NodeStatus::FAILURE;
  }
  
  config().blackboard->get("encontrados", find_players);

  find_players = find_players + encontrado_yolo;
  setOutput("encontrados", find_players);

  RCLCPP_INFO(node_->get_logger(), "\033[1m[BT] Persona detectada: %d\033[0m", encontrado_yolo);
  return BT::NodeStatus::SUCCESS;

}
}  // namespace detectperson


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<detectperson::DetectPersonNode>("DetectPersonNode");
}