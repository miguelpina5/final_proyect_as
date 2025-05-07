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

#ifndef DETECTPERSONNODE_HPP_
#define DETECTPERSONNODE_HPP_

#include <memory>
#include <iostream>

#include "rclcpp/node.hpp"
#include "rclcpp/macros.hpp"

#include "yolo_msgs/msg/detection_array.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/action_node.h"

namespace detectperson
{

class DetectPersonNode : public BT::SyncActionNode
{
public:
  DetectPersonNode(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  bool person_detected_;
  int encontrados;
  int encontrado_yolo;
};

}  // namespace detectperson

#endif  // DETECTPERSONNODE_HPP_