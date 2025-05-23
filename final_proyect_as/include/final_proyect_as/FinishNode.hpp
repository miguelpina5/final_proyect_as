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

#ifndef FINISHNODE_HPP_
#define FINISHNODE_HPP_

#include <memory>
#include <iostream>

#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/node.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace finish
{

class FinishNode : public BT::ConditionNode
{
public:
  explicit FinishNode(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace finish

#endif  // FINISHNODE_HPP_