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

#ifndef REQUESTPLAYERSNODE_HPP_
#define REQUESTPLAYERSNODE_HPP_

#include <memory>
#include <iostream>

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/node.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace requestPlayers
{

class RequestPlayersNode : public BT::SyncActionNode
{
public:
  explicit RequestPlayersNode(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  int num_players_;
};

}  // namespace requestPlayers

#endif  // REQUESTPLAYERSNODE_HPP_
