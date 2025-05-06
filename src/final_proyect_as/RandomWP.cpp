// Copyright 2021 Intelligent Robotics Lab
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

#include <string>
#include <iostream>
#include <vector>

#include "bt_nav/GetWaypoint.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bt_nav
{

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  coords_ = {
    {0.53, 14.73}, {5.53, 18.79}, {17.02, 28.56},
    {17.27, 14.28}, {23.74, 14.96}, {32.50, 19.02},
    {-3.06, 24.17}
  };

  orientation_ = {
    {0, 1}, {0, 0}, {1, 0}, {0, 1}, {0, 1}, {0, 1}
  };

  std::random_device rd;
  gen_ = std::mt19937(rd());
  dist_ = std::uniform_int_distribution<size_t>(0, coords_.size() - 1);

}

void
GetWaypoint::halt()
{
}

BT::NodeStatus
GetWaypoint::tick()
{
  getInput("players", jugadores_);
  setOutput("Wps", wps_array_);


  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nav

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav::GetWaypoint>("GetWaypoint");
}