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

#ifndef BT_NAV__GETWAYPOINT_HPP_
#define BT_NAV__GETWAYPOINT_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace bt_nav
{

class GetWaypoint : public BT::ActionNodeBase
{
public:
  explicit GetWaypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
    {
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("way_point")
    });
  }

private:
  geometry_msgs::msg::PoseStamped wp_;
  nav_msgs::msg::Path wps_array_;
};

}  // namespace bt_nav

#endif  // BT_NAV__GETWAYPOINT_HPP_