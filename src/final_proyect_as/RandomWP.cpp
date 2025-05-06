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

#include "bt_nav/RandomWP.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <random>
#include "std_msgs/msg/bool.hpp"
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

}

BT::PortsList GetWaypoint::providedPorts()
{
  return { 
    BT::InputPort<int>("players")
  };
}

void
GetWaypoint::halt()
{
}

BT::NodeStatus
GetWaypoint::tick()
{
  getInput("players", jugadores_);
  config().blackboard->get("first_time", first_time);

  if(first_time){
    geometry_msgs::msg::PoseStamped ps;
    wps_array_.header.stamp = this->now();
    wps_array_.header.frame_id = "map";

    for(int i = 0; i != jugadores_ + 1; i++){
      geometry_msgs::msg::PoseStamped ps;
      idx = random1to6_no_repeat();

      ps.pose.position.x = coords_[idx].first;
      ps.pose.position.y = coords_[idx].second;
      ps.pose.position.z = 0;

      ps.pose.orientation.x = orientation_[idx].first;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = 0.0;
      ps.pose.orientation.w = orientation_[idx].second;

      wps_array_.poses.push_back(ps);
    }

    config().blackboard->set<nav_msgs::msg::Path>("Wps", wps_array_);
    first_time = false;
    config().blackboard->set<bool>("first_time", first_time);
  }  

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nav

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav::GetWaypoint>("GetWaypoint");
}