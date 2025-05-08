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

RandomWP::RandomWP(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  coords_ = {
    {0.53, 14.73},  // Escondite 1 Lado derecho lab
    {5.53, 18.79},  // Escondite 2 Lado izquierdo lab
    {17.02, 28.56}, // Escondite 3 Ascensor
    {17.06, 14.07}, // Escondite 4 Circulo
    {23.74, 14.96}, // Escondite 5 Pasillo derecha
  };

  orientation_ = {
    {1, 0},  // Escondite 1 lado derecho lab
    {1, 0},  // Escondite 2 lado izquierdo lab
    {0  , 1},  // Escondite 3 Ascensor
    {-0.73624, 0.67672}, // Escondite 4 Circulo
    {-1, 0.0},  // Escondite 5 Pasillo derecha
  };

}

BT::PortsList RandomWP::providedPorts()
{
  return { 
    BT::InputPort<int>("players")
  };
}

void
RandomWP::halt()
{
}

BT::NodeStatus
RandomWP::tick()
{
  getInput("players", jugadores_);
  config().blackboard->get("first_time", first_time_);

  if(first_time_){
    geometry_msgs::msg::PoseStamped ps;
    wps_array_.header.frame_id = "map";
    random_number = random_unique_array(jugadores_ + 1);

    for(int i = 0; i != jugadores_ + 1; i++){
      geometry_msgs::msg::PoseStamped ps;
      idx = random_number[i];

      ps.pose.position.x = coords_[idx].first;
      ps.pose.position.y = coords_[idx].second;
      ps.pose.position.z = 0;

      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = orientation_[idx].first;;
      ps.pose.orientation.w = orientation_[idx].second;
      std::cout << "Indice a buscar: " << idx << std::endl;
      wps_array_.poses.push_back(ps);
    }

    config().blackboard->set<nav_msgs::msg::Path>("Wps", wps_array_);
    first_time_ = false;
    config().blackboard->set<bool>("first_time", first_time_);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nav

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav::RandomWP>("RandomWP");
}