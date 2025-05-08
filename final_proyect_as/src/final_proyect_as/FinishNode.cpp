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

#include "final_proyect_as/FinishNode.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace finish
{

FinishNode::FinishNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList FinishNode::providedPorts()
{
  return {
    BT::InputPort<int>("players"),
    BT::InputPort<int>("encontrados")
  };
}

BT::NodeStatus FinishNode::tick()
{
  int num_players_;
  int find_players;
  int i;
  int escondites = num_players_ + 1;
  nav_msgs::msg::Path wps_array_;
  

  getInput("players", num_players_);
  getInput("encontrados", find_players);
  config().blackboard->get("i", i);
  config().blackboard->get("Wps", wps_array_);
  
  std::cout << "Jugadores: " << num_players_ << "\nEncontrados hasta el momento: " << find_players << std::endl;

  if (find_players == num_players_ || escondites == i) {

    if (find_players == num_players_){
      std::cout << "TODOS LOS JUGADORES HAN SIDO ENCONTRADOS, HE GANADO" << std::endl;
    } else {
     std::cout << "FINALIZADO EL NÚMERO DE INTENTOS, HE PERDIDO" << std::endl;
    }
    std::cout << "Yendo al punto de inicio." << std::endl;
    i = 0;

    geometry_msgs::msg::PoseStamped ps;

    ps.pose.position.x = -3.06;
    ps.pose.position.y = 24.17;
    ps.pose.position.z = 0;

    ps.pose.orientation.x = 0.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = -0.7;
    ps.pose.orientation.w = 0.7;
    wps_array_.poses[i] = ps;
    
    config().blackboard->set<nav_msgs::msg::Path>("Wps", wps_array_);
    config().blackboard->set<int>("i", i);
    std::cout << "** NAVIGATION SUCCEEDED **" << std::endl;
    return BT::NodeStatus::SUCCESS;

  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace finish

// Registro del nodo para que el plugin exporte BT_RegisterNodesFromPlugin

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<finish::FinishNode>("FinishNode");
}