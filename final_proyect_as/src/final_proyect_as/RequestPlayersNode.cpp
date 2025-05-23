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
#include <stdlib.h>
#include "final_proyect_as/RequestPlayersNode.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"


namespace requestPlayers
{
  

RequestPlayersNode::RequestPlayersNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList RequestPlayersNode::providedPorts()
{
  return { 
    BT::OutputPort<int>("players"),
    BT::OutputPort<int>("encontrados"),
    BT::OutputPort<int>("num_intento")
  };
}

BT::NodeStatus RequestPlayersNode::tick()
{
  system("clear");

  std::cout << CYAN << R"(
   ╔═══════════════════════════════════════════════════════════════════════════════════════════╗
   ║ ████████  ████████  ████████  ████████  ███    ██  ██████    ████████  ████████  ████████ ║
   ║ ███       ███       ███       ██    ██  ████   ██  ██   ██      ██        ██     ███      ║
   ║ ██████    ████████  ███       ██    ██  ██ ██  ██  ██    ██     ██        ██     ██████   ║
   ║ ███            ███  ███       ██    ██  ██  ██ ██  ██   ██      ██        ██     ███      ║
   ║ ████████  ████████  ████████  ████████  ██   ████  ██████    ████████     ██     ████████ ║
   ╚═══════════════════════════════════════════════════════════════════════════════════════════╝)" 
   << RESET << std::endl;

  std::cout << "¿Cuántos jugadores van a participar? ";
  std::cin >> num_players_;

  if (num_players_ <= 0 || num_players_ > 2) {
    std::cerr << "Número no válido. Tiene q haber mínimo 1 jugador y máximo 2" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  int find_players = 0;
  int num_attempts = num_players_ + 1;
  int i = 0;
  // Guardamos en el blackboard
  setOutput("players", num_players_);
  setOutput("encontrados", find_players);
  setOutput("num_intento", num_attempts);
  config().blackboard->set<bool>("first_time", true);
  config().blackboard->set<int>("i", i);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace requestPlayers


// Registro del nodo para que el plugin exporte BT_RegisterNodesFromPlugin
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<requestPlayers::RequestPlayersNode>("RequestPlayersNode");
}