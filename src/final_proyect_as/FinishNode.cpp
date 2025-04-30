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
  int players = 0;
  int encontrados = 0;

  if (!getInput("players", players)) {
    std::cerr << "No se pudo obtener el número de jugadores desde el blackboard" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("encontrados", encontrados)) {
    std::cerr << "No se pudo obtener el número de encontrados desde el blackboard" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::cout << "Jugadores: " << players << " - Encontrados: " << encontrados << std::endl;

  if (encontrados == players) {
    std::cout << "Todos los jugadores han sido encontrados." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace finish