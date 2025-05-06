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
#include <memory>
#include <vector>
#include <algorithm>
#include <random>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "final_proyect_as/DetectPersonNode.hpp"
#include "final_proyect_as/FinishNode.hpp"
#include "final_proyect_as/RequestPlayersNode.hpp"

using namespace BT;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("combined_bt_node");
  int n_jugadores = 2;

  // Definimos lista completa de coordenadas
  std::vector<std::pair<double,double>> coords = {
    {0.53, 14.73}, {5.53, 18.79}, {17.02, 28.56},
    {17.27, 14.28}, {23.74, 14.96}, {32.50, 19.02}
  };

  // Barajamos y seleccionamos n_jugadores+1 waypoints para patrullar
  std::random_device rd;
  std::mt19937 gen(rd());
  std::shuffle(coords.begin(), coords.end(), gen);
  int sitios = std::min(static_cast<int>(coords.size()), n_jugadores + 1);

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  waypoints.reserve(sitios);
  for (int i = 0; i < sitios; ++i) {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = node->now();
    wp.pose.position.x = coords[i].first;
    wp.pose.position.y = coords[i].second;
    wp.pose.position.z = 0.0;
    wp.pose.orientation.w = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = 1.0;
    waypoints.push_back(wp);
  }

  // Configuración del Behavior Tree
  BehaviorTreeFactory factory;
  SharedLibrary loader;
  // Plugins para patrulla
  factory.registerFromPlugin(loader.getOSName("move_bt_node"));
  factory.registerFromPlugin(loader.getOSName("getwp_bt_node"));
  // Plugins para juego
  factory.registerFromPlugin(loader.getOSName("detectPerson_bt_node"));
  factory.registerFromPlugin(loader.getOSName("finish_bt_node"));
  factory.registerFromPlugin(loader.getOSName("requestPlayer_bt_node"));

  // Carga del XML de árbol principal (elige el adecuado)
  std::string pkgpath_game = ament_index_cpp::get_package_share_directory("final_proyect_as");
  std::string xml_file_game = pkgpath_game + "/BTs/juego_sin_hablar.xml";
  std::string pkgpath_patrol = ament_index_cpp::get_package_share_directory("bt_nav");
  std::string xml_file_patrol = pkgpath_patrol + "/behavior_tree_xml/navigate.xml";

  // Blackboard común
  auto blackboard = Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("waypoints", waypoints);
  blackboard->set("players", n_jugadores);

  // Selección de modo: patrulla primero, luego juego
  // Patrulla todos los waypoints
  Tree patrol_tree = factory.createTreeFromFile(xml_file_patrol, blackboard);
  rclcpp::Rate rate(10);
  for (size_t i = 0; i < waypoints.size() && rclcpp::ok(); ++i) {
    blackboard->set("waypoint", waypoints[i]);
    RCLCPP_INFO(node->get_logger(), "[Patrol] Waypoint %zu: x=%.2f, y=%.2f", i+1,
                waypoints[i].pose.position.x,
                waypoints[i].pose.position.y);

    bool finished = false;
    while (rclcpp::ok() && !finished) {
      rclcpp::spin_some(node);
      finished = patrol_tree.rootNode()->executeTick() != NodeStatus::RUNNING;
      rate.sleep();
    }
    RCLCPP_INFO(node->get_logger(), "[Patrol] Esperando 2 segundos...");
    rclcpp::sleep_for(std::chrono::seconds(2));
  }

  // Ahora ejecutamos la lógica de juego
  Tree game_tree = factory.createTreeFromFile(xml_file_game, blackboard);
  bool finish = false;
  while (rclcpp::ok() && !finish) {
    rclcpp::spin_some(node);
    finish = game_tree.rootNode()->executeTick() != NodeStatus::RUNNING;
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
