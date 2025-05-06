#include "bt_nav/GetWaypoint.hpp"
#include "rclcpp/rclcpp.hpp"
#include <random>

namespace bt_nav
{

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // Recuperamos el node de ROS para hacer logs y stamps
  config().blackboard->get("node", node_);
}

// BT::PortsList GetWaypoint::providedPorts()
// {
//   return {
//     BT::InputPort<int>("players")
//     BT::OutputPort<int>("")
//   };
// }

BT::NodeStatus GetWaypoint::tick()
{
  // Inicialización en el primer tick
  if (!initialized_) {
    int players;
    if (!getInput("players", players)) {
      RCLCPP_ERROR(node_->get_logger(), "[GetWaypoint] No recibí 'players'");
      return BT::NodeStatus::FAILURE;
    }
    max_calls_ = players + 1;

    if (!getInput("waypoints", waypoints_)) {
      RCLCPP_ERROR(node_->get_logger(), "[GetWaypoint] No recibí 'waypoints'");
      return BT::NodeStatus::FAILURE;
    }
    visited_.assign(waypoints_.size(), false);
    initialized_ = true;
  }

  // Si ya hemos llamado las veces necesarias, fallo para que el BT deje de pedir más
  if (call_count_ >= max_calls_) {
    RCLCPP_INFO(node_->get_logger(), "[GetWaypoint] Límite de %d waypoints alcanzado", max_calls_);
    return BT::NodeStatus::FAILURE;
  }

  // Construir lista de índices no visitados
  std::vector<int> candidatos;
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    if (!visited_[i]) {
      candidatos.push_back(i);
    }
  }

  if (candidatos.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[GetWaypoint] Ya no quedan waypoints sin visitar");
    return BT::NodeStatus::FAILURE;
  }

  // Selección aleatoria de uno de los no visitados
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, candidatos.size() - 1);
  int elegido = candidatos[dis(gen)];

  // Marcar como visitado y aumentar contador
  visited_[elegido] = true;
  ++call_count_;

  // Devolverlo al BT
  const auto & wp = waypoints_[elegido];
  setOutput("waypoint", wp);

  RCLCPP_INFO(
    node_->get_logger(),
    "[GetWaypoint] Seleccionado waypoint #%d (x=%.2f, y=%.2f); llamada %d/%d",
    elegido, wp.pose.position.x, wp.pose.position.y,
    call_count_, max_calls_);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nav

// Registro del nodo
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav::GetWaypoint>("GetWaypoint");
}
