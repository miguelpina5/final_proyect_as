#include "bt_nav/GetWaypoint.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace bt_nav
{

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // Recuperamos el node para timestamps
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);
  node_ = node;
}

void GetWaypoint::halt()
{
  // No es interrumpible
}

BT::NodeStatus GetWaypoint::tick()
{
  // Obtenemos el waypoint de la blackboard
  geometry_msgs::msg::PoseStamped wp;
  if (!getInput("waypoint", wp)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetWaypoint"),
      "[GetWaypoint] No se ha recibido el waypoint de entrada");
    return BT::NodeStatus::FAILURE;
  }

  // Actualizamos cabecera
  wp.header.frame_id = "map";
  wp.header.stamp = node_->now();

  // Enviamos el waypoint al siguiente nodo
  setOutput("waypoint", wp);

  RCLCPP_INFO(
    rclcpp::get_logger("GetWaypoint"),
    "[GetWaypoint] Publicando waypoint: x=%.2f, y=%.2f",
    wp.pose.position.x,
    wp.pose.position.y
  );

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nav

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav::GetWaypoint>("GetWaypoint");
}
