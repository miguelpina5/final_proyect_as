// src/bt_nav/GetWaypoint.cpp

// Copyright 2021 Intelligent Robotics Lab
// Modificado para selección aleatoria entre 6 waypoints

#include "bt_nav/GetWaypoint.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace bt_nav
{

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  gen_(std::random_device{}())
{
  // Recuperamos el node desde el blackboard para usar now()
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  // Definimos 6 waypoints en el frame "map"
  std::vector<std::pair<double,double>> coords = {
    {0.53,  14.73},
    {5.53,  18.79},
    {17.02, 28.56},
    {17.27, 14.28},
    {23.74, 14.96},
    {32.50, 19.02}
  };

  for (const auto & c : coords) {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = node->now();

    wp.pose.position.x = c.first;
    wp.pose.position.y = c.second;
    wp.pose.position.z = 0.0;

    // Orientación neutra (identidad)
    wp.pose.orientation.w = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = 1.0;

    waypoints_.push_back(wp);
  }

  // Distribución uniforme entre [0, waypoints_.size()-1]
  dist_ = std::uniform_int_distribution<size_t>(0, waypoints_.size() - 1);
}

void GetWaypoint::halt()
{
  // No hace nada especial al parar
}

BT::NodeStatus GetWaypoint::tick()
{
  // Elegimos un índice aleatorio
  size_t idx = dist_(gen_);
  const auto & wp = waypoints_[idx];

  // Publicamos el waypoint seleccionado
  setOutput("waypoint", wp);

  RCLCPP_INFO(
    rclcpp::get_logger("GetWaypoint"),
    "Waypoint #%zu seleccionado → x=%.2f, y=%.2f",
    idx, wp.pose.position.x, wp.pose.position.y
  );

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nav

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav::GetWaypoint>("GetWaypoint");
}
