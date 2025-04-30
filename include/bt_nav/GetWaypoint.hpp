#ifndef BT_NAV__GETWAYPOINT_HPP_
#define BT_NAV__GETWAYPOINT_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

namespace bt_nav
{

class GetWaypoint : public BT::ActionNodeBase
{
public:
  GetWaypoint(const std::string& name, const BT::NodeConfiguration& config);

  // Registra puertos: recibe número de jugadores y lista de waypoints; devuelve un waypoint
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("players", "Número de jugadores"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints", "Lista de waypoints"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint", "Waypoint seleccionado")
    };
  }

  // Sobrescribe tick y halt
  BT::NodeStatus tick() override;
  void halt() override {}

private:
  bool initialized_{false};
  int call_count_{0};
  int max_calls_{0};
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::vector<bool> visited_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace bt_nav

#endif  // BT_NAV__GETWAYPOINT_HPP_
