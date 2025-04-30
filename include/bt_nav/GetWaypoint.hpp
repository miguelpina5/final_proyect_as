#ifndef BT_NAV__GETWAYPOINT_HPP_
#define BT_NAV__GETWAYPOINT_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace bt_nav
{

class GetWaypoint : public BT::ActionNodeBase
{
public:
  GetWaypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  // No interrumpible
  void halt() override;

  // Ejecución principal
  BT::NodeStatus tick() override;

  // Definición de puertos
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("waypoint", "Waypoint de entrada para mapear y publicar"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint", "Waypoint modificado con frame y timestamp ajustados")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace bt_nav

#endif  // BT_NAV__GETWAYPOINT_HPP_
