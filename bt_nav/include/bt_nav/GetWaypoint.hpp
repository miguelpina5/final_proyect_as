// src/bt_nav/GetWaypoint.hpp

#ifndef BT_NAV__GETWAYPOINT_HPP_
#define BT_NAV__GETWAYPOINT_HPP_

#include <string>
#include <vector>
#include <random>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_nav
{

class GetWaypoint : public BT::ActionNodeBase
{
public:
  GetWaypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() override;
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint") };
  }

private:
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::mt19937 gen_;
  std::uniform_int_distribution<size_t> dist_;
};

}  // namespace bt_nav

#endif  // BT_NAV__GETWAYPOINT_HPP_
