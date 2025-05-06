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

#ifndef BT_NAV__GETWAYPOINT_HPP_
#define BT_NAV__GETWAYPOINT_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace bt_nav
{

inline int random1to6_no_repeat() {
    // Generador y device estáticos para conservar el estado
    static std::random_device rd;
    static std::mt19937 gen(rd());
    // Pool de números disponibles
    static std::vector<int> pool = {1,2,3,4,5,6};

    // Si ya no quedan números, vuelvo a rellenar
    if (pool.empty()) {
        pool = {1,2,3,4,5,6};
    }

    // Elijo aleatoriamente un índice en [0, pool.size()-1]
    std::uniform_int_distribution<size_t> dist(0, pool.size()-1);
    size_t idx = dist(gen);

    // Extraigo el valor y lo quito del pool
    int value = pool[idx];
    pool.erase(pool.begin() + idx);

    return value;
}
class GetWaypoint : public BT::ActionNodeBase
{
public:
  explicit GetWaypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

    std::vector<std::pair<double,double>> coords_;
    std::vector<std::pair<double,double>> orientation_;
    

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts();

private:
    
  nav_msgs::msg::Path wps_array_;
  bool first_time;
};

}  // namespace bt_nav

#endif  // BT_NAV__GETWAYPOINT_HPP_