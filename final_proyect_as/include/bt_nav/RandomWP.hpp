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

#ifndef BT_NAV__RandomWP_HPP_
#define BT_NAV__RandomWP_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <random>
#include <algorithm>

namespace bt_nav
{

  void fill_rec(std::vector<int> &out, int n) {
    if (static_cast<int>(out.size()) >= n) {
        // Caso base: ya tenemos n elementos
        return;
    }
    // Generador estático para conservar estado entre llamadas
    static std::random_device rd;
    static std::mt19937 gen(rd());

    // Distribución en [1, n]
    std::uniform_int_distribution<int> dist(1, n);
    int num = dist(gen);

    // Si no está aún en 'out', lo añadimos
    if (std::find(out.begin(), out.end(), num) == out.end()) {
        out.push_back(num);
    }

    // Llamada recursiva hasta tener n elementos
    fill_rec(out, n);
}

  std::vector<int> random_unique_array(int n) {
    std::vector<int> result;
    result.reserve(n);
    fill_rec(result, n);
    return result;
  }

class RandomWP : public BT::ActionNodeBase
{
public:
  explicit RandomWP(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

    std::vector<std::pair<double,double>> coords_;
    std::vector<std::pair<double,double>> orientation_;
    

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts();

private:
    
  nav_msgs::msg::Path wps_array_;
  bool first_time_;
  int jugadores_;
  size_t idx;
  std::vector<int> random_number; 
};

}  // namespace bt_nav

#endif  // BT_NAV__RandomWP_HPP_