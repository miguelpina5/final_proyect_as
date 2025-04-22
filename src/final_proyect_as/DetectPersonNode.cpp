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
#include <vector>
#include <string>

#include "final_proyect_as/DetectPersonNode.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/macros.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace detectperson
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

DetectPersonNode::DetectPersonNode()
: LifecycleNode("DetectPersonNode"){}

CallbackReturn
DetectPersonNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Configuring...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
DetectPersonNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Activating...");

  array3D_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "detection_3d", 10, std::bind(&DetectPersonNode::array3D_callback, this, _1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn
DetectPersonNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Deactivating...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
DetectPersonNode::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Cleaning Up...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
DetectPersonNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Shutting Down...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
DetectPersonNode::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Error State");

  return CallbackReturn::SUCCESS;
}

void
DetectPersonNode::array3D_callback(vision_msgs::msg::Detection3DArray::UniquePtr detectionArray)
{
  std::vector<std::string> id_list;

  for(const auto& detection : detectionArray->detections){
    if(detection.results[0].hypothesis.class_id == "person"){
      std::string id = "0";//detection.results[0].hypothesis.id;

      bool ya_existe = false;

      for (const auto& existente : id_list) {
        if (id == existente) {
          ya_existe = true;
          break;
        }
      }

      if (!ya_existe) {
        id_list.push_back(id);
      }
    }   
  }

  int persons = id_list.size();

  if (persons == 0){
    RCLCPP_INFO(get_logger(), "No hay personas en este escondite");

  } else{
    RCLCPP_INFO(get_logger(), "Número de personas encontradas: %d\n", persons);

    for (int i = 0; i < persons; i++) {
      RCLCPP_INFO(get_logger(), "ID %d: %s", i, id_list[i].c_str());
    }

  }
}
}
