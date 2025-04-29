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

#include "yolo_msgs/msg/detection_array.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

#include "rclcpp/rclcpp.hpp"

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

  id_list_.clear();

  detection_sub_ = create_subscription<yolo_msgs::msg::DetectionArray>(
    "input_detection", rclcpp::SensorDataQoS().reliable(),
    std::bind(&DetectPersonNode::detection_callback, this, _1));

  timer_ = create_wall_timer(5s, [this]() {

    int persons = id_list_.size();
    RCLCPP_INFO(get_logger(), "Número de personas detectadas: %d", persons);
    
    blackboard()->set("num_persons", persons);
    // int num_persons = blackboard()->get<int>("num_personas");
    this->deactivate();
  });

  return CallbackReturn::SUCCESS;
}

CallbackReturn
DetectPersonNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Deactivating...");

  timer_->cancel();

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
DetectPersonNode::detection_callback(
  const yolo_msgs::msg::DetectionArray::ConstSharedPtr & msg)
{

  for(const auto & detection : msg->detections){
    if(detection.class_name == "person"){
      int id = detection.class_id;

      bool ya_existe = false;

      for (const auto& existente : id_list_) {
        if (id == existente) {
          ya_existe = true;
          break;
        }
      }

      if (!ya_existe) {
        id_list_.push_back(id);
        RCLCPP_INFO(get_logger(), "ID %d", id);
      }
    }
  }
}
}
