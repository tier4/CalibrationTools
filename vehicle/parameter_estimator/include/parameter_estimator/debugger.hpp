//
//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef PARAMETER_ESTIMATOR__DEBUGGER_HPP_
#define PARAMETER_ESTIMATOR__DEBUGGER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

#include <iostream>
#include <string>

struct Debugger
{
  explicit Debugger(std::string name, rclcpp::Node * node)
  {
    // QoS setup
    static constexpr std::size_t queue_size = 1;
    rclcpp::QoS durable_qos(queue_size);
    durable_qos.transient_local();  // option for latching

    pub_debug_ = rclcpp::create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>(
      node, "~/debug_values/" + name, durable_qos);
    debug_values_.data.resize(num_debug_values_, 0.0);
  }
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr pub_debug_;
  void publishDebugValue() { pub_debug_->publish(debug_values_); }
  static constexpr std::uint8_t num_debug_values_ = 20;
  mutable tier4_debug_msgs::msg::Float32MultiArrayStamped debug_values_;
};

#endif  // PARAMETER_ESTIMATOR__DEBUGGER_HPP_
