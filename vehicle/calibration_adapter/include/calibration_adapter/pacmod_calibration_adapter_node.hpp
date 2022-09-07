//
// Copyright 2020 Tier IV, Inc. All rights reserved.
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
//

#ifndef CALIBRATION_ADAPTER__PACMOD_CALIBRATION_ADAPTER_NODE_HPP_
#define CALIBRATION_ADAPTER__PACMOD_CALIBRATION_ADAPTER_NODE_HPP_

#include "calibration_adapter/calibration_adapter_node_base.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tier4_calibration_msgs/msg/float32_stamped.hpp"
#include "tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp"

class PacmodCalibrationAdapterNode : public CalibrationAdapterNodeBase
{
public:
  using SteeringWheelStatusStamped = tier4_vehicle_msgs::msg::SteeringWheelStatusStamped;
  PacmodCalibrationAdapterNode();

private:
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_handle_status_;
  rclcpp::Subscription<SteeringWheelStatusStamped>::SharedPtr sub_handle_status_;
  void callbackSteeringWheelStatus(const SteeringWheelStatusStamped::ConstSharedPtr msg);
};

#endif  // CALIBRATION_ADAPTER__PACMOD_CALIBRATION_ADAPTER_NODE_HPP_
