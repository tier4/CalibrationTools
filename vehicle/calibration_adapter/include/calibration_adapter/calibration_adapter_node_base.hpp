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

#ifndef CALIBRATION_ADAPTER__CALIBRATION_ADAPTER_NODE_BASE_HPP_
#define CALIBRATION_ADAPTER__CALIBRATION_ADAPTER_NODE_BASE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_vehicle_msgs/msg/engage.hpp"
#include "tier4_calibration_msgs/msg/bool_stamped.hpp"
#include "tier4_calibration_msgs/msg/float32_stamped.hpp"
#include "tier4_vehicle_msgs/msg/actuation_command_stamped.hpp"
#include "tier4_vehicle_msgs/msg/actuation_status_stamped.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"

class CalibrationAdapterNodeBase : public rclcpp::Node
{
public:
  using ActuationCommandStamped = tier4_vehicle_msgs::msg::ActuationCommandStamped;
  using ActuationStatusStamped = tier4_vehicle_msgs::msg::ActuationStatusStamped;
  using Float32Stamped = tier4_calibration_msgs::msg::Float32Stamped;
  using BoolStamped = tier4_calibration_msgs::msg::BoolStamped;
  using EngageStatus = autoware_auto_vehicle_msgs::msg::Engage;
  using SteeringAngleStatus = autoware_auto_vehicle_msgs::msg::SteeringReport;
  CalibrationAdapterNodeBase();

private:
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_accel_status_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_brake_status_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_steer_status_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_accel_cmd_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_brake_cmd_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_steer_cmd_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_steering_angle_status_;
  rclcpp::Publisher<BoolStamped>::SharedPtr pub_is_engage_;

  rclcpp::Subscription<ActuationCommandStamped>::SharedPtr sub_actuation_command_;
  rclcpp::Subscription<ActuationStatusStamped>::SharedPtr sub_actuation_status_;
  rclcpp::Subscription<SteeringAngleStatus>::SharedPtr sub_steering_angle_status_;
  rclcpp::Subscription<EngageStatus>::SharedPtr sub_engage_status_;
  void callbackSteeringAngleStatus(const SteeringAngleStatus::ConstSharedPtr msg);
  void onActuationCmd(const ActuationCommandStamped::ConstSharedPtr msg);
  void onActuationStatus(const ActuationStatusStamped::ConstSharedPtr msg);
  void onEngageStatus(const EngageStatus::SharedPtr msg);
};

#endif  // CALIBRATION_ADAPTER__CALIBRATION_ADAPTER_NODE_BASE_HPP_
