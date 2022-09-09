// Copyright 2021 Tier IV, Inc.
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

#ifndef VEHICLE_CMD_ANALYZER__VEHICLE_CMD_ANALYZER_HPP_
#define VEHICLE_CMD_ANALYZER__VEHICLE_CMD_ANALYZER_HPP_

#include "vehicle_cmd_analyzer/debug_values.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

class VehicleCmdAnalyzer : public rclcpp::Node
{
private:
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    sub_vehicle_cmd_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr pub_debug_;
  rclcpp::TimerBase::SharedPtr timer_control_;

  std::shared_ptr<autoware_auto_control_msgs::msg::AckermannControlCommand> vehicle_cmd_ptr_{
    nullptr};

  // timer callback
  double control_rate_;
  double wheelbase_;

  // for calculating dt
  std::shared_ptr<rclcpp::Time> prev_control_time_{nullptr};

  double prev_target_vel_;
  std::array<double, 3> prev_target_d_vel_ = {};
  double prev_target_acc_;

  // debug values
  DebugValues debug_values_;

  void callbackVehicleCommand(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);

  void callbackTimerControl();

  void publishDebugData();

  double getDt();
  std::pair<double, double> differentiateVelocity(const double dt);
  double differentiateAcceleration(const double dt);
  double calcLateralAcceleration() const;

public:
  explicit VehicleCmdAnalyzer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

#endif  // VEHICLE_CMD_ANALYZER__VEHICLE_CMD_ANALYZER_HPP_
