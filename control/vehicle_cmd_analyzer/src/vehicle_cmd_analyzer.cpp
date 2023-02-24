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

#include "vehicle_cmd_analyzer/vehicle_cmd_analyzer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

VehicleCmdAnalyzer::VehicleCmdAnalyzer(const rclcpp::NodeOptions & options)
: Node("vehicle_cmd_analyzer", options)
{
  control_rate_ = declare_parameter("control_rate", 30.0);

  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  wheelbase_ = vehicle_info.wheel_base_m;

  sub_vehicle_cmd_ =
    this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd", rclcpp::QoS(10),
      std::bind(&VehicleCmdAnalyzer::callbackVehicleCommand, this, std::placeholders::_1));
  pub_debug_ = create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>(
    "~/debug_values", rclcpp::QoS{1});

  // Timer
  {
    auto timer_callback = std::bind(&VehicleCmdAnalyzer::callbackTimerControl, this);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_));
    timer_control_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      this->get_clock(), period, std::move(timer_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_control_, nullptr);
  }
}

void VehicleCmdAnalyzer::callbackVehicleCommand(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
  vehicle_cmd_ptr_ =
    std::make_shared<autoware_auto_control_msgs::msg::AckermannControlCommand>(*msg);
}

void VehicleCmdAnalyzer::callbackTimerControl()
{
  // wait for initial pointers
  if (!vehicle_cmd_ptr_) {
    return;
  }

  // publish debug data
  publishDebugData();
}

void VehicleCmdAnalyzer::publishDebugData()
{
  const double dt = getDt();
  const double a_lat = calcLateralAcceleration();
  const auto [d_vel, dd_vel] = differentiateVelocity(dt);

  // set debug values
  debug_values_.setValues(DebugValues::TYPE::DT, dt);
  debug_values_.setValues(
    DebugValues::TYPE::CURRENT_TARGET_VEL, vehicle_cmd_ptr_->longitudinal.speed);
  debug_values_.setValues(DebugValues::TYPE::CURRENT_TARGET_D_VEL, d_vel);
  debug_values_.setValues(DebugValues::TYPE::CURRENT_TARGET_DD_VEL, dd_vel);
  debug_values_.setValues(
    DebugValues::TYPE::CURRENT_TARGET_ACC, vehicle_cmd_ptr_->longitudinal.acceleration);
  debug_values_.setValues(DebugValues::TYPE::CURRENT_TARGET_D_ACC, differentiateAcceleration(dt));
  debug_values_.setValues(DebugValues::TYPE::CURRENT_TARGET_LATERAL_ACC, a_lat);

  // publish debug values
  tier4_debug_msgs::msg::Float32MultiArrayStamped debug_msg{};
  debug_msg.stamp = this->now();
  for (const auto & v : debug_values_.getValues()) {
    debug_msg.data.push_back(v);
  }
  pub_debug_->publish(debug_msg);
}

double VehicleCmdAnalyzer::getDt()
{
  double dt;
  if (!prev_control_time_) {
    dt = 1.0 / control_rate_;
    prev_control_time_ = std::make_shared<rclcpp::Time>(this->now());
  } else {
    dt = (this->now() - *prev_control_time_).seconds();
    *prev_control_time_ = this->now();
  }
  const double max_dt = 1.0 / control_rate_ * 2.0;
  const double min_dt = 1.0 / control_rate_ * 0.5;
  return std::max(std::min(dt, max_dt), min_dt);
}

std::pair<double, double> VehicleCmdAnalyzer::differentiateVelocity(const double dt)
{
  if (!prev_target_vel_) {
    prev_target_vel_ = vehicle_cmd_ptr_->longitudinal.speed;
    prev_target_d_vel_.at(2) = 0.0;
    return {0.0, 0.0};
  }
  const double d_vel = (vehicle_cmd_ptr_->longitudinal.speed - prev_target_vel_) / dt;
  const double dd_vel = (d_vel - prev_target_d_vel_.at(0)) / 2 / dt;
  prev_target_vel_ = vehicle_cmd_ptr_->longitudinal.speed;
  for (int i = 0; i < 2; i++) {
    prev_target_d_vel_.at(i) = prev_target_d_vel_.at(i + 1);
  }
  prev_target_d_vel_.at(2) = d_vel;
  return {d_vel, dd_vel};
}

double VehicleCmdAnalyzer::differentiateAcceleration(const double dt)
{
  if (!prev_target_acc_) {
    prev_target_acc_ = vehicle_cmd_ptr_->longitudinal.acceleration;
    return 0.0;
  }
  const double d_acc = (vehicle_cmd_ptr_->longitudinal.acceleration - prev_target_acc_) / dt;
  prev_target_acc_ = vehicle_cmd_ptr_->longitudinal.acceleration;
  return d_acc;
}

double VehicleCmdAnalyzer::calcLateralAcceleration() const
{
  const double delta = vehicle_cmd_ptr_->lateral.steering_tire_angle;
  const double vel = vehicle_cmd_ptr_->longitudinal.speed;
  const double a_lat = vel * vel * std::sin(delta) / wheelbase_;
  return a_lat;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(VehicleCmdAnalyzer)
