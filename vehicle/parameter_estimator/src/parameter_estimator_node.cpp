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

#include "parameter_estimator/parameter_estimator_node.hpp"

#include <memory>
#include <utility>
#include <vector>

ParameterEstimatorNode::ParameterEstimatorNode(const rclcpp::NodeOptions & node_options)
: Node("parameter_estimator", node_options)
{
  using std::placeholders::_1;

  // QoS setup
  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);
  durable_qos.transient_local();  // option for latching

  // get parameter
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  wheel_base_ = vehicle_info.wheel_base_m;
  use_auto_mode_ = this->declare_parameter<bool>("use_auto_mode", true);
  update_hz_ = this->declare_parameter<double>("update_hz", 10.0);
  covariance_ = this->declare_parameter<double>("initial_covariance", 1.0);
  forgetting_factor_ = this->declare_parameter<double>("forgetting_factor", 0.999);
  select_gear_ratio_estimator = this->declare_parameter<bool>("select_gear_ratio_estimator", true);
  select_steer_offset_estimator =
    this->declare_parameter<bool>("select_steer_offset_estimator", true);
  select_wheel_base_estimator = this->declare_parameter<bool>("select_wheel_base_estimator", true);
  params_.valid_max_steer_rad = this->declare_parameter<double>("valid_max_steer_rad", 0.05);
  params_.valid_min_velocity = this->declare_parameter<double>("valid_min_velocity", 0.5);
  params_.valid_min_angular_velocity =
    this->declare_parameter<double>("valid_min_angular_velocity", 0.1);
  params_.is_showing_debug_info = this->declare_parameter<bool>("is_showing_debug_info", true);

  const auto estimated_gear_ratio =
    this->declare_parameter<std::vector<double>>("gear_ratio", {15.7, 0.053, 0.047});
  steer_offset_estimator_ =
    std::make_unique<SteerOffsetEstimator>(this, params_, covariance_, forgetting_factor_, 0);
  wheel_base_estimator_ = std::make_unique<WheelBaseEstimator>(
    this, params_, covariance_, forgetting_factor_, wheel_base_);
  gear_ratio_estimator_ = std::make_unique<GearRatioEstimator>(
    this, params_, covariance_, forgetting_factor_, estimated_gear_ratio);

  // subscriber
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "input/imu_twist", queue_size, std::bind(&ParameterEstimatorNode::callbackImu, this, _1));
  sub_vehicle_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/vehicle_twist", queue_size,
    std::bind(&ParameterEstimatorNode::callbackVehicleTwist, this, _1));

  if (select_gear_ratio_estimator) {
    sub_steer_wheel_ = create_subscription<tier4_calibration_msgs::msg::Float32Stamped>(
      "input/handle_status", queue_size,
      std::bind(&ParameterEstimatorNode::callbackSteerWheel, this, _1));
  }

  if (select_steer_offset_estimator || select_wheel_base_estimator) {
    sub_steer_ = create_subscription<tier4_calibration_msgs::msg::Float32Stamped>(
      "input/steer", queue_size, std::bind(&ParameterEstimatorNode::callbackSteer, this, _1));
  }
  sub_control_mode_report_ =
    create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
      "input/control_mode", queue_size,
      std::bind(&ParameterEstimatorNode::callbackControlModeReport, this, _1));

  initTimer(1.0 / update_hz_);
}

void ParameterEstimatorNode::initTimer(double period_s)
{
  auto timer_callback = std::bind(&ParameterEstimatorNode::timerCallback, this);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void ParameterEstimatorNode::timerCallback()
{
  auto & clk = *this->get_clock();
  const auto & is_debug = params_.is_showing_debug_info;
  if (!vehicle_twist_ptr_) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("parameter_estimator"), is_debug, "vehicle_twist_ptr_ is null");
    return;
  } else if (!steer_ptr_ && (select_steer_offset_estimator || select_wheel_base_estimator)) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("parameter_estimator"), is_debug, "steer_ptr_ is null");
    return;
  } else if (!steer_wheel_ptr_ && select_gear_ratio_estimator) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("parameter_estimator"), is_debug, "steer_wheel_ptr_ is null");
    return;
  } else if (!imu_ptr_) {
    RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("parameter_estimator"), is_debug, "imu_ptr_ is null");
    return;
  } else if (auto_mode_duration_ < 0.5 && use_auto_mode_) {
    RCLCPP_INFO_STREAM_THROTTLE(
      rclcpp::get_logger("parameter_estimator"), clk, 5000, "not auto mode");
    return;
  } else {
    RCLCPP_INFO_STREAM_THROTTLE(
      rclcpp::get_logger("parameter_estimator"), clk, 5000, "running parameter estimator");
  }

  {
    VehicleData v = {};
    v.velocity = vehicle_twist_ptr_->twist.linear.x;
    v.angular_velocity = -imu_ptr_->angular_velocity.z;
    if (select_steer_offset_estimator || select_wheel_base_estimator) {
      v.steer = steer_ptr_->data;
    }
    if (select_gear_ratio_estimator) {
      v.handle = steer_wheel_ptr_->data;
    }
    v.wheel_base = wheel_base_;
    if (select_steer_offset_estimator) {
      steer_offset_estimator_->setData(v);
      steer_offset_estimator_->processData();
      steer_offset_estimator_->Run();
    }
    if (select_wheel_base_estimator) {
      wheel_base_estimator_->setData(v);
      wheel_base_estimator_->processData();
      wheel_base_estimator_->Run();
    }
    if (select_gear_ratio_estimator) {
      gear_ratio_estimator_->setData(v);
      gear_ratio_estimator_->processData();
      gear_ratio_estimator_->Run();
    }
  }
}

void ParameterEstimatorNode::callbackVehicleTwist(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  vehicle_twist_ptr_ = msg;
}

void ParameterEstimatorNode::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  imu_ptr_ = msg;
}

void ParameterEstimatorNode::callbackSteer(
  const tier4_calibration_msgs::msg::Float32Stamped::ConstSharedPtr msg)
{
  steer_ptr_ = msg;
}

void ParameterEstimatorNode::callbackSteerWheel(
  const tier4_calibration_msgs::msg::Float32Stamped::ConstSharedPtr msg)
{
  steer_wheel_ptr_ = msg;
}

void ParameterEstimatorNode::callbackControlModeReport(
  const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg)
{
  auto & clk = *this->get_clock();
  control_mode_ptr_ = msg;
  if (control_mode_ptr_->mode == autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS) {
    auto_mode_duration_ = (this->now().seconds() - last_manual_time_);
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("parameter_estimator"), clk, 5000,
      "[parameter_estimator] control mode duration: " << auto_mode_duration_);
  } else {
    auto_mode_duration_ = 0;
    last_manual_time_ = this->now().seconds();
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("parameter_estimator"), clk, 5000,
      "[parameter_estimator] control mode : manual");
  }
}
