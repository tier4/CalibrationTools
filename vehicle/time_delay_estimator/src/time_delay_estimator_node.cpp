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

#include "time_delay_estimator/time_delay_estimator_node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

double validateRange(
  rclcpp::Node * node, const double min, const double max, const double val, std::string name)
{
  auto & clk = *node->get_clock();
  if (min < std::abs(val) && std::abs(val) < max) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk, 5000,
      "[time_delay_estimator] " << name << ": " << val);
    return val;
  }
  RCLCPP_DEBUG_STREAM_THROTTLE(
    rclcpp::get_logger("time_delay_estimator"), clk, 5000, name << " is out of range: " << val);
  return 0.0;
}

double addOffset(const double val, const double offset) { return val + offset; }

TimeDelayEstimatorNode::TimeDelayEstimatorNode(const rclcpp::NodeOptions & node_options)
: Node("time_delay_estimator", node_options)
{
  using std::placeholders::_1;

  // QoS setup
  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);
  durable_qos.transient_local();  // option for latching

  // get parameter
  detect_manual_engage_ = this->declare_parameter<bool>("detect_manual_engage", true);
  params_.sampling_hz = this->declare_parameter<double>("data/sampling_hz", 30.0);
  params_.estimation_hz = this->declare_parameter<double>("data/estimation_hz", 10.0);
  params_.sampling_duration = this->declare_parameter<double>("data/sampling_duration", 5.0);
  params_.validation_duration = this->declare_parameter<double>("data/validation_duration", 1.0);
  params_.valid_peak_cross_correlation_threshold =
    this->declare_parameter<double>("data/valid_peak_cross_correlation_threshold", 0.8);
  params_.valid_delay_index_ratio =
    this->declare_parameter<double>("data/valid_delay_index_ratio", 0.1);
  params_.cutoff_hz_input = this->declare_parameter<double>("filter/cutoff_hz_input", 0.5);
  params_.cutoff_hz_output = this->declare_parameter<double>("filter/cutoff_hz_output", 0.1);
  params_.is_showing_debug_info = this->declare_parameter<bool>("is_showing_debug_info", true);
  params_.is_test_mode = this->declare_parameter<bool>("test/is_test_mode", false);
  params_.num_interpolation = this->declare_parameter<int>("data/num_interpolation", 3);
  params_.reset_at_disengage = this->declare_parameter<bool>("reset_at_disengage", false);
  estimator_type_ = this->declare_parameter<std::string>("estimator_type", "cc");
  bool use_weight_for_cross_correlation =
    this->declare_parameter<bool>("use_weight_for_cross_correlation", false);
  params_.sampling_delta_time = 1.0 / params_.sampling_hz;
  params_.estimation_delta_time = 1.0 / params_.estimation_hz;
  params_.data_size = static_cast<int>(params_.sampling_hz * params_.sampling_duration);
  params_.validation_size = static_cast<int>(params_.sampling_hz * params_.validation_duration);
  valid_steer_.min = this->declare_parameter<double>("steer/valid_min_steer", 0.05);
  valid_steer_.max = this->declare_parameter<double>("steer/valid_max_steer", 1.0);
  steer_offset_ = this->declare_parameter<double>("steer/offset_value", 0.0);
  valid_accel_.min = this->declare_parameter<double>("accel/valid_min_accel", 0.05);
  valid_accel_.max = this->declare_parameter<double>("accel/valid_max_accel", 1.0);
  accel_offset_ = this->declare_parameter<double>("accel/offset_value", 0.0);
  valid_brake_.min = this->declare_parameter<double>("brake/valid_min_brake", 0.05);
  valid_brake_.max = this->declare_parameter<double>("brake/valid_max_brake", 1.0);
  brake_offset_ = this->declare_parameter<double>("brake/offset_value", 0.0);

  last_manual_time_ = this->now().seconds();
  // input
  sub_control_mode_ = create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "~/input/control_mode", queue_size,
    std::bind(&TimeDelayEstimatorNode::callbackControlModeReport, this, _1));

  // response
  sub_accel_cmd_ = create_subscription<Float32Stamped>(
    "~/input/accel_cmd", queue_size,
    std::bind(&TimeDelayEstimatorNode::callbackAccelCmd, this, _1));
  sub_brake_cmd_ = create_subscription<Float32Stamped>(
    "~/input/brake_cmd", queue_size,
    std::bind(&TimeDelayEstimatorNode::callbackBrakeCmd, this, _1));
  sub_steer_cmd_ = create_subscription<Float32Stamped>(
    "~/input/steer_cmd", queue_size,
    std::bind(&TimeDelayEstimatorNode::callbackSteerCmd, this, _1));
  sub_accel_status_ = create_subscription<Float32Stamped>(
    "~/input/accel_status", queue_size,
    std::bind(&TimeDelayEstimatorNode::callbackAccelStatus, this, _1));
  sub_brake_status_ = create_subscription<Float32Stamped>(
    "~/input/brake_status", queue_size,
    std::bind(&TimeDelayEstimatorNode::callbackBrakeStatus, this, _1));
  sub_steer_status_ = create_subscription<Float32Stamped>(
    "~/input/steer_status", queue_size,
    std::bind(&TimeDelayEstimatorNode::callbackSteerStatus, this, _1));
  sub_is_engaged_ = create_subscription<BoolStamped>(
    "~/input/is_engage", queue_size, std::bind(&TimeDelayEstimatorNode::callbackEngage, this, _1));

  params_.total_data_size =
    static_cast<int>(params_.sampling_duration * params_.sampling_hz * params_.num_interpolation) +
    1;
  accel_data_ = std::make_unique<TimeDelayEstimator>(
    this, params_, "accel", params_.total_data_size, use_weight_for_cross_correlation);
  brake_data_ = std::make_unique<TimeDelayEstimator>(
    this, params_, "brake", params_.total_data_size, use_weight_for_cross_correlation);
  steer_data_ = std::make_unique<TimeDelayEstimator>(
    this, params_, "steer", params_.total_data_size, use_weight_for_cross_correlation);
  if (params_.is_test_mode) {
    test_data_ = std::make_unique<TimeDelayEstimator>(
      this, params_, "test", params_.total_data_size, use_weight_for_cross_correlation);
  }

  pub_time_delay_test_ = create_publisher<TimeDelay>("~/output/test_time_delay", durable_qos);
  pub_time_delay_accel_ = create_publisher<TimeDelay>("~/output/accel_cmd_delay", durable_qos);
  pub_time_delay_steer_ = create_publisher<TimeDelay>("~/output/steer_cmd_delay", durable_qos);
  pub_time_delay_brake_ = create_publisher<TimeDelay>("~/output/brake_cmd_delay", durable_qos);

  const auto period_s = params_.sampling_delta_time;
  // data processing callback
  {
    auto data_processing_callback = std::bind(&TimeDelayEstimatorNode::timerDataCollector, this);
    const auto period_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
    timer_data_processing_ =
      std::make_shared<rclcpp::GenericTimer<decltype(data_processing_callback)>>(
        this->get_clock(), period_ns, std::move(data_processing_callback),
        this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_data_processing_, nullptr);
  }
  // estimation callback
  {
    auto estimation_callback = std::bind(&TimeDelayEstimatorNode::estimateTimeDelay, this);
    const auto period_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
    timer_estimation_ = std::make_shared<rclcpp::GenericTimer<decltype(estimation_callback)>>(
      this->get_clock(), period_ns, std::move(estimation_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_estimation_, nullptr);
  }
}

void TimeDelayEstimatorNode::timerDataCollector()
{
  if (params_.is_test_mode) {
    callbackTestData();
    test_data_->preprocessData(this);
  }
  if (std::min(auto_mode_duration_, engage_duration_) < 5.0 && detect_manual_engage_) {
    if (params_.reset_at_disengage) {
      accel_data_->resetEstimator();
      brake_data_->resetEstimator();
      steer_data_->resetEstimator();
    }
    return;
  }
  // ====accel delay estimation
  if (accel_status_ptr_ && accel_cmd_ptr_) {
    accel_data_->preprocessData(this);
  }
  if (brake_status_ptr_ && brake_cmd_ptr_) {
    brake_data_->preprocessData(this);
  }
  if (steer_status_ptr_ && steer_cmd_ptr_) {
    steer_data_->preprocessData(this);
  }
}
bool TimeDelayEstimatorNode::estimateTimeDelay()
{
  std::chrono::system_clock::time_point start, end;
  start = std::chrono::system_clock::now();

  if (params_.is_test_mode) {
    // ====test delay estimation
    pub_time_delay_test_->publish(test_data_->estimateTimeDelay(this, estimator_type_));
  }

  if (std::min(auto_mode_duration_, engage_duration_) < 5.0 && detect_manual_engage_) {
    return false;
  }
  auto & clk = *this->get_clock();

  // ====accel delay estimation
  if (!accel_status_ptr_ || !accel_cmd_ptr_) {
    RCLCPP_INFO_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk, 5000,
      "[time_delay_estimator] : empty accel ptr");
  } else {
    pub_time_delay_accel_->publish(accel_data_->estimateTimeDelay(this, estimator_type_));
  }

  // ====brake delay estimation
  if (!brake_status_ptr_ || !brake_cmd_ptr_) {
    RCLCPP_INFO_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk, 5000,
      "[time_delay_estimator] : empty brake ptr");
  } else {
    pub_time_delay_brake_->publish(brake_data_->estimateTimeDelay(this, estimator_type_));
  }
  // ====steer delay estimation
  if (!steer_status_ptr_ || !steer_cmd_ptr_) {
    RCLCPP_INFO_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk, 5000,
      "[time_delay_estimator] : empty steer ptr");
  } else {
    pub_time_delay_steer_->publish(steer_data_->estimateTimeDelay(this, estimator_type_));
  }

  end = std::chrono::system_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  RCLCPP_DEBUG_STREAM_THROTTLE(
    rclcpp::get_logger("time_delay_estimator"), clk, 10000, "duration[microSec]: " << elapsed);
  return true;
}

void TimeDelayEstimatorNode::callbackAccelCmd(const Float32Stamped::ConstSharedPtr msg)
{
  accel_cmd_ptr_ = msg;
  const auto & t = this->get_clock()->now().seconds();
  // accel
  double accel_input =
    validateRange(this, valid_accel_.min, valid_accel_.max, accel_cmd_ptr_->data, "accel input");
  const double accel_input_offset = addOffset(accel_input, accel_offset_);
  accel_data_->input_.setValue(accel_input_offset, t);
}

void TimeDelayEstimatorNode::callbackBrakeCmd(const Float32Stamped::ConstSharedPtr msg)
{
  brake_cmd_ptr_ = msg;
  const auto & t = this->get_clock()->now().seconds();
  // brake
  double brake_input =
    validateRange(this, valid_brake_.min, valid_brake_.max, brake_cmd_ptr_->data, "brake input");
  const double brake_input_offset = addOffset(brake_input, brake_offset_);
  brake_data_->input_.setValue(brake_input_offset, t);
}

void TimeDelayEstimatorNode::callbackSteerCmd(const Float32Stamped::ConstSharedPtr msg)
{
  steer_cmd_ptr_ = msg;
  const auto & t = this->get_clock()->now().seconds();
  double steer_input =
    validateRange(this, valid_steer_.min, valid_steer_.max, steer_cmd_ptr_->data, "steer input");
  steer_input = math_utils::normalize(steer_input, -1.0, 1.0);
  const double steer_input_offset = addOffset(steer_input, steer_offset_);
  steer_data_->input_.setValue(steer_input_offset, t);
}

void TimeDelayEstimatorNode::callbackAccelStatus(const Float32Stamped::ConstSharedPtr msg)
{
  accel_status_ptr_ = msg;
  const auto & t = this->get_clock()->now().seconds();
  double accel_response = validateRange(
    this, valid_accel_.min, valid_accel_.max, accel_status_ptr_->data, "accel response");
  const double accel_response_offset = addOffset(accel_response, accel_offset_);
  accel_data_->response_.setValue(accel_response_offset, t);
}

void TimeDelayEstimatorNode::callbackBrakeStatus(const Float32Stamped::ConstSharedPtr msg)
{
  brake_status_ptr_ = msg;
  const auto & t = this->get_clock()->now().seconds();
  double brake_response = validateRange(
    this, valid_brake_.min, valid_brake_.max, brake_status_ptr_->data, "brake response");
  const double brake_response_offset = addOffset(brake_response, brake_offset_);
  brake_data_->response_.setValue(brake_response_offset, t);
}

void TimeDelayEstimatorNode::callbackSteerStatus(const Float32Stamped::ConstSharedPtr msg)
{
  steer_status_ptr_ = msg;
  const auto & t = this->get_clock()->now().seconds();
  double steer_response = validateRange(
    this, valid_steer_.min, valid_steer_.max, steer_status_ptr_->data, "steer response");
  steer_response = math_utils::normalize(steer_response, -1.0, 1.0);
  const double steer_response_offset = addOffset(steer_response, steer_offset_);
  steer_data_->response_.setValue(steer_response_offset, t);
}

void TimeDelayEstimatorNode::callbackControlModeReport(const ControlModeReport::ConstSharedPtr msg)
{
  auto & clk = *this->get_clock();
  control_mode_ptr_ = msg;
  if (control_mode_ptr_->mode == autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS) {
    auto_mode_duration_ = (this->now().seconds() - last_manual_time_);
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk, 5000,
      "[time_delay_estimator] control mode duration: " << auto_mode_duration_);
  } else {
    auto_mode_duration_ = 0;
    last_manual_time_ = this->now().seconds();
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk, 5000,
      "[time_delay_estimator] control mode : manual");
  }
}

void TimeDelayEstimatorNode::callbackEngage(const IsEngaged::ConstSharedPtr msg)
{
  engage_mode_ = msg->data;
  auto & clk = *this->get_clock();
  if (engage_mode_) {
    engage_duration_ = (this->now().seconds() - last_disengage_time_);
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk, 5000,
      "[time_delay_estimator] engage duration: " << engage_duration_);
  } else {
    engage_duration_ = 0;
    last_disengage_time_ = this->now().seconds();
    RCLCPP_INFO_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk, 5000,
      "[time_delay_estimator] engage mode : disengage");
  }
}

void TimeDelayEstimatorNode::callbackTestData()
{
  cnt_++;
  double dt = 1.0 / params_.sampling_hz;
  static double delay;
  if (cnt_ % 500 == 0) {
    if (delay < 0.6) {
      delay += 0.1;
    } else {
      delay = 0;
    }
  }
  auto & clk = *this->get_clock();
  RCLCPP_DEBUG_STREAM_THROTTLE(
    rclcpp::get_logger("time_delay_estimator"), clk, 1000,
    "[time_delay_estimator] delay: " << delay << " cnt_: " << cnt_);
  double t = cnt_ * dt;
  const double w = 2.0 * M_PI * 0.5;
  const double Amp = 0.5;
  const double multiplier = 0.1 * (cnt_ % 100);
  const double base = 0.5;
  double input = Amp * multiplier * std::sin(w * t) + base;
  double response = Amp * multiplier * std::sin(w * (t - delay)) + base;
  input = math_utils::normalize(input, Amp * 10 + base, -Amp * 10 + base);
  response = math_utils::normalize(response, Amp * 10 + base, -Amp * 10 + base);
  test_data_->input_.setValue(input, t);
  test_data_->response_.setValue(response, t);
}
