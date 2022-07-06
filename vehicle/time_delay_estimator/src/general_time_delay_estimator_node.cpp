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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "time_delay_estimator/general_time_delay_estimator_node.hpp"

double validateRange(
  rclcpp::Node * node, const double min, const double max, const double val,
  std::string name)
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

double addOffset(const double val, const double offset)
{
  return val + offset;
}

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
  // params_.is_test_mode = this->declare_parameter<bool>("test/is_test_mode", false);
  params_.num_interpolation = this->declare_parameter<int>("data/num_interpolation", 3);
  params_.reset_at_disengage = this->declare_parameter<bool>("reset_at_disengage", false);
  bool use_weight_for_cross_correlation =
    this->declare_parameter<bool>("use_weight_for_cross_correlation", false);
  params_.sampling_delta_time = 1.0 / params_.sampling_hz;
  params_.estimation_delta_time = 1.0 / params_.estimation_hz;
  params_.data_size = static_cast<int>(params_.sampling_hz * params_.sampling_duration);
  params_.validation_size = static_cast<int>(params_.sampling_hz * params_.validation_duration);
  valid_input_.min = this->declare_parameter<double>("min_valid_value", 0.05);
  valid_input_.max = this->declare_parameter<double>("max_valid_value", 1.00);
  input_offset_ = this->declare_parameter<double>("offset_value", 0.0);
  data_name_ = this->declare_parameter<std::string>("data_name", "test");

  last_manual_time_ = this->now().seconds();
  // input
  sub_control_mode_report_ =
    create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "~/input/control_mode", queue_size,
    std::bind(&TimeDelayEstimatorNode::callbackControlModeReport, this, _1));

  // response
  sub_input_cmd_ = create_subscription<Float32Stamped>(
    "~/input/input_cmd", queue_size,
    std::bind(&TimeDelayEstimatorNode::callbackInputCmd, this, _1));
  sub_input_status_ = create_subscription<Float32Stamped>(
    "~/input/input_status", queue_size,
    std::bind(&TimeDelayEstimatorNode::callbackInputStatus, this, _1));
  sub_is_engaged_ = create_subscription<BoolStamped>(
    "~/input/is_engage", queue_size,
    std::bind(&TimeDelayEstimatorNode::callbackEngage, this, _1));

  params_.total_data_size =
    static_cast<int>(params_.sampling_duration * params_.sampling_hz * params_.num_interpolation) +
    1;
  collected_data_ = std::make_unique<TimeDelayEstimator>(
    this, params_, data_name_, params_.total_data_size, use_weight_for_cross_correlation);

  pub_time_delay_ = create_publisher<TimeDelay>("~/output/time_delay", durable_qos);

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
  if (std::min(auto_mode_duration_, engage_duration_) < 5.0 && detect_manual_engage_) {
    if (params_.reset_at_disengage) {
      collected_data_->resetEstimator();
    }
    return;
  }
  if (input_status_ptr_ && input_cmd_ptr_) {
    collected_data_->preprocessData(this);
  }
}
bool TimeDelayEstimatorNode::estimateTimeDelay()
{
  std::chrono::system_clock::time_point start, end;
  start = std::chrono::system_clock::now();

  if (std::min(auto_mode_duration_, engage_duration_) < 5.0 && detect_manual_engage_) {
    return false;
  }
  auto & clk = *this->get_clock();

  // ====data delay estimation
  if (!input_status_ptr_ || !input_cmd_ptr_) {
    RCLCPP_INFO_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk, 5000,
      "[time_delay_estimator] : empty input ptr");
  } else {
    pub_time_delay_->publish(collected_data_->estimateTimeDelay(this, "cc"));
  }

  end = std::chrono::system_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  RCLCPP_DEBUG_STREAM_THROTTLE(
    rclcpp::get_logger("time_delay_estimator"), clk, 10000, "duration[microSec]: " << elapsed);
  return true;
}


void TimeDelayEstimatorNode::callbackInputCmd(const Float32Stamped::ConstSharedPtr msg)
{
  input_cmd_ptr_ = msg;
  const auto & t = this->get_clock()->now().seconds();
  double input =
    validateRange(
    this, valid_input_.min, valid_input_.max,
    input_cmd_ptr_->data, data_name_);
  const double input_offset = addOffset(input, input_offset_);
  collected_data_->input_.setValue(input_offset, t);
}

void TimeDelayEstimatorNode::callbackInputStatus(const Float32Stamped::ConstSharedPtr msg)
{
  input_status_ptr_ = msg;
  const auto & t = this->get_clock()->now().seconds();
  double input_response = validateRange(
    this, valid_input_.min, valid_input_.max, input_status_ptr_->data,
    data_name_ + " response");
  const double input_response_offset = addOffset(input_response, input_offset_);
  collected_data_->response_.setValue(input_response_offset, t);
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
