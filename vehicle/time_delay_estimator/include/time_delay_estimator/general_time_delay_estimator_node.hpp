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

#ifndef TIME_DELAY_ESTIMATOR__GENERAL_TIME_DELAY_ESTIMATOR_NODE_HPP_
#define TIME_DELAY_ESTIMATOR__GENERAL_TIME_DELAY_ESTIMATOR_NODE_HPP_

#include "estimator_utils/math_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "time_delay_estimator/data_processor.hpp"
#include "time_delay_estimator/parameters.hpp"
#include "time_delay_estimator/time_delay_estimator.hpp"

#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tier4_calibration_msgs/msg/bool_stamped.hpp"
#include "tier4_calibration_msgs/msg/float32_stamped.hpp"
#include "tier4_calibration_msgs/msg/time_delay.hpp"

#include <cmath>
#include <deque>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

class TimeDelayEstimatorNode : public rclcpp::Node
{
  using Float32Stamped = tier4_calibration_msgs::msg::Float32Stamped;
  using ControlModeReport = autoware_auto_vehicle_msgs::msg::ControlModeReport;
  using IsEngaged = tier4_calibration_msgs::msg::BoolStamped;
  using BoolStamped = tier4_calibration_msgs::msg::BoolStamped;
  using TimeDelay = tier4_calibration_msgs::msg::TimeDelay;

private:
  // output delay
  rclcpp::Publisher<TimeDelay>::SharedPtr pub_time_delay_;

  // input subscription
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    sub_control_mode_report_;

  // response subscription
  rclcpp::Subscription<Float32Stamped>::SharedPtr sub_input_cmd_;
  rclcpp::Subscription<Float32Stamped>::SharedPtr sub_input_status_;
  rclcpp::Subscription<BoolStamped>::SharedPtr sub_is_engaged_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_estimation_;
  rclcpp::TimerBase::SharedPtr timer_data_processing_;

  Float32Stamped::ConstSharedPtr input_cmd_ptr_;
  Float32Stamped::ConstSharedPtr input_status_ptr_;
  ControlModeReport::ConstSharedPtr control_mode_ptr_;
  std::string estimator_type_;
  std::string data_name_;

  void callbackInputCmd(const Float32Stamped::ConstSharedPtr msg);
  void callbackInputStatus(const Float32Stamped::ConstSharedPtr msg);
  void callbackControlModeReport(const ControlModeReport::ConstSharedPtr msg);
  void callbackVehicleEngage(const ControlModeReport::ConstSharedPtr msg);
  void callbackEngage(const IsEngaged::ConstSharedPtr msg);

  // use diff instead of raw data
  double auto_mode_duration_ = 0;
  double last_manual_time_;
  double last_disengage_time_;
  double engage_duration_;
  bool detect_manual_engage_;
  bool engage_mode_ = false;
  // saturation
  MinMax valid_input_;
  double input_offset_;

  std::unique_ptr<TimeDelayEstimator> collected_data_;
  // for ros parameters
  Params params_;

  void timerCallback();
  void timerDataCollector();
  bool estimateTimeDelay();

public:
  explicit TimeDelayEstimatorNode(const rclcpp::NodeOptions & node_options);
  ~TimeDelayEstimatorNode() {}
};

#endif  // TIME_DELAY_ESTIMATOR__GENERAL_TIME_DELAY_ESTIMATOR_NODE_HPP_
