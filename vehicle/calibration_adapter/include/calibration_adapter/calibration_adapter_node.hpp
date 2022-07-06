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

#ifndef CALIBRATION_ADAPTER__CALIBRATION_ADAPTER_NODE_HPP_
#define CALIBRATION_ADAPTER__CALIBRATION_ADAPTER_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "calibration_adapter/calibration_adapter_node_base.hpp"
#include "tier4_calibration_msgs/msg/float32_stamped.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "estimator_utils/math_utils.hpp"

class CalibrationAdapterNode : public CalibrationAdapterNodeBase
{
  using Velocity = autoware_auto_vehicle_msgs::msg::VelocityReport;
  using ControlCommandStamped = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
public:
  CalibrationAdapterNode();

private:
  double acceleration_ = 0.0;
  const double dif_twist_time_ = 0.2;   // 200ms
  const std::size_t twist_vec_max_size_ = 100;
  double lowpass_cutoff_value_;
  std::vector<TwistStamped> twist_vec_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_acceleration_status_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_acceleration_cmd_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_steering_angle_cmd_;
  rclcpp::Subscription<ControlCommandStamped>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<Velocity>::SharedPtr sub_twist_;
  template<class T>
  T getNearestTimeDataFromVec(
    const T base_data, const double back_time, const std::vector<T> & vec);
  double getAccel(
    const TwistStamped & prev_twist,
    const TwistStamped & current_twist,
    const double dt);
  template<class T>
  void pushDataToVec(const T data, const std::size_t max_size, std::vector<T> * vec);
  void callbackControlCmd(const ControlCommandStamped::ConstSharedPtr msg);
  void callbackTwistStatus(const Velocity::ConstSharedPtr msg);
};

#endif  // CALIBRATION_ADAPTER__CALIBRATION_ADAPTER_NODE_HPP_
