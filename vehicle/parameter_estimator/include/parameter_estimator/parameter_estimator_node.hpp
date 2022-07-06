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

#ifndef PARAMETER_ESTIMATOR__PARAMETER_ESTIMATOR_NODE_HPP_
#define PARAMETER_ESTIMATOR__PARAMETER_ESTIMATOR_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "tier4_calibration_msgs/msg/float32_stamped.hpp"
#include "estimator_utils/estimator_base.hpp"
#include "parameter_estimator/gear_ratio_estimator.hpp"
#include "parameter_estimator/parameters.hpp"
#include "parameter_estimator/steer_offset_estimator.hpp"
#include "parameter_estimator/wheel_base_estimator.hpp"

class ParameterEstimatorNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vehicle_twist_;
  rclcpp::Subscription<tier4_calibration_msgs::msg::Float32Stamped>::SharedPtr sub_steer_;
  rclcpp::Subscription<tier4_calibration_msgs::msg::Float32Stamped>::SharedPtr sub_steer_wheel_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    sub_control_mode_report_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void initTimer(double period_s);

  geometry_msgs::msg::TwistStamped::ConstSharedPtr vehicle_twist_ptr_;
  sensor_msgs::msg::Imu::ConstSharedPtr imu_ptr_;
  tier4_calibration_msgs::msg::Float32Stamped::ConstSharedPtr steer_ptr_;
  tier4_calibration_msgs::msg::Float32Stamped::ConstSharedPtr steer_wheel_ptr_;
  autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr control_mode_ptr_;

  /**
   * ros parameters
   **/
  Params params_;
  double use_auto_mode_;
  double wheel_base_;
  double update_hz_;
  double covariance_;
  double forgetting_factor_;
  bool select_gear_ratio_estimator;
  bool select_steer_offset_estimator;
  bool select_wheel_base_estimator;

  double auto_mode_duration_ = 0;
  double last_manual_time_ = 0;

  std::unique_ptr<SteerOffsetEstimator> steer_offset_estimator_;
  std::unique_ptr<WheelBaseEstimator> wheel_base_estimator_;
  std::unique_ptr<GearRatioEstimator> gear_ratio_estimator_;

  void callbackVehicleTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void callbackSteer(const tier4_calibration_msgs::msg::Float32Stamped::ConstSharedPtr msg);
  void callbackSteerWheel(const tier4_calibration_msgs::msg::Float32Stamped::ConstSharedPtr msg);
  void callbackControlModeReport(
    const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg);
  void timerCallback();
  bool updateGearRatio();

public:
  explicit ParameterEstimatorNode(const rclcpp::NodeOptions & node_options);
};

#endif  // PARAMETER_ESTIMATOR__PARAMETER_ESTIMATOR_NODE_HPP_
