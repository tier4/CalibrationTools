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

#ifndef EXTRINSIC_MANUAL_CALIBRATOR__EXTRINSIC_MANUAL_CALIBRATOR_NODE_HPP_
#define EXTRINSIC_MANUAL_CALIBRATOR__EXTRINSIC_MANUAL_CALIBRATOR_NODE_HPP_

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <string>

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif
#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tier4_calibration_msgs/srv/extrinsic_calibrator.hpp"

class ExtrinsicManualCalibratorNode : public rclcpp::Node
{
public:
  explicit ExtrinsicManualCalibratorNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr server_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::string tf_parameter_ns_;
  bool done_;

  void calibrationRequestCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);

  void calibrationDoneCallback(std_msgs::msg::Bool::ConstSharedPtr done);

  geometry_msgs::msg::Pose getTfParameters(const rclcpp::AsyncParametersClient::SharedPtr & client);
  void setTfParameters(
    const rclcpp::AsyncParametersClient::SharedPtr & client, const geometry_msgs::msg::Pose & pose);
};

#endif  // EXTRINSIC_MANUAL_CALIBRATOR__EXTRINSIC_MANUAL_CALIBRATOR_NODE_HPP_
