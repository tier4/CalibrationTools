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

#ifndef EXTRINSIC_CALIBRATION_MANAGER__EXTRINSIC_CALIBRATION_MANAGER_NODE_HPP_
#define EXTRINSIC_CALIBRATION_MANAGER__EXTRINSIC_CALIBRATION_MANAGER_NODE_HPP_

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include "tier4_calibration_msgs/srv/extrinsic_calibration_manager.hpp"
#include "tier4_calibration_msgs/srv/extrinsic_calibrator.hpp"

class ExtrinsicCalibrationManagerNode : public rclcpp::Node
{
public:
  explicit ExtrinsicCalibrationManagerNode(const rclcpp::NodeOptions & options);
  void spin();

private:
  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrationManager>::SharedPtr server_;
  std::vector<rclcpp::Client<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr> clients_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  struct TargetClient
  {
    std::string parent_frame;
    std::string child_frame;
    tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request request;
    tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response response;
    rclcpp::Client<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr client;
    bool estimated = false;
  };

  std::vector<TargetClient> target_clients_;
  std::string dst_path_;
  YAML::Node yaml_node_;

  static constexpr int yaml_precision_ = 6;

  std::string parent_frame_;
  std::vector<std::string> child_frames_;
  std::string client_ns_;
  double threshold_;

  void calibrationRequestCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrationManager::Request>
      request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrationManager::Response>
      response);

  bool createTargetClient(
    const YAML::Node & yaml_node, const std::string & parent_frame, const std::string & child_frame,
    const std::string & client_ns, const rclcpp::CallbackGroup::SharedPtr & callback_group,
    TargetClient & target_client);
  geometry_msgs::msg::Pose getPoseFromYaml(
    const YAML::Node & yaml_node, const std::string & parent_frame,
    const std::string & child_frame);
  bool dumpCalibrationConfig(
    const std::string & path, const std::vector<TargetClient> & target_clients);
};

#endif  // EXTRINSIC_CALIBRATION_MANAGER__EXTRINSIC_CALIBRATION_MANAGER_NODE_HPP_
