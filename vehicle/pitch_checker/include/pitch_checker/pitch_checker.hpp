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

#ifndef PITCH_CHECKER__PITCH_CHECKER_HPP_
#define PITCH_CHECKER__PITCH_CHECKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/utils.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif
#include "tier4_autoware_utils/ros/transform_listener.hpp"

struct TfInfo
{
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
};

class PitchChecker : public rclcpp::Node
{
public:
  explicit PitchChecker(const rclcpp::NodeOptions & node_options);

private:
  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void initTimer(double period_s);

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_flag_server_;

  std::vector<TfInfo> tf_info_vec_;
  std::map<std::pair<int, int>, std::vector<TfInfo>> tf_map_;
  std::map<std::pair<int, int>, std::vector<TfInfo>> tf_map_unified_;
  std::string output_file_;
  double update_hz_;
  int get_tf_count_ = 0;
  static constexpr int MAX_TF_COUNT_ = 1e08;

  void timerCallback();
  bool onSaveService(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  bool getTf();
  void pitchInfoToMap();
  bool writeMap();
  TfInfo getMedianPitchTfInfo(const std::vector<TfInfo> & tf_info_vec);
};

#endif  // PITCH_CHECKER__PITCH_CHECKER_HPP_
