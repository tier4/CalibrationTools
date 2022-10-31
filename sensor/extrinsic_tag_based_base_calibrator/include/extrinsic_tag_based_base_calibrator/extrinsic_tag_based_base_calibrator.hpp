// Copyright 2022 Tier IV, Inc.
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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__EXTRINSIC_TAG_BASED_BASE_CALIBRATOR_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__EXTRINSIC_TAG_BASED_BASE_CALIBRATOR_HPP_

#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <unordered_set>
#include <vector>

class ExtrinsicTagBasedBaseCalibrator : public rclcpp::Node
{
public:
  explicit ExtrinsicTagBasedBaseCalibrator(const rclcpp::NodeOptions & options);

protected:
  void visualizationTimerCallback();
  std_msgs::msg::ColorRGBA getNextColor();

  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  double waypoint_tag_size_;
  double wheel_tag_size_;
  double ground_tag_size_;
  std::vector<int> waypoint_tag_ids_;
  int left_wheel_tag_id_;
  int right_wheel_tag_id_;
  std::vector<int> wheel_tag_ids_;
  std::vector<int> ground_tag_ids_;
  std::vector<int> intrinsic_calibration_tag_ids_;

  std::unordered_set<int> waypoint_tag_ids_set_;
  std::unordered_set<int> wheel_tag_ids_set_;
  std::unordered_set<int> ground_tag_ids_set_;

  std::vector<CalibrationScene> scenes_;

  std::size_t next_color_index_;
  std::vector<std_msgs::msg::ColorRGBA> precomputed_colors_;
};

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__EXTRINSIC_TAG_BASED_BASE_CALIBRATOR_HPP_
