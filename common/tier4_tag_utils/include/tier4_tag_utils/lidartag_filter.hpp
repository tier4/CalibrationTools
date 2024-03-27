// Copyright 2024 Tier IV, Inc.
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

#ifndef TIER4_TAG_UTILS__LIDARTAG_FILTER_HPP_
#define TIER4_TAG_UTILS__LIDARTAG_FILTER_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_tag_utils/lidartag_hypothesis.hpp>

#include <lidartag_msgs/msg/lidar_tag_detection_array.hpp>

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace tier4_tag_utils
{

class LidartagFilter : public rclcpp::Node
{
public:
  explicit LidartagFilter(const rclcpp::NodeOptions & options);

protected:
  void lidarTagDetectionsCallback(
    const lidartag_msgs::msg::LidarTagDetectionArray::SharedPtr detections_msg);

  void updateHypothesis(
    const lidartag_msgs::msg::LidarTagDetection & detection, const std_msgs::msg::Header & header);

  void publishFilteredDetections(const std_msgs::msg::Header & header);

  // Parameters
  double max_no_observation_time_;
  double new_hypothesis_distance_;
  double new_hypothesis_translation_;
  double new_hypothesis_rotation_;
  double measurement_noise_translation_;
  double measurement_noise_rotation_;
  double process_noise_translation_;
  double process_noise_translation_dot_;
  double process_noise_rotation_;
  double process_noise_rotation_dot_;

  // ROS Interface
  rclcpp::Publisher<lidartag_msgs::msg::LidarTagDetectionArray>::SharedPtr pub_;

  rclcpp::Subscription<lidartag_msgs::msg::LidarTagDetectionArray>::SharedPtr sub_;

  // Hypotheses
  std::unordered_map<int, LidartagHypothesis> hypotheses_map_;
  std::unordered_map<int, lidartag_msgs::msg::LidarTagDetection> detection_map_;
};

}  // namespace tier4_tag_utils

#endif  // TIER4_TAG_UTILS__LIDARTAG_FILTER_HPP_
