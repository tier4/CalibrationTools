// Copyright 2024 TIER IV, Inc.
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

#ifndef TIER4_TAG_UTILS__APRILTAG_FILTER_HPP_
#define TIER4_TAG_UTILS__APRILTAG_FILTER_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_tag_utils/apriltag_hypothesis.hpp>

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace tier4_tag_utils
{

class ApriltagFilter : public rclcpp::Node
{
public:
  explicit ApriltagFilter(const rclcpp::NodeOptions & options);

protected:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);

  void detectionsCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr detections_msg);

  void publishFilteredDetections(const std_msgs::msg::Header & header);

  // Parameters
  std::unordered_map<std::string, double> tag_sizes_map_;

  double max_no_observation_time_;
  double new_hypothesis_translation_;
  double measurement_noise_translation_;
  double process_noise_translation_;
  double min_tag_size_;
  double max_tag_distance_;
  double max_allowed_homography_error_;
  int max_hamming_error_;
  double min_margin_;

  // ROS Interface
  rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
    apriltag_detections_array_sub_;

  // ROS Data
  sensor_msgs::msg::CameraInfo camera_info_;
  image_geometry::PinholeCameraModel pinhole_camera_model_;

  std::unordered_map<std::string, ApriltagHypothesis> hypotheses_map_;
  std::unordered_map<std::string, apriltag_msgs::msg::AprilTagDetection> detection_map_;
};

}  // namespace tier4_tag_utils

#endif  // TIER4_TAG_UTILS__APRILTAG_FILTER_HPP_
