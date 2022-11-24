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

#include <rclcpp/time.hpp>
#include <tier4_tag_utils/apriltag_filter.hpp>
#include <tier4_tag_utils/types.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <tf2_eigen/tf2_eigen.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace tier4_tag_utils
{

ApriltagFilter::ApriltagFilter(const rclcpp::NodeOptions & options)
: Node("apriltag_filter_node", options)
{
  min_tag_size_ = this->declare_parameter<double>("min_tag_size");
  max_tag_distance_ = this->declare_parameter<double>("max_tag_distance");
  max_allowed_homography_error_ = this->declare_parameter<double>("max_allowed_homography_error");
  max_hamming_error_ = this->declare_parameter<int>("max_hamming_error");
  min_margin_ = this->declare_parameter<double>("min_margin");
  ;
  max_no_observation_time_ = this->declare_parameter<double>("max_no_observation_time");
  new_hypothesis_transl_ = this->declare_parameter<double>("new_hypothesis_transl");
  measurement_noise_transl_ = this->declare_parameter<double>("measurement_noise_transl");
  process_noise_transl_ = this->declare_parameter<double>("process_noise_transl");

  std::vector<int64_t> tag_ids = this->declare_parameter<std::vector<int64_t>>("tag_ids");
  std::vector<double> tag_sizes = this->declare_parameter<std::vector<double>>("tag_sizes");

  if (tag_ids.size() != tag_sizes.size()) {
    throw std::invalid_argument("Tag ids and sizes must be of the same size");
  }

  for (std::size_t i = 0; i < tag_ids.size(); i++) {
    tag_sizes_map_[tag_ids[i]] = tag_sizes[i];
  }

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::QoS(1).best_effort(),
    std::bind(&ApriltagFilter::cameraInfoCallback, this, std::placeholders::_1));

  apriltag_detections_array_sub_ =
    this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "apriltag/detection_array", 1,
      std::bind(&ApriltagFilter::detectionsCallback, this, std::placeholders::_1));

  pub_ = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
    "apriltag/filtered/detection_array", 10);
}

void ApriltagFilter::detectionsCallback(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr detections_msg)
{
  rclcpp::Time timestamp = rclcpp::Time(detections_msg->header.stamp);

  // Filter apriltag detections that are too far away from the sensor
  double max_distance_px = min_tag_size_ * pinhole_camera_model_.fx() / max_tag_distance_;

  apriltag_msgs::msg::AprilTagDetectionArray filtered_detections;
  filtered_detections.header = detections_msg->header;

  for (auto & detection : detections_msg->detections) {
    const int & corners_size = detection.corners.size();
    double max_side_distance = 0.0;
    double max_homography_error = 0.0;

    cv::Mat_<double> H(3, 3);
    std::memcpy(H.data, detection.homography.data(), 9 * sizeof(double));
    cv::Mat H_inv = H.inv();

    for (int i = 0; i < corners_size; i++) {
      const auto & p1 = detection.corners[i];
      const auto & p2 = detection.corners[(i + 1) % corners_size];

      cv::Mat_<double> p_corner(3, 1);
      p_corner(0, 0) = detection.corners[i].x;
      p_corner(1, 0) = detection.corners[i].y;
      p_corner(2, 0) = 1.0;

      cv::Mat p_corner2 = H_inv * p_corner;

      // According to the equation (x2, y2, 1) = H *(x1, y1, 1) the third component should be 1.0
      double h_error = std::abs(p_corner2.at<double>(2, 0) - 1.0);
      max_homography_error = std::max(max_homography_error, h_error);

      double side_distance = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
      max_side_distance = std::max(max_side_distance, side_distance);
    }

    // We discard detections that are theoretically detected too far away
    if (max_side_distance < max_distance_px) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Discarding apriltag: size " << max_side_distance
                                                   << " px. Expecting at least " << max_distance_px
                                                   << " px");
      continue;
    }

    // We also discard detections with an unreliable homography
    if (
      max_homography_error > max_allowed_homography_error_ ||
      detection.hamming > max_hamming_error_ || detection.decision_margin < min_margin_) {
      RCLCPP_WARN(
        get_logger(), "Discarding apriltag. h=%d m=%.2f", detection.hamming,
        detection.decision_margin);
      continue;
    }

    filtered_detections.detections.push_back(detection);
  }

  for (auto & detection : filtered_detections.detections) {
    detection_map_[detection.id] = detection;
    std::vector<cv::Point2d> corners;

    for (auto & c : detection.corners) {
      corners.push_back(cv::Point2d(c.x, c.y));
    }

    if (hypotheses_map_.count(detection.id) == 0) {
      hypotheses_map_[detection.id] = ApriltagHypothesis(detection.id, pinhole_camera_model_);
      auto & h = hypotheses_map_[detection.id];
      h.setMaxConvergenceThreshold(0.0);
      h.setMaxNoObservationTime(max_no_observation_time_);
      h.setMeasurementNoise(measurement_noise_transl_);
      h.setMinConvergenceTime(std::numeric_limits<double>::max());
      h.setNewHypothesisThreshold(new_hypothesis_transl_);
      h.setProcessNoise(process_noise_transl_);
      h.setTagSize(tag_sizes_map_[detection.id]);
      h.update(corners, timestamp);
    } else {
      hypotheses_map_[detection.id].update(corners, timestamp);
    }
  }

  std::vector<int> erase_list;
  for (auto it = hypotheses_map_.begin(); it != hypotheses_map_.end(); it++) {
    if (!it->second.update(timestamp)) {
      erase_list.push_back(it->first);
    }
  }

  for (auto id : erase_list) {
    hypotheses_map_.erase(id);
  }

  publishFilteredDetections(detections_msg->header);
}

void ApriltagFilter::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
{
  camera_info_ = *camera_info_msg;
  pinhole_camera_model_.fromCameraInfo(camera_info_);
}

void ApriltagFilter::publishFilteredDetections(const std_msgs::msg::Header & header)
{
  apriltag_msgs::msg::AprilTagDetectionArray filtered_detections;

  for (auto it = hypotheses_map_.begin(); it != hypotheses_map_.end(); it++) {
    const auto & base_detection = detection_map_[it->first];
    apriltag_msgs::msg::AprilTagDetection detection;
    detection = base_detection;

    auto filtered_points = it->second.getFilteredPoints2d();

    for (int i = 0; i < 4; i++) {
      detection.corners[i].x = filtered_points[i].x;
      detection.corners[i].y = filtered_points[i].y;
    }

    filtered_detections.detections.push_back(detection);
  }

  filtered_detections.header = header;
  pub_->publish(filtered_detections);
}

}  // namespace tier4_tag_utils

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_tag_utils::ApriltagFilter)
