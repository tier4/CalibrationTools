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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <rclcpp/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tier4_tag_utils/lidartag_filter.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tier4_tag_utils
{

LidartagFilter::LidartagFilter(const rclcpp::NodeOptions & options)
: Node("extrinsic_tag_based_calibrator_node", options)
{
  max_no_observation_time_ = this->declare_parameter<double>("max_no_observation_time");
  new_hypothesis_distance_ = this->declare_parameter<double>("new_hypothesis_distance");

  new_hypothesis_translation_ = this->declare_parameter<double>("new_hypothesis_translation");
  new_hypothesis_rotation_ = this->declare_parameter<double>("new_hypothesis_rotation");
  measurement_noise_translation_ = this->declare_parameter<double>("measurement_noise_translation");
  measurement_noise_rotation_ = this->declare_parameter<double>("measurement_noise_rotation");
  process_noise_translation_ = this->declare_parameter<double>("process_noise_translation");
  process_noise_translation_dot_ = this->declare_parameter<double>("process_noise_translation_dot");
  process_noise_rotation_ = this->declare_parameter<double>("process_noise_rotation");
  process_noise_rotation_dot_ = this->declare_parameter<double>("process_noise_rotation_dot");

  sub_ = this->create_subscription<lidartag_msgs::msg::LidarTagDetectionArray>(
    "lidartag/detections_array", 1,
    std::bind(&LidartagFilter::lidarTagDetectionsCallback, this, std::placeholders::_1));

  pub_ = this->create_publisher<lidartag_msgs::msg::LidarTagDetectionArray>(
    "lidartag/filtered/detections_array", 10);
}

void LidartagFilter::lidarTagDetectionsCallback(
  const lidartag_msgs::msg::LidarTagDetectionArray::SharedPtr detections_msg)
{
  const auto & header = detections_msg->header;

  for (auto & detection : detections_msg->detections) {
    updateHypothesis(detection, header);
  }

  rclcpp::Time timestamp = rclcpp::Time(header.stamp);

  std::vector<int> erase_list;
  for (auto it = hypotheses_map_.begin(); it != hypotheses_map_.end(); it++) {
    if (!it->second.update(timestamp)) {
      erase_list.push_back(it->first);
    }
  }

  for (auto id : erase_list) {
    hypotheses_map_.erase(id);
  }

  publishFilteredDetections(header);
}

void LidartagFilter::updateHypothesis(
  const lidartag_msgs::msg::LidarTagDetection & detection, const std_msgs::msg::Header & header)
{
  rclcpp::Time timestamp = rclcpp::Time(header.stamp);
  detection_map_[detection.id] = detection;

  cv::Matx31d translation_cv;
  cv::Matx33d rotation_cv;

  Eigen::Isometry3d pose_eigen;
  tf2::fromMsg(detection.pose, pose_eigen);

  Eigen::Vector3d translation_eigen = pose_eigen.translation();
  Eigen::Matrix3d rotation_eigen = pose_eigen.rotation();

  cv::eigen2cv(translation_eigen, translation_cv);
  cv::eigen2cv(rotation_eigen, rotation_cv);

  if (hypotheses_map_.count(detection.id) == 0) {
    hypotheses_map_[detection.id] = LidartagHypothesis(detection.id);
    auto & h = hypotheses_map_[detection.id];

    h.setMaxNoObservationTime(max_no_observation_time_);
    h.setMinConvergenceTime(std::numeric_limits<double>::max());

    h.setMaxConvergenceThreshold(0.0, 0.0, 0.0, 0.0);
    h.setMeasurementNoise(measurement_noise_translation_, measurement_noise_rotation_);
    h.setNewHypothesisThreshold(new_hypothesis_translation_, new_hypothesis_rotation_);
    h.setProcessNoise(
      process_noise_translation_, process_noise_translation_dot_, process_noise_rotation_,
      process_noise_rotation_dot_);
    h.update(translation_cv, rotation_cv, detection.size, timestamp);
  } else {
    hypotheses_map_[detection.id].update(translation_cv, rotation_cv, detection.size, timestamp);
  }
}

void LidartagFilter::publishFilteredDetections(const std_msgs::msg::Header & header)
{
  lidartag_msgs::msg::LidarTagDetectionArray filtered_detections;

  for (auto it = hypotheses_map_.begin(); it != hypotheses_map_.end(); it++) {
    const auto & base_detection = detection_map_[it->first];
    lidartag_msgs::msg::LidarTagDetection detection = base_detection;

    auto rotation = it->second.getFilteredRotation();
    auto translation = it->second.getFilteredTranslation();
    auto corners = it->second.getFilteredPoints();

    Eigen::Matrix3d rotation_eigen;
    Eigen::Vector3d translation_eigen;

    cv::cv2eigen(rotation, rotation_eigen);
    cv::cv2eigen(translation, translation_eigen);

    Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
    pose_matrix.block<3, 3>(0, 0) = rotation_eigen;
    pose_matrix.block<3, 1>(0, 3) = translation_eigen;

    Eigen::Isometry3d pose_isometry(pose_matrix);
    auto pose_msg = tf2::toMsg(pose_isometry);

    detection.pose = pose_msg;
    detection.vertices.clear();
    for (auto & corner : corners) {
      geometry_msgs::msg::Point p;
      p.x = corner.x;
      p.y = corner.y;
      p.z = corner.z;

      detection.vertices.push_back(p);
    }

    filtered_detections.detections.push_back(detection);
  }

  filtered_detections.header = header;
  pub_->publish(filtered_detections);
}

}  // namespace tier4_tag_utils

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tier4_tag_utils::LidartagFilter)
