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

#ifndef TIER4_TAG_UTILS__APRILTAG_HYPOTHESIS_HPP_
#define TIER4_TAG_UTILS__APRILTAG_HYPOTHESIS_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/time.hpp>
#include <tier4_tag_utils/types.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <vector>

namespace tier4_tag_utils
{

class ApriltagHypothesis
{
public:
  ApriltagHypothesis() = default;
  ApriltagHypothesis(int id, image_geometry::PinholeCameraModel & pinhole_camera_model);
  ~ApriltagHypothesis();

  ApriltagHypothesis(const ApriltagHypothesis &) = default;
  ApriltagHypothesis(ApriltagHypothesis &&) = default;
  ApriltagHypothesis & operator=(const ApriltagHypothesis &) = default;
  ApriltagHypothesis & operator=(ApriltagHypothesis &&) = default;

  bool update(const std::vector<cv::Point2d> & corners, const rclcpp::Time & stamp);
  bool update(const rclcpp::Time & stamp);

  int getId() const;
  std::vector<cv::Point2d> getLatestPoints2d() const;
  std::vector<cv::Point2d> getFilteredPoints2d() const;
  cv::Point2d getCenter2d() const;
  cv::Point2d getCenter2d(const std::vector<cv::Point2d> & corners) const;

  std::vector<cv::Point3d> getLatestPoints3d() const;
  std::vector<cv::Point3d> getFilteredPoints3d() const;
  std::vector<cv::Point3d> getPoints3d(const std::vector<cv::Point2d> & corners) const;
  cv::Point3d getCenter3d() const;
  cv::Point3d getCenter3d(const std::vector<cv::Point3d> & corners) const;

  bool converged() const;

  void setDynamicsModel(DynamicsModel dynamics_model);
  void setMinConvergenceTime(double convergence_time);
  void setMaxNoObservationTime(double time);
  void setMaxConvergenceThreshold(double transl);
  void setNewHypothesisThreshold(double transl);
  void setMeasurementNoise(double transl);
  void setProcessNoise(double transl);
  void setTagSize(double size);

protected:
  void reset();

  void initKalman(const std::vector<cv::Point2d> & corners);

  void initStaticKalman(const std::vector<cv::Point2d> & corners);

  cv::Mat toState(const cv::Point2d & corner);

  double convergence_transl_;
  double new_hypothesis_transl_;
  double min_convergence_time_;
  double max_no_observation_time_;
  double tag_size_;

  // Kalman related
  cv::KalmanFilter kalman_filters_[4];
  double process_noise_transl_;

  double measurement_noise_transl_;

  // General variables
  bool first_observation_;
  DynamicsModel dynamics_model_;
  int id_;
  rclcpp::Time first_observation_timestamp_;
  rclcpp::Time last_observation_timestamp_;
  image_geometry::PinholeCameraModel pinhole_camera_model_;

  std::vector<cv::Point2d> latest_corner_points_2d_, filtered_corner_points_2d_;
};

}  // namespace tier4_tag_utils

#endif  // TIER4_TAG_UTILS__APRILTAG_HYPOTHESIS_HPP_
