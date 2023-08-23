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

#ifndef TIER4_TAG_UTILS__LIDARTAG_HYPOTHESIS_HPP_
#define TIER4_TAG_UTILS__LIDARTAG_HYPOTHESIS_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/time.hpp>
#include <tier4_tag_utils/types.hpp>

#include <vector>

namespace tier4_tag_utils
{

class LidartagHypothesis
{
public:
  LidartagHypothesis() = default;
  explicit LidartagHypothesis(int id);

  bool update(
    const cv::Matx31d & pose_translation, const cv::Matx33d & pose_rotation, double tag_size,
    const rclcpp::Time & stamp);
  bool update(const rclcpp::Time & stamp);

  int getId() const;
  std::vector<cv::Point3d> getLatestPoints();
  cv::Matx33d getLatestRotation() const;
  cv::Matx31d getLatestTranslation() const;

  std::vector<cv::Point3d> getFilteredPoints();
  cv::Matx33d getFilteredRotation() const;
  cv::Matx31d getFilteredTranslation() const;
  cv::Point3d getCenter() const;

  double getTransCov() const;
  double getTransDotCov() const;
  double getRotCov() const;
  double getRotDotCov() const;
  double getSpeed() const;

  bool converged() const;
  double timeSinceFirstObservation(const rclcpp::Time & stamp) const;
  double timeSinceLastObservation(const rclcpp::Time & stamp) const;

  void setDynamicsModel(DynamicsModel dynamics_mode);
  void setMinConvergenceTime(double convergence_time);
  void setMaxNoObservationTime(double time);
  void setMaxConvergenceThreshold(double transl, double tansl_dot, double angle, double angle_dot);
  void setNewHypothesisThreshold(double transl, double angle);
  void setMeasurementNoise(double transl, double angle);
  void setProcessNoise(double transl, double transl_dot, double rot, double rot_dot);

protected:
  void reset();

  void initKalman(const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix);

  void initStaticKalman(
    const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix);

  void initConstantVelocityKalman(
    const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix);

  cv::Mat toState(const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix);

  cv::Mat toObservation(
    const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix);

  void fixState(cv::Mat & old_state, cv::Mat & new_state);
  void fixState(cv::Mat & new_state);

  cv::Matx31d rot2euler(const cv::Matx33d & rotation_matrix);
  cv::Matx33d euler2rot(const cv::Matx31d & euler);

  DynamicsModel dynamics_model_;

  double convergence_transl_;
  double convergence_transl_dot_;
  double convergence_rot_;
  double convergence_rot_dot_;
  double new_hypothesis_transl_;
  double new_hypothesis_rot_;
  double min_convergence_time_;
  double max_no_observation_time_;

  // Kalman related
  cv::KalmanFilter kalman_filter_;
  double process_noise_transl_;
  double process_noise_transl_dot_;
  double process_noise_rot_;
  double process_noise_rot_dot_;

  double measurement_noise_transl_;
  double measurement_noise_rot_;

  // General variables
  int id_;
  bool first_observation_;
  rclcpp::Time first_observation_timestamp_;
  rclcpp::Time last_observation_timestamp_;

  double tag_size_;

  std::vector<cv::Point3d> latest_corner_points_, filtered_corner_points;

  cv::Matx33d latest_rotation_matrix_, filtered_rotation_matrix_;
  cv::Matx31d latest_translation_vector_, filtered_translation_vector_;
  double estimated_speed_;
};

}  // namespace tier4_tag_utils

#endif  // TIER4_TAG_UTILS__LIDARTAG_HYPOTHESIS_HPP_
