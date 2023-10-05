// Copyright 2023 Tier IV, Inc.
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

#ifndef EXTRINSIC_TAG_BASED_PNP_CALIBRATOR__CALIBRATION_ESTIMATOR_HPP_
#define EXTRINSIC_TAG_BASED_PNP_CALIBRATOR__CALIBRATION_ESTIMATOR_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/time.hpp>
#include <tier4_tag_utils/apriltag_hypothesis.hpp>
#include <tier4_tag_utils/lidartag_hypothesis.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <lidartag_msgs/msg/lidar_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <tf2/utils.h>

#include <memory>
#include <unordered_map>
#include <vector>

class CalibrationEstimator
{
public:
  CalibrationEstimator();

  void update(const apriltag_msgs::msg::AprilTagDetectionArray & msg);
  void update(const lidartag_msgs::msg::LidarTagDetectionArray & msg);
  void update(const apriltag_msgs::msg::AprilTagDetection & msg, const rclcpp::Time & time_stamp);
  void update(const lidartag_msgs::msg::LidarTagDetection & msg, const rclcpp::Time & time_stamp);
  bool update(const rclcpp::Time & timestamp);

  void getCalibrationPoints(
    std::vector<cv::Point3d> & object_points, std::vector<cv::Point2d> & image_pointsbool,
    bool use_estimated);

  bool calibrate();

  bool converged() const;
  bool valid() const;

  std::vector<std::shared_ptr<tier4_tag_utils::LidartagHypothesis>> getActiveLidartagHypotheses()
    const;
  std::vector<std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>> getActiveApriltagHypotheses()
    const;

  std::vector<std::shared_ptr<tier4_tag_utils::LidartagHypothesis>> getConvergedLidartagHypotheses()
    const;
  std::vector<std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>> getConvergedApriltagHypotheses()
    const;

  void setCameraModel(const sensor_msgs::msg::CameraInfo & camera_info);
  tf2::Transform getCurrentPose() const;
  void getCurrentPose(cv::Matx31d & trans_vector, cv::Matx33d & rot_matrix) const;
  tf2::Transform getFilteredPose() const;
  void getFilteredPose(cv::Matx31d & trans_vector, cv::Matx33d & rot_matrix) const;

  // Parameters setters
  void setCrossvalidationTrainingRatio(double ratio);
  void setCalibrationConvergenceCriteria(int min_pairs, double min_area_percentage);
  void setMinPnpPairs(int min_pairs);
  void setMinConvergenceTime(double convergence_time);
  void setMaxNoObservationTime(double time);
  void setNewHypothesisDistance(double distance);
  void setTagSizes(std::vector<int64_t> & tag_id, std::vector<double> & tag_sizes);

  void setLidartagMaxConvergenceThreshold(
    double transl, double tansl_dot, double angle, double angle_dot);
  void setLidartagNewHypothesisThreshold(double transl, double angle);
  void setLidartagMeasurementNoise(double transl, double angle);
  void setLidartagProcessNoise(double transl, double transl_dot, double rot, double rot_dot);

  void setApriltagMaxConvergenceThreshold(double transl);
  void setApriltagNewHypothesisThreshold(double transl);
  void setApriltagMeasurementNoise(double transl);
  void setApriltagProcessNoise(double transl);

  double getNewHypothesisDistance() const;
  double getCalibrationCoveragePercentage() const;
  int getCurrentCalibrationPairsNumber() const;
  double getCrossValidationReprojError() const;

private:
  void getCalibrationPointsIdBased(
    std::vector<cv::Point3d> & object_points, std::vector<cv::Point2d> & image_points,
    bool use_estimated);

  void getCalibrationPointsIdless(
    std::vector<cv::Point3d> & object_points, std::vector<cv::Point2d> & image_points,
    bool use_estimated);

  bool calibrate(
    const std::vector<cv::Point3d> & object_points, const std::vector<cv::Point2d> & image_points,
    cv::Matx31d & translation_vector, cv::Matx33d & rotation_matrix);

  tf2::Transform toTf2(
    const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix) const;

  void computeCrossValidationReprojError(
    const std::vector<cv::Point3d> & object_points, const std::vector<cv::Point2d> & image_points);

  // Parameters

  // Calibration convergence criteria
  int convergence_min_pairs_;
  double convergence_min_area_percentage_;
  double crossvalidation_training_ratio_;

  // Hypothesis convergence criteria
  int min_pnp_pairs_;
  double min_convergence_time_;
  double max_no_observation_time_;
  double new_hypothesis_distance_;

  // Lidartag estimation parameters
  double lidartag_convergence_transl_;
  double lidartag_convergence_transl_dot_;
  double lidartag_convergence_rot_;
  double lidartag_convergence_rot_dot_;
  double lidartag_new_hypothesis_transl_;
  double lidartag_new_hypothesis_rot_;

  double lidartag_process_noise_transl_;
  double lidartag_process_noise_transl_dot_;
  double lidartag_process_noise_rot_;
  double lidartag_process_noise_rot_dot_;
  double lidartag_measurement_noise_transl_;
  double lidartag_measurement_noise_rot_;

  // Apriltag estimation parameters
  double apriltag_convergence_transl_;
  double apriltag_new_hypothesis_transl_;
  double apriltag_process_noise_transl_;
  double apriltag_measurement_noise_transl_;

  image_geometry::PinholeCameraModel pinhole_camera_model_;

  std::unordered_map<int, double> tag_sizes_map_;

  // Hypotheses
  std::unordered_map<int, std::shared_ptr<tier4_tag_utils::LidartagHypothesis>>
    active_lidartag_hypotheses_;
  std::unordered_map<int, std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>>
    active_apriltag_hypotheses_;

  std::vector<std::shared_ptr<tier4_tag_utils::LidartagHypothesis>> converged_lidartag_hypotheses_;
  std::vector<std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>> converged_apriltag_hypotheses_;

  // Output
  double crossval_reproj_error_;
  bool valid_;
  cv::Matx33d hypothesis_rotation_matrix_, observation_rotation_matrix_;
  cv::Matx31d hypothesis_translation_vector_, observation_translation_vector_;
};

#endif  // EXTRINSIC_TAG_BASED_PNP_CALIBRATOR__CALIBRATION_ESTIMATOR_HPP_
