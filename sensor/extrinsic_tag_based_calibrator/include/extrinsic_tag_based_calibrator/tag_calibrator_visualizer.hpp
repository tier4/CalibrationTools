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

#ifndef EXTRINSIC_TAG_BASED_CALIBRATOR_TAG_CALIBRATOR_VISUALIZER_H_
#define EXTRINSIC_TAG_BASED_CALIBRATOR_TAG_CALIBRATOR_VISUALIZER_H_

#include <Eigen/Core>
#include <extrinsic_tag_based_calibrator/apriltag_hypothesis.hpp>
#include <extrinsic_tag_based_calibrator/calibration_estimator.hpp>
#include <extrinsic_tag_based_calibrator/lidartag_hypothesis.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <lidartag_msgs/msg/lidar_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <string>

class TagCalibratorVisualizer
{
public:
  enum class HypothesisType  // C++11 scoped enum
  {
    Active,
    Converged
  };

  TagCalibratorVisualizer(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub);

  void setTagSizes(std::vector<int64_t> & tag_id, std::vector<double> & tag_sizes);

  void setBaseFrame(const std::string & lidar_frame);
  void setCameraFrame(const std::string & camera_frame);
  void setLidarFrame(const std::string & lidar_frame);

  void setCameraModel(const sensor_msgs::msg::CameraInfo & camera_info);
  void setCameraLidarTransform(
    const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix);
  void setBaseLidarTransform(
    const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix);

  void drawCalibrationStatus(const CalibrationEstimator & estimation, rclcpp::Time & stamp);

  void drawAprilTagDetections(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr & apriltag_detection_array);

  void drawLidarTagDetections(
    const lidartag_msgs::msg::LidarTagDetectionArray::SharedPtr & lidartag_detections_array);

  void displayImagePoints(const std::vector<cv::Point2d> & points, const rclcpp::Time & stamp);

  void displayObjectPoints(const std::vector<cv::Point3d> & points, const rclcpp::Time & stamp);

  void setMinConvergenceTime(double convergence_time);
  void setMaxNoObservationTime(double time);
  void setLidartagMaxConvergenceThreshold(
    double transl, double tansl_dot, double angle, double angle_dot);
  void setApriltagMaxConvergenceThreshold(double transl);

private:
  void drawLidartagHypotheses(
    const std::vector<std::shared_ptr<LidartagHypothesis>> & h_vector, HypothesisType type,
    rclcpp::Time & stamp, visualization_msgs::msg::MarkerArray & marker_array);
  void drawApriltagHypotheses(
    const std::vector<std::shared_ptr<ApriltagHypothesis>> & h_vector, HypothesisType type,
    rclcpp::Time & stamp, visualization_msgs::msg::MarkerArray & marker_array);
  void drawCalibrationZone(
    rclcpp::Time & stamp, visualization_msgs::msg::MarkerArray & marker_array);
  void drawCalibrationStatusText(
    const CalibrationEstimator & estimator, rclcpp::Time & stamp,
    visualization_msgs::msg::MarkerArray & marker_array);

  std::vector<cv::Point3d> get3dpoints(apriltag_msgs::msg::AprilTagDetection & detection);
  void computeTransforms();

  double min_convergence_time_;
  double max_no_observation_time_;
  double lidartag_convergence_transl_;
  double lidartag_convergence_transl_dot_;
  double lidartag_convergence_rot_;
  double lidartag_convergence_rot_dot_;
  double apriltag_convergence_transl_;

  bool valid_base_lidar_transform_;
  bool valid_camera_lidar_transform_;
  bool valid_transforms_;
  double circle_diameter_;

  std::unordered_map<int, double> tag_sizes_map_;

  std::string base_frame_;
  std::string camera_frame_;
  std::string lidar_frame_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  sensor_msgs::msg::CameraInfo camera_info_;
  image_geometry::PinholeCameraModel pinhole_camera_model_;

  cv::Affine3d camera_lidar_transform_;
  cv::Affine3d lidar_camera_transform_;
  cv::Affine3d base_lidar_transform_;
  cv::Affine3d lidar_base_transform_;
  cv::Affine3d base_camera_transform_;
  cv::Affine3d camera_base_transform_;
};

#endif  // EXTRINSIC_TAG_BASED_CALIBRATOR_TAG_CALIBRATOR_VISUALIZER_HPP_
