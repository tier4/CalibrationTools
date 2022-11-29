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

#include <extrinsic_tag_based_base_calibrator/intrinsics_calibration/intrinsics_calibrator.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace extrinsic_tag_based_base_calibrator
{

void IntrinsicsCalibrator::setCalibrationImageFiles(
  const std::vector<std::string> & image_file_names)
{
  calibration_image_file_names_ = image_file_names;
}

bool IntrinsicsCalibrator::calibrate(IntrinsicParameters & intrinsics)
{
  RCLCPP_INFO(
    rclcpp::get_logger("intrinsics_calibrator"), "Calibration images: %lu",
    calibration_image_file_names_.size());

  // Extract calibration points from the images
  extractCalibrationPoints();

  intrinsics.size = size_;
  intrinsics.camera_matrix = cv::Mat_<double>::zeros(3, 3);
  intrinsics.dist_coeffs = cv::Mat_<double>::zeros(3, 3);
  intrinsics.undistorted_camera_matrix = cv::Mat_<double>::zeros(3, 3);
  intrinsics.camera_matrix(0, 0) = intrinsics.undistorted_camera_matrix(0, 0) = 1.0;
  intrinsics.camera_matrix(1, 1) = intrinsics.undistorted_camera_matrix(1, 1) = 1.0;

  // Calibrate
  RCLCPP_INFO(
    rclcpp::get_logger("intrinsics_calibrator"), "Calibration views: %lu", image_points_.size());

  int flags = 0;

  if (!use_tangent_distortion_) {
    flags |= cv::CALIB_ZERO_TANGENT_DIST;
  }

  if (num_radial_distortion_coeffs_ < 3) {
    flags |= cv::CALIB_FIX_K3;
  }

  if (num_radial_distortion_coeffs_ < 2) {
    flags |= cv::CALIB_FIX_K2;
  }

  if (num_radial_distortion_coeffs_ < 1) {
    flags |= cv::CALIB_FIX_K1;
  }

  double reproj_error = cv::calibrateCamera(
    object_points_, image_points_, intrinsics.size, intrinsics.camera_matrix,
    intrinsics.dist_coeffs, rvecs_, tvecs_, flags);

  RCLCPP_INFO(
    rclcpp::get_logger("intrinsics_calibrator"), "Tangent distortion: %d", use_tangent_distortion_);
  RCLCPP_INFO(
    rclcpp::get_logger("intrinsics_calibrator"), "Radial distortion coeffs: %d",
    num_radial_distortion_coeffs_);

  RCLCPP_INFO(rclcpp::get_logger("intrinsics_calibrator"), "Reproj_error: %.2f", reproj_error);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("intrinsics_calibrator"), "Camera matrix:\n"
                                                   << intrinsics.camera_matrix);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("intrinsics_calibrator"), "Distortion coefficients:\n"
                                                   << intrinsics.dist_coeffs);

  // New calibration matrix
  intrinsics.undistorted_camera_matrix = getOptimalNewCameraMatrix(
    intrinsics.camera_matrix, intrinsics.dist_coeffs, intrinsics.size, 0, intrinsics.size);

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("intrinsics_calibrator"), "Undistorted camera matrix:\n"
                                                   << intrinsics.undistorted_camera_matrix);

  if (debug_) {
    writeDebugImages(intrinsics);
  }

  return true;
}

}  // namespace extrinsic_tag_based_base_calibrator
