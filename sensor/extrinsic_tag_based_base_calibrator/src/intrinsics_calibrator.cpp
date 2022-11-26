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

#include <extrinsic_tag_based_base_calibrator/intrinsics_calibrator.hpp>
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
  intrinsics.size.height = -1;
  intrinsics.size.width = -1;

  intrinsics.camera_matrix = cv::Mat_<double>::zeros(3, 3);
  intrinsics.dist_coeffs = cv::Mat_<double>::zeros(3, 3);
  intrinsics.undistorted_camera_matrix = cv::Mat_<double>::zeros(3, 3);
  intrinsics.camera_matrix(0, 0) = intrinsics.undistorted_camera_matrix(0, 0) = 1.0;
  intrinsics.camera_matrix(1, 1) = intrinsics.undistorted_camera_matrix(1, 1) = 1.0;

  RCLCPP_INFO(
    rclcpp::get_logger("intrinsics_calibrator"), "Calibration images: %lu",
    calibration_image_file_names_.size());

  std::vector<std::vector<cv::Point2f>>
    image_points;  // They need to be single precision due to the calibration api
  std::vector<std::vector<cv::Point3f>> object_points;

  std::unordered_map<std::string, std::vector<int>> filtered_image_file_name_to_calibration_id_map;

  std::unordered_set<int> calibration_tag_ids_set;

  for (const auto id : calibration_tag_ids_) {
    calibration_tag_ids_set.insert(id);
  }

  std::vector<cv::Point3f> single_tag_object_points = {
    {-1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, -1.0, 0.0}, {-1.0, -1.0, 0.0}};

  for (std::size_t i = 0; i < calibration_image_file_names_.size(); ++i) {
    RCLCPP_INFO(
      rclcpp::get_logger("intrinsics_calibrator"), "calibration image id: %lu file name: %s", i,
      calibration_image_file_names_[i].c_str());

    cv::Mat grayscale_img = cv::imread(
      calibration_image_file_names_[i], cv::IMREAD_GRAYSCALE | cv::IMREAD_IGNORE_ORIENTATION);

    RCLCPP_INFO(
      rclcpp::get_logger("intrinsics_calibrator"), "rows: %d cols: %d", grayscale_img.rows,
      grayscale_img.cols);

    assert(intrinsics.size.height == -1 || intrinsics.size.height == grayscale_img.rows);
    assert(intrinsics.size.width == -1 || intrinsics.size.width == grayscale_img.cols);
    intrinsics.size = grayscale_img.size();

    auto raw_detections = detector_.detect(grayscale_img);

    // Filter detections
    std::vector<ApriltagDetection> filtered_detections;
    std::copy_if(
      raw_detections.begin(), raw_detections.end(), std::back_inserter(filtered_detections),
      [&calibration_tag_ids_set](const auto & detection) {
        return calibration_tag_ids_set.count(detection.id) > 0;
      });

    // Extract points
    for (const auto & detection : filtered_detections) {
      filtered_image_file_name_to_calibration_id_map[calibration_image_file_names_[i]].push_back(
        image_points.size());

      std::vector<cv::Point2f> corners;
      std::transform(
        detection.corners.begin(), detection.corners.end(), std::back_inserter(corners),
        [](const cv::Point2d & corner) { return cv::Point2f(corner.x, corner.y); });

      image_points.push_back(corners);
      object_points.push_back(single_tag_object_points);
    }
  }

  // Calibrate
  RCLCPP_INFO(
    rclcpp::get_logger("intrinsics_calibrator"), "Calibration views: %lu", image_points.size());
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

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
    object_points, image_points, intrinsics.size, intrinsics.camera_matrix, intrinsics.dist_coeffs,
    rvecs, tvecs, flags);

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

  auto draw_corners = [](cv::Mat & img, std::vector<cv::Point2f> & corners, cv::Scalar color) {
    std::vector<double> edge_sizes;

    for (std::size_t i = 0; i < corners.size(); ++i) {
      std::size_t j = (i + 1) % corners.size();
      edge_sizes.push_back(cv::norm(corners[i] - corners[j]));
    }

    double tag_size = *std::max_element(edge_sizes.begin(), edge_sizes.end());

    for (std::size_t i = 0; i < corners.size(); ++i) {
      std::size_t j = (i + 1) % corners.size();
      cv::line(
        img, corners[i], corners[j], color, std::max(1, static_cast<int>(tag_size / 256)),
        cv::LINE_AA);
    }
  };

  if (debug_) {
    for (auto it = filtered_image_file_name_to_calibration_id_map.begin();
         it != filtered_image_file_name_to_calibration_id_map.end(); it++) {
      const std::string input_file_name = it->first;
      std::size_t name_start_pos = input_file_name.find_last_of("/\\");
      std::size_t name_end_pos = input_file_name.find_last_of(".\\");
      std::string raw_name =
        input_file_name.substr(name_start_pos + 1, name_end_pos - name_start_pos - 1);

      std::string distorted_image_name = raw_name + "_distorted.jpg";
      std::string undistorted_image_name = raw_name + "_undistorted.jpg";

      cv::Mat distorted_img =
        cv::imread(input_file_name, cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION);
      cv::Mat undistorted_img;

      cv::undistort(
        distorted_img, undistorted_img, intrinsics.camera_matrix, intrinsics.dist_coeffs,
        intrinsics.undistorted_camera_matrix);

      for (auto i : it->second) {
        std::vector<cv::Point2f> distorted_projected_points,
          undistorted_projected_points;  // need to be float due to the api

        cv::projectPoints(
          object_points[i], rvecs[i], tvecs[i], intrinsics.camera_matrix, intrinsics.dist_coeffs,
          distorted_projected_points);

        cv::projectPoints(
          object_points[i], rvecs[i], tvecs[i], intrinsics.undistorted_camera_matrix, cv::Mat(),
          undistorted_projected_points);

        draw_corners(distorted_img, distorted_projected_points, cv::Scalar(0, 255, 0));
        draw_corners(undistorted_img, undistorted_projected_points, cv::Scalar(255, 0, 255));
      }

      cv::imwrite(distorted_image_name, distorted_img);
      cv::imwrite(undistorted_image_name, undistorted_img);
    }
  }

  return true;
}

}  // namespace extrinsic_tag_based_base_calibrator
