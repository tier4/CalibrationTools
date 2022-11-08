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

  RCLCPP_INFO(
    rclcpp::get_logger("intrinsics_calibrator"), "Calibration images: %lu",
    calibration_image_file_names_.size());

  std::vector<std::vector<cv::Point2f>> image_points;
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::string> filtered_image_file_names;

  std::unordered_set<int> calibration_tag_ids_set;

  for (const auto id : calibration_tag_ids_) {
    calibration_tag_ids_set.insert(id);
  }

  std::vector<cv::Point3f> single_tag_object_points = {
    cv::Point3f(-1.0, -1.0, 0.0), cv::Point3f(1.0, -1.0, 0.0), cv::Point3f(1.0, 1.0, 0.0),
    cv::Point3f(-1.0, 1.0, 0.0)};

  for (std::size_t i = 0; i < calibration_image_file_names_.size(); ++i) {
    RCLCPP_INFO(
      rclcpp::get_logger("intrinsics_calibrator"), "calibration image id: %lu file name: %s", i,
      calibration_image_file_names_[i].c_str());

    cv::Mat grayscale_img = cv::imread(calibration_image_file_names_[i], cv::IMREAD_GRAYSCALE);

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
      image_points.push_back(detection.corners);
      object_points.push_back(single_tag_object_points);
      filtered_image_file_names.push_back(calibration_image_file_names_[i]);
    }
  }

  // Calibrate
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  cv::calibrateCamera(
    object_points, image_points, intrinsics.size, intrinsics.camera_matrix, intrinsics.dist_coeffs,
    rvecs, tvecs);
  intrinsics.camera_matrix.convertTo(intrinsics.camera_matrix, CV_32F);
  intrinsics.dist_coeffs.convertTo(intrinsics.dist_coeffs, CV_32F);

  std::cout << "camera_matrix: \n" << intrinsics.camera_matrix << std::endl;
  std::cout << "dist_coeffs: \n" << intrinsics.dist_coeffs << std::endl;

  // New calibration matrix
  intrinsics.undistorted_camera_matrix = getOptimalNewCameraMatrix(
    intrinsics.camera_matrix, intrinsics.dist_coeffs, intrinsics.size, 0, intrinsics.size);
  intrinsics.undistorted_camera_matrix.convertTo(intrinsics.undistorted_camera_matrix, CV_32F);
  std::cout << "undistorted_camera_matrix: \n" << intrinsics.undistorted_camera_matrix << std::endl;

  auto draw_corners = [](cv::Mat & img, std::vector<cv::Point2f> & corners, cv::Scalar color) {
    std::vector<float> edge_sizes;

    for (std::size_t i = 0; i < corners.size(); ++i) {
      std::size_t j = (i + 1) % corners.size();
      edge_sizes.push_back(cv::norm(corners[i] - corners[j]));
    }

    float tag_size = *std::max_element(edge_sizes.begin(), edge_sizes.end());

    for (std::size_t i = 0; i < corners.size(); ++i) {
      std::size_t j = (i + 1) % corners.size();
      cv::line(img, corners[i], corners[j], color, static_cast<int>(tag_size / 256), cv::LINE_AA);
    }
  };

  if (debug_) {
    for (std::size_t i = 0; i < filtered_image_file_names.size(); ++i) {
      const std::string input_file_name = filtered_image_file_names[i];
      std::size_t name_start_pos = input_file_name.find_last_of("/\\");
      std::size_t name_end_pos = input_file_name.find_last_of(".\\");
      std::string raw_name =
        input_file_name.substr(name_start_pos + 1, name_end_pos - name_start_pos - 1);

      std::string distorted_image_name = raw_name + "_distorted.jpg";
      std::string undistorted_image_name = raw_name + "_undistorted.jpg";
      std::cout << "Raw name: " << raw_name << std::endl;

      cv::Mat distorted_img = cv::imread(input_file_name, cv::IMREAD_COLOR);
      cv::Mat undistorted_img;

      cv::undistort(
        distorted_img, undistorted_img, intrinsics.camera_matrix, intrinsics.dist_coeffs,
        intrinsics.undistorted_camera_matrix);

      std::vector<cv::Point2f> distorted_projected_points, undistorted_projected_points;

      cv::projectPoints(
        object_points[i], rvecs[i], tvecs[i], intrinsics.camera_matrix, intrinsics.dist_coeffs,
        distorted_projected_points);

      cv::projectPoints(
        object_points[i], rvecs[i], tvecs[i], intrinsics.undistorted_camera_matrix, cv::Mat(),
        undistorted_projected_points);

      draw_corners(distorted_img, distorted_projected_points, cv::Scalar(0, 255, 0));
      draw_corners(undistorted_img, undistorted_projected_points, cv::Scalar(255, 0, 255));

      cv::imwrite(distorted_image_name, distorted_img);
      cv::imwrite(undistorted_image_name, undistorted_img);
    }
  }

  return true;
}

}  // namespace extrinsic_tag_based_base_calibrator
