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

#include <extrinsic_tag_based_base_calibrator/intrinsics_calibration/apriltag_calibrator.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace extrinsic_tag_based_base_calibrator
{

void ApriltagBasedCalibrator::extractCalibrationPoints()
{
  size_.height = -1;
  size_.width = -1;

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

    assert(size_.height == -1 || size_.height == grayscale_img.rows);
    assert(size_.width == -1 || size_.width == grayscale_img.cols);
    size_ = grayscale_img.size();

    auto detections_map = detector_.detect(grayscale_img);
    auto detections = detections_map[TagType::IntrinsicCalibrationTag];

    // Extract points
    for (const auto & detection : detections) {
      filtered_image_file_name_to_calibration_id_map_[calibration_image_file_names_[i]].push_back(
        image_points_.size());

      std::vector<cv::Point2f> corners;
      std::transform(
        detection.image_corners.begin(), detection.image_corners.end(), std::back_inserter(corners),
        [](const cv::Point2d & corner) { return cv::Point2f(corner.x, corner.y); });

      image_points_.push_back(corners);
      object_points_.push_back(single_tag_object_points);
    }
  }
}

void ApriltagBasedCalibrator::writeDebugImages(const IntrinsicParameters & intrinsics)
{
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

  for (auto it = filtered_image_file_name_to_calibration_id_map_.begin();
       it != filtered_image_file_name_to_calibration_id_map_.end(); it++) {
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
        object_points_[i], rvecs_[i], tvecs_[i], intrinsics.camera_matrix, intrinsics.dist_coeffs,
        distorted_projected_points);

      cv::projectPoints(
        object_points_[i], rvecs_[i], tvecs_[i], intrinsics.undistorted_camera_matrix, cv::Mat(),
        undistorted_projected_points);

      draw_corners(distorted_img, distorted_projected_points, cv::Scalar(0, 255, 0));
      draw_corners(undistorted_img, undistorted_projected_points, cv::Scalar(255, 0, 255));
    }

    cv::imwrite(distorted_image_name, distorted_img);
    cv::imwrite(undistorted_image_name, undistorted_img);
  }
}

}  // namespace extrinsic_tag_based_base_calibrator
