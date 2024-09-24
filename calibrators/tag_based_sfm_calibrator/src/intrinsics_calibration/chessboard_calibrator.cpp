// Copyright 2024 TIER IV, Inc.
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

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tag_based_sfm_calibrator/intrinsics_calibration/chessboard_calibrator.hpp>

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace tag_based_sfm_calibrator
{

void ChessboardBasedCalibrator::extractCalibrationPoints()
{
  size_.height = -1;
  size_.width = -1;

  cv::SimpleBlobDetector::Params params;
  params.filterByArea = true;
  params.minArea = 1e3;
  params.maxArea = 2 * 1e5;
  cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(params);

  std::vector<cv::Point3f> template_points;
  for (int j = 0; j < rows_; j++) {
    for (int i = 0; i < cols_; i++) {
      template_points.emplace_back(i, j, 0.0);
    }
  }

  for (std::size_t i = 0; i < calibration_image_file_names_.size(); ++i) {
    cv::Mat grayscale_img = cv::imread(
      calibration_image_file_names_[i], cv::IMREAD_GRAYSCALE | cv::IMREAD_IGNORE_ORIENTATION);

    assert(size_.height == -1 || size_.height == grayscale_img.rows);
    assert(size_.width == -1 || size_.width == grayscale_img.cols);
    size_ = grayscale_img.size();

    RCLCPP_INFO(
      rclcpp::get_logger("intrinsics_calibrator"), "rows: %d cols: %d", grayscale_img.rows,
      grayscale_img.cols);

    cv::Size pattern(cols_, rows_);    // w x h format
    std::vector<cv::Point2f> centers;  // this will be filled by the detected centers

    bool found = cv::findCirclesGrid(
      grayscale_img, pattern, centers, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING,
      blobDetector);

    RCLCPP_INFO(
      rclcpp::get_logger("intrinsics_calibrator"),
      "calibration image id: %lu file name: %s found=%d", i,
      calibration_image_file_names_[i].c_str(), found);

    if (found) {
      filtered_image_file_names_.push_back(calibration_image_file_names_[i]);
      object_points_.push_back(template_points);
      image_points_.push_back(centers);
    }
  }
}

void ChessboardBasedCalibrator::writeDebugImages(const IntrinsicParameters & intrinsics)
{
  cv::Size pattern(cols_, rows_);  // w x h format

  for (std::size_t i = 0; i < filtered_image_file_names_.size(); i++) {
    const std::string input_file_name = filtered_image_file_names_[i];
    std::size_t name_start_pos = input_file_name.find_last_of("/\\");
    std::size_t name_end_pos = input_file_name.find_last_of(".\\");
    std::string raw_name =
      input_file_name.substr(name_start_pos + 1, name_end_pos - name_start_pos - 1);

    std::string distorted_image_name = raw_name + "_distorted.jpg";
    std::string undistorted_image_name = raw_name + "_undistorted.jpg";

    cv::Mat distorted_img =
      cv::imread(input_file_name, cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION);
    cv::Mat undistorted_img;

    cv::drawChessboardCorners(distorted_img, pattern, cv::Mat(image_points_[i]), true);

    cv::undistort(
      distorted_img, undistorted_img, intrinsics.camera_matrix, intrinsics.dist_coeffs,
      intrinsics.undistorted_camera_matrix);

    std::vector<cv::Point2f> projected_points;  // need to be float due to the api

    cv::projectPoints(
      object_points_[i], rvecs_[i], tvecs_[i], intrinsics.undistorted_camera_matrix, cv::Mat(),
      projected_points);

    cv::drawChessboardCorners(undistorted_img, pattern, cv::Mat(projected_points), true);

    cv::imwrite(distorted_image_name, distorted_img);
    cv::imwrite(undistorted_image_name, undistorted_img);
  }
}

}  // namespace tag_based_sfm_calibrator
