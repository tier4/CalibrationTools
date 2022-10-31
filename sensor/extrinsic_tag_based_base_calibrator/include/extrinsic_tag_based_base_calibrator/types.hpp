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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__TYPES_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__TYPES_HPP_

#include <opencv2/core.hpp>

#include <string>
#include <vector>

struct ApriltagParameters
{
  std::string family;
  int max_hamming;
  float min_margin;
  float max_h_error;
  float quad_decimate;
  float quad_sigma;
  int nthreads;
  bool debug;
  bool refine_edges;
};

struct ApriltagDetection
{
  int id;
  std::vector<cv::Point2f> corners;
  cv::Point2f center;
  cv::Matx33f pose_rotation;
  cv::Matx31f pose_translation;
  float size;
};

class IntrinsicParameters
{
public:
  bool loadCalibration(const std::string file_name)
  {
    cv::FileStorage fs(file_name, cv::FileStorage::READ);

    if (!fs.isOpened()) {
      return false;
    }

    fs["size"] >> size;
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    fs["undistorted_camera_matrix"] >> undistorted_camera_matrix;

    return true;
  }

  void saveCalibration(const std::string file_name)
  {
    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
    fs << "size" << size;
    fs << "camera_matrix" << camera_matrix;
    fs << "dist_coeffs" << dist_coeffs;
    fs << "undistorted_camera_matrix" << undistorted_camera_matrix;
  }

  bool isValid()
  {
    return !size.empty() && camera_matrix.at<float>(0, 0) > 0.f &&
           camera_matrix.at<float>(1, 1) > 0.f && camera_matrix.at<float>(0, 2) > 0.f &&
           camera_matrix.at<float>(1, 2) > 0.f && undistorted_camera_matrix.at<float>(0, 0) > 0.f &&
           undistorted_camera_matrix.at<float>(1, 1) > 0.f &&
           undistorted_camera_matrix.at<float>(0, 2) > 0.f &&
           undistorted_camera_matrix.at<float>(1, 2) > 0.f;
  }

  cv::Size size;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  cv::Mat undistorted_camera_matrix;
};

struct CalibrationScene
{
  std::vector<ApriltagDetection> calibration_sensor_detections;
  std::vector<std::vector<ApriltagDetection>> external_camera_detections;
};

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__TYPES_HPP_
