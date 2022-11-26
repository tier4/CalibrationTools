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
#include <opencv2/core/affine.hpp>

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace extrinsic_tag_based_base_calibrator
{

struct ApriltagParameters
{
  std::string family;
  int max_hamming;
  double min_margin;
  double max_homography_error;
  double quad_decimate;
  double quad_sigma;
  int nthreads;
  bool debug;
  bool refine_edges;
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
    return !size.empty() && camera_matrix(0, 0) > 0.0 && camera_matrix(1, 1) > 0.0 &&
           camera_matrix(0, 2) > 0.0 && camera_matrix(1, 2) > 0.0 &&
           undistorted_camera_matrix(0, 0) > 0.0 && undistorted_camera_matrix(1, 1) > 0.0 &&
           undistorted_camera_matrix(0, 2) > 0.0 && undistorted_camera_matrix(1, 2) > 0.0;
  }

  cv::Size size;
  cv::Mat_<double> camera_matrix;
  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> undistorted_camera_matrix;
};

struct ApriltagDetection
{
  int id;
  std::vector<cv::Point2d> corners;
  cv::Point2d center;
  cv::Affine3d pose;
  double size;
};

struct LidartagDetection
{
  int id;
  cv::Affine3d pose;
  double size;
};

struct ExternalCameraFrame
{
  std::string image_filename;
  std::vector<ApriltagDetection> detections;
};

struct CalibrationScene
{
  std::vector<LidartagDetection> calibration_lidar_detections;
  std::vector<ApriltagDetection> calibration_camera_detections;
  std::vector<ExternalCameraFrame> external_camera_frames;
};
struct UID
{
  UID()
  : is_camera(false),
    is_tag(false),
    is_waypoint_tag(false),
    is_ground_tag(false),
    is_wheel_tag(false),
    scene_id(-1),
    frame_id(-1),
    tag_id(-1)
  {
  }

  UID(
    bool is_camera, bool is_waypoint, bool is_ground_tag, bool is_wheel_tag, int scene_id,
    int frame_id, int tag_id)
  : is_camera(is_camera),
    is_waypoint_tag(is_waypoint),
    is_ground_tag(is_ground_tag),
    is_wheel_tag(is_wheel_tag),
    scene_id(scene_id),
    frame_id(frame_id),
    tag_id(tag_id)
  {
    is_tag = is_waypoint_tag || is_ground_tag || is_wheel_tag;
  }

  std::string to_string() const
  {
    if (is_camera) {
      return "s" + std::to_string(scene_id) + "_c" + std::to_string(frame_id);
    } else if (is_waypoint_tag) {
      return "s" + std::to_string(scene_id) + "_w" + std::to_string(tag_id);
    } else if (is_ground_tag) {
      return "g" + std::to_string(tag_id);
    } else if (is_wheel_tag) {
      return "w" + std::to_string(tag_id);
    } else {
      throw std::invalid_argument("Invalid UID");
    }
  }

  bool operator<(const UID & other) const
  {
    return (this->to_string().compare(other.to_string())) > 0;
  }

  static UID makeCameraUID(int scene_id, int frame_id)
  {
    return UID(true, false, false, false, scene_id, frame_id, -1);
  }

  static UID makeWaypointUID(int scene_id, int waypoint_id)
  {
    return UID(false, true, false, false, scene_id, -1, waypoint_id);
  }

  static UID makeGroundTagUID(int tag_id) { return UID(false, false, true, false, -1, -1, tag_id); }

  static UID makeWheelTagUID(int tag_id) { return UID(false, false, false, true, -1, -1, tag_id); }

  bool is_camera;
  bool is_tag;
  bool is_waypoint_tag;
  bool is_ground_tag;
  bool is_wheel_tag;

  int scene_id;
  int frame_id;
  int tag_id;
};

struct CalibrationData
{
  using Ptr = std::shared_ptr<CalibrationData>;

  static constexpr int POSE_OPT_DIM = 7;
  static constexpr int SHRD_GROUND_TAG_POSE_DIM = 5;
  static constexpr int INDEP_GROUND_TAG_POSE_DIM = 3;
  static constexpr int INTRINSICS_DIM = 6;

  static constexpr int ROTATION_W_INDEX = 0;
  static constexpr int ROTATION_X_INDEX = 1;
  static constexpr int ROTATION_Y_INDEX = 2;
  static constexpr int ROTATION_Z_INDEX = 3;
  static constexpr int TRANSLATION_X_INDEX = 4;
  static constexpr int TRANSLATION_Y_INDEX = 5;
  static constexpr int TRANSLATION_Z_INDEX = 6;

  static constexpr int INTRINSICS_CX_INDEX = 0;
  static constexpr int INTRINSICS_CY_INDEX = 1;
  static constexpr int INTRINSICS_FX_INDEX = 2;
  static constexpr int INTRINSICS_FY_INDEX = 3;
  static constexpr int INTRINSICS_K1_INDEX = 4;
  static constexpr int INTRINSICS_K2_INDEX = 5;

  static constexpr int GROUND_TAG_D_INDEX = 4;
  static constexpr int GROUND_TAG_YAW_INDEX = 0;
  static constexpr int GROUND_TAG_X_INDEX = 1;
  static constexpr int GROUND_TAG_Y_INDEX = 2;

  std::vector<CalibrationScene> scenes;

  std::set<int> detected_tag_ids_set;

  std::map<UID, std::shared_ptr<cv::Affine3d>> initial_external_camera_poses;
  std::map<UID, std::shared_ptr<std::array<double, INTRINSICS_DIM>>>
    initial_external_camera_intrinsics;

  std::map<UID, std::shared_ptr<cv::Affine3d>> initial_tag_poses_map;
  std::vector<std::shared_ptr<cv::Affine3d>> initial_waypoint_tag_poses;
  std::vector<std::shared_ptr<cv::Affine3d>> initial_ground_tag_poses;
  std::shared_ptr<cv::Affine3d> initial_left_wheel_tag_pose;
  std::shared_ptr<cv::Affine3d> initial_right_wheel_tag_pose;

  std::map<UID, std::shared_ptr<cv::Affine3d>> optimized_external_camera_poses;
  std::map<UID, std::shared_ptr<std::array<double, INTRINSICS_DIM>>>
    optimized_external_camera_intrinsics;
  std::map<UID, std::shared_ptr<cv::Affine3d>> optimized_tag_poses_map;
  std::vector<std::shared_ptr<cv::Affine3d>> optimized_waypoint_tag_poses;
  std::vector<std::shared_ptr<cv::Affine3d>> optimized_ground_tag_poses;
  std::shared_ptr<cv::Affine3d> optimized_left_wheel_tag_pose;
  std::shared_ptr<cv::Affine3d> optimized_right_wheel_tag_pose;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__TYPES_HPP_
