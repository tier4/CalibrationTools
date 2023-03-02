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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CALIBRATION_TYPES_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CALIBRATION_TYPES_HPP_

#include <extrinsic_tag_based_base_calibrator/scene_types.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace extrinsic_tag_based_base_calibrator
{

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

  UID main_calibration_sensor_uid;
  std::unordered_map<UID, std::vector<UID>> uid_connections_map;
  std::map<std::pair<UID, UID>, cv::Affine3d> detections_relative_poses_map;
  std::set<std::pair<UID, UID>> invalid_pairs_set;

  std::map<UID, IntrinsicParameters> calibration_camera_intrinsics_map_;

  // Placeholders using during optimization
  std::map<UID, std::shared_ptr<cv::Affine3d>> initial_sensor_poses_map;
  std::map<UID, std::shared_ptr<std::array<double, INTRINSICS_DIM>>> initial_camera_intrinsics_map;

  std::map<UID, std::shared_ptr<cv::Affine3d>> initial_tag_poses_map;
  std::map<UID, std::shared_ptr<cv::Affine3d>> initial_ground_tag_poses_map;
  std::shared_ptr<cv::Affine3d> initial_left_wheel_tag_pose;
  std::shared_ptr<cv::Affine3d> initial_right_wheel_tag_pose;

  std::map<UID, std::shared_ptr<cv::Affine3d>> optimized_sensor_poses_map;
  std::map<UID, std::shared_ptr<std::array<double, INTRINSICS_DIM>>>
    optimized_camera_intrinsics_map;

  std::map<UID, std::shared_ptr<cv::Affine3d>> optimized_tag_poses_map;
  std::map<UID, std::shared_ptr<cv::Affine3d>> optimized_ground_tag_poses_map;
  std::shared_ptr<cv::Affine3d> optimized_left_wheel_tag_pose;
  std::shared_ptr<cv::Affine3d> optimized_right_wheel_tag_pose;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CALIBRATION_TYPES_HPP_
