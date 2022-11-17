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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CALIBRATION_PROBLEM_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CALIBRATION_PROBLEM_HPP_

#include <extrinsic_tag_based_base_calibrator/types.hpp>

#include <array>
#include <map>
#include <memory>
#include <set>
#include <vector>

namespace extrinsic_tag_based_base_calibrator
{

class CalibrationProblem
{
public:
  static constexpr int POSE_OPT_DIM = CalibrationData::POSE_OPT_DIM;
  static constexpr int SHRD_GROUND_TAG_POSE_DIM = CalibrationData::SHRD_GROUND_TAG_POSE_DIM;
  static constexpr int INDEP_GROUND_TAG_POSE_DIM = CalibrationData::INDEP_GROUND_TAG_POSE_DIM;
  static constexpr int INTRINSICS_DIM = CalibrationData::INTRINSICS_DIM;

  static constexpr int ROTATION_W_INDEX = CalibrationData::ROTATION_W_INDEX;
  static constexpr int ROTATION_X_INDEX = CalibrationData::ROTATION_X_INDEX;
  static constexpr int ROTATION_Y_INDEX = CalibrationData::ROTATION_Y_INDEX;
  static constexpr int ROTATION_Z_INDEX = CalibrationData::ROTATION_Z_INDEX;
  static constexpr int TRANSLATION_X_INDEX = CalibrationData::TRANSLATION_X_INDEX;
  static constexpr int TRANSLATION_Y_INDEX = CalibrationData::TRANSLATION_Y_INDEX;
  static constexpr int TRANSLATION_Z_INDEX = CalibrationData::TRANSLATION_Z_INDEX;

  static constexpr int INTRINSICS_CX_INDEX = CalibrationData::INTRINSICS_CX_INDEX;
  static constexpr int INTRINSICS_CY_INDEX = CalibrationData::INTRINSICS_CY_INDEX;
  static constexpr int INTRINSICS_FX_INDEX = CalibrationData::INTRINSICS_FX_INDEX;
  static constexpr int INTRINSICS_FY_INDEX = CalibrationData::INTRINSICS_FY_INDEX;
  static constexpr int INTRINSICS_K1_INDEX = CalibrationData::INTRINSICS_K1_INDEX;
  static constexpr int INTRINSICS_K2_INDEX = CalibrationData::INTRINSICS_K2_INDEX;

  static constexpr int GROUND_TAG_Z_INDEX = CalibrationData::GROUND_TAG_Z_INDEX;
  static constexpr int GROUND_TAG_YAW_INDEX = CalibrationData::GROUND_TAG_YAW_INDEX;
  static constexpr int GROUND_TAG_X_INDEX = CalibrationData::GROUND_TAG_X_INDEX;
  static constexpr int GROUND_TAG_Y_INDEX = CalibrationData::GROUND_TAG_Y_INDEX;

  static constexpr int RESIDUAL_DIM = 8;

  void setTagIds(
    std::vector<int> & waypoint_tag_ids, std::vector<int> & ground_tag_ids, int left_wheel_tag_id,
    int right_wheel_tag_id);

  void setOptimizeIntrinsics(bool ba_optimize_intrinsics);
  void setShareIntrinsics(bool ba_share_intrinsics);
  void setForceSharedGroundPlane(bool ba_force_shared_ground_plane);

  void setExternalCameraIntrinsics(IntrinsicParameters & intrinsics);
  void setCalibrationSensorIntrinsics(IntrinsicParameters & intrinsics);

  void setData(CalibrationData::Ptr & data);
  void dataToPlaceholders();
  void placeholdersToData();
  void evaluate();
  void solve();
  void writeDebugImages();

  void getMarkers();

protected:
  void pose3dToPlaceholder(
    cv::Affine3d pose, std::array<double, POSE_OPT_DIM> & placeholder, bool invert);
  void placeholderToPose3d(
    const std::array<double, POSE_OPT_DIM> & placeholder, std::shared_ptr<cv::Affine3d> & pose,
    bool invert);
  void pose3dToGroundTagPlaceholder(
    cv::Affine3d pose, std::array<double, SHRD_GROUND_TAG_POSE_DIM> & shrd_placeholder,
    std::array<double, INDEP_GROUND_TAG_POSE_DIM> & indep_placeholder, bool invert);
  void groundTagPlaceholderToPose3d(
    const std::array<double, SHRD_GROUND_TAG_POSE_DIM> & shrd_placeholder,
    const std::array<double, INDEP_GROUND_TAG_POSE_DIM> & indep_placeholder,
    std::shared_ptr<cv::Affine3d> & pose, bool invert);

  std::set<int> waypoint_tag_ids_set_;
  std::set<int> ground_tag_ids_set_;
  std::set<int> wheel_tag_ids_set_;

  int left_wheel_tag_id_;
  int right_wheel_tag_id_;

  bool optimize_intrinsics_;
  bool share_intrinsics_;
  bool force_shared_ground_plane_;

  IntrinsicParameters calibration_sensor_intrinsics_;
  IntrinsicParameters external_camera_intrinsics_;

  CalibrationData::Ptr data_;

  // Optimization placeholders
  std::map<UID, std::array<double, POSE_OPT_DIM>> pose_opt_map;
  std::array<double, SHRD_GROUND_TAG_POSE_DIM> shrd_ground_tag_pose_opt;
  std::map<UID, std::array<double, INDEP_GROUND_TAG_POSE_DIM>> indep_ground_tag_pose_opt_map;
  std::map<UID, std::array<double, INTRINSICS_DIM>> intrinsics_opt_map;
  std::array<double, INTRINSICS_DIM> shared_intrinsics_opt;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CALIBRATION_PROBLEM_HPP_
