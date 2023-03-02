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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CALIBRATION_PROBLEM_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CALIBRATION_PROBLEM_HPP_

#include <extrinsic_tag_based_base_calibrator/calibration_types.hpp>
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

  static constexpr int GROUND_TAG_D_INDEX = CalibrationData::GROUND_TAG_D_INDEX;
  static constexpr int GROUND_TAG_YAW_INDEX = CalibrationData::GROUND_TAG_YAW_INDEX;
  static constexpr int GROUND_TAG_X_INDEX = CalibrationData::GROUND_TAG_X_INDEX;
  static constexpr int GROUND_TAG_Y_INDEX = CalibrationData::GROUND_TAG_Y_INDEX;

  static constexpr int RESIDUAL_DIM = 8;

  /*!
   * Sets whether or not the external camera intrinsics should be optimized during ba
   * @param[in] ba_optimize_intrinsics whether or not the external camera intrinsics should be
   * optimized
   */
  void setOptimizeIntrinsics(bool ba_optimize_intrinsics);

  /*!
   * Sets whether or not the external camera intrinsics are shared among observations
   * @param[in] ba_share_intrinsics whether or not the external camera intrinsics are shared among
   * observations
   */
  void setShareIntrinsics(bool ba_share_intrinsics);

  /*!
   * Sets whether or not the ground tags are forced to lie on a single plane during ba
   * @param[in] ba_force_shared_ground_plane whether or not the ground tags are forced to lie on a
   * single plane
   */
  void setForceSharedGroundPlane(bool ba_force_shared_ground_plane);

  /*!
   * Sets the external camera intrinsics
   * @param[in] intrinsics the camera intrinsics
   */
  void setExternalCameraIntrinsics(IntrinsicParameters & intrinsics);

  /*!
   * Sets the calibration cameras intrinsics
   * @param[in] calibration_camera_intrinsics_map the camera intrinsics
   */
  void setCalibrationCameraIntrinsics(
    std::map<UID, IntrinsicParameters> & calibration_camera_intrinsics_map);

  /*!
   * Sets the calibration lidars intrinsics
   * @param[in] calibration_lidar_virtual_f the camera intrinsics
   */
  void setCalibrationLidarIntrinsics(double calibration_lidar_virtual_f);

  /*!
   * Sets UIDs corresponding to the left and wheel tags
   * @param[in] left_wheel_tag_uid the left wheel tag uid
   * @param[in] right_wheel_tag_uid the right wheel tag uid
   */
  void setWheelTagUIDs(UID left_wheel_tag_uid, UID right_wheel_tag_uid);

  /*!
   * Sets whether the waypoint poses should be fixed during optimization. This is used for base
   * lidar calibration
   * @param[in] fix_waypoint_poses whether the waypoint poses should be fixed during optimization
   */
  void setFixedWaypointPoses(bool fix_waypoint_poses);

  /*!
   * Sets the calibration data
   * @param[in] data the calibration data
   */
  void setData(CalibrationData::Ptr & data);

  /*!
   * Formats the input data into optimization placeholders
   */
  void dataToPlaceholders();

  /*!
   * Extracts the information from the optimization placeholders and formats it into the calibration
   * data structure
   */
  void placeholdersToData();

  /*!
   * Evaluates the current optimization variables with the BA cost function
   */
  void evaluate();

  /*!
   * Formulates and solves the BA problem
   */
  void solve();

  /*!
   * Generates debug image displaying the detections with the poses and intrinsics resulting from BA
   */
  void writeDebugImages();

protected:
  /*!
   * Converts a 3d pose into a normal tag optimization placeholder
   * @param[in] pose the input 3d pose
   * @param[out] placeholder the output placeholder
   * @param[in] invert whether or not the pose should be inverted
   */
  void pose3dToPlaceholder(
    cv::Affine3d pose, std::array<double, POSE_OPT_DIM> & placeholder, bool invert);

  /*!
   * Converts a normal tag optimization placeholder into a 3d pose
   * @param[in] placeholder the input placeholder
   * @param[out] pose the output 3d pose
   * @param[in] invert whether or not the pose should be inverted
   */
  void placeholderToPose3d(
    const std::array<double, POSE_OPT_DIM> & placeholder, std::shared_ptr<cv::Affine3d> & pose,
    bool invert);

  /*!
   * Converts a 3d pose into a ground tag optimization placeholder
   * @param[in] pose the input 3d pose
   * @param[out] shrd_placeholder the output placeholder shared among all ground tags
   * @param[out] indep_placeholder the output placeholder independent from other ground tags
   */
  void pose3dToGroundTagPlaceholder(
    cv::Affine3d tag_pose, cv::Affine3d ground_pose,
    std::array<double, SHRD_GROUND_TAG_POSE_DIM> & shrd_placeholder,
    std::array<double, INDEP_GROUND_TAG_POSE_DIM> & indep_placeholder);

  /*!
   * Converts a ground tag optimization placeholder into a 3d pose
   * @param[in] shrd_placeholder the input placeholder shared among all ground tags
   * @param[in] indep_placeholder the input placeholder independent from other ground tags
   * @param[out] pose the output 3d pose
   */
  void groundTagPlaceholderToPose3d(
    const std::array<double, SHRD_GROUND_TAG_POSE_DIM> & shrd_placeholder,
    const std::array<double, INDEP_GROUND_TAG_POSE_DIM> & indep_placeholder,
    std::shared_ptr<cv::Affine3d> & pose);

  // std::set<int> waypoint_tag_ids_set_; on the way to be deprecated
  // std::set<int> ground_tag_ids_set_;
  // std::set<int> wheel_tag_ids_set_;

  UID left_wheel_tag_uid_;
  UID right_wheel_tag_uid_;

  bool optimize_intrinsics_;
  bool share_intrinsics_;
  bool force_shared_ground_plane_;

  double calibration_lidar_intrinsics_;
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
