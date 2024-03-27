// Copyright 2024 Tier IV, Inc.
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

#ifndef TAG_BASED_SFM_CALIBRATOR__CERES__SENSOR_RESIDUAL_HPP_
#define TAG_BASED_SFM_CALIBRATOR__CERES__SENSOR_RESIDUAL_HPP_

#include <tag_based_sfm_calibrator/calibration_types.hpp>
#include <tag_based_sfm_calibrator/types.hpp>

namespace tag_based_sfm_calibrator
{

struct SensorResidual
{
  template <class T>
  using Vector2 = Eigen::Matrix<T, 2, 1>;

  template <class T>
  using Vector3 = Eigen::Matrix<T, 3, 1>;

  template <class T>
  using Vector4 = Eigen::Matrix<T, 4, 1>;

  template <class T>
  using Vector6 = Eigen::Matrix<T, 6, 1>;

  template <typename T>
  using Matrix3 = Eigen::Matrix<T, 3, 3>;

  static constexpr int POSE_OPT_DIM = CalibrationData::POSE_OPT_DIM;
  static constexpr int SHRD_GROUND_TAG_POSE_DIM =
    CalibrationData::SHRD_GROUND_TAG_POSE_DIM;  // cSpell:ignore SHRD
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
  static constexpr int NUM_CORNERS = 4;
};

}  // namespace tag_based_sfm_calibrator

#endif  // TAG_BASED_SFM_CALIBRATOR__CERES__SENSOR_RESIDUAL_HPP_
