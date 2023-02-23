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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CAMERA_RESIDUAL_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CAMERA_RESIDUAL_HPP_

#include <extrinsic_tag_based_base_calibrator/calibration_types.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <opencv2/core.hpp>

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <array>
#include <memory>

namespace extrinsic_tag_based_base_calibrator
{

struct CameraResidual
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
  static constexpr int NUM_CORNERS = 4;

  CameraResidual(
    const UID & camera_uid, const IntrinsicParameters & intrinsics,
    const ApriltagDetection & detection,
    const std::array<double, CalibrationData::POSE_OPT_DIM> & fixed_camera_pose_inv,
    bool fix_camera_pose, bool optimize_intrinsics, bool is_ground_tag)
  : camera_uid_(camera_uid),
    intrinsics_(intrinsics),
    detection_(detection),
    fix_camera_pose_(fix_camera_pose),
    optimize_intrinsics_(optimize_intrinsics),
    is_ground_tag_(is_ground_tag)
  {
    fx_ = intrinsics.undistorted_camera_matrix(0, 0);
    fy_ = intrinsics.undistorted_camera_matrix(1, 1);
    cx_ = intrinsics.undistorted_camera_matrix(0, 2);
    cy_ = intrinsics.undistorted_camera_matrix(1, 2);

    tag_size_ = detection.size;

    for (int j = 0; j < NUM_CORNERS; ++j) {
      observed_corners_[j] =
        Eigen::Vector2d(detection.image_corners[j].x, detection.image_corners[j].y);
    }

    if (fix_camera_pose) {
      const Eigen::Map<const Eigen::Vector4d> rotation_map(fixed_camera_pose_inv.data());
      const Eigen::Map<const Eigen::Vector3d> translation_map(
        fixed_camera_pose_inv.data() + TRANSLATION_X_INDEX);

      fixed_camera_rotation_inv_ = rotation_map;
      fixed_camera_translation_inv_ = translation_map;
    }
  }

  /*!
   * Auxiliar method to construct a 3d rotation matrix representing a 2d rotatin matrix
   * @param[in] yaw the rotation angle in the Z axis
   */
  template <typename T>
  Eigen::Matrix<T, 3, 3> rotationMatrixFromYaw(const T & yaw) const
  {
    const T cos = ceres::cos(yaw);
    const T sin = ceres::sin(yaw);
    Eigen::Matrix<T, 3, 3> rotation;
    rotation << cos, -sin, T(0.0), sin, cos, T(0.0), T(0.0), T(0.0), T(1.0);
    return rotation;
  }

  /*!
   * The implementation of the cost function
   * @param[in] camera_pose_inv The pose from the camera to the origin
   * @param[in] camera_intrinsics The camera intrinsics
   * @param[in] tag_pose The pose of the tag to project into the camera
   * @param[in] tag_rotation_z The pose from the ground plane to the origin (only used when using
   * ground tags)
   * @param[in] tag_pose_2d The pose from ground plane to the tag (only used when using ground tags)
   * @param[in] residuals The residual error of projecting the tag into the camera
   * @returns success status
   */
  template <typename T>
  bool impl(
    const T * const camera_pose_inv, const T * const camera_intrinsics, const T * const tag_pose,
    const T * const tag_rotation_z, const T * const tag_pose_2d, T * residuals) const
  {
    double hsize = 0.5 * tag_size_;

    Vector3<T> template_corners_[NUM_CORNERS] = {
      {T(-hsize), T(hsize), T(0.0)},
      {T(hsize), T(hsize), T(0.0)},
      {T(hsize), T(-hsize), T(0.0)},
      {T(-hsize), T(-hsize), T(0.0)}};

    Vector3<T> corners_wcs[NUM_CORNERS];
    Vector3<T> corners_ccs[NUM_CORNERS];

    const Eigen::Map<const Vector4<T>> camera_rotation_inv_map(camera_pose_inv);
    const Eigen::Map<const Vector3<T>> camera_translation_inv_map(
      &camera_pose_inv[TRANSLATION_X_INDEX]);
    const Eigen::Map<const Vector6<T>> camera_intrinsics_map(camera_intrinsics);

    auto transform_corners =
      [](auto & quaternion, auto & translation, auto & input_corners, auto & output_corners) {
        for (int i = 0; i < NUM_CORNERS; i++) {
          output_corners[i] = quaternion * input_corners[i] + translation;
        }
      };

    // Template corners to World coordinate system (wcs)
    if (!is_ground_tag_) {
      const Eigen::Map<const Vector4<T>> tag_rotation_map(tag_pose);
      const Eigen::Map<const Vector3<T>> tag_translation_map(&tag_pose[TRANSLATION_X_INDEX]);

      Eigen::Quaternion<T> tag_quaternion = {
        tag_rotation_map(ROTATION_W_INDEX), tag_rotation_map(ROTATION_X_INDEX),
        tag_rotation_map(ROTATION_Y_INDEX), tag_rotation_map(ROTATION_Z_INDEX)};

      tag_quaternion = tag_quaternion.normalized();

      transform_corners(tag_quaternion, tag_translation_map, template_corners_, corners_wcs);

    } else {
      const Eigen::Map<const Vector4<T>> tag_rotation_map(tag_rotation_z);
      Eigen::Quaternion<T> tag_quaternion = {
        tag_rotation_map(ROTATION_W_INDEX), tag_rotation_map(ROTATION_X_INDEX),
        tag_rotation_map(ROTATION_Y_INDEX), tag_rotation_map(ROTATION_Z_INDEX)};

      const Matrix3<T> tag_rotation_2d = rotationMatrixFromYaw(tag_pose_2d[GROUND_TAG_YAW_INDEX]);
      Vector3<T> tag_translation_2d(
        tag_pose_2d[GROUND_TAG_X_INDEX], tag_pose_2d[GROUND_TAG_Y_INDEX], T(0));
      transform_corners(tag_rotation_2d, tag_translation_2d, template_corners_, corners_wcs);

      tag_quaternion = tag_quaternion.normalized().inverse();
      Vector3<T> translation =
        T(-1.0) * (tag_quaternion * Vector3<T>(T(0), T(0), tag_rotation_z[GROUND_TAG_D_INDEX]));
      transform_corners(tag_quaternion, translation, corners_wcs, corners_wcs);
    }

    // World corners to camera coordinate system (ccs)
    if (fix_camera_pose_) {
      const Eigen::Map<const Eigen::Vector4d> fixed_camera_rotation_inv_map(
        fixed_camera_rotation_inv_.data());
      const Eigen::Map<const Eigen::Vector3d> fixed_camera_translation_inv_map(
        fixed_camera_translation_inv_.data());

      Eigen::Quaternion<T> fixed_camera_rotation_inv_quaternion = {
        T(1) * fixed_camera_rotation_inv_map(ROTATION_W_INDEX),
        T(1) * fixed_camera_rotation_inv_map(ROTATION_X_INDEX),
        T(1) * fixed_camera_rotation_inv_map(ROTATION_Y_INDEX),
        T(1) * fixed_camera_rotation_inv_map(ROTATION_Z_INDEX)};

      fixed_camera_rotation_inv_quaternion = fixed_camera_rotation_inv_quaternion.normalized();
      transform_corners(
        fixed_camera_rotation_inv_quaternion, fixed_camera_translation_inv_map, corners_wcs,
        corners_ccs);

    } else {
      Eigen::Quaternion<T> camera_rotation_inv_quaternion = {
        camera_rotation_inv_map(ROTATION_W_INDEX), camera_rotation_inv_map(ROTATION_X_INDEX),
        camera_rotation_inv_map(ROTATION_Y_INDEX), camera_rotation_inv_map(ROTATION_Z_INDEX)};

      camera_rotation_inv_quaternion = camera_rotation_inv_quaternion.normalized();
      transform_corners(
        camera_rotation_inv_quaternion, camera_translation_inv_map, corners_wcs, corners_ccs);
    }

    // Compute the reprojection error residuals
    auto compute_reproj_error_point = [&](
                                        auto & predicted_ccs, auto observed_ics, auto * residuals) {
      const T & cx = camera_intrinsics_map(INTRINSICS_CX_INDEX);
      const T & cy = camera_intrinsics_map(INTRINSICS_CY_INDEX);
      const T & fx = camera_intrinsics_map(INTRINSICS_FX_INDEX);
      const T & fy = camera_intrinsics_map(INTRINSICS_FY_INDEX);
      const T & k1 = camera_intrinsics_map(INTRINSICS_K1_INDEX);
      const T & k2 = camera_intrinsics_map(INTRINSICS_K2_INDEX);

      const T xp = predicted_ccs.x() / predicted_ccs.z();
      const T yp = predicted_ccs.y() / predicted_ccs.z();
      const T r2 = xp * xp + yp * yp;
      const T d = 1.0 + r2 * (k1 + k2 * r2);
      const T predicted_ics_x = cx + fx * d * xp;
      const T predicted_ics_y = cy + fy * d * yp;

      residuals[0] = predicted_ics_x - observed_ics.x();
      residuals[1] = predicted_ics_y - observed_ics.y();
    };

    for (int i = 0; i < NUM_CORNERS; i++) {
      compute_reproj_error_point(corners_ccs[i], observed_corners_[i], residuals + 2 * i);
    }

    return true;
  }

  /*!
   * The cost function wrapper for the case where the tag is in the ground and the intrinsics are
   * optimized
   * @param[in] camera_pose_inv The pose from the camera to the origin
   * @param[in] camera_intrinsics The camera intrinsics
   * @param[in] tag_rotation_z The pose from the ground plane to the origin (only used when using
   * ground tags)
   * @param[in] tag_pose_2d The pose from ground plane to the tag (only used when using ground tags)
   * @param[in] residuals The residual error of projecting the tag into the camera
   * @returns success status
   */
  template <typename T>
  bool operator()(
    const T * const camera_pose_inv, const T * const camera_intrinsics, const T * const tag_rot_z,
    const T * const tag_pose_2d, T * residuals) const
  {
    assert(fix_camera_pose_ == false);
    assert(optimize_intrinsics_ == true);
    assert(is_ground_tag_ == true);

    return impl(
      camera_pose_inv, camera_intrinsics, static_cast<T *>(nullptr), tag_rot_z, tag_pose_2d,
      residuals);

    return false;
  }

  /*!
   * The cost function wrapper for the following casesL
   *     - the tag is in the ground and the intrinsics are not optimized
   *     - the camera is not fixed and the intrinsics are optimized
   * @param[in] arg1 The pose from the camera to the origin
   * @param[in] arg2 The camera intrinsics
   * @param[in] arg3 The pose from the ground plane to the origin (only used when using ground tags)
   * @param[in] residuals The residual error of projecting the tag into the camera
   * @returns success status
   */
  template <typename T>
  bool operator()(
    const T * const arg1, const T * const arg2, const T * const arg3, T * residuals) const
  {
    if (is_ground_tag_) {
      // Case where the tag is in the ground and the intrinsics are not optimized
      assert(fix_camera_pose_ == false);
      assert(optimize_intrinsics_ == false);

      std::array<T, INTRINSICS_DIM> intrinsics{T(1.0) * cx_, T(1.0) * cy_, T(1.0) * fx_,
                                               T(1.0) * fy_, T(0.0),       T(0.0)};

      return impl(arg1, intrinsics.data(), static_cast<T *>(nullptr), arg2, arg3, residuals);

    } else {
      // Case where the camera is not fixed and the intrinsics are optimized
      assert(fix_camera_pose_ == false);
      assert(optimize_intrinsics_ == true);

      return impl(
        arg1, arg2, arg3, static_cast<T *>(nullptr), static_cast<T *>(nullptr), residuals);
    }
  }

  /*!
   * The cost function wrapper for one of the following cases:
   *   - the basic case of a normal tag with the intrinsics not being optimized
   *   - the case of a fixed tag with intrinsics being optimized
   * @param[in] camera_pose_inv The pose from the camera to the origin
   * @param[in] tag_pose The pose of the tag
   * @param[in] residuals The residual error of projecting the tag into the camera
   * @returns success status
   */
  template <typename T>
  bool operator()(const T * const arg1, const T * const arg2, T * residuals) const
  {
    assert(fix_camera_pose_ == false);
    assert(is_ground_tag_ == false);
    assert(optimize_intrinsics_ == false);

    const T * const camera_pose_inv = arg1;
    const T * const tag_pose = arg2;

    std::array<T, INTRINSICS_DIM> intrinsics{T(1.0) * cx_, T(1.0) * cy_, T(1.0) * fx_,
                                             T(1.0) * fy_, T(0.0),       T(0.0)};

    return impl(
      camera_pose_inv, intrinsics.data(), tag_pose, static_cast<T *>(nullptr),
      static_cast<T *>(nullptr), residuals);
  }

  /*!
   * The cost function wrapper for the case where the camera has a fixed pose and intrinsics
   * @param[in] tag_pose The pose of the tag
   * @param[in] residuals The residual error of projecting the tag into the camera
   * @returns success status
   */
  template <typename T>
  bool operator()(const T * const tag_pose, T * residuals) const
  {
    assert(fix_camera_pose_ == true);
    assert(optimize_intrinsics_ == false);
    assert(is_ground_tag_ == false);

    std::array<T, INTRINSICS_DIM> intrinsics{T(1.0) * cx_, T(1.0) * cy_, T(1.0) * fx_,
                                             T(1.0) * fy_, T(0.0),       T(0.0)};

    return impl(
      static_cast<T *>(nullptr), intrinsics.data(), tag_pose, static_cast<T *>(nullptr),
      static_cast<T *>(nullptr), residuals);
  }

  /*!
   * Residual factory function of a normal tag & camera residual without exposing much templates
   * @param[in] camera_uid The detection's uid
   * @param[in] intrinsics The camera intrinsics
   * @param[in] detection The tag detection
   * @param[in] fixed_camera_pose_inv The fixed pose of the camera
   * @param[in] fix_camera_pose Whether the camera pose should be fixed during optimization
   * @param[in] optimize_intrinsics Whether the intrinsics should be optimized
   * @returns the ceres residual
   */
  static ceres::CostFunction * createTagResidual(
    const UID & camera_uid, const IntrinsicParameters & intrinsics,
    const ApriltagDetection & detection,
    std::array<double, CalibrationData::POSE_OPT_DIM> & fixed_camera_pose_inv, bool fix_camera_pose,
    bool optimize_intrinsics)
  {
    auto f = new CameraResidual(
      camera_uid, intrinsics, detection, fixed_camera_pose_inv, fix_camera_pose,
      optimize_intrinsics, false);

    if (fix_camera_pose && !optimize_intrinsics) {
      return (new ceres::AutoDiffCostFunction<
              CameraResidual,
              RESIDUAL_DIM,   // 4 corners x 2 residuals
              POSE_OPT_DIM>(  // 7 tag pose parameters
        f));
    } else if (!fix_camera_pose && !optimize_intrinsics) {
      return (new ceres::AutoDiffCostFunction<
              CameraResidual,
              RESIDUAL_DIM,   // 4 corners x 2 residuals
              POSE_OPT_DIM,   // 7 camera pose parameters
              POSE_OPT_DIM>(  // 7 tag pose parameters
        f));
    } else if (fix_camera_pose && optimize_intrinsics) {
      return (new ceres::AutoDiffCostFunction<
              CameraResidual,
              RESIDUAL_DIM,    // 4 corners x 2 residuals
              INTRINSICS_DIM,  // 6 camera intrinsic parameters
              POSE_OPT_DIM>(   // 7 tag pose parameters
        f));
    } else if (!fix_camera_pose && optimize_intrinsics) {
      return (new ceres::AutoDiffCostFunction<
              CameraResidual,
              RESIDUAL_DIM,    // 4 corners x 2 residuals
              POSE_OPT_DIM,    // 7 camera pose parameters
              INTRINSICS_DIM,  // 6 camera intrinsic parameters
              POSE_OPT_DIM>(   // 7 tag pose parameters
        f));
    }

    return nullptr;
  }

  /*!
   * Residual factory function of a ground tag & camera residual without exposing much templates
   * @param[in] camera_uid The detection's uid
   * @param[in] intrinsics The camera intrinsics
   * @param[in] detection The tag detection
   * @param[in] optimize_intrinsics Whether the intrinsics should be optimized
   * @returns the ceres residual
   */
  static ceres::CostFunction * createGroundTagResidual(
    const UID & camera_uid, const IntrinsicParameters & intrinsics,
    const ApriltagDetection & detection,
    std::array<double, CalibrationData::POSE_OPT_DIM> & fixed_camera_pose_inv, bool fix_camera_pose,
    bool optimize_intrinsics)
  {
    auto f = new CameraResidual(
      camera_uid, intrinsics, detection, fixed_camera_pose_inv, fix_camera_pose,
      optimize_intrinsics, true);

    if (optimize_intrinsics) {
      return (new ceres::AutoDiffCostFunction<
              CameraResidual,
              RESIDUAL_DIM,                // 4 corners x 2 residuals
              POSE_OPT_DIM,                // 7 camera pose parameters
              INTRINSICS_DIM,              // 6 camera intrinsic parameters
              SHRD_GROUND_TAG_POSE_DIM,    // 5 shared ground pose parameters (quat/z) parameters
              INDEP_GROUND_TAG_POSE_DIM>(  // 3 2D-ground tag pose (yaw/x/y)  parameters
        f));
    } else {
      return (new ceres::AutoDiffCostFunction<
              CameraResidual,
              RESIDUAL_DIM,                // 4 corners x 2 residuals
              POSE_OPT_DIM,                // 7 camera pose parameters
              SHRD_GROUND_TAG_POSE_DIM,    // 5 shared ground pose parameters (quat/z) parameters
              INDEP_GROUND_TAG_POSE_DIM>(  // 3 2D-ground tag pose (yaw/x/y)  parameters
        f));
    }
  }

  double cx_;
  double cy_;
  double fx_;
  double fy_;
  double tag_size_;

  Eigen::Vector2d observed_corners_[NUM_CORNERS];

  UID camera_uid_;
  IntrinsicParameters intrinsics_;
  ApriltagDetection detection_;
  Eigen::Vector4d fixed_camera_rotation_inv_;
  Eigen::Vector3d fixed_camera_translation_inv_;
  bool fix_camera_pose_;
  bool optimize_intrinsics_;
  bool is_ground_tag_;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CAMERA_RESIDUAL_HPP_
