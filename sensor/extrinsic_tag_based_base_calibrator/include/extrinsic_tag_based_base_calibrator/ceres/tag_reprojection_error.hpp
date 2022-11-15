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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__TAG_REPROJECTION_ERROR_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__TAG_REPROJECTION_ERROR_HPP_

#include <extrinsic_tag_based_base_calibrator/ceres/ceres_functions.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <opencv2/core.hpp>

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <array>
#include <memory>

namespace extrinsic_tag_based_base_calibrator
{

struct TagReprojectionError
{
  TagReprojectionError(
    const UID & camera_uid, const IntrinsicParameters & intrinsics,
    const ApriltagDetection & detection,
    const std::shared_ptr<std::array<double, CalibrationData::POSE_OPT_DIM>> &
      fixed_camera_pose_inv,
    const std::shared_ptr<std::array<double, CalibrationData::POSE_OPT_DIM>> & fixed_tag_pose,
    bool fix_camera_pose, bool optimize_intrinsics, bool fix_tag_pose, bool is_ground_tag)
  : camera_uid_(camera_uid),
    intrinsics_(intrinsics),
    detection_(detection),
    fix_camera_pose_(fix_camera_pose),
    optimize_intrinsics_(optimize_intrinsics),
    fix_tag_pose_(fix_tag_pose),
    is_ground_tag_(is_ground_tag)
  {
    fx_ = intrinsics.undistorted_camera_matrix(0, 0);
    fy_ = intrinsics.undistorted_camera_matrix(1, 1);
    cx_ = intrinsics.undistorted_camera_matrix(0, 2);
    cy_ = intrinsics.undistorted_camera_matrix(1, 2);

    tag_size_ = detection.size;

    for (int j = 0; j < 4; ++j) {
      observed_corners_[j] = Eigen::Vector2d(
        static_cast<double>(detection.corners[j].x), static_cast<double>(detection.corners[j].y));
    }

    if (fix_camera_pose) {
      const Eigen::Map<const Eigen::Matrix<double, 4, 1>> rotation_map(
        fixed_camera_pose_inv->data());
      const Eigen::Map<const Eigen::Matrix<double, 3, 1>> translation_map(
        fixed_camera_pose_inv->data() + 4);

      fixed_camera_rotation_inv_ = rotation_map;
      fixed_camera_translation_inv_ = translation_map;
    }

    if (fix_tag_pose) {
      const Eigen::Map<const Eigen::Matrix<double, 4, 1>> rotation_map(fixed_tag_pose->data());
      const Eigen::Map<const Eigen::Matrix<double, 3, 1>> translation_map(
        fixed_tag_pose->data() + 4);

      fixed_tag_rotation_ = rotation_map;
      fixed_tag_translation_ = translation_map;
    }
  }

  template <typename T>
  bool impl(
    const T * const camera_pose_inv, const T * const camera_intrinsics, const T * const tag_pose,
    const T * const tag_rotation_z, const T * const tag_pose_2d, T * residuals) const
  {
    double hsize = 0.5 * tag_size_;

    Eigen::Matrix<T, 3, 1> template_corners_[4] = {
      {T(-hsize), T(hsize), T(0.0)},
      {T(hsize), T(hsize), T(0.0)},
      {T(hsize), T(-hsize), T(0.0)},
      {T(-hsize), T(-hsize), T(0.0)}};

    Eigen::Matrix<T, 3, 1> corners_wcs[4];
    Eigen::Matrix<T, 3, 1> corners_ccs[4];

    const Eigen::Map<const Eigen::Matrix<T, 4, 1>> camera_rotation_inv_map(camera_pose_inv);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> camera_translation_inv_map(&camera_pose_inv[4]);
    const Eigen::Map<const Eigen::Matrix<T, 6, 1>> camera_intrinsics_map(camera_intrinsics);

    assert(fix_tag_pose_ == false);

    // Template corners to World coordinate system (wcs)
    if (!is_ground_tag_) {
      const Eigen::Map<const Eigen::Matrix<T, 4, 1>> tag_rotation_map(tag_pose);
      const Eigen::Map<const Eigen::Matrix<T, 3, 1>> tag_translation_map(&tag_pose[4]);
      transformCorners(tag_rotation_map, tag_translation_map, template_corners_, corners_wcs);
    } else {
      const Eigen::Map<const Eigen::Matrix<T, 4, 1>> tag_rotation_map(tag_rotation_z);
      const T & tag_z = tag_rotation_z[4];

      Eigen::Matrix<T, 3, 3> tag_rotation_2d = rotationMatrixFromYaw(tag_pose_2d[0]);
      const Eigen::Map<const Eigen::Matrix<T, 2, 1>> tag_translation_2d(&tag_pose_2d[1]);
      transformCorners2(
        tag_rotation_map, tag_z, tag_rotation_2d, tag_translation_2d, template_corners_,
        corners_wcs);
    }

    // World corners to camera coordinate system (ccs)
    if (fix_camera_pose_) {
      const Eigen::Map<const Eigen::Matrix<double, 4, 1>> rotation_map(
        fixed_camera_rotation_inv_.data());
      const Eigen::Map<const Eigen::Matrix<double, 3, 1>> translation_map(
        fixed_camera_translation_inv_.data());
      transformCorners(rotation_map, translation_map, corners_wcs, corners_ccs);
    } else {
      transformCorners(
        camera_rotation_inv_map, camera_translation_inv_map, corners_wcs, corners_ccs);
    }

    // Compute the reprojection error residuals
    auto compute_reproj_error_point = [&](
                                        auto & predicted_ccs, auto observed_ics, auto * residuals) {
      const T & cx = camera_intrinsics_map(0);
      const T & cy = camera_intrinsics_map(1);
      const T & fx = camera_intrinsics_map(2);
      const T & fy = camera_intrinsics_map(3);
      const T & k1 = camera_intrinsics_map(4);
      const T & k2 = camera_intrinsics_map(5);

      const T xp = predicted_ccs.x() / predicted_ccs.z();
      const T yp = predicted_ccs.y() / predicted_ccs.z();
      const T r2 = xp * xp + yp * yp;
      const T d = 1.0 + r2 * (k1 + k2 * r2);
      const T predicted_ics_x = cx + fx * d * xp;
      const T predicted_ics_y = cy + fy * d * yp;

      residuals[0] = predicted_ics_x - observed_ics.x();
      residuals[1] = predicted_ics_y - observed_ics.y();
    };

    for (int i = 0; i < 4; i++) {
      compute_reproj_error_point(corners_ccs[i], observed_corners_[i], residuals + 2 * i);
    }

    return true;
  }

  // Case where the tag is in the ground and the intrinsics are optimized
  template <typename T>
  bool operator()(
    const T * const camera_pose_inv, const T * const camera_intrinsics, const T * const tag_rot_z,
    const T * const tag_pose_2d, T * residuals) const
  {
    assert(fix_camera_pose_ == false);
    assert(optimize_intrinsics_ == true);
    assert(fix_tag_pose_ == false);
    assert(is_ground_tag_ == true);

    return impl(
      camera_pose_inv, camera_intrinsics, static_cast<T *>(nullptr), tag_rot_z, tag_pose_2d,
      residuals);

    return false;
  }

  template <typename T>
  bool operator()(
    const T * const arg1, const T * const arg2, const T * const arg3, T * residuals) const
  {
    if (is_ground_tag_) {
      // Case where the tag is in the ground and not the intrinsics are not optimized
      assert(fix_camera_pose_ == false);
      assert(optimize_intrinsics_ == false);
      assert(fix_tag_pose_ == false);

      return impl(
        arg1, static_cast<T *>(nullptr), static_cast<T *>(nullptr), arg2, arg3, residuals);

    } else {
      // Case where the camera is not fixed and the intrinsics are optimized
      assert(fix_camera_pose_ == false);
      assert(optimize_intrinsics_ == true);
      assert(fix_tag_pose_ == false);

      return impl(
        arg1, arg2, arg3, static_cast<T *>(nullptr), static_cast<T *>(nullptr), residuals);
    }
  }

  // Case where the camera has fixed intrinsics
  template <typename T>
  bool operator()(const T * const camera_pose_inv, const T * const tag_pose, T * residuals) const
  {
    assert(fix_camera_pose_ == false);
    assert(optimize_intrinsics_ == false);
    assert(fix_tag_pose_ == false);
    assert(is_ground_tag_ == false);

    std::array<T, 6> intrinsics{T(1.0) * cx_, T(1.0) * cy_, T(1.0) * fx_,
                                T(1.0) * fy_, T(0.0),       T(0.0)};

    return impl(
      camera_pose_inv, intrinsics.data(), tag_pose, static_cast<T *>(nullptr),
      static_cast<T *>(nullptr), residuals);
  }

  // Case where the camera has known pose and fixed intrinsics
  template <typename T>
  bool operator()(const T * const tag_pose, T * residuals) const
  {
    assert(fix_camera_pose_ == true);
    assert(optimize_intrinsics_ == false);
    assert(fix_tag_pose_ == false);
    assert(is_ground_tag_ == false);

    std::array<T, 6> intrinsics{T(1.0) * cx_, T(1.0) * cy_, T(1.0) * fx_,
                                T(1.0) * fy_, T(0.0),       T(0.0)};

    return impl(
      static_cast<T *>(nullptr), intrinsics.data(), tag_pose, static_cast<T *>(nullptr),
      static_cast<T *>(nullptr), residuals);
  }

  static ceres::CostFunction * createTagResidual(
    const UID & camera_uid, const IntrinsicParameters & intrinsics,
    const ApriltagDetection & detection,
    std::shared_ptr<std::array<double, CalibrationData::POSE_OPT_DIM>> & fixed_camera_pose_inv,
    std::shared_ptr<std::array<double, CalibrationData::POSE_OPT_DIM>> & fixed_tag_pose,
    bool fix_camera_pose, bool optimize_intrinsics, bool fix_tag_pose)
  {
    auto f = new TagReprojectionError(
      camera_uid, intrinsics, detection, fixed_camera_pose_inv, fixed_tag_pose, fix_camera_pose,
      optimize_intrinsics, fix_tag_pose, false);

    if (fix_camera_pose && !optimize_intrinsics) {
      return (new ceres::AutoDiffCostFunction<
              TagReprojectionError,
              8,   // 4 corners x 2 residuals
              7>(  // 7 tag pose parameters
        f));
    } else if (!fix_camera_pose && !optimize_intrinsics) {
      return (new ceres::AutoDiffCostFunction<
              TagReprojectionError,
              8,   // 4 corners x 2 residuals
              7,   // 7 camera pose parameters
              7>(  // 7 tag pose parameters
        f));
    } else if (fix_camera_pose && optimize_intrinsics) {
      return (new ceres::AutoDiffCostFunction<
              TagReprojectionError,
              8,   // 4 corners x 2 residuals
              6,   // 6 camera intrinsic parameters
              7>(  // 7 tag pose parameters
        f));
    } else if (!fix_camera_pose && optimize_intrinsics) {
      return (new ceres::AutoDiffCostFunction<
              TagReprojectionError,
              8,   // 4 corners x 2 residuals
              7,   // 7 camera pose parameters
              6,   // 6 camera intrinsic parameters
              7>(  // 7 tag pose parameters
        f));
    }

    return nullptr;
  }

  static ceres::CostFunction * createGroundTagResidual(
    const UID & camera_uid, const IntrinsicParameters & intrinsics,
    const ApriltagDetection & detection, bool optimize_intrinsics)
  {
    auto f = new TagReprojectionError(
      camera_uid, intrinsics, detection, nullptr, nullptr, false, optimize_intrinsics, false, true);

    if (optimize_intrinsics) {
      return (new ceres::AutoDiffCostFunction<
              TagReprojectionError,
              8,   // 4 corners x 2 residuals
              7,   // 7 camera pose parameters
              6,   // 6 camera intrinsic parameters
              5,   // 5 shared ground pose parameters (quat/z) parameters
              3>(  // 3 2D-ground tag pose (yaw/x/y)  parameters
        f));
    } else {
      return (new ceres::AutoDiffCostFunction<
              TagReprojectionError,
              8,   // 4 corners x 2 residuals
              7,   // 7 camera pose parameters
              5,   // 5 shared ground pose parameters (quat/z) parameters
              3>(  // 3 2D-ground tag pose (yaw/x/y)  parameters
        f));
    }
  }

  double cx_;
  double cy_;
  double fx_;
  double fy_;
  double tag_size_;

  Eigen::Vector2d observed_corners_[4];

  UID camera_uid_;
  IntrinsicParameters intrinsics_;
  ApriltagDetection detection_;
  Eigen::Vector4d fixed_camera_rotation_inv_;
  Eigen::Vector3d fixed_camera_translation_inv_;
  Eigen::Vector4d fixed_tag_rotation_;
  Eigen::Vector3d fixed_tag_translation_;
  bool fix_camera_pose_;
  bool optimize_intrinsics_;
  bool fix_tag_pose_;
  bool is_ground_tag_;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__TAG_REPROJECTION_ERROR_HPP_
