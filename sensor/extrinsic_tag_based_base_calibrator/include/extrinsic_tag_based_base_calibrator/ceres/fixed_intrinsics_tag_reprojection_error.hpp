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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__FIXED_INTRINSICS_TAG_REPROJECTION_ERROR_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__FIXED_INTRINSICS_TAG_REPROJECTION_ERROR_HPP_

#include <extrinsic_tag_based_base_calibrator/ceres/ceres_functions.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <opencv2/core.hpp>

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <array>

namespace extrinsic_tag_based_base_calibrator
{

struct FixedIntrinsicsTagReprojectionError
{
  enum FixedPoseType {
    FixedCameraPose,  // the sensor camera pose uses optimization, but the pose itself is fixed
    FixedTagPose,     // the waypoints are fixed for base-lidar calibration
    NoFixedPose,
  };

  FixedIntrinsicsTagReprojectionError(
    const UID & camera_uid, const IntrinsicParameters & intrinsics,
    const ApriltagDetection & detection)
  : camera_uid_(camera_uid),
    intrinsics_(intrinsics),
    detection_(detection),
    fixed_pose_type_(NoFixedPose)
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
  }

  FixedIntrinsicsTagReprojectionError(
    const UID & camera_uid, const IntrinsicParameters & intrinsics,
    const ApriltagDetection & detection, std::array<double, 10> fixed_pose,
    FixedPoseType fixed_pose_type)
  : FixedIntrinsicsTagReprojectionError(camera_uid, intrinsics, detection)
  {
    const Eigen::Map<const Eigen::Matrix<double, 4, 1>> rotation_map(fixed_pose.data());
    const Eigen::Map<const Eigen::Matrix<double, 3, 1>> translation_map(fixed_pose.data() + 4);

    fixed_pose_type_ = fixed_pose_type;
    if (fixed_pose_type == NoFixedPose) {
      throw std::invalid_argument("Invalid argument");
    } else if (fixed_pose_type == FixedCameraPose) {
      fixed_camera_rotation_inv_ = rotation_map;
      fixed_camera_translation_inv_ = translation_map;
    } else if (fixed_pose_type == FixedTagPose) {
      fixed_tag_pose_ = fixed_pose;
    }
  }

  template <typename T>
  bool operator()(const T * const camera_pose_inv, const T * const tag_pose, T * residuals) const
  {
    double hsize = 0.5 * tag_size_;

    Eigen::Matrix<T, 3, 1> template_corners_[4] = {
      {T(-hsize), T(hsize), T(0.0)},
      {T(hsize), T(hsize), T(0.0)},
      {T(hsize), T(-hsize), T(0.0)},
      {T(-hsize), T(-hsize), T(0.0)}};

    Eigen::Matrix<T, 3, 1> corners_wcs[4];
    Eigen::Matrix<T, 3, 1> corners_ccs[4];

    const Eigen::Map<const Eigen::Matrix<T, 4, 1>> tag_rotation_map(tag_pose);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> tag_translation_map(&tag_pose[4]);

    const Eigen::Map<const Eigen::Matrix<T, 4, 1>> camera_rotation_inv_map(camera_pose_inv);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> camera_translation_inv_map(&camera_pose_inv[4]);

    assert(fixed_pose_type_ != FixedTagPose);

    // Template corners to World coordinate system (wcs)
    transformCorners(tag_rotation_map, tag_translation_map, template_corners_, corners_wcs);

    // World corners to camera coordinate system (ccs)
    if (fixed_pose_type_ == FixedCameraPose) {
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
      const T predicted_ics_x = cx_ + fx_ * (predicted_ccs.x() / predicted_ccs.z());
      const T predicted_ics_y = cy_ + fy_ * (predicted_ccs.y() / predicted_ccs.z());

      residuals[0] = predicted_ics_x - observed_ics.x();
      residuals[1] = predicted_ics_y - observed_ics.y();
    };

    for (int i = 0; i < 4; i++) {
      compute_reproj_error_point(corners_ccs[i], observed_corners_[i], residuals + 2 * i + 0);
      compute_reproj_error_point(corners_ccs[i], observed_corners_[i], residuals + 2 * i + 1);
    }

    return true;
  }

  template <typename T>
  bool operator()(const T * const tag_pose, T * residuals) const
  {
    assert(fixed_pose_type_ != NoFixedPose);

    (*this)(static_cast<T *>(nullptr), tag_pose, residuals);

    return true;
  }

  static ceres::CostFunction * createResidual(
    const UID & camera_uid, const IntrinsicParameters & intrinsics,
    const ApriltagDetection & detection)
  {
    auto f = new FixedIntrinsicsTagReprojectionError(camera_uid, intrinsics, detection);
    return (new ceres::AutoDiffCostFunction<
            FixedIntrinsicsTagReprojectionError,
            8,   // 4 corners x 2 residuals
            7,   // 7 camera pose parameters
            7>(  // 7 tag pose parameters
      f));
  }

  static ceres::CostFunction * createResidualWithFixedCameraPose(
    const UID & camera_uid, const IntrinsicParameters & intrinsics,
    const ApriltagDetection & detection, const std::array<double, 10> & camera_pose_inv)
  {
    auto f = new FixedIntrinsicsTagReprojectionError(
      camera_uid, intrinsics, detection, camera_pose_inv, FixedCameraPose);
    return (new ceres::AutoDiffCostFunction<
            FixedIntrinsicsTagReprojectionError,
            8,   // 4 corners x 2 residuals
            7>(  // 7 tag pose parameters
      f));
  }

  double cx_;
  double cy_;
  double fx_;
  double fy_;
  double tag_size_;

  // double template_corners_[4][3] = {
  //   {-1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, -1.0, 0.0}, {-1.0, -1.0, 0.0}};
  Eigen::Vector2d observed_corners_[4];

  UID camera_uid_;
  IntrinsicParameters intrinsics_;
  ApriltagDetection detection_;
  Eigen::Vector4d fixed_camera_rotation_inv_;
  Eigen::Vector3d fixed_camera_translation_inv_;
  std::array<double, 10> fixed_tag_pose_;
  FixedPoseType fixed_pose_type_;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__FIXED_INTRINSICS_TAG_REPROJECTION_ERROR_HPP_
