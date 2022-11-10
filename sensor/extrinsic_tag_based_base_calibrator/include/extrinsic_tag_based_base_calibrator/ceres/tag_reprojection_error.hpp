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

#include <algorithm>
#include <array>

namespace extrinsic_tag_based_base_calibrator
{

struct TagReprojectionError
{
  enum FixedPoseType {
    FixedCameraPose,  // the sensor camera pose uses optimization, but the pose itself is fixed
    FixedTagPose,     // the waypoints are fixed for base-lidar calibration
    NoFixedPose,
  };

  TagReprojectionError(
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
      template_corners_[j][0] *= 0.5 * detection.size;
      template_corners_[j][1] *= 0.5 * detection.size;
    }

    for (int j = 0; j < 4; ++j) {
      observed_corners_[j][0] = static_cast<double>(detection.corners[j].x);
      observed_corners_[j][1] = static_cast<double>(detection.corners[j].y);
    }
  }

  TagReprojectionError(
    const UID & camera_uid, const IntrinsicParameters & intrinsics,
    const ApriltagDetection & detection, std::array<double, 10> fixed_pose,
    FixedPoseType fixed_pose_type)
  : TagReprojectionError(camera_uid, intrinsics, detection)
  {
    fixed_pose_type_ = fixed_pose_type;
    if (fixed_pose_type == NoFixedPose) {
      throw std::invalid_argument("Invalid argument");
    } else if (fixed_pose_type == FixedCameraPose) {
      fixed_camera_pose_inv_ = fixed_pose;
    } else if (fixed_pose_type == FixedTagPose) {
      std::copy(fixed_pose.begin(), fixed_pose.begin() + 7, fixed_tag_pose_.begin());
      // fixed_tag_pose_ = fixed_pose;
    }
  }

  template <typename T>
  bool operator()(const T * const camera_pose_inv, const T * const tag_pose, T * residuals) const
  {
    T corner1_wcs[3];
    T corner2_wcs[3];
    T corner3_wcs[3];
    T corner4_wcs[3];

    if (fixed_pose_type_ == FixedTagPose) {
      assert(false);
    } else {
      QuaternionRotatePoint2(tag_pose, template_corners_[0], corner1_wcs);
      QuaternionRotatePoint2(tag_pose, template_corners_[1], corner2_wcs);
      QuaternionRotatePoint2(tag_pose, template_corners_[2], corner3_wcs);
      QuaternionRotatePoint2(tag_pose, template_corners_[3], corner4_wcs);
    }

    corner1_wcs[0] += tag_pose[4];
    corner1_wcs[1] += tag_pose[5];
    corner1_wcs[2] += tag_pose[6];

    corner2_wcs[0] += tag_pose[4];
    corner2_wcs[1] += tag_pose[5];
    corner2_wcs[2] += tag_pose[6];

    corner3_wcs[0] += tag_pose[4];
    corner3_wcs[1] += tag_pose[5];
    corner3_wcs[2] += tag_pose[6];

    corner4_wcs[0] += tag_pose[4];
    corner4_wcs[1] += tag_pose[5];
    corner4_wcs[2] += tag_pose[6];

    T corner1_ccs[3];
    T corner2_ccs[3];
    T corner3_ccs[3];
    T corner4_ccs[3];

    if (fixed_pose_type_ == FixedCameraPose) {
      auto fixed_camera_pose_inv = fixed_camera_pose_inv_.data();
      QuaternionRotatePoint2(fixed_camera_pose_inv, corner1_wcs, corner1_ccs);
      QuaternionRotatePoint2(fixed_camera_pose_inv, corner2_wcs, corner2_ccs);
      QuaternionRotatePoint2(fixed_camera_pose_inv, corner3_wcs, corner3_ccs);
      QuaternionRotatePoint2(fixed_camera_pose_inv, corner4_wcs, corner4_ccs);

      corner1_ccs[0] += fixed_camera_pose_inv[4];
      corner1_ccs[1] += fixed_camera_pose_inv[5];
      corner1_ccs[2] += fixed_camera_pose_inv[6];

      corner2_ccs[0] += fixed_camera_pose_inv[4];
      corner2_ccs[1] += fixed_camera_pose_inv[5];
      corner2_ccs[2] += fixed_camera_pose_inv[6];

      corner3_ccs[0] += fixed_camera_pose_inv[4];
      corner3_ccs[1] += fixed_camera_pose_inv[5];
      corner3_ccs[2] += fixed_camera_pose_inv[6];

      corner4_ccs[0] += fixed_camera_pose_inv[4];
      corner4_ccs[1] += fixed_camera_pose_inv[5];
      corner4_ccs[2] += fixed_camera_pose_inv[6];
    } else {
      QuaternionRotatePoint2(camera_pose_inv, corner1_wcs, corner1_ccs);
      QuaternionRotatePoint2(camera_pose_inv, corner2_wcs, corner2_ccs);
      QuaternionRotatePoint2(camera_pose_inv, corner3_wcs, corner3_ccs);
      QuaternionRotatePoint2(camera_pose_inv, corner4_wcs, corner4_ccs);

      corner1_ccs[0] += camera_pose_inv[4];
      corner1_ccs[1] += camera_pose_inv[5];
      corner1_ccs[2] += camera_pose_inv[6];

      corner2_ccs[0] += camera_pose_inv[4];
      corner2_ccs[1] += camera_pose_inv[5];
      corner2_ccs[2] += camera_pose_inv[6];

      corner3_ccs[0] += camera_pose_inv[4];
      corner3_ccs[1] += camera_pose_inv[5];
      corner3_ccs[2] += camera_pose_inv[6];

      corner4_ccs[0] += camera_pose_inv[4];
      corner4_ccs[1] += camera_pose_inv[5];
      corner4_ccs[2] += camera_pose_inv[6];
    }

    const T k1 = fixed_pose_type_ == FixedCameraPose ? T(1) : camera_pose_inv[7];
    const T k2 = fixed_pose_type_ == FixedCameraPose ? T(1) : camera_pose_inv[8];
    const T f = fixed_pose_type_ == FixedCameraPose ? T(1) : camera_pose_inv[9];

    T fx = f + fx_;
    T fy = f + fy_;

    const T corner1_xp = corner1_ccs[0] / corner1_ccs[2];
    const T corner1_yp = corner1_ccs[1] / corner1_ccs[2];
    const T corner1_r2 = corner1_xp * corner1_xp + corner1_yp * corner1_yp;
    const T corner1_distortion = 1.0 + corner1_r2 * (k1 + k2 * corner1_r2);
    const T predicted_corner1_x = cx_ + fx * corner1_distortion * corner1_xp;
    const T predicted_corner1_y = cy_ + fy * corner1_distortion * corner1_yp;

    const T corner2_xp = corner2_ccs[0] / corner2_ccs[2];
    const T corner2_yp = corner2_ccs[1] / corner2_ccs[2];
    const T corner2_r2 = corner2_xp * corner2_xp + corner2_yp * corner2_yp;
    const T corner2_distortion = 1.0 + corner2_r2 * (k1 + k2 * corner2_r2);
    const T predicted_corner2_x = cx_ + fx * corner2_distortion * corner2_xp;
    const T predicted_corner2_y = cy_ + fy * corner2_distortion * corner2_yp;

    const T corner3_xp = corner3_ccs[0] / corner3_ccs[2];
    const T corner3_yp = corner3_ccs[1] / corner3_ccs[2];
    const T corner3_r2 = corner3_xp * corner3_xp + corner3_yp * corner3_yp;
    const T corner3_distortion = 1.0 + corner3_r2 * (k1 + k2 * corner3_r2);
    const T predicted_corner3_x = cx_ + fx * corner3_distortion * corner3_xp;
    const T predicted_corner3_y = cy_ + fy * corner3_distortion * corner3_yp;

    const T corner4_xp = corner4_ccs[0] / corner4_ccs[2];
    const T corner4_yp = corner4_ccs[1] / corner4_ccs[2];
    const T corner4_r2 = corner4_xp * corner4_xp + corner4_yp * corner4_yp;
    const T corner4_distortion = 1.0 + corner4_r2 * (k1 + k2 * corner4_r2);
    const T predicted_corner4_x = cx_ + fx * corner4_distortion * corner4_xp;
    const T predicted_corner4_y = cy_ + fy * corner4_distortion * corner4_yp;

    residuals[0] = predicted_corner1_x - observed_corners_[0][0];
    residuals[1] = predicted_corner1_y - observed_corners_[0][1];

    residuals[2] = predicted_corner2_x - observed_corners_[1][0];
    residuals[3] = predicted_corner2_y - observed_corners_[1][1];

    residuals[4] = predicted_corner3_x - observed_corners_[2][0];
    residuals[5] = predicted_corner3_y - observed_corners_[2][1];

    residuals[6] = predicted_corner4_x - observed_corners_[3][0];
    residuals[7] = predicted_corner4_y - observed_corners_[3][1];

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
    auto f = new TagReprojectionError(camera_uid, intrinsics, detection);
    return (new ceres::AutoDiffCostFunction<
            TagReprojectionError,
            8,   // 4 corners x 2 residuals
            10,  // 10 camera pose parameters
            7>(  // 7 tag pose parameters
      f));
  }

  static ceres::CostFunction * createResidualWithFixedCameraPose(
    const UID & camera_uid, const IntrinsicParameters & intrinsics,
    const ApriltagDetection & detection, const std::array<double, 10> & camera_pose_inv)
  {
    auto f =
      new TagReprojectionError(camera_uid, intrinsics, detection, camera_pose_inv, FixedCameraPose);
    return (new ceres::AutoDiffCostFunction<
            TagReprojectionError,
            8,   // 4 corners x 2 residuals
            7>(  // 7 tag pose parameters
      f));
  }

  double cx_;
  double cy_;
  double fx_;
  double fy_;
  double tag_size_;

  double template_corners_[4][3] = {
    {-1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, -1.0, 0.0}, {-1.0, -1.0, 0.0}};
  double observed_corners_[4][2];

  UID camera_uid_;
  IntrinsicParameters intrinsics_;
  ApriltagDetection detection_;
  std::array<double, 10> fixed_camera_pose_inv_;
  std::array<double, 7> fixed_tag_pose_;
  FixedPoseType fixed_pose_type_;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__TAG_REPROJECTION_ERROR_HPP_
