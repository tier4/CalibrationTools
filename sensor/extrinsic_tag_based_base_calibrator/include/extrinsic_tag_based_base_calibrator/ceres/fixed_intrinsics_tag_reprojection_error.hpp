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

#include <extrinsic_tag_based_base_calibrator/residuals/ceres_functions.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <opencv2/core.hpp>

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace extrinsic_tag_based_base_calibrator
{

struct FixedIntrinsicsTagReprojectionError
{
  FixedIntrinsicsTagReprojectionError(
    const IntrinsicParameters & intrinsics, const ApriltagDetection & detection)
  {
    fx_ = intrinsics.undistorted_camera_matrix(0, 0);
    fy_ = intrinsics.undistorted_camera_matrix(1, 1);
    cx_ = intrinsics.undistorted_camera_matrix(0, 2);
    cy_ = intrinsics.undistorted_camera_matrix(1, 0);

    tag_size_ = detection.size;

    for (int j = 0; j < 4; ++j) {
      template_corners_[j][0] *= detection.size;
      template_corners_[j][1] *= detection.size;
    }

    for (int j = 0; j < 4; ++j) {
      observed_corners_[j][0] = static_cast<double>(detection.corners[j].x);
      observed_corners_[j][1] = static_cast<double>(detection.corners[j].y);
    }
  }

  template <typename T>
  bool operator()(const T * const camera_pose_inv, const T * const tag_pose, T * residuals) const
  {
    T corner1_wcs[3];
    T corner2_wcs[3];
    T corner3_wcs[3];
    T corner4_wcs[3];

    QuaternionRotatePointDouble(tag_pose, template_corners_[0], corner1_wcs);
    corner1_wcs[0] += tag_pose[4];
    corner1_wcs[1] += tag_pose[5];
    corner1_wcs[2] += tag_pose[6];

    QuaternionRotatePointDouble(tag_pose, template_corners_[1], corner2_wcs);
    corner2_wcs[0] += tag_pose[4];
    corner2_wcs[1] += tag_pose[5];
    corner2_wcs[2] += tag_pose[6];

    QuaternionRotatePointDouble(tag_pose, template_corners_[2], corner3_wcs);
    corner3_wcs[0] += tag_pose[4];
    corner3_wcs[1] += tag_pose[5];
    corner3_wcs[2] += tag_pose[6];

    QuaternionRotatePointDouble(tag_pose, template_corners_[3], corner4_wcs);
    corner4_wcs[0] += tag_pose[4];
    corner4_wcs[1] += tag_pose[5];
    corner4_wcs[2] += tag_pose[6];

    T corner1_ccs[3];
    T corner2_ccs[3];
    T corner3_ccs[3];
    T corner4_ccs[3];

    ceres::QuaternionRotatePoint(camera_pose_inv, corner1_wcs, corner1_ccs);
    corner1_ccs[0] += tag_pose[4];
    corner1_ccs[1] += tag_pose[5];
    corner1_ccs[2] += tag_pose[6];

    ceres::QuaternionRotatePoint(camera_pose_inv, corner2_wcs, corner2_ccs);
    corner2_ccs[0] += tag_pose[4];
    corner2_ccs[1] += tag_pose[5];
    corner2_ccs[2] += tag_pose[6];

    ceres::QuaternionRotatePoint(camera_pose_inv, corner3_wcs, corner3_ccs);
    corner3_ccs[0] += tag_pose[4];
    corner3_ccs[1] += tag_pose[5];
    corner3_ccs[2] += tag_pose[6];

    ceres::QuaternionRotatePoint(camera_pose_inv, corner4_wcs, corner4_ccs);
    corner4_ccs[0] += tag_pose[4];
    corner4_ccs[1] += tag_pose[5];
    corner4_ccs[2] += tag_pose[6];

    const T predicted_corner1_x = cx_ + fx_ * (corner1_ccs[0] / corner1_ccs[2]);
    const T predicted_corner1_y = cy_ + fy_ * (corner1_ccs[1] / corner1_ccs[2]);

    const T predicted_corner2_x = cx_ + fx_ * (corner2_ccs[0] / corner2_ccs[2]);
    const T predicted_corner2_y = cy_ + fy_ * (corner2_ccs[1] / corner2_ccs[2]);

    const T predicted_corner3_x = cx_ + fx_ * (corner3_ccs[0] / corner3_ccs[2]);
    const T predicted_corner3_y = cy_ + fy_ * (corner3_ccs[1] / corner3_ccs[2]);

    const T predicted_corner4_x = cx_ + fx_ * (corner4_ccs[0] / corner4_ccs[2]);
    const T predicted_corner4_y = cy_ + fy_ * (corner4_ccs[1] / corner4_ccs[2]);

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

  static ceres::CostFunction * Create(
    const IntrinsicParameters & intrinsics, const ApriltagDetection & detection)
  {
    return (new ceres::AutoDiffCostFunction<
            FixedIntrinsicsTagReprojectionError,
            8,   // 4 corners x 2 residuals
            7,   // 7 camera pose parameters
            7>(  // 7 tag pose parameters
      new FixedIntrinsicsTagReprojectionError(intrinsics, detection)));
  }

  double cx_;
  double cy_;
  double fx_;
  double fy_;
  double tag_size_;

  double template_corners_[4][3] = {
    {-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}};
  double observed_corners_[4][2];
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__FIXED_INTRINSICS_TAG_REPROJECTION_ERROR_HPP_
