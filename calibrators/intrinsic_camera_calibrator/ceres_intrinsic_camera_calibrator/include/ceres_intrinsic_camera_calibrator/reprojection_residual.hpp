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

#ifndef CERES_INTRINSIC_CAMERA_CALIBRATOR__REPROJECTION_RESIDUAL_HPP_
#define CERES_INTRINSIC_CAMERA_CALIBRATOR__REPROJECTION_RESIDUAL_HPP_

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct ReprojectionResidual
{
  template <class T>
  using Vector3 = Eigen::Matrix<T, 3, 1>;

  static constexpr int POSE_OPT_DIM = 7;

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

  static constexpr int RESIDUAL_DIM = 2;

  ReprojectionResidual(
    const cv::Point3f & object_point, const cv::Point2f & image_point, int radial_distortion_coeffs,
    bool use_tangential_distortion, int rational_distortion_coeffs)
  {
    object_point_ = Eigen::Vector3d(object_point.x, object_point.y, object_point.z);
    image_point_ = Eigen::Vector2d(image_point.x, image_point.y);
    radial_distortion_coeffs_ = radial_distortion_coeffs;
    use_tangential_distortion_ = use_tangential_distortion;
    rational_distortion_coeffs_ = rational_distortion_coeffs;
  }

  /*!
   * The cost function representing the reprojection error
   * @param[in] camera_intrinsics The camera intrinsics
   * @param[in] board_pose The pose from ground plane to the tag (only used when using ground tags)
   * @param[in] residuals The residual error of projecting the tag into the camera
   * @returns success status
   */
  template <typename T>
  bool operator()(
    const T * const camera_intrinsics, const T * const board_pose, T * residuals) const
  {
    const Eigen::Map<const Vector3<T>> board_translation(&board_pose[TRANSLATION_X_INDEX]);

    Eigen::Quaternion<T> board_quaternion = {
      board_pose[ROTATION_W_INDEX], board_pose[ROTATION_X_INDEX], board_pose[ROTATION_Y_INDEX],
      board_pose[ROTATION_Z_INDEX]};

    board_quaternion = board_quaternion.normalized();

    Vector3<T> object_point_ccs = board_quaternion * (T(1.0) * object_point_) + board_translation;

    const T null_value = T(0.0);
    int distortion_index = 4;
    const T & cx = camera_intrinsics[INTRINSICS_CX_INDEX];
    const T & cy = camera_intrinsics[INTRINSICS_CY_INDEX];
    const T & fx = camera_intrinsics[INTRINSICS_FX_INDEX];
    const T & fy = camera_intrinsics[INTRINSICS_FY_INDEX];
    const T & k1 =
      radial_distortion_coeffs_ > 0 ? camera_intrinsics[distortion_index++] : null_value;
    const T & k2 =
      radial_distortion_coeffs_ > 1 ? camera_intrinsics[distortion_index++] : null_value;
    const T & k3 =
      radial_distortion_coeffs_ > 2 ? camera_intrinsics[distortion_index++] : null_value;
    const T & p1 = use_tangential_distortion_ ? camera_intrinsics[distortion_index++] : null_value;
    const T & p2 = use_tangential_distortion_ ? camera_intrinsics[distortion_index++] : null_value;
    const T & k4 =
      rational_distortion_coeffs_ > 0 ? camera_intrinsics[distortion_index++] : null_value;
    const T & k5 =
      rational_distortion_coeffs_ > 1 ? camera_intrinsics[distortion_index++] : null_value;
    const T & k6 =
      rational_distortion_coeffs_ > 2 ? camera_intrinsics[distortion_index++] : null_value;

    const T xp = object_point_ccs.x() / object_point_ccs.z();
    const T yp = object_point_ccs.y() / object_point_ccs.z();
    const T r2 = xp * xp + yp * yp;
    const T dn = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const T dd = 1.0 + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2;
    const T d = dn / dd;
    const T xy = xp * yp;
    const T tdx = 2.0 * p1 * xy + p2 * (r2 + 2.0 * xp * xp);
    const T tdy = 2.0 * p2 * xy + p1 * (r2 + 2.0 * yp * yp);

    const T predicted_ics_x = cx + fx * (xp * d + tdx);
    const T predicted_ics_y = cy + fy * (yp * d + tdy);

    residuals[0] = predicted_ics_x - image_point_.x();
    residuals[1] = predicted_ics_y - image_point_.y();

    return true;
  }

  /*!
   * Residual factory method
   * @param[in] object_point The object point
   * @param[in] image_point The image point
   * @param[in] radial_distortion_coeffs The number of radial distortion coefficients
   * @param[in] use_tangential_distortion Whether to use or not tangential distortion
   * @returns the ceres residual
   */
  static ceres::CostFunction * createResidual(
    const cv::Point3f & object_point, const cv::Point2f & image_point, int radial_distortion_coeffs,
    bool use_tangential_distortion, int rational_distortion_coeffs)
  {
    auto f = new ReprojectionResidual(
      object_point, image_point, radial_distortion_coeffs, use_tangential_distortion,
      rational_distortion_coeffs);

    int distortion_coefficients = radial_distortion_coeffs +
                                  2 * static_cast<int>(use_tangential_distortion) +
                                  rational_distortion_coeffs;
    ceres::CostFunction * cost_function = nullptr;

    switch (distortion_coefficients) {
      case 0:
        cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionResidual, RESIDUAL_DIM, 4, POSE_OPT_DIM>(f);
        break;
      case 1:
        cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionResidual, RESIDUAL_DIM, 5, POSE_OPT_DIM>(f);
        break;
      case 2:
        cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionResidual, RESIDUAL_DIM, 6, POSE_OPT_DIM>(f);
        break;
      case 3:
        cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionResidual, RESIDUAL_DIM, 7, POSE_OPT_DIM>(f);
        break;
      case 4:
        cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionResidual, RESIDUAL_DIM, 8, POSE_OPT_DIM>(f);
        break;
      case 5:
        cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionResidual, RESIDUAL_DIM, 9, POSE_OPT_DIM>(f);
        break;
      case 6:
        cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionResidual, RESIDUAL_DIM, 10, POSE_OPT_DIM>(f);
        break;
      case 7:
        cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionResidual, RESIDUAL_DIM, 11, POSE_OPT_DIM>(f);
        break;
      case 8:
        cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionResidual, RESIDUAL_DIM, 12, POSE_OPT_DIM>(f);
        break;
      default:
        throw std::runtime_error("Invalid number of distortion coefficients");
    }

    return cost_function;
  }

  Eigen::Vector3d object_point_;
  Eigen::Vector2d image_point_;
  int radial_distortion_coeffs_;
  bool use_tangential_distortion_;
  int rational_distortion_coeffs_;
};

#endif  // CERES_INTRINSIC_CAMERA_CALIBRATOR__REPROJECTION_RESIDUAL_HPP_
