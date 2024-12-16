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

#ifndef CERES_INTRINSIC_CAMERA_CALIBRATOR__DISTORTION_COEFFICIENTS_RESIDUAL_HPP_
#define CERES_INTRINSIC_CAMERA_CALIBRATOR__DISTORTION_COEFFICIENTS_RESIDUAL_HPP_

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>

struct DistortionCoefficientsResidual
{
  static constexpr int RESIDUAL_DIM = 8;

  DistortionCoefficientsResidual(
    int radial_distortion_coeffs, bool use_tangential_distortion, int rational_distortion_coeffs)
  {
    radial_distortion_coeffs_ = radial_distortion_coeffs;
    use_tangential_distortion_ = use_tangential_distortion;
    rational_distortion_coeffs_ = rational_distortion_coeffs;
  }

  /*!
   * The cost function representing the reprojection error
   * @param[in] camera_intrinsics The camera intrinsics
   * @param[in] residuals The residual error of projecting the tag into the camera
   * @returns success status
   */
  template <typename T>
  bool operator()(const T * const camera_intrinsics, T * residuals) const
  {
    const T null_value = T(0.0);
    int distortion_index = 4;
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

    residuals[0] = k1;
    residuals[1] = k2;
    residuals[2] = k3;
    residuals[3] = p1;
    residuals[4] = p2;
    residuals[5] = k4;
    residuals[6] = k5;
    residuals[7] = k6;

    return true;
  }

  /*!
   * Residual factory method
   * @param[in] radial_distortion_coeffs The number of radial distortion coefficients
   * @param[in] use_tangential_distortion Whether to use or not tangential distortion
   * @param[in] rational_distortion_coeffs The number of rational distortion coefficients
   * @returns the ceres residual
   */
  static ceres::CostFunction * createResidual(
    int radial_distortion_coeffs, bool use_tangential_distortion, int rational_distortion_coeffs)
  {
    auto f = new DistortionCoefficientsResidual(
      radial_distortion_coeffs, use_tangential_distortion, rational_distortion_coeffs);

    int distortion_coefficients = radial_distortion_coeffs +
                                  2 * static_cast<int>(use_tangential_distortion) +
                                  rational_distortion_coeffs;
    ceres::CostFunction * cost_function = nullptr;

    switch (distortion_coefficients) {
      case 0:
        cost_function =
          new ceres::AutoDiffCostFunction<DistortionCoefficientsResidual, RESIDUAL_DIM, 4>(f);
        break;
      case 1:
        cost_function =
          new ceres::AutoDiffCostFunction<DistortionCoefficientsResidual, RESIDUAL_DIM, 5>(f);
        break;
      case 2:
        cost_function =
          new ceres::AutoDiffCostFunction<DistortionCoefficientsResidual, RESIDUAL_DIM, 6>(f);
        break;
      case 3:
        cost_function =
          new ceres::AutoDiffCostFunction<DistortionCoefficientsResidual, RESIDUAL_DIM, 7>(f);
        break;
      case 4:
        cost_function =
          new ceres::AutoDiffCostFunction<DistortionCoefficientsResidual, RESIDUAL_DIM, 8>(f);
        break;
      case 5:
        cost_function =
          new ceres::AutoDiffCostFunction<DistortionCoefficientsResidual, RESIDUAL_DIM, 9>(f);
        break;
      case 6:
        cost_function =
          new ceres::AutoDiffCostFunction<DistortionCoefficientsResidual, RESIDUAL_DIM, 10>(f);
        break;
      case 7:
        cost_function =
          new ceres::AutoDiffCostFunction<DistortionCoefficientsResidual, RESIDUAL_DIM, 11>(f);
        break;
      case 8:
        cost_function =
          new ceres::AutoDiffCostFunction<DistortionCoefficientsResidual, RESIDUAL_DIM, 12>(f);
        break;
      default:
        throw std::runtime_error("Invalid number of distortion coefficients");
    }

    return cost_function;
  }

  int radial_distortion_coeffs_;
  bool use_tangential_distortion_;
  int rational_distortion_coeffs_;
};

#endif  // CERES_INTRINSIC_CAMERA_CALIBRATOR__DISTORTION_COEFFICIENTS_RESIDUAL_HPP_
