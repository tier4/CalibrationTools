// Copyright 2025 Tier IV, Inc.
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

#ifndef CERES_INTRINSIC_CAMERA_CALIBRATOR__FOV_RESIDUAL_HPP_
#define CERES_INTRINSIC_CAMERA_CALIBRATOR__FOV_RESIDUAL_HPP_

#include <Eigen/Core>

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>

#include <algorithm>
#include <utility>
#include <vector>

struct FOVResidual
{
  static constexpr int INTRINSICS_CX_INDEX = 0;
  static constexpr int INTRINSICS_CY_INDEX = 1;
  static constexpr int INTRINSICS_FX_INDEX = 2;
  static constexpr int INTRINSICS_FY_INDEX = 3;

  static constexpr int RESIDUAL_DIM = 8;

  static constexpr int UNDIST_ITERS = 100;  // cSpell:ignore UNDIST

  FOVResidual(
    int radial_distortion_coeffs, bool use_tangential_distortion, int rational_distortion_coeffs,
    int width, int height)
  {
    radial_distortion_coeffs_ = radial_distortion_coeffs;
    use_tangential_distortion_ = use_tangential_distortion;
    rational_distortion_coeffs_ = rational_distortion_coeffs;
    width_ = width;
    height_ = height;
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
    const T depth = T(1.0);
    const std::vector<T> shifts = {T(0.01), T(0.03), T(0.05), T(0.1),
                                   T(0.3),  T(0.5),  T(1.0),  T(3.0)};

    const T width_t = T(width_);
    const T height_t = T(height_);
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

    // cSpell:ignore backprojection
    auto apply_residual = [this, residuals, shifts, width_t, height_t, cx, cy, fx, fy, k1, k2, k3,
                           p1, p2, k4, k5, k6, depth](
                            const int & idx, const T & u, const T & v,
                            const T & backprojection_err_thr = T(10.0)) -> void {
      residuals[idx] = T(0.0);
      auto [x, y] = imageToCamera(u, v, cx, cy, fx, fy, k1, k2, k3, p1, p2, k4, k5, k6, depth);
      auto [u_bpr, v_bpr] =
        cameraToImage(x, y, cx, cy, fx, fy, k1, k2, k3, p1, p2, k4, k5, k6, depth);
      auto backprojection_err = ceres::sqrt(ceres::pow(u - u_bpr, 2) + ceres::pow(v - v_bpr, 2));
      if (ceres::IsNaN(backprojection_err) || backprojection_err > backprojection_err_thr) {
        return;
      }
      auto sign_shift_x = u <= T(0.0) ? T(-1.0) : u >= width_t - T(1.0) ? T(1.0) : T(0.0);
      auto sign_shift_y = v <= T(0.0) ? T(-1.0) : v >= height_t - T(1.0) ? T(1.0) : T(0.0);
      for (const auto & shift : shifts) {
        auto [u_shifted, v_shifted] = cameraToImage(
          x + shift * sign_shift_x, y + shift * sign_shift_y, cx, cy, fx, fy, k1, k2, k3, p1, p2,
          k4, k5, k6, depth);
        auto residual = getFovResidual(u_shifted, v_shifted);
        // Weigh the residuals by the backprojection error
        residuals[idx] += residual * (T(1.0) / (backprojection_err + T(1.0)));
      }
    };

    // Middle top
    apply_residual(0, width_t / T(2.0) - T(1.0), T(0.0));

    // Middle left
    apply_residual(1, T(0.0), height_t / T(2.0) - T(1.0));

    // Middle bottom
    apply_residual(2, width_t / T(2.0) - T(1.0), height_t - T(1.0));

    // Middle right
    apply_residual(3, width_t - T(1.0), height_t / T(2.0) - T(1.0));

    // Top left
    apply_residual(4, T(0.0), T(0.0));

    // Top right
    apply_residual(5, width_t - T(1.0), T(0.0));

    // Bottom left
    apply_residual(6, T(0.0), height_t - T(1.0));

    // Bottom right
    apply_residual(7, width_t - T(1.0), height_t - T(1.0));

    return true;
  }

  /*!
   * Calculates FOV residual (closest border) for given pixel in image coordinate system
   * @param[in] u The pixel x coordinate
   * @param[in] v The pixel y coordinate
   * @returns The residual
   */
  template <typename T>
  T getFovResidual(const T u, const T v) const
  {
    T width_t = T(width_);
    T height_t = T(height_);

    if (u >= T(0.0) && u <= width_t - T(1.0) && v >= T(0.0) && v <= height_t - T(1.0)) {
      T closest_u = std::min(u, width_t - u - T(1.0)) / (std::max(height_t, width_t) - T(1.0));
      T closest_v = std::min(v, height_t - v - T(1.0)) / (std::max(height_t, width_t) - T(1.0));
      return std::min(closest_u, closest_v);
    }
    return T(0.0);
  }

  /*!
   * Converts a real world coordinate to a pixel (distorted) coordinate
   * @param[in] x The x coordinate in camera frame
   * @param[in] y The y coordinate in camera frame
   * @param[in] cx The camera center x coordinate
   * @param[in] cy The camera center y coordinate
   * @param[in] fx The focal length x
   * @param[in] fy The focal length y
   * @param[in] k1 The radial distortion coefficient k1
   * @param[in] k2 The radial distortion coefficient k2
   * @param[in] k3 The radial distortion coefficient k3
   * @param[in] p1 The tangential distortion coefficient p1
   * @param[in] p2 The tangential distortion coefficient p2
   * @param[in] k4 The rational distortion coefficient k4
   * @param[in] k5 The rational distortion coefficient k5
   * @param[in] k6 The rational distortion coefficient k6
   * @param[in] depth The depth of the point
   * @returns The pixel x and y coordinates
   */
  template <typename T>
  std::pair<T, T> cameraToImage(
    const T x, const T y, const T cx, const T cy, const T fx, const T fy, const T k1, const T k2,
    const T k3, const T p1, const T p2, const T k4, const T k5, const T k6, T depth = T(1.0)) const
  {
    const T xp = x / depth;
    const T yp = y / depth;
    const T r2 = xp * xp + yp * yp;
    const T d = getRadialDist(xp, yp, k1, k2, k3, k4, k5, k6);
    const T xy = xp * yp;
    const T tdx = 2.0 * p1 * xy + p2 * (r2 + 2.0 * xp * xp);
    const T tdy = 2.0 * p2 * xy + p1 * (r2 + 2.0 * yp * yp);

    const T u = cx + fx * (xp * d + tdx);
    const T v = cy + fy * (yp * d + tdy);

    return std::make_pair(u, v);
  }

  /*!
   * Converts a pixel coordinate to a point (undistorted) in camera coordinates system
   * @param[in] u The x coordinate of pixel in image coordinate system
   * @param[in] v The y coordinate of pixel in image coordinate system
   * @param[in] cx The camera center x coordinate
   * @param[in] cy The camera center y coordinate
   * @param[in] fx The focal length x
   * @param[in] fy The focal length y
   * @param[in] k1 The radial distortion coefficient k1
   * @param[in] k2 The radial distortion coefficient k2
   * @param[in] k3 The radial distortion coefficient k3
   * @param[in] p1 The tangential distortion coefficient p1
   * @param[in] p2 The tangential distortion coefficient p2
   * @param[in] k4 The rational distortion coefficient k4
   * @param[in] k5 The rational distortion coefficient k5
   * @param[in] k6 The rational distortion coefficient k6
   * @param[in] depth The depth of the point
   * @returns The real world x and y coordinates in camera frame
   */
  template <typename T>
  std::pair<T, T> imageToCamera(
    const T u, const T v, const T cx, const T cy, const T fx, const T fy, const T k1, const T k2,
    const T k3, const T p1, const T p2, const T k4, const T k5, const T k6, T depth = T(1.0),
    const T tol = T(1e-6)) const
  {
    T xp = (u - cx) / fx;
    T yp = (v - cy) / fy;

    for (int i = 0; i < UNDIST_ITERS; i++) {
      const T r2 = xp * xp + yp * yp;
      const T d = getRadialDist(xp, yp, k1, k2, k3, k4, k5, k6);
      const T xy = xp * yp;
      const T tdx = 2.0 * p1 * xy + p2 * (r2 + 2.0 * xp * xp);
      const T tdy = 2.0 * p2 * xy + p1 * (r2 + 2.0 * yp * yp);

      const T predicted_x = xp * d + tdx;
      const T predicted_y = yp * d + tdy;

      const T delta_x = (u - cx) / fx - predicted_x;
      const T delta_y = (v - cy) / fy - predicted_y;

      if (delta_x * delta_x + delta_y * delta_y < tol) {
        break;
      }

      xp += delta_x;
      yp += delta_y;
    }

    const T x = xp * depth;
    const T y = yp * depth;

    return std::make_pair(x, y);
  }

  // cSpell:ignore Maclaurin
  /*!
   * Calculates radial distortion
   *
   * Approximation is applied if any rational distortion coefficient is negative. This approximation
   * follows Taylor series expansion of a function around x0=0, also known as a Maclaurin series.
   * In given context, only constant term of P''(x0) is not equal to zero, which simplifies the
   * polynomial to a 2nd degree polynomial.
   *
   * @param[in] x Normalized input coordinate along x-axis
   * @param[in] y Normalized input coordinate along y-axis
   * @param[in] k1 The radial distortion coefficient k1
   * @param[in] k2 The radial distortion coefficient k2
   * @param[in] k3 The radial distortion coefficient k3
   * @param[in] k4 The rational distortion coefficient k4
   * @param[in] k5 The rational distortion coefficient k5
   * @param[in] k6 The rational distortion coefficient k6
   * @returns The 3rd degree polynomial coefficients
   */
  template <typename T>
  T getRadialDist(
    const T x, const T y, const T k1, const T k2, const T k3, const T k4, const T k5,
    const T k6) const
  {
    const T r2 = x * x + y * y;

    // Use approximation if any rational distortion coefficient is negative
    if (k4 < T(0.0) || k5 < T(0.0) || k6 < T(0.0)) {
      const T dn = 1.0 + k1 * r2;
      const T dd = 1.0 + k4 * r2;
      return dn / dd;
    }

    const T dn = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const T dd = 1.0 + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2;
    return dn / dd;
  }

  /*!
   * Residual factory method
   * @param[in] object_point The object point
   * @param[in] image_point The image point
   * @param[in] radial_distortion_coeffs The number of radial distortion coefficients
   * @param[in] use_tangential_distortion Whether to use or not tangential distortion
   * @param[in] width the source image width
   * @param[in] height the source image height
   * @returns the ceres residual
   */
  static ceres::CostFunction * createResidual(
    int radial_distortion_coeffs, bool use_tangential_distortion, int rational_distortion_coeffs,
    int width, int height)
  {
    auto f = new FOVResidual(
      radial_distortion_coeffs, use_tangential_distortion, rational_distortion_coeffs, width,
      height);

    int distortion_coefficients = radial_distortion_coeffs +
                                  2 * static_cast<int>(use_tangential_distortion) +
                                  rational_distortion_coeffs;
    ceres::CostFunction * cost_function = nullptr;

    switch (distortion_coefficients) {
      case 0:
        cost_function = new ceres::AutoDiffCostFunction<FOVResidual, RESIDUAL_DIM, 4>(f);
        break;
      case 1:
        cost_function = new ceres::AutoDiffCostFunction<FOVResidual, RESIDUAL_DIM, 5>(f);
        break;
      case 2:
        cost_function = new ceres::AutoDiffCostFunction<FOVResidual, RESIDUAL_DIM, 6>(f);
        break;
      case 3:
        cost_function = new ceres::AutoDiffCostFunction<FOVResidual, RESIDUAL_DIM, 7>(f);
        break;
      case 4:
        cost_function = new ceres::AutoDiffCostFunction<FOVResidual, RESIDUAL_DIM, 8>(f);
        break;
      case 5:
        cost_function = new ceres::AutoDiffCostFunction<FOVResidual, RESIDUAL_DIM, 9>(f);
        break;
      case 6:
        cost_function = new ceres::AutoDiffCostFunction<FOVResidual, RESIDUAL_DIM, 10>(f);
        break;
      case 7:
        cost_function = new ceres::AutoDiffCostFunction<FOVResidual, RESIDUAL_DIM, 11>(f);
        break;
      case 8:
        cost_function = new ceres::AutoDiffCostFunction<FOVResidual, RESIDUAL_DIM, 12>(f);
        break;
      default:
        throw std::runtime_error("Invalid number of distortion coefficients");
    }

    return cost_function;
  }

  int radial_distortion_coeffs_;
  bool use_tangential_distortion_;
  int rational_distortion_coeffs_;
  int width_;
  int height_;
};

#endif  // CERES_INTRINSIC_CAMERA_CALIBRATOR__FOV_RESIDUAL_HPP_
