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
#include <cstddef>
#include <utility>
#include <vector>

template <typename T>
struct CameraPoint
{
  T x;
  T y;
  T depth;
  T sign_shift_x;
  T sign_shift_y;
  T backprojection_err;  // cSpell:ignore backprojection
};

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
    int width, int height, const std::vector<CameraPoint<double>> & camera_points)
  {
    radial_distortion_coeffs_ = radial_distortion_coeffs;
    use_tangential_distortion_ = use_tangential_distortion;
    rational_distortion_coeffs_ = rational_distortion_coeffs;
    width_ = width;
    height_ = height;
    camera_points_ = camera_points;
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

    if (RESIDUAL_DIM != shifts.size()) {
      throw std::runtime_error("The number of residuals should match the number of shifts");
    }
    std::fill(residuals, residuals + RESIDUAL_DIM, T(0.0));
    for (const auto & cp : camera_points_) {
      for (std::size_t i = 0; i < shifts.size(); i++) {
        auto [u_shifted, v_shifted] = cameraToImage(
          cp.x + shifts[i] * cp.sign_shift_x, cp.y + shifts[i] * cp.sign_shift_y, cx, cy, fx, fy,
          k1, k2, k3, p1, p2, k4, k5, k6, depth);
        auto residual = getFovResidual(u_shifted, v_shifted);
        // Weigh the residuals by the backprojection error
        auto residual_weighted = residual / (cp.backprojection_err + T(1.0));
        // Increase residual magnitude & soften
        residual_weighted = ceres::log(T(1.0) + residual_weighted);
        residuals[i] += residual_weighted;
      }
    }

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
  static std::pair<T, T> cameraToImage(
    const T x, const T y, const T cx, const T cy, const T fx, const T fy, const T k1, const T k2,
    const T k3, const T p1, const T p2, const T k4, const T k5, const T k6, T depth = T(1.0))
  {
    const T xp = x / depth;
    const T yp = y / depth;
    const T r2 = xp * xp + yp * yp;
    const T dn = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const T dd = 1.0 + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2;
    const T d = dn / dd;
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
  static std::pair<T, T> imageToCamera(
    const T u, const T v, const T cx, const T cy, const T fx, const T fy, const T k1, const T k2,
    const T k3, const T p1, const T p2, const T k4, const T k5, const T k6, T depth = T(1.0),
    const T tol = T(1e-6))
  {
    T xp = (u - cx) / fx;
    T yp = (v - cy) / fy;

    for (int i = 0; i < UNDIST_ITERS; i++) {
      const T r2 = xp * xp + yp * yp;
      const T dn = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
      const T dd = 1.0 + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2;
      const T d = dn / dd;
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
    int width, int height, const std::vector<CameraPoint<double>> & camera_points)
  {
    auto f = new FOVResidual(
      radial_distortion_coeffs, use_tangential_distortion, rational_distortion_coeffs, width,
      height, camera_points);

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
  std::vector<CameraPoint<double>> camera_points_;
};

/*!
 * Get points around image border represented in camera frame
 * @param[in] camera_intrinsics The camera intrinsics
 * @param[in] radial_distortion_coeffs The number of radial distortion coefficients
 * @param[in] use_tangential_distortion Whether to use or not tangential distortion
 * @param[in] rational_distortion_coeffs The number of rational distortion coefficients
 * @param[in] width the source image width
 * @param[in] height the source image height
 * @returns The points in camera frame
 */
template <typename T>
std::vector<CameraPoint<T>> getCameraPoints(
  const T * const camera_intrinsics, int radial_distortion_coeffs, int use_tangential_distortion,
  int rational_distortion_coeffs, const T width, const T height)
{
  const T null_value = T(0.0);
  const T depth = T(1.0);

  int distortion_index = 0;
  const T & cx = camera_intrinsics[distortion_index++];
  const T & cy = camera_intrinsics[distortion_index++];
  const T & fx = camera_intrinsics[distortion_index++];
  const T & fy = camera_intrinsics[distortion_index++];
  const T & k1 = radial_distortion_coeffs > 0 ? camera_intrinsics[distortion_index++] : null_value;
  const T & k2 = radial_distortion_coeffs > 1 ? camera_intrinsics[distortion_index++] : null_value;
  const T & k3 = radial_distortion_coeffs > 2 ? camera_intrinsics[distortion_index++] : null_value;
  const T & p1 = use_tangential_distortion ? camera_intrinsics[distortion_index++] : null_value;
  const T & p2 = use_tangential_distortion ? camera_intrinsics[distortion_index++] : null_value;
  const T & k4 =
    rational_distortion_coeffs > 0 ? camera_intrinsics[distortion_index++] : null_value;
  const T & k5 =
    rational_distortion_coeffs > 1 ? camera_intrinsics[distortion_index++] : null_value;
  const T & k6 =
    rational_distortion_coeffs > 2 ? camera_intrinsics[distortion_index++] : null_value;

  auto getPoints = [width, height, cx, cy, fx, fy, k1, k2, k3, p1, p2, k4, k5, k6, depth](
                     const T u, const T v, std::vector<CameraPoint<T>> & camera_points,
                     const T backprojection_err_thr = T(10.0)) -> bool {
    auto [x, y] =
      FOVResidual::imageToCamera<T>(u, v, cx, cy, fx, fy, k1, k2, k3, p1, p2, k4, k5, k6, depth);
    auto [u_bpr, v_bpr] =
      FOVResidual::cameraToImage<T>(x, y, cx, cy, fx, fy, k1, k2, k3, p1, p2, k4, k5, k6, depth);
    auto backprojection_err = ceres::sqrt(ceres::pow(u - u_bpr, 2) + ceres::pow(v - v_bpr, 2));
    if (ceres::IsNaN(backprojection_err) || backprojection_err > backprojection_err_thr) {
      return false;
    }
    auto sign_shift_x = u <= T(0.0) ? T(-1.0) : u >= width - T(1.0) ? T(1.0) : T(0.0);
    auto sign_shift_y = v <= T(0.0) ? T(-1.0) : v >= height - T(1.0) ? T(1.0) : T(0.0);
    camera_points.push_back({x, y, depth, sign_shift_x, sign_shift_y, backprojection_err});
    return true;
  };

  std::vector<CameraPoint<T>> camera_points;
  std::size_t valid_points = 0;

  // Middle top
  valid_points += getPoints(width / T(2.0) - T(1.0), T(0.0), camera_points);

  // Middle left
  valid_points += getPoints(T(0.0), height / T(2.0) - T(1.0), camera_points);

  // Middle bottom
  valid_points += getPoints(width / T(2.0) - T(1.0), height - T(1.0), camera_points);

  // Middle right
  valid_points += getPoints(width - T(1.0), height / T(2.0) - T(1.0), camera_points);

  // Top left
  valid_points += getPoints(T(0.0), T(0.0), camera_points);

  // Top right
  valid_points += getPoints(width - T(1.0), T(0.0), camera_points);

  // Bottom left
  valid_points += getPoints(T(0.0), height - T(1.0), camera_points);

  // Bottom right
  valid_points += getPoints(width - T(1.0), height - T(1.0), camera_points);

  LOG(INFO) << "Valid camera points: " << valid_points << " / 8" << std::endl;

  return camera_points;
}

#endif  // CERES_INTRINSIC_CAMERA_CALIBRATOR__FOV_RESIDUAL_HPP_
