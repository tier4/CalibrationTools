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

#ifndef CERES_INTRINSIC_CAMERA_CALIBRATOR__CERES_CAMERA_INTRINSICS_OPTIMIZER_HPP_
#define CERES_INTRINSIC_CAMERA_CALIBRATOR__CERES_CAMERA_INTRINSICS_OPTIMIZER_HPP_

#include <Eigen/Dense>
#include <opencv2/core/affine.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

#include <array>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

class CeresCameraIntrinsicsOptimizer
{
public:
  static constexpr int POSE_OPT_DIM = 7;
  static constexpr int INTRINSICS_DIM = 12;

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

  /*!
   * Sets the number of radial distortion coefficients
   * @param[in] radial_distortion_coefficients number of radial distortion coefficients
   * optimized
   */
  void setRadialDistortionCoefficients(int radial_distortion_coefficients);

  /*!
   * Sets the use of tangential distortion
   * @param[in] use_tangential_distortion whether or not to use tangential distortion
   */
  void setTangentialDistortion(bool use_tangential_distortion);

  /*!
   * Sets the number of rational distortion coefficients
   * @param[in] rational_distortion_coefficients number of radial distortion coefficients
   * optimized
   */
  void setRationalDistortionCoefficients(int rational_distortion_coefficients);

  /*!
   * Sets the verbose mode
   * @param[in] verbose whether or not to use tangential distortion
   */
  void setVerbose(bool verbose);

  /*!
   * Sets the calibration data
   * @param[in] camera_matrix the initial 3x3 camera matrix
   * @param[in] distortion_coeffs the initial 5d distortion coefficients in the opencv formulation
   * @param[in] object_points the calibration object points
   * @param[in] image_points the calibration image points
   * @param[in] rvecs the calibration boards initial poses
   * @param[in] tvecs the calibration boards initial poses
   */
  void setData(
    const cv::Mat_<double> & camera_matrix, const cv::Mat_<double> & distortion_coeffs,
    const std::vector<std::vector<cv::Point3f>> & object_points,
    const std::vector<std::vector<cv::Point2f>> & image_points, const std::vector<cv::Mat> & rvecs,
    const std::vector<cv::Mat> & tvecs);

  /*!
   * Extract the solution from the optimization
   * @param[in] camera_matrix the optimized 3x3 camera matrix
   * @param[in] distortion_coeffs the optimized 5d distortion coefficients in the opencv formulation
   * @param[in] rvecs the calibration boards optimized poses
   * @param[in] tvecs the calibration boards optimized poses
   * @return the reprojection RMS error as in the opencv formulation
   */
  double getSolution(
    cv::Mat_<double> & camera_matrix, cv::Mat_<double> & distortion_coeffs,
    std::vector<cv::Mat> & rvecs, std::vector<cv::Mat> & tvecs);

  /*!
   * Formats the input data into optimization placeholders
   */
  void dataToPlaceholders();

  /*!
   * Extracts the information from the optimization placeholders and formats it into the calibration
   * data structure
   */
  void placeholdersToData();

  /*!
   * Evaluates the current optimization variables with the ceres cost function
   */
  void evaluate();

  /*!
   * Formulates and solves the optimization problem
   */
  void solve();

protected:
  int radial_distortion_coefficients_;
  bool use_tangential_distortion_;
  int rational_distortion_coefficients_;
  bool verbose_;
  cv::Mat_<double> camera_matrix_;
  cv::Mat_<double> distortion_coeffs_;

  std::vector<std::vector<cv::Point3f>> object_points_;
  std::vector<std::vector<cv::Point2f>> image_points_;
  std::vector<cv::Mat> rvecs_, tvecs_;

  // Optimization placeholders
  std::vector<std::array<double, POSE_OPT_DIM>> pose_placeholders_;
  std::array<double, INTRINSICS_DIM> intrinsics_placeholder_;
};

#endif  // CERES_INTRINSIC_CAMERA_CALIBRATOR__CERES_CAMERA_INTRINSICS_OPTIMIZER_HPP_
