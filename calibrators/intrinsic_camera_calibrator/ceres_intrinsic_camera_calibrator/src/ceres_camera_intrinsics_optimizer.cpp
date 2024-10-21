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

#include <Eigen/Core>
#include <ceres_intrinsic_camera_calibrator/ceres_camera_intrinsics_optimizer.hpp>
#include <ceres_intrinsic_camera_calibrator/reprojection_residual.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <numeric>
#include <vector>

void CeresCameraIntrinsicsOptimizer::setRadialDistortionCoefficients(
  int radial_distortion_coefficients)
{
  radial_distortion_coefficients_ = radial_distortion_coefficients;
}

void CeresCameraIntrinsicsOptimizer::setTangentialDistortion(bool use_tangential_distortion)
{
  use_tangential_distortion_ = use_tangential_distortion;
}

void CeresCameraIntrinsicsOptimizer::setVerbose(bool verbose) { verbose_ = verbose; }

void CeresCameraIntrinsicsOptimizer::setData(
  const cv::Mat_<double> & camera_matrix, const cv::Mat_<double> & distortion_coeffs,
  const std::vector<std::vector<cv::Point3f>> & object_points,
  const std::vector<std::vector<cv::Point2f>> & image_points, const std::vector<cv::Mat> & rvecs,
  const std::vector<cv::Mat> & tvecs)
{
  camera_matrix_ = camera_matrix.clone();
  distortion_coeffs_ = distortion_coeffs.clone();
  object_points_ = object_points;
  image_points_ = image_points;
  rvecs_ = rvecs;
  tvecs_ = tvecs;
}

double CeresCameraIntrinsicsOptimizer::getSolution(
  cv::Mat_<double> & camera_matrix, cv::Mat_<double> & distortion_coeffs,
  std::vector<cv::Mat> & rvecs, std::vector<cv::Mat> & tvecs)
{
  camera_matrix = camera_matrix_;
  distortion_coeffs = distortion_coeffs_;
  rvecs = rvecs_;
  tvecs = tvecs_;

  double total_error = 0.0;
  std::size_t total_points = 0;

  auto get_reprojection_error = [](auto & image_points, auto & projected_points) -> double {
    cv::Mat x = cv::Mat(2 * image_points.size(), 1, CV_32F, image_points.data());
    cv::Mat y = cv::Mat(2 * projected_points.size(), 1, CV_32F, projected_points.data());
    double total_error = cv::norm(x - y, cv::NORM_L2SQR);
    return total_error;
  };

  for (std::size_t i = 0; i < object_points_.size(); i++) {
    std::vector<cv::Point2f> projected_points;
    total_points += object_points_[i].size();

    cv::projectPoints(
      object_points_[i], rvecs[i], tvecs[i], camera_matrix, distortion_coeffs, projected_points);

    total_error += get_reprojection_error(projected_points, image_points_[i]);
  }

  double rms_error = std::sqrt(total_error / total_points);
  return rms_error;
}

void CeresCameraIntrinsicsOptimizer::dataToPlaceholders()
{
  // Convert the intrinsics
  intrinsics_placeholder_[INTRINSICS_CX_INDEX] = camera_matrix_(0, 2);
  intrinsics_placeholder_[INTRINSICS_CY_INDEX] = camera_matrix_(1, 2);
  intrinsics_placeholder_[INTRINSICS_FX_INDEX] = camera_matrix_(0, 0);
  intrinsics_placeholder_[INTRINSICS_FY_INDEX] = camera_matrix_(1, 1);

  double k1 = distortion_coeffs_(0);
  double k2 = distortion_coeffs_(1);
  double p1 = distortion_coeffs_(2);
  double p2 = distortion_coeffs_(3);
  double k3 = distortion_coeffs_(4);

  int index = 4;

  if (radial_distortion_coefficients_ > 0) {
    intrinsics_placeholder_[index++] = k1;
  }
  if (radial_distortion_coefficients_ > 1) {
    intrinsics_placeholder_[index++] = k2;
  }
  if (radial_distortion_coefficients_ > 2) {
    intrinsics_placeholder_[index++] = k3;
  }
  if (use_tangential_distortion_) {
    intrinsics_placeholder_[index++] = p1;
    intrinsics_placeholder_[index++] = p2;
  }

  // Convert the revcs, tvecs into the placeholders
  pose_placeholders_.resize(object_points_.size());

  for (std::size_t i = 0; i < object_points_.size(); i++) {
    cv::Mat rmat;
    cv::Rodrigues(rvecs_[i], rmat);

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    cv::cv2eigen(tvecs_[i], translation);
    cv::cv2eigen(rmat, rotation);
    Eigen::Quaterniond quat(rotation);

    std::array<double, POSE_OPT_DIM> & placeholder = pose_placeholders_[i];
    placeholder[ROTATION_W_INDEX] = quat.w();
    placeholder[ROTATION_X_INDEX] = quat.x();
    placeholder[ROTATION_Y_INDEX] = quat.y();
    placeholder[ROTATION_Z_INDEX] = quat.z();
    placeholder[TRANSLATION_X_INDEX] = translation.x();
    placeholder[TRANSLATION_Y_INDEX] = translation.y();
    placeholder[TRANSLATION_Z_INDEX] = translation.z();
  }
}

void CeresCameraIntrinsicsOptimizer::placeholdersToData()
{
  // Convert the intrinsics
  camera_matrix_(0, 2) = intrinsics_placeholder_[INTRINSICS_CX_INDEX];
  camera_matrix_(1, 2) = intrinsics_placeholder_[INTRINSICS_CY_INDEX];
  camera_matrix_(0, 0) = intrinsics_placeholder_[INTRINSICS_FX_INDEX];
  camera_matrix_(1, 1) = intrinsics_placeholder_[INTRINSICS_FY_INDEX];

  distortion_coeffs_ = cv::Mat_<double>::zeros(5, 1);
  double & k1 = distortion_coeffs_(0);
  double & k2 = distortion_coeffs_(1);
  double & p1 = distortion_coeffs_(2);
  double & p2 = distortion_coeffs_(3);
  double & k3 = distortion_coeffs_(4);

  int index = 4;

  if (radial_distortion_coefficients_ > 0) {
    k1 = intrinsics_placeholder_[index++];
  }
  if (radial_distortion_coefficients_ > 1) {
    k2 = intrinsics_placeholder_[index++];
  }
  if (radial_distortion_coefficients_ > 2) {
    k3 = intrinsics_placeholder_[index++];
  }
  if (use_tangential_distortion_) {
    p1 = intrinsics_placeholder_[index++];
    p2 = intrinsics_placeholder_[index++];
  }

  // Convert the revcs, tvecs into the placeholders
  rvecs_.resize(pose_placeholders_.size());
  tvecs_.resize(pose_placeholders_.size());

  for (std::size_t i = 0; i < pose_placeholders_.size(); i++) {
    const auto & placeholder = pose_placeholders_[i];
    const double scale = 1.0 / std::sqrt(
                                 placeholder[0] * placeholder[0] + placeholder[1] * placeholder[1] +
                                 placeholder[2] * placeholder[2] + placeholder[3] * placeholder[3]);

    Eigen::Quaterniond quat = Eigen::Quaterniond(
      scale * placeholder[ROTATION_W_INDEX], scale * placeholder[ROTATION_X_INDEX],
      scale * placeholder[ROTATION_Y_INDEX], scale * placeholder[ROTATION_Z_INDEX]);

    Eigen::Vector3d translation = Eigen::Vector3d(
      placeholder[TRANSLATION_X_INDEX], placeholder[TRANSLATION_Y_INDEX],
      placeholder[TRANSLATION_Z_INDEX]);

    Eigen::Matrix3d rotation = quat.toRotationMatrix();

    cv::Matx33d cv_rot;
    cv::Matx31d cv_transl;
    cv::eigen2cv(translation, cv_transl);
    cv::eigen2cv(rotation, cv_rot);

    cv::Mat rvec, tvec;
    tvec = cv::Mat(cv_transl);

    cv::Rodrigues(cv_rot, rvec);

    rvecs_[i] = rvec;
    tvecs_[i] = tvec;
  }
}

void CeresCameraIntrinsicsOptimizer::evaluate()
{
  // Start developing the ceres optimizer
  double total_calibration_error = 0.0;
  auto get_reprojection_error =
    [](const auto & image_points, const auto & projected_points) -> double {
    double total_error = 0;
    for (size_t i = 0; i < projected_points.size(); i++) {
      double error = cv::norm(image_points[i] - projected_points[i]);
      total_error += error * error;
    }
    return std::sqrt(total_error / projected_points.size());
  };

  for (std::size_t i = 0; i < object_points_.size(); i++) {
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(
      object_points_[i], rvecs_[i], tvecs_[i], camera_matrix_, distortion_coeffs_,
      projected_points);

    double calibration_error = get_reprojection_error(projected_points, image_points_[i]);

    total_calibration_error += calibration_error;
  }

  if (verbose_) {
    printf("summary | calibration_error=%.3f\n", total_calibration_error / object_points_.size());
  }

  double total_ceres_error = 0;

  for (std::size_t view_index = 0; view_index < object_points_.size(); view_index++) {
    const auto & view_object_points = object_points_[view_index];
    const auto & view_image_points = image_points_[view_index];
    auto & pose_placeholder = pose_placeholders_[view_index];

    for (std::size_t point_index = 0; point_index < view_object_points.size(); point_index++) {
      auto f = ReprojectionResidual(
        view_object_points[point_index], view_image_points[point_index],
        radial_distortion_coefficients_, use_tangential_distortion_);
      std::array<double, 2> residuals;
      f(intrinsics_placeholder_.data(), pose_placeholder.data(), residuals.data());
      total_ceres_error += residuals[0] * residuals[0] + residuals[1] * residuals[1];
    }
  }

  if (verbose_) {
    std::cout << "total_ceres_error: " << 0.5 * total_ceres_error << std::endl;
  }
}

void CeresCameraIntrinsicsOptimizer::solve()
{
  ceres::Problem problem;

  for (std::size_t view_index = 0; view_index < object_points_.size(); view_index++) {
    const auto & view_object_points = object_points_[view_index];
    const auto & view_image_points = image_points_[view_index];
    auto & pose_placeholder = pose_placeholders_[view_index];

    for (std::size_t point_index = 0; point_index < view_object_points.size(); point_index++) {
      problem.AddResidualBlock(
        ReprojectionResidual::createResidual(
          view_object_points[point_index], view_image_points[point_index],
          radial_distortion_coefficients_, use_tangential_distortion_),
        nullptr,  // L2
        intrinsics_placeholder_.data(), pose_placeholder.data());
    }
  }

  double initial_cost = 0.0;
  std::vector<double> residuals;
  ceres::Problem::EvaluateOptions eval_opt;
  eval_opt.num_threads = 1;
  problem.GetResidualBlocks(&eval_opt.residual_blocks);
  problem.Evaluate(eval_opt, &initial_cost, &residuals, nullptr, nullptr);

  if (verbose_) {
    std::cout << "Initial cost: " << initial_cost;
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = verbose_;
  options.max_num_iterations = 500;
  options.function_tolerance = 1e-10;
  options.gradient_tolerance = 1e-14;
  options.num_threads = 8;
  options.use_inner_iterations = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (verbose_) {
    std::cout << "Report: " << summary.FullReport();
  }
}
