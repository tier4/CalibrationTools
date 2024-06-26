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

#include <marker_radar_lidar_calibrator/marker_radar_lidar_calibrator.hpp>
#include <marker_radar_lidar_calibrator/sensor_residual.hpp>
#include <marker_radar_lidar_calibrator/transformation_estimator.hpp>

#include <numeric>

namespace marker_radar_lidar_calibrator
{

TransformationEstimator::TransformationEstimator(
  Eigen::Isometry3d initial_radar_to_lidar_eigen,
  Eigen::Isometry3d initial_radar_optimization_to_radar_eigen,
  Eigen::Isometry3d radar_optimization_to_lidar_eigen)
{
  initial_radar_to_lidar_eigen_ = initial_radar_to_lidar_eigen;
  initial_radar_optimization_to_radar_eigen_ = initial_radar_optimization_to_radar_eigen;
  radar_optimization_to_lidar_eigen_ = radar_optimization_to_lidar_eigen;
}

void TransformationEstimator::setPoints(
  pcl::PointCloud<PointType>::Ptr lidar_points_ocs,
  pcl::PointCloud<PointType>::Ptr radar_points_rcs)
{
  lidar_points_ocs_ = lidar_points_ocs;
  radar_points_rcs_ = radar_points_rcs;
}

void TransformationEstimator::setDelta(double delta_cos, double delta_sin)
{
  delta_cos_ = delta_cos;
  delta_sin_ = delta_sin;
}

void TransformationEstimator::estimateYawOnlyTransformation()
{
  RCLCPP_INFO(
    rclcpp::get_logger("marker_radar_lidar_calibrator"), "Estimate yaw only 2d transformation");
  Eigen::Matrix3d delta_rotation;
  delta_rotation << delta_cos_, -delta_sin_, 0.0, delta_sin_, delta_cos_, 0.0, 0.0, 0.0, 1.0;
  Eigen::Isometry3d delta_transformation = Eigen::Isometry3d::Identity();
  delta_transformation.linear() = delta_rotation;

  calibrated_radar_to_lidar_transformation_ = delta_transformation * initial_radar_to_lidar_eigen_;
}

void TransformationEstimator::estimateSVDTransformation(
  ExtrinsicReflectorBasedCalibrator::TransformationType transformation_type)
{
  if (transformation_type == ExtrinsicReflectorBasedCalibrator::TransformationType::svd_2d) {
    RCLCPP_INFO(
      rclcpp::get_logger("marker_radar_lidar_calibrator"), "Estimate 2D SVD transformation");
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("marker_radar_lidar_calibrator"), "Estimate 3D SVD transformation");
  }

  pcl::registration::TransformationEstimationSVD<PointType, PointType> estimator;
  Eigen::Matrix4f full_radar_to_radar_optimization_transformation;
  estimator.estimateRigidTransformation(
    *lidar_points_ocs_, *radar_points_rcs_, full_radar_to_radar_optimization_transformation);
  Eigen::Isometry3d calibrated_radar_to_radar_optimization_transformation(
    full_radar_to_radar_optimization_transformation.cast<double>());

  if (transformation_type == ExtrinsicReflectorBasedCalibrator::TransformationType::svd_2d) {
    // Check that is is actually a 2D transformation
    auto calibrated_radar_to_radar_optimization_rpy = tier4_autoware_utils::getRPY(
      tf2::toMsg(calibrated_radar_to_radar_optimization_transformation).orientation);
    double calibrated_radar_to_radar_optimization_z =
      calibrated_radar_to_radar_optimization_transformation.translation().z();
    double calibrated_radar_to_radar_optimization_roll =
      calibrated_radar_to_radar_optimization_rpy.x;
    double calibrated_radar_to_radar_optimization_pitch =
      calibrated_radar_to_radar_optimization_rpy.y;

    if (
      calibrated_radar_to_radar_optimization_z != 0.0 ||
      calibrated_radar_to_radar_optimization_roll != 0.0 ||
      calibrated_radar_to_radar_optimization_pitch != 0.0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("marker_radar_lidar_calibrator"),
        "The estimated 2D translation was not really 2D. Continue at your own risk. z=%.3f "
        "roll=%.3f "
        "pitch=%.3f",
        calibrated_radar_to_radar_optimization_z, calibrated_radar_to_radar_optimization_roll,
        calibrated_radar_to_radar_optimization_pitch);
    }

    calibrated_radar_to_radar_optimization_transformation.translation().z() =
      (initial_radar_to_lidar_eigen_ * radar_optimization_to_lidar_eigen_.inverse())
        .translation()
        .z();
  }

  calibrated_radar_to_lidar_transformation_ =
    calibrated_radar_to_radar_optimization_transformation * radar_optimization_to_lidar_eigen_;
}

void TransformationEstimator::estimateZeroRollTransformation()
{
  RCLCPP_INFO(
    rclcpp::get_logger("marker_radar_lidar_calibrator"),
    "Estimate 3D transformation with roll is always zero");

  ceres::Problem problem;

  Eigen::Vector3d translation = initial_radar_optimization_to_radar_eigen_.translation();
  Eigen::Matrix3d rotation = initial_radar_optimization_to_radar_eigen_.rotation();
  Eigen::Vector3d euler_angle = rotation.eulerAngles(0, 1, 2);

  // params: x, y, z, pitch, yaw
  std::vector<double> params = {
    translation[0], translation[1], translation[2], euler_angle[1], euler_angle[2]};

  std::string initial_params_str = std::accumulate(
    std::next(params.begin()), params.end(),
    std::to_string(params.front()),  // Initialize with the first element
    [](const std::string & a, const auto & b) { return a + " " + std::to_string(b); });
  std::string initial_params_msg = "initial params (x,y,z,pith,yaw): " + initial_params_str;
  RCLCPP_INFO(
    rclcpp::get_logger("marker_radar_lidar_calibrator"), "%s", initial_params_msg.c_str());

  for (size_t i = 0; i < lidar_points_ocs_->points.size(); i++) {
    auto lidar_point = lidar_points_ocs_->points[i];
    auto radar_point = radar_points_rcs_->points[i];

    Eigen::Vector4d radar_point_eigen(radar_point.x, radar_point.y, radar_point.z, 1);
    Eigen::Vector4d lidar_point_eigen(lidar_point.x, lidar_point.y, lidar_point.z, 1);

    ceres::CostFunction * cost_function = new ceres::AutoDiffCostFunction<SensorResidual, 3, 5>(
      new SensorResidual(radar_point_eigen, lidar_point_eigen));
    problem.AddResidualBlock(cost_function, NULL, params.data());
  }

  // Solve
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;  // cSpell:ignore schur
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 500;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::string report = summary.FullReport();
  RCLCPP_INFO(rclcpp::get_logger("marker_radar_lidar_calibrator"), "%s", report.c_str());

  std::string calibrated_params_str = std::accumulate(
    std::next(params.begin()), params.end(),
    std::to_string(params.front()),  // Initialize with the first element
    [](const std::string & a, const auto & b) { return a + " " + std::to_string(b); });
  std::string calibrated_params_msg =
    "calibrated params (x,y,z,pitch,yaw): " + calibrated_params_str;
  RCLCPP_INFO(
    rclcpp::get_logger("marker_radar_lidar_calibrator"), "%s", calibrated_params_msg.c_str());

  Eigen::Isometry3d calibrated_3d_radar_optimization_to_radar_transformation =
    Eigen::Isometry3d::Identity();
  calibrated_3d_radar_optimization_to_radar_transformation.pretranslate(
    Eigen::Vector3d(params[0], params[1], params[2]));
  Eigen::Quaterniond q(
    Eigen::AngleAxisd(params[4], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(params[3], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
  calibrated_3d_radar_optimization_to_radar_transformation.rotate(q);

  calibrated_radar_to_lidar_transformation_ =
    calibrated_3d_radar_optimization_to_radar_transformation.inverse() *
    radar_optimization_to_lidar_eigen_;
}

Eigen::Isometry3d TransformationEstimator::getTransformation()
{
  return calibrated_radar_to_lidar_transformation_;
}

}  // namespace marker_radar_lidar_calibrator
