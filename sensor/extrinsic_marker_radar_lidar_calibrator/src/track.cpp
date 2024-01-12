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

#include <Eigen/Dense>
#include <extrinsic_marker_radar_lidar_calibrator/track.hpp>
#include <extrinsic_marker_radar_lidar_calibrator/types.hpp>
#include <kalman_filter/kalman_filter.hpp>

namespace extrinsic_marker_radar_lidar_calibrator
{

Track::Track(
  builtin_interfaces::msg::Time & t0, const KalmanFilter & initial_lidar_filter,
  const KalmanFilter & initial_radar_filter, double lidar_convergence_thresh,
  double radar_convergence_thresh, double timeout_thresh, double max_matching_thresh)
: latest_update_time_(t0),
  lidar_filter_(initial_lidar_filter),
  radar_filter_(initial_radar_filter),
  lidar_convergence_thresh_(lidar_convergence_thresh),
  radar_convergence_thresh_(radar_convergence_thresh),
  timeout_thresh_(timeout_thresh),
  max_matching_thresh_(max_matching_thresh),
  first_observation_(true)
{
}

bool Track::match(const Eigen::Vector3d & lidar_detection, const Eigen::Vector3d & radar_detection)
{
  return (lidar_detection - getLidarEstimation()).norm() < max_matching_thresh_ &&
         (radar_detection - getRadarEstimation()).norm() < max_matching_thresh_;
}

bool Track::partialMatch(
  const Eigen::Vector3d & lidar_detection, const Eigen::Vector3d & radar_detection)
{
  return (lidar_detection - getLidarEstimation()).norm() < max_matching_thresh_ ||
         (radar_detection - getRadarEstimation()).norm() < max_matching_thresh_;
}

void Track::update(const Eigen::Vector3d & lidar_detection, const Eigen::Vector3d & radar_detection)
{
  lidar_filter_.predict(Eigen::Vector3d::Zero());
  lidar_filter_.update(lidar_detection);
  radar_filter_.predict(Eigen::Vector3d::Zero());
  radar_filter_.update(radar_detection);
  return;
}

void Track::updateIfMatch(
  const Eigen::Vector3d & lidar_detection, const Eigen::Vector3d & radar_detection)
{
  if (match(lidar_detection, radar_detection)) {
    update(lidar_detection, radar_detection);
  }
}

bool Track::converged()
{
  Eigen::MatrixXd lidar_p_matrix, radar_p_matrix;

  lidar_filter_.getP(lidar_p_matrix);
  double lidar_max_p = lidar_p_matrix.diagonal().maxCoeff();

  radar_filter_.getP(radar_p_matrix);
  double radar_max_p = radar_p_matrix.diagonal().maxCoeff();

  std::cout << "lidar p: " << lidar_max_p << " | " << radar_convergence_thresh_ << std::endl
            << std::flush;
  std::cout << "radar p: " << radar_max_p << " | " << radar_convergence_thresh_ << std::endl
            << std::flush;

  return lidar_max_p < lidar_convergence_thresh_ && radar_max_p < radar_convergence_thresh_;
}

bool Track::timedOut(builtin_interfaces::msg::Time & time) const
{
  const auto dt = (rclcpp::Time(time) - rclcpp::Time(latest_update_time_)).seconds();
  return dt < 0 || dt > timeout_thresh_;
}

Eigen::Vector3d Track::getLidarEstimation()
{
  Eigen::MatrixXd lidar_estimation;
  lidar_filter_.getX(lidar_estimation);
  return Eigen::Vector3d(lidar_estimation.reshaped());
}

Eigen::Vector3d Track::getRadarEstimation()
{
  Eigen::MatrixXd radar_estimation;
  radar_filter_.getX(radar_estimation);
  return Eigen::Vector3d(radar_estimation.reshaped());
}

TrackFactory::TrackFactory(
  double initial_lidar_cov, double initial_radar_cov, double lidar_measurement_cov,
  double radar_measurement_cov, double lidar_process_cov, double radar_process_cov,
  double lidar_convergence_thresh, double radar_convergence_thresh, double timeout_thresh,
  double max_matching_distance)
: initial_lidar_cov_(initial_lidar_cov * initial_lidar_cov),
  initial_radar_cov_(initial_radar_cov * initial_radar_cov),
  lidar_convergence_thresh_(lidar_convergence_thresh * lidar_convergence_thresh),
  radar_convergence_thresh_(radar_convergence_thresh * radar_convergence_thresh),
  timeout_thresh_(timeout_thresh),
  max_matching_distance_(max_matching_distance)
{
  lidar_measurement_cov = lidar_measurement_cov * lidar_measurement_cov;
  radar_measurement_cov = radar_measurement_cov * radar_measurement_cov;
  lidar_process_cov = lidar_process_cov * lidar_process_cov;
  radar_process_cov = radar_process_cov * radar_process_cov;

  lidar_filter_.setA(Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, 1.0));
  lidar_filter_.setB(Eigen::DiagonalMatrix<double, 3>(0.0, 0.0, 0.0));
  lidar_filter_.setC(Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, 1.0));
  lidar_filter_.setR(Eigen::DiagonalMatrix<double, 3>(
    lidar_measurement_cov, lidar_measurement_cov, lidar_measurement_cov));
  lidar_filter_.setQ(
    Eigen::DiagonalMatrix<double, 3>(lidar_process_cov, lidar_process_cov, lidar_process_cov));

  radar_filter_.setA(Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, 1.0));
  radar_filter_.setB(Eigen::DiagonalMatrix<double, 3>(0.0, 0.0, 0.0));
  radar_filter_.setC(Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, 1.0));
  radar_filter_.setR(Eigen::DiagonalMatrix<double, 3>(
    radar_measurement_cov, radar_measurement_cov, radar_measurement_cov));
  radar_filter_.setQ(
    Eigen::DiagonalMatrix<double, 3>(radar_process_cov, radar_process_cov, radar_process_cov));
}

Track TrackFactory::makeTrack(
  const Eigen::Vector3d & lidar_detection, const Eigen::Vector3d & radar_detection,
  builtin_interfaces::msg::Time & t0)
{
  auto lidar_filter = lidar_filter_;
  auto radar_filter = radar_filter_;
  Eigen::DiagonalMatrix<double, 3> lidar_p0(
    initial_lidar_cov_, initial_lidar_cov_, initial_lidar_cov_);
  Eigen::DiagonalMatrix<double, 3> radar_p0(
    initial_radar_cov_, initial_radar_cov_, initial_radar_cov_);
  lidar_filter.init(lidar_detection, lidar_p0);
  radar_filter.init(radar_detection, radar_p0);

  return Track(
    t0, lidar_filter, radar_filter, lidar_convergence_thresh_, radar_convergence_thresh_,
    timeout_thresh_, max_matching_distance_);
}

}  // namespace extrinsic_marker_radar_lidar_calibrator
