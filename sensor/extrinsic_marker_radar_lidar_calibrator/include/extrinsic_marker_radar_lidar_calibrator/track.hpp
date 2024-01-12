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

#ifndef EXTRINSIC_MARKER_RADAR_LIDAR_CALIBRATOR__TRACK_HPP_
#define EXTRINSIC_MARKER_RADAR_LIDAR_CALIBRATOR__TRACK_HPP_

#include <Eigen/Dense>
#include <builtin_interfaces/msg/time.hpp>
#include <extrinsic_marker_radar_lidar_calibrator/types.hpp>
#include <kalman_filter/kalman_filter.hpp>
#include <rclcpp/time.hpp>

#include <memory>

namespace extrinsic_marker_radar_lidar_calibrator
{

class Track
{
public:
  bool match(const Eigen::Vector3d & lidar_detection, const Eigen::Vector3d & radar_detection);
  bool partialMatch(
    const Eigen::Vector3d & lidar_detection, const Eigen::Vector3d & radar_detection);
  void update(const Eigen::Vector3d & lidar_detection, const Eigen::Vector3d & radar_detection);
  void updateIfMatch(
    const Eigen::Vector3d & lidar_detection, const Eigen::Vector3d & radar_detection);
  bool converged();
  bool timedOut(builtin_interfaces::msg::Time & time) const;

  Eigen::Vector3d getLidarEstimation();
  Eigen::Vector3d getRadarEstimation();

protected:
  Track(
    builtin_interfaces::msg::Time & t0, const KalmanFilter & initial_lidar_filter,
    const KalmanFilter & initial_radar_filter, double lidar_convergence_thresh,
    double radar_convergence_thresh, double timeout_thresh, double max_matching_thresh);

  builtin_interfaces::msg::Time latest_update_time_;
  KalmanFilter lidar_filter_, radar_filter_;

  double lidar_convergence_thresh_;
  double radar_convergence_thresh_;
  double timeout_thresh_;
  double max_matching_thresh_;
  bool first_observation_;

  friend class TrackFactory;
};

class TrackFactory
{
public:
  using Ptr = std::shared_ptr<TrackFactory>;

  TrackFactory(
    double initial_lidar_cov, double initial_radar_cov, double lidar_measurement_cov,
    double radar_measurement_cov, double lidar_process_cov, double radar_process_cov,
    double lidar_convergence_tresh, double radar_convergence_thresh, double timeout_thresh,
    double max_matching_distance);

  Track makeTrack(
    const Eigen::Vector3d & lidar_detection, const Eigen::Vector3d & radar_detection,
    builtin_interfaces::msg::Time & t0);

  KalmanFilter lidar_filter_, radar_filter_;

  double initial_lidar_cov_;
  double initial_radar_cov_;
  double lidar_convergence_thresh_;
  double radar_convergence_thresh_;
  double timeout_thresh_;
  double max_matching_distance_;
};

}  // namespace extrinsic_marker_radar_lidar_calibrator

#endif  // EXTRINSIC_MARKER_RADAR_LIDAR_CALIBRATOR__TRACK_HPP_
