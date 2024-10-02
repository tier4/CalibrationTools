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

#ifndef MARKER_RADAR_LIDAR_CALIBRATOR__TRANSFORMATION_ESTIMATOR_HPP_
#define MARKER_RADAR_LIDAR_CALIBRATOR__TRANSFORMATION_ESTIMATOR_HPP_

#include <Eigen/Dense>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <marker_radar_lidar_calibrator/marker_radar_lidar_calibrator.hpp>
#include <marker_radar_lidar_calibrator/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ceres/ceres.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <tf2/convert.h>

namespace marker_radar_lidar_calibrator
{

class TransformationEstimator
{
public:
  TransformationEstimator(
    Eigen::Isometry3d initial_radar_to_lidar_eigen,
    Eigen::Isometry3d initial_radar_to_radar_optimization_eigen,
    Eigen::Isometry3d radar_optimization_to_lidar_eigen);
  void setPoints(
    pcl::PointCloud<common_types::PointType>::Ptr lidar_points_ocs,
    pcl::PointCloud<common_types::PointType>::Ptr radar_points_rcs);
  void setDelta(double delta_cos, double delta_sin);
  void estimateYawOnlyTransformation();
  void estimateSVDTransformation(
    ExtrinsicReflectorBasedCalibrator::TransformationType transformation_type);
  void estimateZeroRollTransformation();
  Eigen::Isometry3d getTransformation();

  double delta_cos_;
  double delta_sin_;
  pcl::PointCloud<common_types::PointType>::Ptr lidar_points_ocs_;
  pcl::PointCloud<common_types::PointType>::Ptr radar_points_rcs_;
  Eigen::Isometry3d calibrated_radar_to_lidar_transformation_;

  Eigen::Isometry3d initial_radar_to_lidar_eigen_;
  Eigen::Isometry3d initial_radar_optimization_to_radar_eigen_;
  Eigen::Isometry3d radar_optimization_to_lidar_eigen_;
};

}  // namespace marker_radar_lidar_calibrator

#endif  // MARKER_RADAR_LIDAR_CALIBRATOR__TRANSFORMATION_ESTIMATOR_HPP_
