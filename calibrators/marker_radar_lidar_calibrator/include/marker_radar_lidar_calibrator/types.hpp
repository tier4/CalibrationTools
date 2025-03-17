// Copyright 2024 TIER IV, Inc.
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

#pragma once

#include <Eigen/Dense>

#include <geometry_msgs/msg/point.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace marker_radar_lidar_calibrator
{

namespace common_types
{
using PointType = pcl::PointXYZ;
}

struct BackgroundModel
{
public:
  using TreeType = pcl::KdTreeFLANN<common_types::PointType>;  // cSpell:ignore FLANN
  using index_t = std::uint32_t;

  BackgroundModel()
  : valid_(false),
    leaf_size_(0.0),
    min_point_(
      std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max(), 1.f),
    max_point_(
      -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
      -std::numeric_limits<float>::max(), 1.f),
    pointcloud_(new pcl::PointCloud<common_types::PointType>)
  {
  }

  bool valid_;
  double leaf_size_;
  Eigen::Vector4f min_point_;
  Eigen::Vector4f max_point_;
  std::unordered_set<index_t> set_;
  pcl::PointCloud<common_types::PointType>::Ptr pointcloud_;
  TreeType tree_;
};

enum class TransformationType { svd_2d, yaw_only_rotation_2d, svd_3d, zero_roll_3d };

struct TransformationResult
{
  pcl::PointCloud<common_types::PointType>::Ptr lidar_points_ocs;
  pcl::PointCloud<common_types::PointType>::Ptr radar_points_rcs;
  std::unordered_map<TransformationType, Eigen::Isometry3d>
    calibrated_radar_to_lidar_transformations;

  void clear()
  {
    lidar_points_ocs.reset();
    radar_points_rcs.reset();
    calibrated_radar_to_lidar_transformations.clear();
  }
};

struct CalibrationErrorMetrics
{
  std::vector<double> calibrated_distance_errors;
  std::vector<double> calibrated_yaw_errors;
  std::vector<double> avg_crossval_calibrated_distance_errors;
  std::vector<double> avg_crossval_calibrated_yaw_errors;
  std::vector<double> std_crossval_calibrated_distance_errors;
  std::vector<double> std_crossval_calibrated_yaw_errors;

  void clear()
  {
    calibrated_distance_errors.clear();
    calibrated_yaw_errors.clear();
    avg_crossval_calibrated_distance_errors.clear();
    avg_crossval_calibrated_yaw_errors.clear();
    std_crossval_calibrated_distance_errors.clear();
    std_crossval_calibrated_yaw_errors.clear();
  }
};

struct OutputMetrics
{
  int num_of_converged_tracks = 0;
  std::vector<int> num_of_samples;
  std::unordered_map<TransformationType, CalibrationErrorMetrics> methods;
  std::vector<geometry_msgs::msg::Point> detections;

  void clear()
  {
    num_of_converged_tracks = 0;
    num_of_samples.clear();
    for (auto & [type, metrics] : methods) {
      metrics.clear();
    }
    detections.clear();
  }
};

struct Track
{
  int id;
  Eigen::Vector3d lidar_estimation;
  Eigen::Vector3d radar_estimation;
  double distance_error;
  double yaw_error;
};

}  // namespace marker_radar_lidar_calibrator
