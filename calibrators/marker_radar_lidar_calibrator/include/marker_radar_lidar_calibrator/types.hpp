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

#ifndef MARKER_RADAR_LIDAR_CALIBRATOR__TYPES_HPP_
#define MARKER_RADAR_LIDAR_CALIBRATOR__TYPES_HPP_

#include <Eigen/Dense>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <limits>
#include <memory>
#include <unordered_set>

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

}  // namespace marker_radar_lidar_calibrator

#endif  // MARKER_RADAR_LIDAR_CALIBRATOR__TYPES_HPP_
