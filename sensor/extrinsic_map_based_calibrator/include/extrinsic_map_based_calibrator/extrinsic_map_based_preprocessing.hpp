// Copyright 2022 Tier IV, Inc.
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

#ifndef EXTRINSIC_MAP_BASED_CALIBRATOR__EXTRINSIC_MAP_BASED_PREPROCESSING_HPP_
#define EXTRINSIC_MAP_BASED_CALIBRATOR__EXTRINSIC_MAP_BASED_PREPROCESSING_HPP_

#include "extrinsic_map_based_calibrator/pointcloud_matcher.hpp"
#include "pcl/PCLPointCloud2.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_ros/transforms.hpp"

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

namespace extrinsic_map_base_calibrator
{

struct RansacConfig
{
  int max_iteration;
  double voxel_grid_size;
  double distance_threshold;
  double min_radius;
};

struct ClipWallPointCloudConfig
{
  MatchingConfig matching_config;
  double clipping_threshold;
};

struct PreprocessingConfig
{
  RansacConfig ransac_config;
  ClipWallPointCloudConfig clip_config;
};

class ExtrinsicMapBasedPreprocessing
{
private:
  PreprocessingConfig config_{};
  PointCloudMatcher matcher_;
  matchingResult prematched_result_;

public:
  explicit ExtrinsicMapBasedPreprocessing();
  PointCloudT::Ptr preprocessing(
    const PointCloudT::Ptr & map_pointcloud_with_wall_pcl,
    const PointCloudT::Ptr & map_pointcloud_without_wall_pcl,
    const PointCloudT::Ptr & sensor_pointcloud_pcl);
  PointCloudT::Ptr preprocessing(
    const PointCloudT::Ptr & map_point_cloud_with_wall,
    const PointCloudT::Ptr & sensor_pointcloud_pcl);
  void downsamplingOnFloor(
    const PointCloudT::Ptr & pcl_sensor, PointCloudT::Ptr & pcl_filtered_sensor) const;
  PointCloudT::Ptr removeWallPointcloud(
    const PointCloudT::Ptr & sensor_point_cloud, const PointCloudT::Ptr & map_point_cloud_with_wall,
    const PointCloudT::Ptr & map_point_cloud_without_wall);
  void setConfig(PreprocessingConfig & config) { config_ = config; };
  matchingResult getPrematchedResult() { return prematched_result_; };
};

}  // namespace extrinsic_map_base_calibrator
#endif  // EXTRINSIC_MAP_BASED_CALIBRATOR__EXTRINSIC_MAP_BASED_PREPROCESSING_HPP_
