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

#ifndef EXTRINSIC_MAP_BASED_CALIBRATOR__POINTCLOUD_PREPROCESSING_HPP_
#define EXTRINSIC_MAP_BASED_CALIBRATOR__POINTCLOUD_PREPROCESSING_HPP_

#include <string>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include "pcl/PCLPointCloud2.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"
#include "pcl/pcl_base.h"
#include "pcl/common/pca.h"
#include "pcl/point_cloud.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/ModelCoefficients.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.h"
#include "extrinsic_map_based_calibrator/pointcloud_matcher.hpp"

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

namespace extrinsic_map_base_calibrator
{

struct RansacConfig{
  int max_iteration;
  double voxel_grid_size;
  double distance_threshold;
  double min_radius;
};

struct ClipWallPointCloudConfig{
  MatchingConfig matching_config;
  double clipping_threshold;
};

struct PreprocessingConfig{
  RansacConfig ransac_config;
  ClipWallPointCloudConfig clip_config;
};

class ExtrinsicMapBasedPreprocessing
{
private:
  PreprocessingConfig config_{};
  PointCloudMatcher matcher_;
  matchingResult prematched_result_;
  int min_plane_points_ = 500;
  float max_cos_distance_ = 0.3;
  double max_inlier_distance_ = 0.05;
  int max_iterations_ = 1000;


public:
  explicit ExtrinsicMapBasedPreprocessing();
  PointCloudT::Ptr preprocessing(
    const PointCloudT::Ptr & map_pointcloud_with_wall_pcl,
    const PointCloudT::Ptr & map_pointcloud_without_wall_pcl,
    const PointCloudT::Ptr & sensor_pointcloud_pcl,
    PointCloudT::Ptr & inliers_pointcloud_pcl);
  PointCloudT::Ptr preprocessing(
    const PointCloudT::Ptr & map_point_cloud_with_wall,
    const PointCloudT::Ptr & sensor_pointcloud_pcl);
  void downsamplingOnFloor(
    const PointCloudT::Ptr & pcl_sensor,
    PointCloudT::Ptr & pcl_filtered_sensor,
    PointCloudT::Ptr & inliers_pointcloud_pcl) const;
  bool extractGroundPlane(
    const PointCloudT::Ptr & pointcloud,
    Eigen::Vector4d & model,
    PointCloudT::Ptr & plane_pointcloud/* ,
    PointCloudT::Ptr & target_pointcloud */) const;
  void separatePointCloud(
    const PointCloudT::Ptr & pointcloud,
    const Eigen::Vector4d & model,
    PointCloudT::Ptr & plane_pointcloud,
    PointCloudT::Ptr & target_pointcloud) const;
  PointCloudT::Ptr removeWallPointcloud(
    const PointCloudT::Ptr & sensor_point_cloud,
    const PointCloudT::Ptr & map_point_cloud_with_wall,
    const PointCloudT::Ptr & map_point_cloud_without_wall);
  void setConfig(PreprocessingConfig & config){config_ = config;};
  matchingResult getPrematchedResult(){return prematched_result_;};
};

}  // namespace extrinsic_map_base_calibrator
#endif  // EXTRINSIC_MAP_BASED_CALIBRATOR__POINTCLOUD_PREPROCESSING_HPP_
