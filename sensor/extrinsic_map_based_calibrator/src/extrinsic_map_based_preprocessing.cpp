// Copyright 2021 Tier IV, Inc.
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

#include "extrinsic_map_based_calibrator/extrinsic_map_based_preprocessing.hpp"

namespace extrinsic_map_base_calibrator
{
ExtrinsicMapBasedPreprocessing::ExtrinsicMapBasedPreprocessing()
{

}

PointCloudT::Ptr ExtrinsicMapBasedPreprocessing::preprocessing(
  const PointCloudT::Ptr & map_pointcloud_with_wall_pcl,
  const PointCloudT::Ptr & map_pointcloud_without_wall_pcl,
  const PointCloudT::Ptr & sensor_pointcloud_pcl)
{
  PointCloudT::Ptr filtered_sensor_pointcloud(new PointCloudT);
  // downsampling on the floor
  downsamplingOnFloor(sensor_pointcloud_pcl, filtered_sensor_pointcloud);

  return removeWallPointcloud(
    filtered_sensor_pointcloud, map_pointcloud_with_wall_pcl,
    map_pointcloud_without_wall_pcl);
}

PointCloudT::Ptr ExtrinsicMapBasedPreprocessing::preprocessing(
  const PointCloudT::Ptr & map_pointcloud_with_wall,
  const PointCloudT::Ptr & sensor_pointcloud_pcl)
{
  // matching of sensor cloud and map cloud
  matcher_.setParameter(config_.clip_config.matching_config);
  prematched_result_ = matcher_.ICPMatching(map_pointcloud_with_wall, sensor_pointcloud_pcl);
  PointCloudT::Ptr matched_sensor_pointcloud(new PointCloudT);
  pcl::transformPointCloud( *sensor_pointcloud_pcl, *matched_sensor_pointcloud, prematched_result_.transformation_matrix);

  PointCloudT::Ptr filtered_sensor_pointcloud(new PointCloudT);
  // downsampling on the floor
  downsamplingOnFloor(matched_sensor_pointcloud, filtered_sensor_pointcloud);

  return filtered_sensor_pointcloud;
}

void ExtrinsicMapBasedPreprocessing::downsamplingOnFloor(
  const PointCloudT::Ptr & pcl_sensor,
  PointCloudT::Ptr & pcl_filtered_sensor)
{
  // ransac
  pcl::PointIndices::Ptr output_inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr output_coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setRadiusLimits(config_.ransac_config.min_radius, std::numeric_limits<double>::max());
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(config_.ransac_config.distance_threshold);
  seg.setInputCloud(pcl_sensor);
  seg.setMaxIterations(config_.ransac_config.max_iteration);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.segment(*output_inliers, *output_coefficients);

  // extract target cloud and floor cloud
  PointCloudT::Ptr cloud_floor(new PointCloudT);
  PointCloudT::Ptr cloud_target(new PointCloudT);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(pcl_sensor);
  extract.setIndices(output_inliers);
  extract.setNegative(false);
  extract.filter(*cloud_floor);
  extract.setNegative(true);
  extract.filter(*cloud_target);

  // voxel grid filtering
  PointCloudT::Ptr cloud_voxel_filtered(new PointCloudT);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(pcl::make_shared<PointCloudT>(*cloud_floor));
  vg.setLeafSize(config_.ransac_config.voxel_grid_size, config_.ransac_config.voxel_grid_size, config_.ransac_config.voxel_grid_size);
  vg.filter(*cloud_voxel_filtered);

  // merge target cloud and downsampled floor cloud
  *pcl_filtered_sensor = *cloud_voxel_filtered + *cloud_target;

}

PointCloudT::Ptr ExtrinsicMapBasedPreprocessing::removeWallPointcloud(
  const PointCloudT::Ptr & sensor_point_cloud,
  const PointCloudT::Ptr & map_point_cloud_with_wall,
  const PointCloudT::Ptr & map_point_cloud_without_wall)
{
  // matching of sensor cloud and map cloud
  matcher_.setParameter(config_.clip_config.matching_config);
  PointCloudT matched_sensor_point_cloud;
  prematched_result_ = matcher_.ICPMatching(map_point_cloud_with_wall, sensor_point_cloud);
  pcl::transformPointCloud( *sensor_point_cloud, matched_sensor_point_cloud, prematched_result_.transformation_matrix);

  // return matched_sensor_point_cloud.makeShared();
  // clip overlapped cloud
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree->setInputCloud(map_point_cloud_without_wall);
  int overlapped_point_num = 0;

  PointCloudT clipped_sensor_point_cloud;
  for (size_t i = 0; i < matched_sensor_point_cloud.points.size(); ++i) {
    std::vector<int> indices;
    std::vector<float> distances;
    kdtree->nearestKSearch(matched_sensor_point_cloud.points[i], 1, indices, distances);

    if (std::sqrt(distances[0]) < config_.clip_config.clipping_threshold) {
      PointCloudT cloud;
      cloud.width = 1;
      cloud.height = 1;
      cloud.is_dense = false;
      cloud.points.resize(cloud.width * cloud.height);
      cloud.points[0] = matched_sensor_point_cloud.points[i];
      clipped_sensor_point_cloud += cloud;
      overlapped_point_num += 1;
    }
  }

  return clipped_sensor_point_cloud.makeShared();
}
}  // namespace map_base_calibrator
