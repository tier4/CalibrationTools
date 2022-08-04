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
  const PointCloudT::Ptr & sensor_pointcloud_pcl,
  PointCloudT::Ptr & inliers_pointcloud_pcl)
{
  PointCloudT::Ptr remove_wall_pointcloud(new PointCloudT);
  remove_wall_pointcloud = removeWallPointcloud(
    sensor_pointcloud_pcl, map_pointcloud_with_wall_pcl,
    map_pointcloud_without_wall_pcl);
  PointCloudT::Ptr filtered_sensor_pointcloud(new PointCloudT);
  downsamplingOnFloor(remove_wall_pointcloud, filtered_sensor_pointcloud, inliers_pointcloud_pcl);
  return filtered_sensor_pointcloud;
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
  PointCloudT::Ptr inliers(new PointCloudT);
  // downsampling on the floor
  downsamplingOnFloor(matched_sensor_pointcloud, filtered_sensor_pointcloud, inliers);

  return filtered_sensor_pointcloud;
}

void ExtrinsicMapBasedPreprocessing::downsamplingOnFloor(
  const PointCloudT::Ptr & pcl_sensor,
  PointCloudT::Ptr & pcl_filtered_sensor,
  PointCloudT::Ptr & inliers_pointcloud_pcl) const

{
  PointCloudT::Ptr cloud_target(new PointCloudT);
  Eigen::Vector4d ground_plane_model;
  extractGroundPlane(pcl_sensor, ground_plane_model, inliers_pointcloud_pcl/* , cloud_target */);
  PointCloudT::Ptr cloud_ground_plane(new PointCloudT);
  separatePointCloud(pcl_sensor, ground_plane_model, cloud_ground_plane, cloud_target);

  // voxel grid filtering
  PointCloudT::Ptr cloud_voxel_filtered(new PointCloudT);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(boost::make_shared<PointCloudT>(*cloud_ground_plane));
  vg.setLeafSize(config_.ransac_config.voxel_grid_size, config_.ransac_config.voxel_grid_size, config_.ransac_config.voxel_grid_size);
  vg.filter(*cloud_voxel_filtered);

  pcl::VoxelGrid<pcl::PointXYZ> vg_target;
  PointCloudT::Ptr filtered_cloud_target(new PointCloudT);
  vg_target.setInputCloud(boost::make_shared<PointCloudT>(*cloud_target));
  vg_target.setLeafSize(config_.ransac_config.voxel_grid_size/2, config_.ransac_config.voxel_grid_size/2, config_.ransac_config.voxel_grid_size/2);
  vg_target.filter(*filtered_cloud_target);

  // merge target cloud and downsampled floor cloud
  *pcl_filtered_sensor = *cloud_voxel_filtered + *filtered_cloud_target;

}

bool ExtrinsicMapBasedPreprocessing::extractGroundPlane(
  const PointCloudT::Ptr & pointcloud, Eigen::Vector4d & model,
  PointCloudT::Ptr & plane_pointcloud/* , PointCloudT::Ptr & target_pointcloud */) const
{
  std::vector<pcl::ModelCoefficients> models;

  // Obtain an idea of the ground plane using PCA
  // under the asumption that the axis with less variance will be the ground plane normal
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(pointcloud);
  Eigen::MatrixXf vectors = pca.getEigenVectors();
  Eigen::Vector3f rough_normal = vectors.col(2);

  // Use RANSAC iteratively until we find the ground plane
  // Since walls can have more points, we filter using the PCA-based hypothesis
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(max_inlier_distance_);
  seg.setMaxIterations(max_iterations_);

  PointCloudT::Ptr iteration_cloud = pointcloud;
  int iteration_size = iteration_cloud->height * iteration_cloud->width;

  while (iteration_size > min_plane_points_) {
    seg.setInputCloud(iteration_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
      break;
    }

    Eigen::Vector3f normal(
      coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    float cos_distance = 1.0 - std::abs(rough_normal.dot(normal));

    if (
      static_cast<int>(inliers->indices.size()) > min_plane_points_ &&
      cos_distance < max_cos_distance_) {
      model = Eigen::Vector4d(
        coefficients->values[0], coefficients->values[1], coefficients->values[2],
        coefficients->values[3]);

      // Extract the ground plane inliers
      extract.setInputCloud(iteration_cloud);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*plane_pointcloud);
      // extract.setNegative(true);
      // extract.filter(*target_pointcloud);
      return true;
    }

    // Extract the inliers from the pointcloud (the detected plane was not the ground plane)
    extract.setInputCloud(iteration_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);

    PointCloudT next_cloud;
    extract.filter(next_cloud);

    iteration_cloud->swap(next_cloud);
    iteration_size = iteration_cloud->height * iteration_cloud->width;
  }

  return false;

}

void ExtrinsicMapBasedPreprocessing::separatePointCloud(
    const PointCloudT::Ptr & pointcloud,
    const Eigen::Vector4d & model,
    PointCloudT::Ptr & plane_pointcloud,
    PointCloudT::Ptr & target_pointcloud) const
  {
    for(auto & p : pointcloud->points){
      double distance = pcl::pointToPlaneDistance(p, model(0), model(1), model(2), model(3));
      if(distance < config_.ransac_config.distance_threshold){
        plane_pointcloud->points.push_back(p);
      }
      else{
        target_pointcloud->points.push_back(p);
      }
    }
  }

PointCloudT::Ptr ExtrinsicMapBasedPreprocessing::removeWallPointcloud(
  const PointCloudT::Ptr & sensor_point_cloud,
  const PointCloudT::Ptr & map_point_cloud_with_wall,
  const PointCloudT::Ptr & map_point_cloud_without_wall)
{
  // matching of sensor cloud and map cloud
  matcher_.setParameter(config_.clip_config.matching_config);
  PointCloudT matched_sensor_point_cloud;
  prematched_result_ = matcher_.GICPMatching(map_point_cloud_with_wall, sensor_point_cloud);
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
