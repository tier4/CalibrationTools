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

#ifndef TIER4_GROUND_PLANE_UTILS__GROUND_PLANE_UTILS_HPP_
#define TIER4_GROUND_PLANE_UTILS__GROUND_PLANE_UTILS_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <tuple>
#include <utility>
#include <vector>

namespace tier4_ground_plane_utils
{

using PointType = pcl::PointXYZ;

struct GroundPlaneExtractorParameters
{
  bool verbose_;
  bool use_crop_box_filter_;
  double crop_box_min_x_;
  double crop_box_min_y_;
  double crop_box_min_z_;
  double crop_box_max_x_;
  double crop_box_max_y_;
  double crop_box_max_z_;
  bool use_pca_rough_normal_;
  double max_inlier_distance_;
  int min_plane_points_;
  int min_plane_points_percentage_;
  double max_cos_distance_;
  int max_iterations_;
  bool remove_outliers_;
  double remove_outlier_tolerance_;
  Eigen::Isometry3d initial_base_to_lidar_transform_;
};

/*!
 * Extracts the ground plane from a pointcloud
 * @param[in] pointcloud the input pointcloud
 * @return A tuple containing wether or not th calibration plane was found, the estimated ground
 * plane model, and the inliers of the respective model
 */
std::tuple<bool, Eigen::Vector4d, pcl::PointCloud<PointType>::Ptr> extractGroundPlane(
  pcl::PointCloud<PointType>::Ptr & pointcloud, const GroundPlaneExtractorParameters & parameters,
  std::vector<Eigen::Vector4d> & outlier_models);

/*!
 * Extracts a plane from a pointcloud
 * @param[in] pointcloud the input pointcloud
 * @param[in] max_inlier_distance maximum allowed inlier distance
 * @param[in] max_iterations max iterations for the ransac algorithm
 * @return A tuple containing a plane model and the inlier indices
 */
std::pair<Eigen::Vector4d, pcl::PointIndices::Ptr> extractPlane(
  pcl::PointCloud<PointType>::Ptr pointcloud, double max_inlier_distance, int max_iterations);

/*!
 * Computes the fitting error of an estimated model and the initial one
 * @param[in] estimated_model the estimated model
 * @param[in] inliers the inliers of the current estimated model
 */
void evaluateModels(
  const Eigen::Vector4d & initial_model, const Eigen::Vector4d & estimated_model,
  pcl::PointCloud<PointType>::Ptr inliers);

/*!
 * Computes a plane model given a pose.
 * The normal of the plane is given by the z-axis of the rotation of the pose
 * @param[in] pointcloud Point cloud to crop
 * @param[in] max_range Range to crop the pointcloud to
 * @return the plane model
 */
Eigen::Vector4d poseToPlaneModel(const Eigen::Isometry3d & pose);

/*!
 * Compute a pose from a plane model a*x + b*y +c*z +d = 0
 * The pose lies has its origin on the z-projection of the plane
 * @param[in] model Point cloud to crop
 * @return the plane pose
 */
Eigen::Isometry3d modelPlaneToPose(const Eigen::Vector4d & model);

/*!
 * Estimate / refine a lidar-base transform given an initial guess and an estimated ground plane
 * @param[in] base_lidar_transform Initial base lidar transform
 * @param[in] ground_plane_model ground plane model
 * @return the refined base lidar pose
 */
Eigen::Isometry3d estimateBaseLidarTransform(
  const Eigen::Isometry3d & initial_base_lidar_transform, const Eigen::Vector4d & model);

/*!
 * Removes the point that are consistent with an input plane from the pointcloud
 * @param[in] input_pointcloud the pointcloud to filter
 * @param[in] outlier_model the model that represents the outliers
 * @param[in] outlier_tolerance the tolerance with which a point is still considered an outlier
 * @return the refined base lidar pose
 */
pcl::PointCloud<PointType>::Ptr removeOutliers(
  pcl::PointCloud<PointType>::Ptr input_pointcloud, const Eigen::Vector4d & outlier_plane_model,
  double outlier_tolerance);

/*!
 * Overwrite the calibrated x, y, and yaw values of the calibrated base lidar transform with the
 * initial ones
 * @param[in] initial_base_lidar_transform_msg the initial base lidar transform msg
 * @param[in] calibrated_base_lidar_transform_msg the calibrated base lidar transform msg
 * @return the calibrated base lidar transform with its x, y, and yaw values being overwritten by
 * the initial ones
 */
geometry_msgs::msg::TransformStamped overwriteXYYawValues(
  const geometry_msgs::msg::TransformStamped & initial_base_lidar_transform_msg,
  const geometry_msgs::msg::TransformStamped & calibrated_base_lidar_transform_msg);

}  // namespace tier4_ground_plane_utils

#endif  // TIER4_GROUND_PLANE_UTILS__GROUND_PLANE_UTILS_HPP_
