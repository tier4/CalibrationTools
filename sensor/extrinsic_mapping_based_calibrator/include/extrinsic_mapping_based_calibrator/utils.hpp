// Copyright 2023 Tier IV, Inc.
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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR__UTILS_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR__UTILS_HPP_

#include <Eigen/Core>
#include <extrinsic_mapping_based_calibrator/types.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/registration.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <vector>

/*!
 * Transform a pointcloud between frames
 * @param[in] source_frame Source frame
 * @param[in] target_frame Target frame
 * @param[out] pc_ptr Output pointcloud
 * @param[in] buffer Tf buffer to get the transforms from
 */
template <typename PointcloudType>
void transformPointcloud(
  const std::string & source_frame, const std::string & target_frame,
  typename PointcloudType::Ptr & pc_ptr, tf2_ros::Buffer & buffer);

/*!
 * Crop a point cloud to a certain radius
 * @param[in] pointcloud Point cloud to crop
 * @param[in] max_range Range to crop the pointcloud to
 * @retval Cropped pointcloud
 */
template <typename PointcloudType>
typename PointcloudType::Ptr cropPointCloud(
  const typename PointcloudType::Ptr & pointcloud, double min_range, double max_range);

/*!
 * Interpolate a transform between two points in time
 * @param[in] t Interpolation time t >t1 and t<=t2
 * @param[in] t1 Left interpolation time
 * @param[in] t2 Righti interpolation time
 * @param[in] m1 Transformation at time t1
 * @param[in] m2 Transformation at time t2
 * @retval Interpolated transform at time t
 */
Eigen::Matrix4f poseInterpolationBase(
  double t, double t1, double t2, Eigen::Matrix4f const & m1, Eigen::Matrix4f const & m2);

/*!
 * Interpolate a transform between two points in time
 * @param[in] t Interpolation time (can be greater than t2 -> extrapolation)
 * @param[in] t1 Left interpolation time
 * @param[in] t2 Righti interpolation time
 * @param[in] m1 Transformation at time t1
 * @param[in] m2 Transformation at time t2
 * @retval Interpolated transform at time t
 */
Eigen::Matrix4f poseInterpolation(
  double t, double t1, double t2, Eigen::Matrix4f const & m1, Eigen::Matrix4f const & m2);

/*!
 * Compute the source to distance pointcloud distance
 * @param[in] estimator Correspondence estimator between source and target
 * @param[in] max_corr_distance Maximum distance allowed to be considered a correspondence [m]
 * @retval Source to distance pointcloud distance
 */
template <class PointType>
float sourceTargetDistance(
  pcl::registration::CorrespondenceEstimation<PointType, PointType> & estimator,
  float max_corr_distance);

/*!
 * Estimate the source->target distance
 * @param[in] source Source pointcloud
 * @param[in] target Target pointcloud
 * @param[in] transform Target to input frame transform
 * @param[in] max_corr_distance Maximum distance allowed to be considered a correspondence [m]
 * @retval Source to distance pointcloud distance
 */
template <class PointType>
float sourceTargetDistance(
  const typename pcl::PointCloud<PointType>::Ptr & source,
  const typename pcl::PointCloud<PointType>::Ptr & target, const Eigen::Matrix4f & transform,
  float max_corr_distance);

/*!
 * Estimate the source->target distance
 * @param[in] source Source pointcloud
 * @param[in] target Target pointcloud
 * @param[in] transform Target to input frame transform
 * @param[in] max_corr_distance Maximum distance allowed to be considered a correspondence [m]
 * @retval Source to distance pointcloud distance
 */
template <class PointType>
float sourceTargetDistance(
  const std::vector<typename pcl::PointCloud<PointType>::Ptr> & sources,
  const std::vector<typename pcl::PointCloud<PointType>::Ptr> & targets,
  const Eigen::Matrix4f & transform, float max_corr_distance);

/*!
 * Find the best transform between pointclouds using a set of registrators and a set
 * of input transforms (initial solutions) in a cascade approach
 * @param[in] input_transforms Input transforms (to be used as initial solutions)
 * @param[in] registrators Vector of registrators to use as a cascade
 * @param[in] verbose verbose mode
 * @param[out] best_transform Output transform containing the best solution
 * @param[out] best_score Output score containing the best solution
 */
template <class RegistratorPtrType, class PointType>
void findBestTransform(
  const std::vector<Eigen::Matrix4f> & input_transforms,
  std::vector<typename RegistratorPtrType::Ptr> & registrators, float eval_max_corr_distance,
  bool verbose, Eigen::Matrix4f & best_transform, float & best_score);

/*!
 * Crop a target pointcloud to the ranges of a sorce one
 * @param[in] initial_source_aligned_pc_ptr Pointcloud to use as a reference to crop a target
 * pointcloud
 * @param[out] target_dense_pc_ptr Pointcloud to be cropped
 */
template <class PointType>
void cropTargetPointcloud(
  const typename pcl::PointCloud<PointType>::Ptr & initial_source_aligned_pc_ptr,
  typename pcl::PointCloud<PointType>::Ptr & target_dense_pc_ptr, float max_radius);

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR__UTILS_HPP_
