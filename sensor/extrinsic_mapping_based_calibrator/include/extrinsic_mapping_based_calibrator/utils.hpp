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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR_UTILS_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR_UTILS_HPP_

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/registration.h>
#include <tf2_ros/buffer.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

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
  typename PointcloudType::Ptr & pc_ptr, tf2_ros::Buffer & buffer)
{
  if (source_frame == target_frame) {
    return;
  }

  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    Eigen::Matrix4f transform =
      tf2::transformToEigen(
        buffer.lookupTransform(target_frame, source_frame, t, timeout).transform)
        .matrix()
        .cast<float>();

    typename PointcloudType::Ptr transformed_pc_ptr(new PointcloudType());
    pcl::transformPointCloud(*pc_ptr, *transformed_pc_ptr, transform);

    pc_ptr.swap(transformed_pc_ptr);

  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(rclcpp::get_logger("tf_buffer"), "could not get initial tf. %s", ex.what());
  }
}

/*!
 * Crop a point cloud to a certain radius
 * @param[in] pointcloud Point cloud to crop
 * @param[in] max_range Range to crop the pointcloud to
 * @retval Cropped pointcloud
 */
pcl::PointCloud<PointType>::Ptr cropPointCloud(
  pcl::PointCloud<PointType>::Ptr & pointcloud, double max_range)
{
  pcl::PointCloud<PointType>::Ptr tmp_ptr(new pcl::PointCloud<PointType>());
  tmp_ptr->reserve(pointcloud->size());
  for (const auto & p : pointcloud->points) {
    if (std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) < max_range) {
      tmp_ptr->points.push_back(p);
    }
  }

  return tmp_ptr;
}

/*!
 * Interpolate a transform between two points in time
 * @param[in] t Interpolation time t >t1 and t<=t2
 * @param[in] t1 Left interpolation time
 * @param[in] t2 Righti interpolation time
 * @param[in] m1 Transformation at time t1
 * @param[in] m2 Transformation at time t2
 * @retval Interpolated transform at time t
 */
Eigen::Matrix4f poseInterpolation(
  double t, double t1, double t2, Eigen::Matrix4f const & m1, Eigen::Matrix4f const & m2)
{
  assert(t >= t1 && t <= t2);

  float alpha = 0.0;
  if (t2 != t1) alpha = (t - t1) / (t2 - t1);

  Eigen::Affine3f aff1(m1);
  Eigen::Affine3f aff2(m2);

  Eigen::Quaternionf rot1(aff1.linear());
  Eigen::Quaternionf rot2(aff2.linear());

  Eigen::Vector3f trans1 = aff1.translation();
  Eigen::Vector3f trans2 = aff2.translation();

  Eigen::Affine3f result;
  result.translation() = (1.0 - alpha) * trans1 + alpha * trans2;
  result.linear() = rot1.slerp(alpha, rot2).toRotationMatrix();

  return result.matrix();
}

/*!
 * Compute the source to distance pointcloud distance
 * @param[in] estimator Correspondence estimator between source and target
 * @retval Source to distance pointcloud distance
 */
template <class PointType>
float sourceTargetDistance(
  pcl::registration::CorrespondenceEstimation<PointType, PointType> & estimator)
{
  pcl::Correspondences correspondences;
  estimator.determineCorrespondences(correspondences);

  int n_points = static_cast<int>(correspondences.size());
  float sum = 0;

  for (int i = 0; i < n_points; ++i) {
    float distance = correspondences[i].distance;
    sum += distance;
  }

  return sum / n_points;
}

/*!
 * Estimate the source->target distance
 * @param[in] source Source pointcloud
 * @param[in] target Target pointcloud
 * @param[in] transform Target to input frame transform
 * @retval Source to distance pointcloud distance
 */
template <class PointType>
float sourceTargetDistance(
  const typename pcl::PointCloud<PointType>::Ptr & source,
  const typename pcl::PointCloud<PointType>::Ptr & target, const Eigen::Matrix4f & transform)
{
  PointcloudType::Ptr source_aligned(new PointcloudType());
  transformPointCloud(*source, *source_aligned, transform);

  pcl::registration::CorrespondenceEstimation<PointType, PointType> estimator;
  estimator.setInputSource(source_aligned);
  estimator.setInputTarget(target);

  return sourceTargetDistance(estimator);
}

/*!
 * Estimate the source->target distance
 * @param[in] source Source pointcloud
 * @param[in] target Target pointcloud
 * @param[in] transform Target to input frame transform
 * @retval Source to distance pointcloud distance
 */
template <class PointType>
float sourceTargetDistance(
  const std::vector<typename pcl::PointCloud<PointType>::Ptr> & sources,
  const std::vector<typename pcl::PointCloud<PointType>::Ptr> & targets,
  const Eigen::Matrix4f & transform)
{
  double distance = 0.0;
  int n = 0;

  assert(sources.size() == targets.size());

  for (std::size_t i = 0; i < sources.size(); i++) {
    PointcloudType::Ptr source_aligned(new PointcloudType());
    transformPointCloud(*sources[i], *source_aligned, transform);

    pcl::registration::CorrespondenceEstimation<PointType, PointType> estimator;
    estimator.setInputSource(source_aligned);
    estimator.setInputTarget(targets[i]);

    n += sources[i]->size();
    distance += sources[i]->size() * sourceTargetDistance(estimator);
  }

  return distance / n;
}

#pragma GCC push_options
#pragma GCC optimize("O0")
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
  std::vector<typename RegistratorPtrType::Ptr> & registrators, bool verbose,
  Eigen::Matrix4f & best_transform, float & best_score)
{
  std::vector<Eigen::Matrix4f> transforms = input_transforms;
  std::vector<std::string> transforms_names;

  for (std::size_t i = 0; i < transforms.size(); i++) {
    transforms_names.push_back("initial_guess_" + std::to_string(i));
  }

  best_transform = Eigen::Matrix4f::Identity();
  best_score = std::numeric_limits<float>::max();
  std::string best_name;

  for (auto & registrator : registrators) {
    Eigen::Matrix4f best_registrator_transform = Eigen::Matrix4f::Identity();
    float best_registrator_score = std::numeric_limits<float>::max();
    std::string best_registrator_name;

    for (std::size_t i = 0; i < transforms.size(); i++) {
      auto & transform = transforms[i];
      auto & transform_name = transforms_names[i];

      typename pcl::PointCloud<PointType>::Ptr aligned_cloud_ptr(new pcl::PointCloud<PointType>());
      registrator->align(*aligned_cloud_ptr, transform);

      Eigen::Matrix4f candidate_transform = registrator->getFinalTransformation();
      float candidate_score = registrator->getFitnessScore();
      std::string candidate_name = registrator->getClassName() + " (" + transform_name + ")";

      if (verbose) {
        std::cout << candidate_name << " score: " << candidate_score << std::endl;
      }

      if (candidate_score < best_registrator_score) {
        best_registrator_transform = candidate_transform;
        best_registrator_score = candidate_score;
        best_registrator_name = candidate_name;
      }
    }

    if (best_registrator_score < best_score) {
      best_transform = best_registrator_transform;
      best_score = best_registrator_score;
      best_name = best_registrator_name;
    }

    transforms.push_back(best_registrator_transform);
    transforms_names.push_back(best_registrator_name);
  }

  if (verbose) {
    std::cout << "Best result: " << best_name << " score: " << best_score << std::endl;
  }
}

/*!
 * Crop a target pointcloud to the ranges of a sorce one
 * @param[in] initial_source_aligned_pc_ptr Pointcloud to use as a reference to crop a target
 * pointcloud
 * @param[out] target_dense_pc_ptr Pointcloud to be cropped
 */
template <class PointType>
void cropTargetPointcloud(
  const typename pcl::PointCloud<PointType>::Ptr & initial_source_aligned_pc_ptr,
  typename pcl::PointCloud<PointType>::Ptr & target_dense_pc_ptr, float margin)
{
  // Obtain data ranges from the source
  Eigen::Array4f min_p, max_p;
  min_p.setConstant(FLT_MAX);
  max_p.setConstant(-FLT_MAX);

  for (const auto & point : *initial_source_aligned_pc_ptr) {
    pcl::Array4fMapConst pt = point.getArray4fMap();
    min_p = min_p.min(pt);
    max_p = max_p.max(pt);
  }

  Eigen::Vector4f min_vector = min_p - margin;
  Eigen::Vector4f max_vector = max_p + margin;
  min_vector.w() = 1.f;
  max_vector.w() = 1.f;

  // Apply the filter
  pcl::CropBox<PointType> boxFilter;
  boxFilter.setMin(min_vector);
  boxFilter.setMax(max_vector);
  boxFilter.setInputCloud(target_dense_pc_ptr);
  boxFilter.filter(*target_dense_pc_ptr);
}
#pragma GCC pop_options

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR_UTILS_HPP_
