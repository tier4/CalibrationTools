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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR_UTILS_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR_UTILS_HPP_

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/registration.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <memory>
#include <string>
#include <vector>

template <typename PointcloudType>
void transformPointcloud(
  const std::string & source_frame,
  const std::string & target_frame,
  typename PointcloudType::Ptr & pc_ptr,
  tf2_ros::Buffer & buffer)
{
  if (source_frame == target_frame) {
    return;
  }

  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    Eigen::Matrix4f transform = tf2::transformToEigen(buffer.lookupTransform(target_frame, source_frame, t, timeout).transform).matrix().cast<float>();

    typename PointcloudType::Ptr transformed_pc_ptr(new PointcloudType());
    pcl::transformPointCloud(*pc_ptr, *transformed_pc_ptr, transform);

    pc_ptr.swap(transformed_pc_ptr);

  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(rclcpp::get_logger("tf_buffer"), "could not get initial tf. %s", ex.what());
  }
}

pcl::PointCloud<PointType>::Ptr cropPointCloud(
  pcl::PointCloud<PointType>::Ptr & pointcloud, double max_range)
{
  pcl::PointCloud<PointType>::Ptr tmp_ptr(new pcl::PointCloud<PointType>());
  tmp_ptr->reserve(pointcloud->size());
  for (const auto & p : pointcloud->points) {

    if (std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z) < max_range) {
      tmp_ptr->points.push_back(p);
    }
  }

  return tmp_ptr;
}

#pragma GCC push_options
#pragma GCC optimize("O0")
Eigen::Matrix4f poseInterpolation(double t, double t1, double t2, Eigen::Matrix4f const& m1, Eigen::Matrix4f const& m2) {

  assert(t >= t1 && t <= t2);

  float alpha = 0.0;
  if (t2 != t1)
    alpha = (t - t1) / (t2 - t1);

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
#pragma GCC pop_options

template <class PointType>
float sourceTargetDistance(pcl::registration::CorrespondenceEstimation<PointType, PointType> & estimator)
{
  pcl::Correspondences correspondences;
  estimator.determineCorrespondences(correspondences);

  assert(correspondences.size() == source.size());

  int n_points = int(correspondences.size());
  float sum = 0;

  for (int i = 0; i < n_points; ++i) {
    //int source_id = correspondences[i].index_query;
    //int target_id = correspondences[i].index_match;
    float distance = correspondences[i].distance;
    sum += distance;
  }

  return sum / n_points;
}

template <class PointType>
void sourceTargetDistance(const pcl::PointCloud<PointType> & source,
  const Eigen::Matrix4f & transform,
  pcl::registration::CorrespondenceEstimation<PointType, PointType> & estimator)
{
  pcl::PointCloud<PointType> source_transformed;
  transformPointCloud(source, source_transformed, transform);

  sourceTargetDistance(source, estimator);
}

template <class PointType>
void findBestTransform(
  const pcl::PointCloud<PointType> & source,
  const std::vector<Eigen::Matrix4f> & input_transforms,
  std::vector< std::shared_ptr<pcl::Registration<PointType, PointType>> > & registratators,
  Eigen::Matrix4f & best_transform,
  float & best_score)
{
  std::vector<Eigen::Matrix4f> transforms = input_transforms;

  best_transform = Eigen::Matrix4f::Identity;
  best_score = std::numeric_limits<float>::max();

  for (auto & registrator : registratators) {

    Eigen::Matrix4f best_registrator_transform = Eigen::Matrix4f::Identity;
    float best_registrator_score = std::numeric_limits<float>::max();

    for (auto & transform : transforms) {
      typename pcl::PointCloud< PointType > ::Ptr aligned_cloud_ptr(new pcl::PointCloud<PointType>());
      registrator->align(*aligned_cloud_ptr, transform);

      Eigen::Matrix4f candidate_transform = registrator->getFinalTransformation();
      float candidate_score = registrator->getFitnessScore();

      if (candidate_score < best_registrator_score) {
        best_registrator_transform = candidate_transform;
        best_registrator_score = candidate_score;
      }
    }

    if (best_registrator_score < best_score) {
        best_transform = best_registrator_transform;
        best_score = best_registrator_score;
    }

    transforms.push_back(best_registrator_transform);
  }
}

template <class PointType>
void cropTargetPointcloud(
  const typename pcl::PointCloud<PointType>::Ptr & initial_source_aligned_pc_ptr,
  typename pcl::PointCloud<PointType>::Ptr & target_dense_pc_ptr,
  float margin)
{
  // Get min max from the source
  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);

  for(const auto & point : *initial_source_aligned_pc_ptr)
  {
    pcl::Array4fMapConst pt = point.getArray4fMap();
    min_p = min_p.min(pt);
    max_p = max_p.max(pt);
  }

  Eigen::Vector4f margin_vector = margin*Eigen::Vector4f::Ones();
  Eigen::Vector4f min_vector = min_p - margin;
  Eigen::Vector4f max_vector = max_p - margin;

  // Apply the filter
  pcl::CropBox<PointType> boxFilter;
  boxFilter.setMin(min_vector);
  boxFilter.setMax(max_vector);
  boxFilter.setInputCloud(target_dense_pc_ptr);
  boxFilter.filter(*target_dense_pc_ptr);
}

/*

*/
#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR_UTILS_HPP_
