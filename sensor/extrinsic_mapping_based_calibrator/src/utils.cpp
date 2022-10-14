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

#include <Eigen/Core>
#include <extrinsic_mapping_based_calibrator/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_pcl_extensions/joint_icp_extended.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/buffer.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <limits>

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

template <typename PointcloudType>
typename PointcloudType::Ptr cropPointCloud(
  const typename PointcloudType::Ptr & pointcloud, double max_range)
{
  typename PointcloudType::Ptr tmp_ptr(new PointcloudType());
  tmp_ptr->reserve(pointcloud->size());
  for (const auto & p : pointcloud->points) {
    if (std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) < max_range) {
      tmp_ptr->points.push_back(p);
    }
  }

  tmp_ptr->width = tmp_ptr->points.size();
  tmp_ptr->height = 1;

  return tmp_ptr;
}

Eigen::Matrix4f poseInterpolationBase(
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

Eigen::Matrix4f poseInterpolation(
  double t, double t1, double t2, Eigen::Matrix4f const & m1, Eigen::Matrix4f const & m2)
{
  assert(t1 < t2);

  if (t >= t1 && t <= t2) {
    return poseInterpolationBase(t, t1, t2, m1, m2);
  }

  double dt = t2 - t1;
  double te = t - t2;
  Eigen::Matrix4f m = m2;
  Eigen::Matrix4f dm = m1.inverse() * m2;

  while (te >= dt) {
    m = m * dm;
    te -= dt;
  }

  auto asd = poseInterpolationBase(te, 0, dt, Eigen::Matrix4f::Identity(), dm);

  return m * asd;
}

template <class PointType>
float sourceTargetDistance(
  pcl::registration::CorrespondenceEstimation<PointType, PointType> & estimator,
  float max_corr_distance)
{
  pcl::Correspondences correspondences;
  estimator.determineCorrespondences(correspondences, max_corr_distance);

  int n_points = static_cast<int>(correspondences.size());
  float sum = 0;

  for (int i = 0; i < n_points; ++i) {
    float distance = correspondences[i].distance;
    sum += distance;
  }

  return sum / n_points;
}

template <class PointType>
float sourceTargetDistance(
  const typename pcl::PointCloud<PointType>::Ptr & source,
  const typename pcl::PointCloud<PointType>::Ptr & target, const Eigen::Matrix4f & transform,
  float max_corr_distance)
{
  typename pcl::PointCloud<PointType>::Ptr source_aligned(new pcl::PointCloud<PointType>());
  transformPointCloud(*source, *source_aligned, transform);

  pcl::registration::CorrespondenceEstimation<PointType, PointType> estimator;
  estimator.setInputSource(source_aligned);
  estimator.setInputTarget(target);

  return sourceTargetDistance(estimator, max_corr_distance);
}

template <class PointType>
float sourceTargetDistance(
  const std::vector<typename pcl::PointCloud<PointType>::Ptr> & sources,
  const std::vector<typename pcl::PointCloud<PointType>::Ptr> & targets,
  const Eigen::Matrix4f & transform, float max_corr_distance)
{
  double distance = 0.0;
  int n = 0;

  assert(sources.size() == targets.size());

  for (std::size_t i = 0; i < sources.size(); i++) {
    typename pcl::PointCloud<PointType>::Ptr source_aligned(new
                                                            typename pcl::PointCloud<PointType>());
    transformPointCloud(*sources[i], *source_aligned, transform);

    pcl::registration::CorrespondenceEstimation<PointType, PointType> estimator;
    estimator.setInputSource(source_aligned);
    estimator.setInputTarget(targets[i]);

    n += sources[i]->size();
    distance += sources[i]->size() * sourceTargetDistance(estimator, max_corr_distance);
  }

  return distance / n;
}

template <class RegistratorPtrType, class PointType>
void findBestTransform(
  const std::vector<Eigen::Matrix4f> & input_transforms,
  std::vector<typename RegistratorPtrType::Ptr> & registrators, float eval_max_corr_distance,
  bool verbose, Eigen::Matrix4f & best_transform, float & best_score)
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
      float candidate_score =
        registrator->getFitnessScore(eval_max_corr_distance * eval_max_corr_distance);
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

template <class PointType>
void cropTargetPointcloud(
  const typename pcl::PointCloud<PointType>::Ptr & initial_source_aligned_pc_ptr,
  typename pcl::PointCloud<PointType>::Ptr & target_dense_pc_ptr, float margin)
{
  // Obtain data ranges from the source
  Eigen::Array4f min_p, max_p;
  min_p.setConstant(std::numeric_limits<float>::max());
  max_p.setConstant(-std::numeric_limits<float>::max());

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

template void transformPointcloud<PointcloudType>(
  const std::string & source_frame, const std::string & target_frame,
  typename PointcloudType::Ptr & pc_ptr, tf2_ros::Buffer & buffer);

template float sourceTargetDistance<PointType>(
  pcl::registration::CorrespondenceEstimation<PointType, PointType> & estimator,
  float max_corr_distance);

template float sourceTargetDistance<PointType>(
  const pcl::PointCloud<PointType>::Ptr & source, const pcl::PointCloud<PointType>::Ptr & target,
  const Eigen::Matrix4f & transform, float max_corr_distance);

template float sourceTargetDistance<PointType>(
  const std::vector<typename pcl::PointCloud<PointType>::Ptr> & sources,
  const std::vector<typename pcl::PointCloud<PointType>::Ptr> & targets,
  const Eigen::Matrix4f & transform, float max_corr_distance);

template PointcloudType::Ptr cropPointCloud<PointcloudType>(
  const PointcloudType::Ptr & pointcloud, double max_range);

template void findBestTransform<pcl::Registration<PointType, PointType>, PointType>(
  const std::vector<Eigen::Matrix4f> & input_transforms,
  std::vector<pcl::Registration<PointType, PointType>::Ptr> & registrators,
  float eval_max_corr_distance, bool verbose, Eigen::Matrix4f & best_transform, float & best_score);

template void
findBestTransform<pcl::JointIterativeClosestPointExtended<PointType, PointType>, PointType>(
  const std::vector<Eigen::Matrix4f> & input_transforms,
  std::vector<pcl::JointIterativeClosestPointExtended<PointType, PointType>::Ptr> & registrators,
  float eval_max_corr_distance, bool verbose, Eigen::Matrix4f & best_transform, float & best_score);

template void cropTargetPointcloud<PointType>(
  const pcl::PointCloud<PointType>::Ptr & initial_source_aligned_pc_ptr,
  pcl::PointCloud<PointType>::Ptr & target_dense_pc_ptr, float margin);
