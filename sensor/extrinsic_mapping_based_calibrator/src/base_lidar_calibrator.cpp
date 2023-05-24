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

#include <extrinsic_mapping_based_calibrator/base_lidar_calibrator.hpp>
#include <extrinsic_mapping_based_calibrator/utils.hpp>
#include <extrinsic_mapping_based_calibrator/voxel_grid_filter_wrapper.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#define UNUSED(x) (void)x;

BaseLidarCalibrator::BaseLidarCalibrator(
  CalibrationParameters::Ptr & parameters, MappingData::Ptr & mapping_data,
  std::shared_ptr<tf2_ros::Buffer> & tf_buffer, tf2_ros::StaticTransformBroadcaster & broadcaster,
  PointPublisher::SharedPtr & augmented_pointcloud_pub,
  PointPublisher::SharedPtr & ground_pointcloud_pub)
: SensorCalibrator(
    mapping_data->mapping_lidar_frame_,
    "base_lidar_calibrator(" + mapping_data->mapping_lidar_frame_ + ")", parameters, mapping_data,
    tf_buffer),
  tf_broadcaster_(broadcaster),
  augmented_pointcloud_pub_(augmented_pointcloud_pub),
  ground_pointcloud_pub_(ground_pointcloud_pub)
{
}

void BaseLidarCalibrator::calibrationCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response)
{
  UNUSED(request);

  Eigen::Matrix4f result;
  float score;
  response->success = calibrate(result, score);
}

bool BaseLidarCalibrator::calibrate(
  __attribute__((unused)) Eigen::Matrix4f & base_link_transform,
  __attribute__((unused)) float & best_score)
{
  auto & last_keyframe = data_->keyframes_and_stopped_.back();
  PointcloudType::Ptr augmented_pointcloud_ptr = getDensePointcloudFromMap(
    last_keyframe->pose_, last_keyframe, parameters_->leaf_size_dense_map_,
    parameters_->max_calibration_range_);

  Eigen::Matrix4f initial_lidar_to_base_transform;
  Eigen::Isometry3d initial_lidar_to_base_affine;

  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    initial_lidar_to_base_affine = tf2::transformToEigen(
      tf_buffer_->lookupTransform(data_->mapping_lidar_frame_, "base_link", t, timeout).transform);

    initial_lidar_to_base_transform = initial_lidar_to_base_affine.matrix().cast<float>();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(rclcpp::get_logger(calibrator_name_), "could not get initial tf. %s", ex.what());

    return false;
  }

  Eigen::Vector3f estimated_normal =
    initial_lidar_to_base_affine.rotation().cast<float>() * Eigen::Vector3f(0.f, 0.f, 1.f);

  PointcloudType::Ptr augmented_pointcloud_base_ptr(new PointcloudType());
  pcl::transformPointCloud(
    *augmented_pointcloud_ptr, *augmented_pointcloud_base_ptr,
    initial_lidar_to_base_transform.inverse());

  pcl::CropBox<PointType> box_filter;
  box_filter.setMin(Eigen::Vector4f(
    parameters_->base_lidar_crop_box_min_x_, parameters_->base_lidar_crop_box_min_y_,
    parameters_->base_lidar_crop_box_min_z_, 1.0));
  box_filter.setMax(Eigen::Vector4f(
    parameters_->base_lidar_crop_box_max_x_, parameters_->base_lidar_crop_box_max_y_,
    parameters_->base_lidar_crop_box_max_z_, 1.0));
  box_filter.setInputCloud(augmented_pointcloud_base_ptr);
  box_filter.filter(*augmented_pointcloud_base_ptr);

  pcl::transformPointCloud(
    *augmented_pointcloud_base_ptr, *augmented_pointcloud_ptr, initial_lidar_to_base_transform);

  PointcloudType::Ptr ground_plane_inliers_ptr(new PointcloudType());
  Eigen::Vector4d ground_plane_model;
  extractGroundPlane(
    augmented_pointcloud_ptr, estimated_normal, ground_plane_model, ground_plane_inliers_ptr);

  publishResults(ground_plane_model, ground_plane_inliers_ptr, augmented_pointcloud_ptr);

  return true;
}

bool BaseLidarCalibrator::extractGroundPlane(
  pcl::PointCloud<PointType>::Ptr & pointcloud, const Eigen::Vector3f & initial_normal,
  Eigen::Vector4d & model, pcl::PointCloud<PointType>::Ptr & inliers_pointcloud)
{
  std::vector<pcl::ModelCoefficients> models;

  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "Rough plane normal. x=%.3f, y=%.3f, z=%.3f",
    initial_normal.x(), initial_normal.y(), initial_normal.z());

  // Use RANSAC iteratively until we find the ground plane
  // Since walls can have more points, we filter using the PCA-based hypothesis
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointType> seg;
  pcl::ExtractIndices<PointType> extract;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(parameters_->base_lidar_max_inlier_distance_);
  seg.setMaxIterations(parameters_->base_lidar_max_iterations_);

  pcl::PointCloud<PointType>::Ptr iteration_cloud(new pcl::PointCloud<PointType>());
  pcl::copyPointCloud(*pointcloud, *iteration_cloud);
  int iteration_size = iteration_cloud->height * iteration_cloud->width;

  while (iteration_size > parameters_->base_lidar_min_plane_points_) {
    seg.setInputCloud(iteration_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
      RCLCPP_WARN(rclcpp::get_logger(calibrator_name_), "No plane found in the pointcloud");
      break;
    }

    Eigen::Vector3f normal(
      coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    float cos_distance = 1.0 - std::abs(initial_normal.dot(normal));

    model = Eigen::Vector4d(
      coefficients->values[0], coefficients->values[1], coefficients->values[2],
      coefficients->values[3]);

    int inlier_size = static_cast<int>(inliers->indices.size());
    double inlier_percentage = 100.0 * inlier_size / pointcloud->size();

    if (
      inlier_size > parameters_->base_lidar_min_plane_points_ &&
      inlier_percentage > parameters_->base_lidar_min_plane_points_percentage_ &&
      cos_distance < parameters_->base_lidar_max_cos_distance_) {
      RCLCPP_INFO(
        rclcpp::get_logger(calibrator_name_), "Plane found: inliers=%ld (%.3f)",
        inliers->indices.size(), inlier_percentage);
      RCLCPP_INFO(
        rclcpp::get_logger(calibrator_name_), "Plane model. a=%.3f, b=%.3f, c=%.3f, d=%.3f",
        model(0), model(1), model(2), model(3));
      RCLCPP_INFO(
        rclcpp::get_logger(calibrator_name_), "Cos distance: %.3f / %.3f", cos_distance,
        parameters_->base_lidar_max_cos_distance_);

      // Extract the ground plane inliers
      extract.setInputCloud(iteration_cloud);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*inliers_pointcloud);
      return true;
    }

    // Extract the inliers from the pointcloud (the detected plane was not the ground plane)
    extract.setInputCloud(iteration_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);

    pcl::PointCloud<PointType> next_cloud;
    extract.filter(next_cloud);

    iteration_cloud->swap(next_cloud);
    iteration_size = iteration_cloud->height * iteration_cloud->width;
  }

  return false;
}

void BaseLidarCalibrator::publishResults(
  const Eigen::Vector4d & ground_model,
  const pcl::PointCloud<PointType>::Ptr & ground_plane_inliers_ptr,
  const pcl::PointCloud<PointType>::Ptr & augmented_pointcloud_ptr)
{
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "Estimated model. a=%f, b=%f, c=%f, d=%f",
    ground_model.x(), ground_model.y(), ground_model.z(), ground_model.w());

  sensor_msgs::msg::PointCloud2 ground_plane_inliers_msg, augmented_pointcloud_msg;
  ground_plane_inliers_ptr->width = ground_plane_inliers_ptr->points.size();
  ground_plane_inliers_ptr->height = 1;
  augmented_pointcloud_ptr->width = augmented_pointcloud_ptr->points.size();
  augmented_pointcloud_ptr->height = 1;

  pcl::toROSMsg(*ground_plane_inliers_ptr, ground_plane_inliers_msg);
  pcl::toROSMsg(*augmented_pointcloud_ptr, augmented_pointcloud_msg);

  ground_plane_inliers_msg.header.frame_id = data_->mapping_lidar_frame_;
  augmented_pointcloud_msg.header.frame_id = data_->mapping_lidar_frame_;

  augmented_pointcloud_pub_->publish(augmented_pointcloud_msg);
  ground_pointcloud_pub_->publish(ground_plane_inliers_msg);

  Eigen::Isometry3d estimated_ground_pose = modelPlaneToPose(ground_model);
  auto estimated_ground_msg = tf2::eigenToTransform(estimated_ground_pose);
  estimated_ground_msg.header.frame_id = data_->mapping_lidar_frame_;
  estimated_ground_msg.child_frame_id = "estimated_ground_pose";
  tf_broadcaster_.sendTransform(estimated_ground_msg);
}

Eigen::Isometry3d BaseLidarCalibrator::modelPlaneToPose(const Eigen::Vector4d & model) const
{
  Eigen::Vector3d n(model(0), model(1), model(2));
  n.normalize();

  Eigen::Vector3d x0 = -n * model(3);

  // To create a real pose we need to invent a basis
  Eigen::Vector3d base_x, base_y, base_z;
  base_z = n;

  Eigen::Vector3d c1 = Eigen::Vector3d(1.0, 0.0, 0.0).cross(n);
  Eigen::Vector3d c2 = Eigen::Vector3d(0.0, 1.0, 0.0).cross(n);
  Eigen::Vector3d c3 = Eigen::Vector3d(0.0, 0.0, 1.0).cross(n);

  // Any non-zero would work but we use the one with the highest norm (there has to be a non zero)
  if (c1.norm() > c2.norm() && c1.norm() > c3.norm()) {
    base_x = c1;
  } else if (c2.norm() > c3.norm()) {
    base_x = c2;
  } else {
    base_x = c3;
  }

  base_y = base_z.cross(base_x);

  Eigen::Matrix3d rot;
  rot.col(0) = base_x.normalized();
  rot.col(1) = base_y.normalized();
  rot.col(2) = base_z.normalized();

  Eigen::Isometry3d pose;
  pose.translation() = x0;
  pose.linear() = rot;

  return pose;
}
