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

#include <extrinsic_mapping_based_calibrator/base_lidar_calibrator.hpp>
#include <extrinsic_mapping_based_calibrator/utils.hpp>
#include <extrinsic_mapping_based_calibrator/voxel_grid_filter_wrapper.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tier4_ground_plane_utils/ground_plane_utils.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

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

std::tuple<bool, Eigen::Matrix4d, float> BaseLidarCalibrator::calibrate()
{
  auto & last_keyframe = data_->keyframes_and_stopped_.back();
  PointcloudType::Ptr augmented_pointcloud_ptr = getDensePointcloudFromMap(
    last_keyframe->pose_, last_keyframe, parameters_->leaf_size_dense_map_,
    parameters_->min_calibration_range_, parameters_->max_calibration_range_);

  Eigen::Isometry3d initial_base_to_lidar_transform;

  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    initial_base_to_lidar_transform = tf2::transformToEigen(
      tf_buffer_->lookupTransform(parameters_->base_frame_, data_->mapping_lidar_frame_, t, timeout)
        .transform);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(rclcpp::get_logger(calibrator_name_), "could not get initial tf. %s", ex.what());

    return std::make_tuple<>(false, Eigen::Matrix4d::Identity(), 0.f);
  }

  PointcloudType::Ptr augmented_pointcloud_base_ptr(new PointcloudType());
  pcl::transformPointCloud(
    *augmented_pointcloud_ptr, *augmented_pointcloud_base_ptr,
    initial_base_to_lidar_transform.cast<float>());

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
    *augmented_pointcloud_base_ptr, *augmented_pointcloud_ptr,
    initial_base_to_lidar_transform.inverse().cast<float>());

  tier4_ground_plane_utils::GroundPlaneExtractorParameters parameters;
  parameters.verbose_ = true;
  parameters.use_crop_box_filter_ = false;
  parameters.use_pca_rough_normal_ = false;
  parameters.max_inlier_distance_ = parameters_->base_lidar_max_inlier_distance_;
  parameters.min_plane_points_ = parameters_->base_lidar_min_plane_points_;
  parameters.min_plane_points_percentage_ = parameters_->base_lidar_min_plane_points_percentage_;
  parameters.max_cos_distance_ = parameters_->base_lidar_max_cos_distance_;
  parameters.max_iterations_ = parameters_->base_lidar_max_iterations_;
  parameters.remove_outliers_ = false;
  parameters.initial_base_to_lidar_transform_ = initial_base_to_lidar_transform;
  std::vector<Eigen::Vector4d> outlier_models;

  auto [status, ground_plane_model, ground_plane_inliers_ptr] =
    tier4_ground_plane_utils::extractGroundPlane(
      augmented_pointcloud_ptr, parameters, outlier_models);

  if (!status) {
    RCLCPP_WARN(
      rclcpp::get_logger(calibrator_name_), "Base calibration failed because not plane was found");
    return std::make_tuple<>(false, Eigen::Matrix4d::Identity(), 0.f);
  }

  publishResults(
    ground_plane_model, last_keyframe->pose_, ground_plane_inliers_ptr, augmented_pointcloud_ptr);

  Eigen::Isometry3d calibrated_base_to_lidar_transform =
    tier4_ground_plane_utils::estimateBaseLidarTransform(
      initial_base_to_lidar_transform, ground_plane_model);

  if (parameters_->base_lidar_overwrite_xy_yaw_) {
    geometry_msgs::msg::TransformStamped initial_base_to_lidar_transform_msg_ =
      tf2::eigenToTransform(initial_base_to_lidar_transform);
    geometry_msgs::msg::TransformStamped calibrated_base_to_lidar_transform_msg =
      tf2::eigenToTransform(calibrated_base_to_lidar_transform);

    calibrated_base_to_lidar_transform_msg = tier4_ground_plane_utils::overwriteXYYawValues(
      initial_base_to_lidar_transform_msg_, calibrated_base_to_lidar_transform_msg);

    calibrated_base_to_lidar_transform =
      tf2::transformToEigen(calibrated_base_to_lidar_transform_msg);
  }

  // Other calibrators look for lidar -> calibration_frame, so we follow suit
  return std::make_tuple<>(true, calibrated_base_to_lidar_transform.inverse().matrix(), 0.f);
}

void BaseLidarCalibrator::publishResults(
  const Eigen::Vector4d & ground_model, const Eigen::Matrix4f & pose,
  const pcl::PointCloud<PointType>::Ptr & ground_plane_inliers_lcs_ptr,
  const pcl::PointCloud<PointType>::Ptr & augmented_pointcloud_lcs_ptr)
{
  PointcloudType::Ptr ground_plane_inliers_mcs_ptr(new PointcloudType());
  PointcloudType::Ptr augmented_pointcloud_mcs_ptr(new PointcloudType());

  pcl::transformPointCloud(*ground_plane_inliers_lcs_ptr, *ground_plane_inliers_mcs_ptr, pose);
  pcl::transformPointCloud(*augmented_pointcloud_lcs_ptr, *augmented_pointcloud_mcs_ptr, pose);

  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "Estimated model. a=%f, b=%f, c=%f, d=%f",
    ground_model.x(), ground_model.y(), ground_model.z(), ground_model.w());

  sensor_msgs::msg::PointCloud2 ground_plane_inliers_msg, augmented_pointcloud_msg;
  ground_plane_inliers_mcs_ptr->width = ground_plane_inliers_mcs_ptr->points.size();
  ground_plane_inliers_mcs_ptr->height = 1;
  augmented_pointcloud_mcs_ptr->width = augmented_pointcloud_mcs_ptr->points.size();
  augmented_pointcloud_mcs_ptr->height = 1;

  pcl::toROSMsg(*ground_plane_inliers_mcs_ptr, ground_plane_inliers_msg);
  pcl::toROSMsg(*augmented_pointcloud_mcs_ptr, augmented_pointcloud_msg);

  ground_plane_inliers_msg.header.frame_id = data_->map_frame_;
  augmented_pointcloud_msg.header.frame_id = data_->map_frame_;

  augmented_pointcloud_pub_->publish(augmented_pointcloud_msg);
  ground_pointcloud_pub_->publish(ground_plane_inliers_msg);

  Eigen::Isometry3d estimated_ground_pose =
    tier4_ground_plane_utils::modelPlaneToPose(ground_model);
  auto estimated_ground_msg = tf2::eigenToTransform(estimated_ground_pose);
  estimated_ground_msg.header.frame_id = data_->mapping_lidar_frame_;
  estimated_ground_msg.child_frame_id = "estimated_ground_pose";
  tf_broadcaster_.sendTransform(estimated_ground_msg);
}
