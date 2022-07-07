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

#include <extrinsic_lidar_to_lidar_2d_calibrator/extrinsic_lidar_to_lidar_2d_calibrator.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>

#include <iostream>

#define UNUSED(x) (void)x;

using namespace std::chrono_literals;

LidarToLidar2DCalibrator::LidarToLidar2DCalibrator(const rclcpp::NodeOptions & options)
: Node("extrinsic_lidar_to_lidar_2d_calibrator", options),
  tf_broascaster_(this),
  got_initial_transform_(false),
  received_request_(false),
  calibration_done_(false)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  sensor_kit_frame_ = this->declare_parameter<std::string>("parent_frame");
  lidar_base_frame_ = this->declare_parameter<std::string>("child_frame");

  broacast_calibration_tf_ = this->declare_parameter<bool>("broacast_calibration_tf", false);
  min_z_ = this->declare_parameter<double>("min_z", 0.2);
  max_z_ = this->declare_parameter<double>("max_z", 0.6);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  source_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("source_points_2d", 10);
  target_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("target_points_2d", 10);

  source_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "source_input_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&LidarToLidar2DCalibrator::sourcePointCloudCallback, this, std::placeholders::_1));

  target_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "target_input_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&LidarToLidar2DCalibrator::targetPointCloudCallback, this, std::placeholders::_1));

  calib_timer_ = rclcpp::create_timer(
    this, get_clock(), 200ms, std::bind(&LidarToLidar2DCalibrator::calibrationimerCallback, this));

  // The service server runs in a dedicated thread
  srv_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  service_server_ = this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
    "extrinsic_calibration",
    std::bind(
      &LidarToLidar2DCalibrator::requestReceivedCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, srv_callback_group_);
}

void LidarToLidar2DCalibrator::requestReceivedCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  // This tool uses several tfs, so for consistency we take the initial calibration using lookups
  UNUSED(request);

  {
    std::unique_lock<std::mutex> lock(mutex_);
    received_request_ = true;
    initial_calibration_ = request->initial_pose;
  }

  // Loop until the calibration finishes
  while (rclcpp::ok()) {
    rclcpp::sleep_for(1s);
    std::unique_lock<std::mutex> lock(mutex_);

    if (calibration_done_) {
      break;
    }

    RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Waiting for the calibration to end");
  }

  response->success = true;
  response->result_pose = output_calibration_msg_;
}

void LidarToLidar2DCalibrator::sourcePointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  source_pointcloud_frame_ = msg->header.frame_id;
  source_pointcloud_header_ = msg->header;
  source_pointcloud_msg_ = msg;
}

void LidarToLidar2DCalibrator::targetPointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  target_pointcloud_frame_ = msg->header.frame_id;
  target_pointcloud_header_ = msg->header;
  target_pointcloud_msg_ = msg;
}

bool LidarToLidar2DCalibrator::checkInitialTransforms()
{
  if (source_pointcloud_frame_ == "" || target_pointcloud_frame_ == "") {
    return false;
  }

  if (got_initial_transform_) {
    return true;
  }

  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    initial_base_to_source_msg_ =
      tf_buffer_->lookupTransform(base_frame_, source_pointcloud_frame_, t, timeout).transform;

    initial_base_to_target_msg_ =
      tf_buffer_->lookupTransform(base_frame_, target_pointcloud_frame_, t, timeout).transform;

    fromMsg(initial_base_to_source_msg_, initial_base_to_source_tf2_);
    fromMsg(initial_base_to_target_msg_, initial_base_to_target_tf2_);
    initial_base_to_source_eigen_ = tf2::transformToEigen(initial_base_to_source_msg_);
    initial_base_to_target_eigen_ = tf2::transformToEigen(initial_base_to_target_msg_);

    base_to_sensor_kit_msg_ =
      tf_buffer_->lookupTransform(base_frame_, sensor_kit_frame_, t, timeout).transform;

    fromMsg(base_to_sensor_kit_msg_, base_to_sensor_kit_tf2_);
    base_to_sensor_kit_eigen_ = tf2::transformToEigen(base_to_sensor_kit_msg_);

    got_initial_transform_ = true;

  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "could not get initial tf. %s", ex.what());
    return false;
  }

  return true;
}

void LidarToLidar2DCalibrator::calibrationimerCallback()
{
  // Make sure we have all the required initial tfs
  if (!checkInitialTransforms()) {
    return;
  }

  // lidar coordinates
  pcl::PointCloud<PointType>::Ptr raw_source_pointcloud_scs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr raw_target_pointcloud_tcs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr source_pointcloud_scs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr target_pointcloud_tcs(new pcl::PointCloud<PointType>);

  // base coordinates
  pcl::PointCloud<PointType>::Ptr raw_source_pointcloud_bcs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr raw_target_pointcloud_bcs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr source_pointcloud_bcs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr target_pointcloud_bcs(new pcl::PointCloud<PointType>);

  // kit coordinate
  pcl::PointCloud<PointType>::Ptr source_pointcloud_kcs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr target_pointcloud_kcs(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*source_pointcloud_msg_, *raw_source_pointcloud_scs);
  pcl::fromROSMsg(*target_pointcloud_msg_, *raw_target_pointcloud_tcs);

  // Check input pointclouds and frames
  // Turn them into base coordinates

  // Eigen::Matrix4f asd = initial_base_to_source_eigen_;
  pcl::transformPointCloud(
    *raw_source_pointcloud_scs, *raw_source_pointcloud_bcs, initial_base_to_source_eigen_);
  pcl::transformPointCloud(
    *raw_target_pointcloud_tcs, *raw_target_pointcloud_bcs, initial_base_to_target_eigen_);

  // Segment points in a range of z
  for (auto & point : raw_source_pointcloud_bcs->points) {
    if (point.z >= min_z_ && point.z <= max_z_) {
      source_pointcloud_bcs->push_back(point);
    }
  }

  for (auto & point : raw_target_pointcloud_bcs->points) {
    if (point.z >= min_z_ && point.z <= max_z_) {
      target_pointcloud_bcs->push_back(point);
    }
  }

  pcl::transformPointCloud(
    *source_pointcloud_bcs, *source_pointcloud_scs, initial_base_to_source_eigen_.inverse());
  pcl::transformPointCloud(
    *target_pointcloud_bcs, *target_pointcloud_tcs, initial_base_to_target_eigen_.inverse());

  RCLCPP_WARN(this->get_logger(), "ICP Source points: %ld", source_pointcloud_bcs->size());
  RCLCPP_WARN(this->get_logger(), "ICP Target points: %ld", target_pointcloud_bcs->size());

  // Turn them into kit coordinates
  pcl::transformPointCloud(
    *source_pointcloud_bcs, *source_pointcloud_kcs, base_to_sensor_kit_eigen_.inverse());
  pcl::transformPointCloud(
    *target_pointcloud_bcs, *target_pointcloud_kcs, base_to_sensor_kit_eigen_.inverse());

  // Discard the z component
  for (auto & point : source_pointcloud_kcs->points) {
    point.z = 0.f;
  }

  for (auto & point : target_pointcloud_kcs->points) {
    point.z = 0.f;
  }

  // Do 2d ICP. Consider using nlopt and or filtering to improve the results
  pcl::PointCloud<PointType>::Ptr source_pointcloud_aligned_kcs(new pcl::PointCloud<PointType>);
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setInputSource(source_pointcloud_kcs);
  icp.setInputTarget(target_pointcloud_kcs);
  icp.setMaxCorrespondenceDistance(0.5);
  icp.setMaximumIterations(200);
  icp.setTransformationEpsilon(0.0);
  icp.align(*source_pointcloud_aligned_kcs);

  if (!icp.hasConverged()) {
    RCLCPP_WARN(this->get_logger(), "ICP did not converge");
    return;
  }

  Eigen::Matrix4f source_transform_transform = icp.getFinalTransformation();

  double init_error = getAlignmentError(source_pointcloud_kcs, target_pointcloud_kcs);
  double final_error = getAlignmentError(source_pointcloud_aligned_kcs, target_pointcloud_kcs);
  double icp_error = icp.getFitnessScore();

  RCLCPP_WARN(this->get_logger(), "init_error: %.2f", init_error);
  RCLCPP_WARN(this->get_logger(), "final_error: %.2f", final_error);
  RCLCPP_WARN(this->get_logger(), "icp_error: %.2f", icp_error);

  Eigen::Affine3d icp_affine = Eigen::Affine3d(source_transform_transform.cast<double>());
  Eigen::Affine3d inital_kit_to_source_eigen =
    base_to_sensor_kit_eigen_.inverse() * initial_base_to_source_eigen_;

  Eigen::Affine3d estimated_kit_to_source_eigen = inital_kit_to_source_eigen * icp_affine;

  geometry_msgs::msg::TransformStamped tf_msg =
    tf2::eigenToTransform(estimated_kit_to_source_eigen);
  tf_msg.header.stamp = source_pointcloud_header_.stamp;
  tf_msg.header.frame_id = sensor_kit_frame_;
  tf_msg.child_frame_id = source_pointcloud_frame_;
  tf_broascaster_.sendTransform(tf_msg);

  // Publish the segmented pointclouds back in their frames (to evaluate visually the calibration)
  sensor_msgs::msg::PointCloud2 source_pointcloud_scs_msg, target_pointcloud_tcs_msg;
  pcl::toROSMsg(*source_pointcloud_scs, source_pointcloud_scs_msg);
  pcl::toROSMsg(*target_pointcloud_tcs, target_pointcloud_tcs_msg);
  source_pointcloud_scs_msg.header = source_pointcloud_header_;
  target_pointcloud_tcs_msg.header = target_pointcloud_header_;
  source_pub_->publish(source_pointcloud_scs_msg);
  target_pub_->publish(target_pointcloud_tcs_msg);
}

double LidarToLidar2DCalibrator::getAlignmentError(
  pcl::PointCloud<PointType>::Ptr source_pointcloud,
  pcl::PointCloud<PointType>::Ptr target_pointcloud)
{
  pcl::Correspondences correspondences;
  pcl::registration::CorrespondenceEstimation<PointType, PointType> correspondance_estimator;
  correspondance_estimator.setInputSource(source_pointcloud);
  correspondance_estimator.setInputTarget(target_pointcloud);
  correspondance_estimator.determineCorrespondences(correspondences);

  double error = 0.0;

  for (unsigned long i = 0; i < correspondences.size(); ++i) {
    // int source_id = correspondences[i].index_query;
    // int target_id = correspondences[i].index_match;
    error += std::sqrt(correspondences[i].distance);
  }

  return error / correspondences.size();
}
