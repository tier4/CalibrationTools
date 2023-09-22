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

#include <extrinsic_lidar_to_lidar_2d_calibrator/extrinsic_lidar_to_lidar_2d_calibrator.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

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

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <iostream>

#define UNUSED(x) (void)x;

LidarToLidar2DCalibrator::LidarToLidar2DCalibrator(const rclcpp::NodeOptions & options)
: Node("extrinsic_lidar_to_lidar_2d_calibrator", options),
  tf_broadcaster_(this),
  got_initial_transform_(false),
  received_request_(false),
  calibration_done_(false),
  first_observation_(true)
{
  using std::chrono_literals::operator""ms;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  sensor_kit_frame_ = this->declare_parameter<std::string>("parent_frame");
  lidar_base_frame_ = this->declare_parameter<std::string>("child_frame");

  broadcast_calibration_tf_ = this->declare_parameter<bool>("broadcast_calibration_tf", false);
  filter_estimations_ = this->declare_parameter<bool>("filter_estimations", true);

  max_calibration_range_ = this->declare_parameter<double>("max_calibration_range", 20.0);
  max_corr_distance_ = this->declare_parameter<double>("max_corr_distance", 0.5);
  max_iterations_ = this->declare_parameter<int>("max_iterations", 100);

  initial_angle_cov_ = this->declare_parameter<double>("initial_angle_cov", 5.0);
  initial_xy_cov_ = this->declare_parameter<double>("initial_xy_cov", 0.05);

  angle_measurement_cov_ = this->declare_parameter<double>("angle_measurement_cov", 0.5);
  angle_process_cov_ = this->declare_parameter<double>("angle_process_cov", 0.1);
  xy_measurement_cov_ = this->declare_parameter<double>("xy_measurement_cov", 0.005);
  xy_process_cov_ = this->declare_parameter<double>("xy_process_cov", 0.001);

  angle_convergence_threshold_ = this->declare_parameter<float>("angle_convergence_threshold", 0.0);
  xy_convergence_threshold_ = this->declare_parameter<float>("xy_convergence_threshold", 0.0);

  initial_angle_cov_ = std::pow(initial_angle_cov_ * M_PI_2 / 180.0, 2);
  initial_xy_cov_ = std::pow(initial_xy_cov_, 2);

  angle_measurement_cov_ = std::pow(angle_measurement_cov_ * M_PI_2 / 180.0, 2);
  angle_process_cov_ = std::pow(angle_process_cov_ * M_PI_2 / 180.0, 2);
  xy_measurement_cov_ = std::pow(xy_measurement_cov_, 2);
  xy_process_cov_ = std::pow(xy_process_cov_, 2);

  angle_convergence_threshold_ = std::pow(angle_convergence_threshold_ * M_PI_2 / 180.0, 2);
  xy_convergence_threshold_ = std::pow(xy_convergence_threshold_, 2);

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
    this, get_clock(), 200ms, std::bind(&LidarToLidar2DCalibrator::calibrationTimerCallback, this));

  // The service server runs in a dedicated thread
  srv_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  service_server_ = this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
    "extrinsic_calibration",
    std::bind(
      &LidarToLidar2DCalibrator::requestReceivedCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, srv_callback_group_);

  // Initialize the filter
  kalman_filter_.setA(Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, 1.0));
  kalman_filter_.setB(Eigen::DiagonalMatrix<double, 3>(0.0, 0.0, 0.0));
  kalman_filter_.setC(Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, 1.0));
  kalman_filter_.setQ(Eigen::DiagonalMatrix<double, 3>(
    angle_measurement_cov_, xy_measurement_cov_, xy_measurement_cov_));
  kalman_filter_.setR(
    Eigen::DiagonalMatrix<double, 3>(angle_process_cov_, xy_process_cov_, xy_process_cov_));

  calibration_gicp_ =
    pcl::make_shared<pcl::GeneralizedIterativeClosestPoint<PointType, PointType>>();
  calibration_icp_ = pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();
  calibration_icp_fine_ = pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();
  calibration_icp_fine_2_ = pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();
  calibration_icp_fine_3_ = pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();
  calibration_registrators_ = {
    calibration_gicp_, calibration_icp_, calibration_icp_fine_, calibration_icp_fine_2_,
    calibration_icp_fine_3_};

  for (auto & calibrator : calibration_registrators_) {
    calibrator->setMaximumIterations(max_iterations_);
    calibrator->setMaxCorrespondenceDistance(max_corr_distance_);
  }

  calibration_icp_fine_->setMaxCorrespondenceDistance(0.2 * max_corr_distance_);
  calibration_icp_fine_2_->setMaxCorrespondenceDistance(0.1 * max_corr_distance_);
  calibration_icp_fine_3_->setMaxCorrespondenceDistance(0.05 * max_corr_distance_);
}

void LidarToLidar2DCalibrator::requestReceivedCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  // This tool uses several tfs, so for consistency we take the initial calibration using lookups
  UNUSED(request);
  using std::chrono_literals::operator""s;

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

    lidar_base_to_source_msg_ =
      tf_buffer_->lookupTransform(lidar_base_frame_, source_pointcloud_frame_, t, timeout)
        .transform;

    fromMsg(base_to_sensor_kit_msg_, base_to_sensor_kit_tf2_);
    base_to_sensor_kit_eigen_ = tf2::transformToEigen(base_to_sensor_kit_msg_);

    fromMsg(lidar_base_to_source_msg_, lidar_base_to_source_tf2_);
    lidar_base_to_source_eigen_ = tf2::transformToEigen(lidar_base_to_source_msg_);

    got_initial_transform_ = true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "could not get initial tf. %s", ex.what());
    return false;
  }

  return true;
}

void LidarToLidar2DCalibrator::calibrationTimerCallback()
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
    if (
      point.z >= min_z_ && point.z <= max_z_ &&
      std::sqrt(point.x * point.x + point.y * point.y) <= max_calibration_range_) {
      source_pointcloud_bcs->push_back(point);
    }
  }

  for (auto & point : raw_target_pointcloud_bcs->points) {
    if (
      point.z >= min_z_ && point.z <= max_z_ &&
      std::sqrt(point.x * point.x + point.y * point.y) <= max_calibration_range_) {
      target_pointcloud_bcs->push_back(point);
    }
  }

  pcl::transformPointCloud(
    *source_pointcloud_bcs, *source_pointcloud_scs, initial_base_to_source_eigen_.inverse());
  pcl::transformPointCloud(
    *target_pointcloud_bcs, *target_pointcloud_tcs, initial_base_to_target_eigen_.inverse());

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

  pcl::PointCloud<PointType>::Ptr source_pointcloud_aligned_kcs(new pcl::PointCloud<PointType>);
  Eigen::Matrix4f best_transform;
  float best_score;

  findBestTransform(
    source_pointcloud_kcs, target_pointcloud_kcs, calibration_registrators_,
    source_pointcloud_aligned_kcs, best_transform, best_score);

  double init_error = getAlignmentError(source_pointcloud_kcs, target_pointcloud_kcs);
  double final_error = getAlignmentError(source_pointcloud_aligned_kcs, target_pointcloud_kcs);

  RCLCPP_INFO(this->get_logger(), "Initial avrage error: %.4f m", init_error);
  RCLCPP_INFO(this->get_logger(), "Final average error: %.4f m", final_error);

  Eigen::Affine3d icp_affine = Eigen::Affine3d(best_transform.cast<double>());
  Eigen::Affine3d inital_kit_to_source_eigen =
    base_to_sensor_kit_eigen_.inverse() * initial_base_to_source_eigen_;

  Eigen::Affine3d estimated_kit_to_source_eigen = icp_affine * inital_kit_to_source_eigen;

  // Optional filtering
  if (filter_estimations_) {
    auto estimated_rpy =
      tier4_autoware_utils::getRPY(tf2::toMsg(estimated_kit_to_source_eigen).orientation);

    Eigen::Vector3d x(
      estimated_rpy.z, estimated_kit_to_source_eigen.translation().x(),
      estimated_kit_to_source_eigen.translation().y());
    Eigen::DiagonalMatrix<double, 3> p0(initial_angle_cov_, initial_xy_cov_, initial_xy_cov_);

    if (first_observation_) {
      kalman_filter_.init(x, p0);
      first_observation_ = false;
    } else {
      kalman_filter_.update(x);
    }

    estimated_rpy.z = kalman_filter_.getXelement(0);
    estimated_kit_to_source_eigen.translation().x() = kalman_filter_.getXelement(1);
    estimated_kit_to_source_eigen.translation().y() = kalman_filter_.getXelement(2);

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.orientation = tier4_autoware_utils::createQuaternionFromRPY(
      estimated_rpy.x, estimated_rpy.y, estimated_rpy.z);

    pose_msg.position.x = kalman_filter_.getXelement(1);
    pose_msg.position.y = kalman_filter_.getXelement(2);
    pose_msg.position.z = estimated_kit_to_source_eigen.translation().z();

    tf2::fromMsg(pose_msg, estimated_kit_to_source_eigen);
  }

  geometry_msgs::msg::TransformStamped tf_msg =
    tf2::eigenToTransform(estimated_kit_to_source_eigen);
  tf_msg.header.stamp = source_pointcloud_header_.stamp;
  tf_msg.header.frame_id = sensor_kit_frame_;
  tf_msg.child_frame_id = source_pointcloud_frame_;
  tf_broadcaster_.sendTransform(tf_msg);

  // Publish the segmented pointclouds back in their frames (to evaluate visually the calibration)
  sensor_msgs::msg::PointCloud2 source_pointcloud_scs_msg, target_pointcloud_tcs_msg;
  pcl::toROSMsg(*source_pointcloud_scs, source_pointcloud_scs_msg);
  pcl::toROSMsg(*target_pointcloud_tcs, target_pointcloud_tcs_msg);
  source_pointcloud_scs_msg.header = source_pointcloud_header_;
  target_pointcloud_tcs_msg.header = target_pointcloud_header_;
  source_pub_->publish(source_pointcloud_scs_msg);
  target_pub_->publish(target_pointcloud_tcs_msg);

  // We perform basic filtering on the estimated angles
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (filter_estimations_) {
      Eigen::MatrixXd p;
      kalman_filter_.getP(p);
      double yaw_cov = p(0, 0);
      double x_cov = p(1, 1);
      double y_cov = p(2, 2);

      if (
        yaw_cov < angle_convergence_threshold_ && x_cov < xy_convergence_threshold_ &&
        y_cov < xy_convergence_threshold_) {
        output_calibration_msg_ =
          tf2::toMsg(estimated_kit_to_source_eigen * lidar_base_to_source_eigen_.inverse());
        calibration_done_ = true;
      }

      RCLCPP_INFO(
        this->get_logger(), "Filter cov: yaw=%.2e, x=%.2e, y=%.2e", yaw_cov, x_cov, y_cov);

      RCLCPP_INFO(
        this->get_logger(), "Convergence thresh: angle=%.2e, xy=%.2e", angle_convergence_threshold_,
        xy_convergence_threshold_);

    } else {
      output_calibration_msg_ =
        tf2::toMsg(estimated_kit_to_source_eigen * lidar_base_to_source_eigen_.inverse());
      calibration_done_ = true;
    }
  }
}

double LidarToLidar2DCalibrator::getAlignmentError(
  pcl::PointCloud<PointType>::Ptr source_pointcloud,
  pcl::PointCloud<PointType>::Ptr target_pointcloud)
{
  pcl::Correspondences correspondences;
  pcl::registration::CorrespondenceEstimation<PointType, PointType> correspondence_estimator;
  correspondence_estimator.setInputSource(source_pointcloud);
  correspondence_estimator.setInputTarget(target_pointcloud);
  correspondence_estimator.determineCorrespondences(correspondences);

  double error = 0.0;
  int n = 0;

  for (std::size_t i = 0; i < correspondences.size(); ++i) {
    if (correspondences[i].distance <= max_corr_distance_) {
      error += std::sqrt(correspondences[i].distance);
      n += 1;
    }
  }

  return error / n;
}

void LidarToLidar2DCalibrator::findBestTransform(
  pcl::PointCloud<PointType>::Ptr & source_pointcloud_ptr,
  pcl::PointCloud<PointType>::Ptr & target_pointcloud_ptr,
  std::vector<pcl::Registration<PointType, PointType>::Ptr> & registratators,
  pcl::PointCloud<PointType>::Ptr & best_aligned_pointcloud_ptr, Eigen::Matrix4f & best_transform,
  float & best_score)
{
  pcl::Correspondences correspondences;
  pcl::registration::CorrespondenceEstimation<PointType, PointType> estimator;
  estimator.setInputSource(source_pointcloud_ptr);
  estimator.setInputTarget(target_pointcloud_ptr);
  estimator.determineCorrespondences(correspondences);

  best_transform = Eigen::Matrix4f::Identity();
  best_score = 0.f;
  for (std::size_t i = 0; i < correspondences.size(); ++i) {
    best_score += correspondences[i].distance;
  }

  best_score /= correspondences.size();

  std::vector<Eigen::Matrix4f> transforms = {best_transform};

  for (auto & registrator : registratators) {
    Eigen::Matrix4f best_registrator_transform = Eigen::Matrix4f::Identity();
    float best_registrator_score = std::numeric_limits<float>::max();
    pcl::PointCloud<PointType>::Ptr best_registrator_aligned_cloud_ptr(
      new pcl::PointCloud<PointType>());

    for (auto & transform : transforms) {
      pcl::PointCloud<PointType>::Ptr aligned_cloud_ptr(new pcl::PointCloud<PointType>());
      registrator->setInputSource(source_pointcloud_ptr);
      registrator->setInputTarget(target_pointcloud_ptr);
      registrator->align(*aligned_cloud_ptr, transform);

      Eigen::Matrix4f candidate_transform = registrator->getFinalTransformation();
      float candidate_score = registrator->getFitnessScore(max_corr_distance_);

      if (candidate_score < best_registrator_score) {
        best_registrator_transform = candidate_transform;
        best_registrator_score = candidate_score;
        best_registrator_aligned_cloud_ptr = aligned_cloud_ptr;
      }
    }

    if (best_registrator_score < best_score) {
      best_transform = best_registrator_transform;
      best_score = best_registrator_score;
      best_aligned_pointcloud_ptr = best_registrator_aligned_cloud_ptr;
    }

    transforms.push_back(best_registrator_transform);
  }
}
