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

#include <extrinsic_lidar_to_lidar_2d_calibrator/extrinsic_lidar_to_lidar_2d_calibrator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
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

#include <algorithm>
#include <iostream>
#include <numeric>

namespace extrinsic_lidar_to_lidar_2d_calibrator
{

LidarToLidar2DCalibrator::LidarToLidar2DCalibrator(const rclcpp::NodeOptions & options)
: Node("extrinsic_lidar_to_lidar_2d_calibrator", options), tf_broadcaster_(this)
{
  using std::chrono_literals::operator""ms;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

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

  filtered_source_initial_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_source_initial_points", 10);
  filtered_source_aligned_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_source_aligned_points", 10);
  filtered_target_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_target_points", 10);
  flat_source_initial_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("flat_source_initial_points", 10);
  flat_source_aligned_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("flat_source_aligned_points", 10);
  flat_target_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("flat_target_points", 10);

  source_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "source_input_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&LidarToLidar2DCalibrator::sourcePointCloudCallback, this, std::placeholders::_1));

  target_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "target_input_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&LidarToLidar2DCalibrator::targetPointCloudCallback, this, std::placeholders::_1));

  calibration_timer_ = rclcpp::create_timer(
    this, get_clock(), 200ms, std::bind(&LidarToLidar2DCalibrator::calibrationTimerCallback, this));

  // The service server runs in a dedicated thread since it is a blocking call
  srv_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  service_server_ = this->create_service<tier4_calibration_msgs::srv::NewExtrinsicCalibrator>(
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
  [[maybe_unused]] const std::shared_ptr<
    tier4_calibration_msgs::srv::NewExtrinsicCalibrator::Request>
    request,
  const std::shared_ptr<tier4_calibration_msgs::srv::NewExtrinsicCalibrator::Response> response)
{
  using std::chrono_literals::operator""s;

  {
    std::unique_lock<std::mutex> lock(mutex_);
    received_request_ = true;
  }

  // Loop until the calibration finishes
  while (rclcpp::ok()) {
    rclcpp::sleep_for(1s);
    std::unique_lock<std::mutex> lock(mutex_);

    if (calibration_done_) {
      break;
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000, "Waiting for the calibration to end");
  }

  tier4_calibration_msgs::msg::CalibrationResult result;
  result.message.data = "Calibration successful";
  result.score = 0.f;
  result.success = true;
  result.transform_stamped = output_calibration_msg_;

  response->results.push_back(result);
}

void LidarToLidar2DCalibrator::sourcePointCloudCallback(
  sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  source_pointcloud_frame_ = msg->header.frame_id;
  source_pointcloud_header_ = msg->header;
  source_pointcloud_msg_ = std::move(msg);
}

void LidarToLidar2DCalibrator::targetPointCloudCallback(
  sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  target_pointcloud_frame_ = msg->header.frame_id;
  target_pointcloud_header_ = msg->header;
  target_pointcloud_msg_ = std::move(msg);
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

    geometry_msgs::msg::Transform initial_base_to_source_msg_ =
      tf_buffer_->lookupTransform(base_frame_, source_pointcloud_frame_, t, timeout).transform;

    geometry_msgs::msg::Transform initial_base_to_target_msg_ =
      tf_buffer_->lookupTransform(base_frame_, target_pointcloud_frame_, t, timeout).transform;

    initial_base_to_source_eigen_ = tf2::transformToEigen(initial_base_to_source_msg_);
    initial_base_to_target_eigen_ = tf2::transformToEigen(initial_base_to_target_msg_);

    got_initial_transform_ = true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get initial tf. %s", ex.what());
    return false;
  }

  return true;
}

void LidarToLidar2DCalibrator::calibrationTimerCallback()
{
  // Make sure we have all the required initial tfs
  if (!checkInitialTransforms() || !source_pointcloud_msg_ || !target_pointcloud_msg_) {
    return;
  }

  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!received_request_) {
      RCLCPP_WARN_ONCE(
        this->get_logger(), "Attempted to calibrate before a request. Only printing this once");
      return;
    }
  }

  // nomenclature
  // raw: non-processed pointcloud
  // flat: null z-component
  // scs = source coordinate system
  // tcs = target coordinate system
  // bcs = base coordinate system

  // lidar coordinates
  pcl::PointCloud<PointType>::Ptr raw_source_pointcloud_scs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr raw_target_pointcloud_tcs(new pcl::PointCloud<PointType>);

  // base coordinates
  pcl::PointCloud<PointType>::Ptr raw_source_pointcloud_bcs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr raw_target_pointcloud_bcs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr filtered_source_pointcloud_bcs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr filtered_source_aligned_pointcloud_bcs(
    new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr filtered_target_pointcloud_bcs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr flat_source_pointcloud_bcs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr flat_source_aligned_pointcloud_bcs(
    new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr flat_target_pointcloud_bcs(new pcl::PointCloud<PointType>);

  pcl::fromROSMsg(*source_pointcloud_msg_, *raw_source_pointcloud_scs);
  pcl::fromROSMsg(*target_pointcloud_msg_, *raw_target_pointcloud_tcs);
  source_pointcloud_msg_.reset();
  target_pointcloud_msg_.reset();

  pcl::transformPointCloud(
    *raw_source_pointcloud_scs, *raw_source_pointcloud_bcs, initial_base_to_source_eigen_);
  pcl::transformPointCloud(
    *raw_target_pointcloud_tcs, *raw_target_pointcloud_bcs, initial_base_to_target_eigen_);

  // Segment points in a range of z
  auto filter_point_range = [&](const auto & point) {
    return point.z >= min_z_ && point.z <= max_z_ &&
           std::sqrt(point.x * point.x + point.y * point.y) <= max_calibration_range_;
  };

  std::copy_if(
    raw_source_pointcloud_bcs->points.begin(), raw_source_pointcloud_bcs->points.end(),
    std::back_inserter(filtered_source_pointcloud_bcs->points), filter_point_range);
  std::copy_if(
    raw_target_pointcloud_bcs->points.begin(), raw_target_pointcloud_bcs->points.end(),
    std::back_inserter(filtered_target_pointcloud_bcs->points), filter_point_range);
  filtered_source_pointcloud_bcs->width = filtered_source_pointcloud_bcs->size();
  filtered_source_pointcloud_bcs->height = 1;
  filtered_target_pointcloud_bcs->width = filtered_target_pointcloud_bcs->size();
  filtered_target_pointcloud_bcs->height = 1;

  // Discard the z-component to perform 2D ICP
  auto discard_z = [](auto & point) { point.z = 0.f; };

  pcl::copyPointCloud(*filtered_source_pointcloud_bcs, *flat_source_pointcloud_bcs);
  pcl::copyPointCloud(*filtered_target_pointcloud_bcs, *flat_target_pointcloud_bcs);

  std::for_each(
    flat_source_pointcloud_bcs->points.begin(), flat_source_pointcloud_bcs->points.end(),
    discard_z);
  std::for_each(
    flat_target_pointcloud_bcs->points.begin(), flat_target_pointcloud_bcs->points.end(),
    discard_z);

  if (flat_source_pointcloud_bcs->size() == 0 || flat_target_pointcloud_bcs->size() == 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Attempting to calibrate with empty clouds. Either the environment/sensor are not suitable "
      "or the parameters need to be relaxed. source=%lu target=%lu",
      filtered_source_pointcloud_bcs->size(), filtered_target_pointcloud_bcs->size());
    return;
  }

  // Perform 2D ICP
  auto [best_source_aligned_pointcloud_bcs, best_transform, best_score] = findBestTransform(
    flat_source_pointcloud_bcs, flat_target_pointcloud_bcs, calibration_registrators_);

  if (std::isnan(best_score) || std::isinf(best_score) || best_score < 0.f) {
    RCLCPP_WARN(this->get_logger(), "ICP did not converge. Skipping iteration");
    return;
  }

  double init_error = getAlignmentError(flat_source_pointcloud_bcs, flat_target_pointcloud_bcs);
  double final_error =
    getAlignmentError(best_source_aligned_pointcloud_bcs, flat_target_pointcloud_bcs);

  RCLCPP_INFO(this->get_logger(), "Initial average error: %.4f m", init_error);
  RCLCPP_INFO(this->get_logger(), "Final average error: %.4f m", final_error);

  Eigen::Affine3d icp_affine = Eigen::Affine3d(best_transform.cast<double>());
  Eigen::Affine3d filtered_affine = icp_affine;

  // Optional filtering
  if (filter_estimations_) {
    // We force our RPY to avoid convention errors
    auto estimated_rpy = tier4_autoware_utils::getRPY(tf2::toMsg(icp_affine).orientation);

    Eigen::Vector3d x(estimated_rpy.z, icp_affine.translation().x(), icp_affine.translation().y());
    Eigen::DiagonalMatrix<double, 3> p0(initial_angle_cov_, initial_xy_cov_, initial_xy_cov_);

    if (first_observation_) {
      kalman_filter_.init(x, p0);
      first_observation_ = false;
    } else {
      kalman_filter_.update(x);
    }

    // cSpell:ignore getXelement
    estimated_rpy.z = kalman_filter_.getXelement(0);
    filtered_affine.translation().x() = kalman_filter_.getXelement(1);
    filtered_affine.translation().y() = kalman_filter_.getXelement(2);

    geometry_msgs::msg::Quaternion quaternion_msg = tier4_autoware_utils::createQuaternionFromRPY(
      estimated_rpy.x, estimated_rpy.y, estimated_rpy.z);
    Eigen::Quaterniond quaternion_eigen(
      quaternion_msg.w, quaternion_msg.x, quaternion_msg.y, quaternion_msg.z);

    filtered_affine.linear() = quaternion_eigen.matrix();
  }

  // Compute the source_to_target_transform
  Eigen::Affine3d target_to_source_transform =
    initial_base_to_target_eigen_.inverse() * filtered_affine * initial_base_to_source_eigen_;

  // Optionally broadcast the current calibration TF
  if (broadcast_calibration_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg = tf2::eigenToTransform(target_to_source_transform);
    tf_msg.header.stamp = target_pointcloud_header_.stamp;
    tf_msg.header.frame_id = target_pointcloud_frame_;
    tf_msg.child_frame_id = source_pointcloud_frame_;
    tf_broadcaster_.sendTransform(tf_msg);
  }

  pcl::transformPointCloud(
    *filtered_source_pointcloud_bcs, *filtered_source_aligned_pointcloud_bcs,
    filtered_affine.matrix().cast<float>());

  pcl::transformPointCloud(
    *flat_source_pointcloud_bcs, *flat_source_aligned_pointcloud_bcs,
    filtered_affine.matrix().cast<float>());

  // Publish visualization pointclouds
  sensor_msgs::msg::PointCloud2 filtered_source_initial_pointcloud_bcs_msg,
    filtered_source_aligned_pointcloud_bcs_msg, filtered_target_pointcloud_bcs_msg;
  pcl::toROSMsg(*filtered_source_pointcloud_bcs, filtered_source_initial_pointcloud_bcs_msg);
  pcl::toROSMsg(
    *filtered_source_aligned_pointcloud_bcs, filtered_source_aligned_pointcloud_bcs_msg);
  pcl::toROSMsg(*filtered_target_pointcloud_bcs, filtered_target_pointcloud_bcs_msg);
  filtered_source_initial_pointcloud_bcs_msg.header.stamp = source_pointcloud_header_.stamp;
  filtered_source_aligned_pointcloud_bcs_msg.header.stamp = source_pointcloud_header_.stamp;
  filtered_target_pointcloud_bcs_msg.header.stamp = target_pointcloud_header_.stamp;
  filtered_source_initial_pointcloud_bcs_msg.header.frame_id = base_frame_;
  filtered_source_aligned_pointcloud_bcs_msg.header.frame_id = base_frame_;
  filtered_target_pointcloud_bcs_msg.header.frame_id = base_frame_;

  filtered_source_initial_pub_->publish(filtered_source_initial_pointcloud_bcs_msg);
  filtered_source_aligned_pub_->publish(filtered_source_aligned_pointcloud_bcs_msg);
  filtered_target_pub_->publish(filtered_target_pointcloud_bcs_msg);

  sensor_msgs::msg::PointCloud2 flat_source_initial_pointcloud_bcs_msg,
    flat_source_aligned_pointcloud_bcs_msg, flat_target_pointcloud_bcs_msg;
  pcl::toROSMsg(*flat_source_pointcloud_bcs, flat_source_initial_pointcloud_bcs_msg);
  pcl::toROSMsg(*flat_source_aligned_pointcloud_bcs, flat_source_aligned_pointcloud_bcs_msg);
  pcl::toROSMsg(*flat_target_pointcloud_bcs, flat_target_pointcloud_bcs_msg);
  flat_source_initial_pointcloud_bcs_msg.header.stamp = source_pointcloud_header_.stamp;
  flat_source_aligned_pointcloud_bcs_msg.header.stamp = source_pointcloud_header_.stamp;
  flat_target_pointcloud_bcs_msg.header.stamp = target_pointcloud_header_.stamp;
  flat_source_initial_pointcloud_bcs_msg.header.frame_id = base_frame_;
  flat_source_aligned_pointcloud_bcs_msg.header.frame_id = base_frame_;
  flat_target_pointcloud_bcs_msg.header.frame_id = base_frame_;

  flat_source_initial_pub_->publish(flat_source_initial_pointcloud_bcs_msg);
  flat_source_aligned_pub_->publish(flat_source_aligned_pointcloud_bcs_msg);
  flat_target_pub_->publish(flat_target_pointcloud_bcs_msg);

  // We perform basic filtering on the estimated angles
  {
    std::unique_lock<std::mutex> lock(mutex_);

    output_calibration_msg_ = tf2::eigenToTransform(target_to_source_transform);
    output_calibration_msg_.header.stamp = target_pointcloud_header_.stamp;
    output_calibration_msg_.header.frame_id = target_pointcloud_frame_;
    output_calibration_msg_.child_frame_id = source_pointcloud_frame_;

    if (!filter_estimations_) {
      calibration_done_ = true;
    } else {
      Eigen::MatrixXd p;
      kalman_filter_.getP(p);
      double yaw_cov = p(0, 0);
      double x_cov = p(1, 1);
      double y_cov = p(2, 2);

      if (
        yaw_cov < angle_convergence_threshold_ && x_cov < xy_convergence_threshold_ &&
        y_cov < xy_convergence_threshold_) {
        calibration_done_ = true;
      }

      RCLCPP_INFO(
        this->get_logger(), "Filter cov: yaw=%.2e, x=%.2e, y=%.2e", yaw_cov, x_cov, y_cov);

      RCLCPP_INFO(
        this->get_logger(), "Convergence thresh: angle=%.2e, xy=%.2e", angle_convergence_threshold_,
        xy_convergence_threshold_);
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
  std::size_t accepted_correspondences = 0;

  for (std::size_t i = 0; i < correspondences.size(); ++i) {
    if (correspondences[i].distance <= max_corr_distance_) {
      error += std::sqrt(correspondences[i].distance);
      accepted_correspondences += 1;
    }
  }

  return error / accepted_correspondences;
}

std::tuple<pcl::PointCloud<PointType>::Ptr, Eigen::Matrix4f, float>
LidarToLidar2DCalibrator::findBestTransform(
  pcl::PointCloud<PointType>::Ptr & source_pointcloud_ptr,
  pcl::PointCloud<PointType>::Ptr & target_pointcloud_ptr,
  std::vector<pcl::Registration<PointType, PointType>::Ptr> & registrators)
{
  pcl::Correspondences correspondences;
  pcl::registration::CorrespondenceEstimation<PointType, PointType> estimator;
  estimator.setInputSource(source_pointcloud_ptr);
  estimator.setInputTarget(target_pointcloud_ptr);
  estimator.determineCorrespondences(correspondences);

  pcl::PointCloud<PointType>::Ptr best_aligned_pointcloud_ptr(new pcl::PointCloud<PointType>);
  Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
  float best_score =
    std::transform_reduce(
      correspondences.cbegin(), correspondences.cend(), 0.f, std::plus<float>{},
      [](const pcl::Correspondence & correspondence) { return correspondence.distance; }) /
    correspondences.size();

  std::vector<Eigen::Matrix4f> transforms = {best_transform};

  for (auto & registrator : registrators) {
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

  return std::make_tuple<>(best_aligned_pointcloud_ptr, best_transform, best_score);
}  // extrinsic_lidar_to_lidar_2d_calibrator

}  // namespace extrinsic_lidar_to_lidar_2d_calibrator
