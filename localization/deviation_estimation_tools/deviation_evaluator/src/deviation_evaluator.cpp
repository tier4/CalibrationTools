// Copyright 2018-2019 Autoware Foundation
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

#include "deviation_evaluator/deviation_evaluator.hpp"

#include "rclcpp/logging.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

// clang-format on
using std::placeholders::_1;

double double_round(const double x, const int n) { return std::round(x * pow(10, n)) / pow(10, n); }

double norm_xy(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double norm_xy_lateral(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, const double yaw)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::abs(dx * std::sin(yaw) - dy * std::cos(yaw));
}

DeviationEvaluator::DeviationEvaluator(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  show_debug_info_ = declare_parameter<bool>("show_debug_info", false);
  stddev_vx_ = declare_parameter<double>("stddev_vx");
  stddev_wz_ = declare_parameter<double>("stddev_wz");
  coef_vx_ = declare_parameter<double>("coef_vx");
  bias_wz_ = declare_parameter<double>("bias_wz");
  save_dir_ = declare_parameter<std::string>("save_dir");
  wait_duration_ = declare_parameter<double>("wait_duration");
  double wait_scale = declare_parameter<double>("wait_scale");
  errors_threshold_.lateral =
    declare_parameter<double>("warn_ellipse_size_lateral_direction") * wait_scale;
  errors_threshold_.long_radius = declare_parameter<double>("warn_ellipse_size") * wait_scale;

  save2YamlFile();

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "in_imu", 1, std::bind(&DeviationEvaluator::callbackImu, this, _1));
  sub_wheel_odometry_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_wheel_odometry", 1, std::bind(&DeviationEvaluator::callbackWheelOdometry, this, _1));
  sub_ndt_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_ndt_pose_with_covariance", 1,
    std::bind(&DeviationEvaluator::callbackNDTPoseWithCovariance, this, _1));
  sub_dr_odom_ = create_subscription<Odometry>(
    "in_ekf_dr_odom", 1, std::bind(&DeviationEvaluator::callbackEKFDROdom, this, _1));
  sub_gt_odom_ = create_subscription<Odometry>(
    "in_ekf_gt_odom", 1, std::bind(&DeviationEvaluator::callbackEKFGTOdom, this, _1));

  pub_calibrated_imu_ = create_publisher<sensor_msgs::msg::Imu>("out_imu", 1);
  pub_calibrated_wheel_odometry_ =
    create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("out_wheel_odometry", 1);
  pub_pose_with_cov_dr_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "out_pose_with_covariance_dr", 1);
  pub_pose_with_cov_gt_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "out_pose_with_covariance_gt", 1);
  pub_init_pose_with_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "out_initial_pose_with_covariance", 1);

  current_ndt_pose_ptr_ = nullptr;
  has_published_initial_pose_ = false;
}

void DeviationEvaluator::callbackImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  msg->angular_velocity.z -= bias_wz_;
  msg->angular_velocity_covariance[2 * 3 + 2] = stddev_wz_ * stddev_wz_;
  pub_calibrated_imu_->publish(*msg);
}

void DeviationEvaluator::callbackWheelOdometry(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  msg->twist.twist.linear.x *= coef_vx_;
  msg->twist.covariance[0] = stddev_vx_ * stddev_vx_;
  pub_calibrated_wheel_odometry_->publish(*msg);
}

void DeviationEvaluator::callbackNDTPoseWithCovariance(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!has_published_initial_pose_) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov;
    pose_with_cov = *msg;
    pose_with_cov.pose.covariance[0 * 6 + 0] = 0.01;
    pose_with_cov.pose.covariance[1 * 6 + 1] = 0.01;
    pose_with_cov.pose.covariance[2 * 6 + 2] = 0.01;
    pose_with_cov.pose.covariance[3 * 6 + 3] = 0.01;
    pose_with_cov.pose.covariance[4 * 6 + 4] = 0.01;
    pose_with_cov.pose.covariance[5 * 6 + 5] = 0.01;
    pub_init_pose_with_cov_->publish(*msg);
    has_published_initial_pose_ = true;
    return;
  }

  const double msg_time = rclcpp::Time(msg->header.stamp).seconds();

  if (msg_time - start_time_ < wait_duration_) {
    pub_pose_with_cov_dr_->publish(*msg);
  }
  pub_pose_with_cov_gt_->publish(*msg);

  if (
    (current_errors_.lateral > errors_threshold_.lateral) &
    (current_errors_.long_radius > errors_threshold_.long_radius)) {
    RCLCPP_INFO(this->get_logger(), "Error got large enough! Initialize EKF");
    start_time_ = msg_time;
    has_published_initial_pose_ = false;
    current_errors_ = Errors();
  }
}

void DeviationEvaluator::callbackEKFDROdom(const Odometry::SharedPtr msg)
{
  PoseStamped::SharedPtr pose_ptr(new PoseStamped);
  pose_ptr->pose = msg->pose.pose;
  pose_ptr->header = msg->header;
  if (!dr_pose_queue_.empty()) {
    if (
      rclcpp::Time(dr_pose_queue_.back()->header.stamp).seconds() >
      rclcpp::Time(pose_ptr->header.stamp).seconds()) {
      dr_pose_queue_.clear();
      RCLCPP_ERROR_STREAM(this->get_logger(), "Timestampe jump detected!");
    }
  }
  dr_pose_queue_.push_back(pose_ptr);
}

void DeviationEvaluator::callbackEKFGTOdom(const Odometry::SharedPtr msg)
{
  if (last_gt_pose_ptr_ == nullptr) {
    last_gt_pose_ptr_.reset(new PoseStamped);
    last_gt_pose_ptr_->header = msg->header;
    last_gt_pose_ptr_->pose = msg->pose.pose;
  }
  if (dr_pose_queue_.size() < 2) return;

  double start_time = rclcpp::Time(dr_pose_queue_.front()->header.stamp).seconds();
  double target_time = rclcpp::Time(last_gt_pose_ptr_->header.stamp).seconds();
  if (start_time > target_time) {
    last_gt_pose_ptr_ = nullptr;
    return;
  }

  geometry_msgs::msg::Pose target_pose;
  try {
    target_pose = interpolatePose(target_time);
  } catch (const std::runtime_error & exception) {
    return;
  }
  current_errors_.long_radius = norm_xy(target_pose.position, last_gt_pose_ptr_->pose.position);
  current_errors_.lateral = norm_xy_lateral(
    target_pose.position, last_gt_pose_ptr_->pose.position, tf2::getYaw(target_pose.orientation));
  last_gt_pose_ptr_ = nullptr;
  dr_pose_queue_.clear();
}

geometry_msgs::msg::Pose DeviationEvaluator::interpolatePose(const double time)
{
  const auto iter_next = std::upper_bound(
    dr_pose_queue_.begin(), dr_pose_queue_.end(), time,
    [](double const & t1, geometry_msgs::msg::PoseStamped::SharedPtr const & t2) -> bool {
      return t1 < rclcpp::Time(t2->header.stamp).seconds();
    });

  if ((iter_next == dr_pose_queue_.begin()) | (iter_next == dr_pose_queue_.end())) {
    throw std::runtime_error("Interpolation failed");
  }

  const double time_start = rclcpp::Time((*(iter_next - 1))->header.stamp).seconds();
  const double time_end = rclcpp::Time((*iter_next)->header.stamp).seconds();
  const double ratio = (time - time_start) / (time_end - time_start);
  return tier4_autoware_utils::calcInterpolatedPose(
    (*(iter_next - 1))->pose, (*iter_next)->pose, ratio);
}

void DeviationEvaluator::save2YamlFile()
{
  std::ofstream file(save_dir_ + "/config.yaml");
  file << "parameters:" << std::endl;
  file << "  stddev_vx: " << double_round(stddev_vx_, 5) << std::endl;
  file << "  stddev_wz: " << double_round(stddev_wz_, 5) << std::endl;
  file << "  bias_rho: " << double_round(coef_vx_, 5) << std::endl;
  file << "  bias_gyro: " << double_round(bias_wz_, 5) << std::endl;
  file.close();
}
