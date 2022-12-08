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

using namespace std::chrono_literals;

double double_round(const double x, const int n) { return std::round(x * pow(10, n)) / pow(10, n); }

DeviationEvaluator::DeviationEvaluator(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  show_debug_info_ = declare_parameter("show_debug_info", false);
  stddev_vx_ = declare_parameter("stddev_vx", 0.7);
  stddev_wz_ = declare_parameter("stddev_wz", 0.01);
  coef_vx_ = declare_parameter("coef_vx", 1.0);
  bias_wz_ = declare_parameter("bias_wz", 0.0);
  period_ = declare_parameter("period", 10.0);
  cut_ = declare_parameter("cut", 9.0);
  save_dir_ = declare_parameter("save_dir", "");

  client_trigger_ekf_dr_ = create_client<std_srvs::srv::SetBool>("out_ekf_dr_trigger", rmw_qos_profile_services_default);
  client_trigger_ekf_gt_ = create_client<std_srvs::srv::SetBool>("out_ekf_gt_trigger", rmw_qos_profile_services_default);

  if (declare_parameter<bool>("need_ekf_initial_trigger"))
  {
    while (!client_trigger_ekf_dr_->wait_for_service(1s)) {
      if (!rclcpp::ok()) break;
      RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for EKF trigger service...");
    }
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    client_trigger_ekf_dr_->async_send_request(
      req,
      [this]([[maybe_unused]] rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result) {}
    );

    while (!client_trigger_ekf_gt_->wait_for_service(1s)) {
      if (!rclcpp::ok()) break;
      RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for EKF trigger service...");
    }
    client_trigger_ekf_gt_->async_send_request(
      req,
      [this]([[maybe_unused]] rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result) {}
    );
    RCLCPP_INFO(this->get_logger(), "EKF initialization finished");
  }

  save2YamlFile();

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "in_imu", 1, std::bind(&DeviationEvaluator::callbackImu, this, _1));
  sub_wheel_odometry_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_wheel_odometry", 1, std::bind(&DeviationEvaluator::callbackWheelOdometry, this, _1));
  sub_ndt_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_ndt_pose_with_covariance", 1,
    std::bind(&DeviationEvaluator::callbackNDTPoseWithCovariance, this, _1));

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
    pose_with_cov.pose.covariance[0] = 1.0;
    pose_with_cov.pose.covariance[1 * 6 + 1] = 1.0;
    pose_with_cov.pose.covariance[2 * 6 + 2] = 0.01;
    pose_with_cov.pose.covariance[3 * 6 + 3] = 0.01;
    pose_with_cov.pose.covariance[4 * 6 + 4] = 0.01;
    pose_with_cov.pose.covariance[5 * 6 + 5] = 0.2;
    pub_init_pose_with_cov_->publish(*msg);
    has_published_initial_pose_ = true;
    return;
  }

  const double msg_time = rclcpp::Time(msg->header.stamp).seconds();

  if (msg_time - start_time_ < period_ - cut_) {
    pub_pose_with_cov_dr_->publish(*msg);
  }
  pub_pose_with_cov_gt_->publish(*msg);

  if (msg_time - start_time_ > period_) {
    DEBUG_INFO(this->get_logger(), "NDT cycle");
    start_time_ = msg_time;
  }
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
