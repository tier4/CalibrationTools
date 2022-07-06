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

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include "deviation_evaluator/deviation_evaluator.hpp"
#include "rclcpp/logging.hpp"

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

// clang-format on
using std::placeholders::_1;


double double_round(const double x, const int n)
{
  return std::round(x * pow(10, n)) / pow(10, n);
}


DeviationEvaluator::DeviationEvaluator(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
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

  save2YamlFile();

  sub_twist_with_cov_ =
    create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance",
    1,
    std::bind(&DeviationEvaluator::callbackTwistWithCovariance, this, _1));
  sub_ndt_pose_with_cov_ =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_ndt_pose_with_covariance",
    1,
    std::bind(&DeviationEvaluator::callbackNDTPoseWithCovariance, this, _1));

  pub_twist_with_cov_ =
    create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "out_twist_with_covariance", 1);
  pub_ndt_pose_with_cov_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "out_pose_with_covariance", 1);

  current_ndt_pose_ptr_ = nullptr;
}

/*
 * callbackTwistWithCovariance
 */
void DeviationEvaluator::callbackTwistWithCovariance(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  msg->twist.twist.linear.x *= coef_vx_;
  msg->twist.twist.angular.z -= bias_wz_;

  msg->twist.covariance[0] = stddev_vx_ * stddev_vx_;
  msg->twist.covariance[0 * 6 + 5] = 0.0;
  msg->twist.covariance[5 * 6 + 0] = 0.0;
  msg->twist.covariance[5 * 6 + 5] = stddev_wz_ * stddev_wz_;
  pub_twist_with_cov_->publish(*msg);
}

/*
 * callbackNDTPoseWithCovariance
 */
void DeviationEvaluator::callbackNDTPoseWithCovariance(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  const double msg_time = rclcpp::Time(msg->header.stamp).seconds();

  // current_ndt_pose_ptr_ = msg;
  if (msg_time - start_time_ > period_) {
    DEBUG_INFO(
      this->get_logger(),
      "NDT cycle"
    );
    start_time_ = msg_time;
  } else if (msg_time - start_time_ < period_ - cut_) {
    // Streaming NDT results
    pub_ndt_pose_with_cov_->publish(*msg);
  } else {
    // Not streaming NDT results
  }
}

/*
 * save2YamlFile
 */
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
