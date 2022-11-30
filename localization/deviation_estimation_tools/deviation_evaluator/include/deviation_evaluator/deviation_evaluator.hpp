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

#ifndef DEVIATION_EVALUATOR__DEVIATION_EVALUATOR_HPP_
#define DEVIATION_EVALUATOR__DEVIATION_EVALUATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tier4_debug_msgs/msg/float64_stamped.hpp"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

class DeviationEvaluator : public rclcpp::Node
{
public:
  DeviationEvaluator(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_wheel_odometry_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_ndt_pose_with_cov_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_calibrated_imu_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    pub_calibrated_wheel_odometry_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_with_cov_dr_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_with_cov_gt_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pub_init_pose_with_cov_;

  bool show_debug_info_;
  std::string save_dir_;
  double start_time_;
  double stddev_vx_;
  double stddev_wz_;
  double coef_vx_;
  double bias_wz_;
  double period_;
  double cut_;

  geometry_msgs::msg::PoseStamped::SharedPtr current_ekf_gt_pose_ptr_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_ndt_pose_ptr_;

  bool has_published_initial_pose_;

  void callbackImu(const sensor_msgs::msg::Imu::SharedPtr msg);

  void callbackWheelOdometry(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  void callbackNDTPoseWithCovariance(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void save2YamlFile();
};

#endif  // DEVIATION_EVALUATOR__DEVIATION_EVALUATOR_HPP_
