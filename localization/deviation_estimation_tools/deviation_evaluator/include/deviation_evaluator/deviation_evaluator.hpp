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
#include <deque>

class DeviationEvaluator : public rclcpp::Node
{
private:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Odometry = nav_msgs::msg::Odometry;
  struct Errors
  {
    double lateral;
    double long_radius;
    Errors() : lateral(0), long_radius(0){};
  };

public:
  DeviationEvaluator(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr
    sub_wheel_odometry_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr
    sub_ndt_pose_with_cov_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_dr_odom_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_gt_odom_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_calibrated_imu_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr
    pub_calibrated_wheel_odometry_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_pose_with_cov_dr_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_pose_with_cov_gt_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr
    pub_init_pose_with_cov_;

  bool show_debug_info_;
  std::string save_dir_;
  double start_time_;
  double stddev_vx_;
  double stddev_wz_;
  double coef_vx_;
  double bias_wz_;
  // double period_;
  // double cut_;
  double wait_duration_;
  Errors errors_threshold_;
  Errors current_errors_;

  std::deque<PoseStamped::SharedPtr> dr_pose_queue_;

  PoseStamped::SharedPtr last_gt_pose_ptr_;


  PoseStamped::SharedPtr current_ekf_gt_pose_ptr_;
  PoseStamped::SharedPtr current_ndt_pose_ptr_;

  bool has_published_initial_pose_;

  void callbackImu(const sensor_msgs::msg::Imu::SharedPtr msg);

  void callbackWheelOdometry(const TwistWithCovarianceStamped::SharedPtr msg);

  void callbackNDTPoseWithCovariance(const PoseWithCovarianceStamped::SharedPtr msg);

  void callbackEKFDROdom(const Odometry::SharedPtr msg);

  void callbackEKFGTOdom(const Odometry::SharedPtr msg);

  geometry_msgs::msg::Pose interpolatePose(const double timestamp_seconds);

  void save2YamlFile();
};

#endif  // DEVIATION_EVALUATOR__DEVIATION_EVALUATOR_HPP_
