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

#ifndef DEVIATION_ESTIMATOR__DEVIATION_ESTIMATOR_HPP_
#define DEVIATION_ESTIMATOR__DEVIATION_ESTIMATOR_HPP_

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tier4_debug_msgs/msg/float64_stamped.hpp"
#include "tier4_debug_msgs/msg/float64_multi_array_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif



class DeviationEstimator : public rclcpp::Node
{
public:
  DeviationEstimator(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    sub_pose_;   //!< @brief measurement pose subscriber
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    sub_twist_raw_;  //!< @brief measurement twist subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_pose_with_cov_;   //!< @brief measurement pose with covariance subscriber
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_twist_with_cov_raw_;  //!< @brief measurement twist with covariance subscriber
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_coef_vx_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_bias_angvel_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_stddev_vx_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_stddev_angvel_;
  rclcpp::TimerBase::SharedPtr timer_control_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool show_debug_info_;
  bool use_pose_with_covariance_;  //!< @brief  use covariance in pose_with_covariance message
  bool use_twist_with_covariance_;  //!< @brief  use covariance in twist_with_covariance message
  bool use_predefined_coef_vx_;
  double predefined_coef_vx_;
  std::string results_path_;
  std::string imu_link_frame_;

  std::vector<geometry_msgs::msg::PoseStamped> pose_all_;
  std::vector<geometry_msgs::msg::TwistStamped> twist_all_;
  std::vector<geometry_msgs::msg::PoseStamped> pose_buf_;
  std::vector<double> coef_vx_list_;
  std::pair<double, double> coef_vx_;
  std::vector<std::vector<double>> bias_angvel_list_;
  std::vector<std::pair<double, double>> bias_angvel_;
  geometry_msgs::msg::Vector3Stamped bias_angvel_base_;
  geometry_msgs::msg::Vector3Stamped bias_angvel_imu_;

  double stddev_vx_;
  double stddev_vx_prime_;
  std::vector<double> stddev_angvel_base_;
  std::vector<double> stddev_angvel_prime_base_;
  std::vector<double> stddev_angvel_prime_imu_;

  double dt_design_;
  double dx_design_;
  double wz_threshold_;
  double vx_threshold_;
  double estimation_freq_;

  std::string output_frame_;
  geometry_msgs::msg::TransformStamped::SharedPtr tf_base2imu_ptr_;

  /**
   * @brief set poseWithCovariance measurement
   */
  void callbackPoseWithCovariance(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief set twistWithCovariance measurement
   */
  void callbackTwistWithCovarianceRaw(geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief set pose measurement
   */
  void callbackPose(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief set twist measurement
   */
  void callbackTwistRaw(geometry_msgs::msg::TwistStamped::SharedPtr msg);

  /**
   * @brief computes update & prediction of EKF for each ekf_dt_[s] time
   */
  void timerCallback();

  /**
    * @brief stock bias for every small sub-trajectory
    */
  void updateBias();

  /**
    * @brief clip radian
    */
  double clipRadian(double rad);

  /**
    * @brief save the results to a text file
    */
  void saveEstimatedParameters();

  /**
    * @brief get yaw from quaternion
    */
  double getYawFromQuat(const geometry_msgs::msg::Quaternion quat_msg);

  /**
    * @brief get stddev
    */
  void estimateStddev();

  /**
    * @brief get stddev prime
    */
  void estimateStddevPrime();

  /**
    * @brief calculate diff x
    */
  std::pair<double, double> calculateErrorPos(
    const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
    const std::vector<geometry_msgs::msg::TwistStamped> & twist_list,
    const bool enable_bias = false);

  /**
    * @brief calculate diff RPY
    */
  std::vector<double> calculateErrorRPY(
    const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
    const std::vector<geometry_msgs::msg::TwistStamped> & twist_list,
    const bool enable_bias = false);

  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr);

//   friend class DeviationEstimatorTestSuite;  // for test code
};
#endif  // DEVIATION_ESTIMATOR__DEVIATION_ESTIMATOR_HPP_
