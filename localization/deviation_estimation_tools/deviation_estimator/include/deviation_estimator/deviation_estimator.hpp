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

#include "deviation_estimator/gyro_bias_module.hpp"
#include "deviation_estimator/utils.hpp"
#include "deviation_estimator/velocity_coef_module.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tier4_debug_msgs/msg/float64_stamped.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

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
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_pose_with_cov_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_wheel_odometry_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
    sub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_coef_vx_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_bias_angvel_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_stddev_vx_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_stddev_angvel_;
  rclcpp::TimerBase::SharedPtr timer_control_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool show_debug_info_;
  bool use_predefined_coef_vx_;
  double predefined_coef_vx_;
  std::string results_path_;
  std::string imu_link_frame_;

  std::vector<geometry_msgs::msg::PoseStamped> pose_all_;
  // std::vector<geometry_msgs::msg::TwistStamped> twist_all_;
  std::vector<tier4_debug_msgs::msg::Float64Stamped> vx_all_;
  std::vector<geometry_msgs::msg::Vector3Stamped> gyro_all_;
  std::vector<geometry_msgs::msg::PoseStamped> pose_buf_;

  double dt_design_;
  double dx_design_;
  double wz_threshold_;
  double vx_threshold_;
  double estimation_freq_;
  double time_window_;
  bool add_bias_uncertainty_;

  std::string output_frame_;
  geometry_msgs::msg::TransformStamped::SharedPtr tf_base2imu_ptr_;

  std::unique_ptr<GyroBiasModule> gyro_bias_module_;
  std::unique_ptr<VelocityCoefModule> vel_coef_module_;

  void callback_pose_with_covariance(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void callback_wheel_odometry(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr wheel_odometry_msg_ptr);

  void callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);

  void timer_callback();

  void update_bias(
    const std::vector<geometry_msgs::msg::PoseStamped> & pose_buf,
    const std::vector<geometry_msgs::msg::TwistStamped> & twist_all);

  double estimate_stddev_velocity(
    const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
    const std::vector<tier4_debug_msgs::msg::Float64Stamped> & vx_list,
    const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
    const double t_window) const;
  
  geometry_msgs::msg::Vector3 estimate_stddev_angular_velocity(
    const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
    const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
    const double t_window) const;

  double add_bias_uncertainty_on_velocity(
    const double stddev_vx, const double stddev_coef_vx) const;

  geometry_msgs::msg::Vector3 add_bias_uncertainty_on_angular_velocity(
    const geometry_msgs::msg::Vector3 stddev_angvel_base,
    const geometry_msgs::msg::Vector3 stddev_angvel_bias_base) const;

  bool get_transform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr);
};
#endif  // DEVIATION_ESTIMATOR__DEVIATION_ESTIMATOR_HPP_
