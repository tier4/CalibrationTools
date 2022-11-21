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

#include "deviation_estimator/utils.hpp"

#include "rclcpp/rclcpp.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <iostream>
#include <vector>

double double_round(const double x, const int n) { return std::round(x * pow(10, n)) / pow(10, n); }

double clipRadian(const double rad)
{
  if (rad < -M_PI) {
    return rad + 2 * M_PI;
  } else if (rad >= M_PI) {
    return rad - 2 * M_PI;
  } else {
    return rad;
  }
}

/*
 * saveEstimatedParameters
 */
void saveEstimatedParameters(
  const std::string output_path, const double stddev_vx, const double stddev_wz,
  const double coef_vx, const double bias_wz,
  const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
  const geometry_msgs::msg::Vector3 & angular_velocity_offset)
{
  std::ofstream file(output_path);
  file << "# Results expressed in base_link" << std::endl;
  file << "# Copy the following to deviation_evaluator.param.yaml" << std::endl;
  file << "stddev_vx: " << double_round(stddev_vx, 5) << std::endl;
  file << "stddev_wz: " << double_round(stddev_wz, 5) << std::endl;
  file << "coef_vx: " << double_round(coef_vx, 5) << std::endl;
  file << "bias_wz: " << double_round(bias_wz, 5) << std::endl;
  file << std::endl;
  file << "# Results expressed in imu_link" << std::endl;
  file << "# Copy the following to imu_corrector.param.yaml" << std::endl;
  file << "angular_velocity_stddev_xx: " << double_round(angular_velocity_stddev.x, 5) << std::endl;
  file << "angular_velocity_stddev_yy: " << double_round(angular_velocity_stddev.y, 5) << std::endl;
  file << "angular_velocity_stddev_zz: " << double_round(angular_velocity_stddev.z, 5) << std::endl;
  file << "angular_velocity_offset_x: " << double_round(angular_velocity_offset.x, 6) << std::endl;
  file << "angular_velocity_offset_y: " << double_round(angular_velocity_offset.y, 6) << std::endl;
  file << "angular_velocity_offset_z: " << double_round(angular_velocity_offset.z, 6) << std::endl;

  file.close();
}

geometry_msgs::msg::Point calculateErrorPos(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_list, const double coef_vx)
{
  double t_prev = rclcpp::Time(twist_list.front().header.stamp).seconds();
  geometry_msgs::msg::Point d_pos;
  double yaw = tf2::getYaw(pose_list.front().pose.orientation);
  for (std::size_t i = 0; i < twist_list.size() - 1; ++i) {
    const double t_cur = rclcpp::Time(twist_list[i + 1].header.stamp).seconds();
    yaw += twist_list[i].twist.angular.z * (t_cur - t_prev);

    d_pos.x += (t_cur - t_prev) * twist_list[i].twist.linear.x * std::cos(yaw) * coef_vx;
    d_pos.y += (t_cur - t_prev) * twist_list[i].twist.linear.x * std::sin(yaw) * coef_vx;

    t_prev = t_cur;
  }
  return d_pos;
}

geometry_msgs::msg::Vector3 calculateErrorRPY(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_list,
  const geometry_msgs::msg::Vector3 & gyro_bias)
{
  double roll_0 = 0, pitch_0 = 0, yaw_0 = 0;
  double roll_1 = 0, pitch_1 = 0, yaw_1 = 0;
  tf2::Quaternion q_tf_0, q_tf_1;
  tf2::fromMsg(pose_list.front().pose.orientation, q_tf_0);
  tf2::fromMsg(pose_list.back().pose.orientation, q_tf_1);
  tf2::Matrix3x3(q_tf_0).getRPY(roll_0, pitch_0, yaw_0);
  tf2::Matrix3x3(q_tf_1).getRPY(roll_1, pitch_1, yaw_1);

  double d_roll = 0, d_pitch = 0, d_yaw = 0;
  double t_prev = rclcpp::Time(twist_list.front().header.stamp).seconds();
  for (std::size_t i = 0; i < twist_list.size() - 1; ++i) {
    double t_cur = rclcpp::Time(twist_list[i + 1].header.stamp).seconds();

    d_roll += (t_cur - t_prev) * (twist_list[i].twist.angular.x - gyro_bias.x);
    d_pitch += (t_cur - t_prev) * (twist_list[i].twist.angular.y - gyro_bias.y);
    d_yaw += (t_cur - t_prev) * (twist_list[i].twist.angular.z - gyro_bias.z);

    t_prev = t_cur;
  }
  geometry_msgs::msg::Vector3 error_rpy;
  error_rpy.x = clipRadian(-roll_1 + roll_0 + d_roll);
  error_rpy.y = clipRadian(-pitch_1 + pitch_0 + d_pitch);
  error_rpy.z = clipRadian(-yaw_1 + yaw_0 + d_yaw);
  return error_rpy;
}

double getMeanAbsVx(const std::vector<geometry_msgs::msg::TwistStamped> & twist_list)
{
  double mean_abs_vx = 0;
  for (const auto & msg : twist_list) {
    mean_abs_vx += abs(msg.twist.linear.x);
  }
  mean_abs_vx /= twist_list.size();
  return mean_abs_vx;
}

double getMeanAbsWz(const std::vector<geometry_msgs::msg::TwistStamped> & twist_list)
{
  double mean_abs_wz = 0;
  for (auto msg : twist_list) {
    mean_abs_wz += abs(msg.twist.angular.z);
  }
  mean_abs_wz /= twist_list.size();
  return mean_abs_wz;
}
