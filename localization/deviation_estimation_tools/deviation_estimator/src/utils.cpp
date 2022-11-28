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
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <iostream>
#include <vector>

double double_round(const double x, const int n) { return std::round(x * pow(10, n)) / pow(10, n); }

double clip_radian(const double rad)
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
 * save_estimated_parameters
 */
void save_estimated_parameters(
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

geometry_msgs::msg::Vector3 interpolate_vector3_stamped(
  const std::vector<geometry_msgs::msg::Vector3Stamped> & vec_list, const double time,
  const double tolerance_sec = 0.1)
{
  std::vector<double> time_list{};
  for (const auto & vec : vec_list) {
    time_list.push_back(rclcpp::Time(vec.header.stamp).seconds());
  }

  const int next_idx =
    std::upper_bound(time_list.begin(), time_list.end(), time) - time_list.begin();

  if (next_idx == 0) {
    if (time_list.front() - time > tolerance_sec) {
      throw std::domain_error("interpolate_vector3_stamped failed! Query time is too small.");
    }
    return vec_list.front().vector;
  } else if (next_idx == time_list.end() - time_list.begin()) {
    if (time - time_list.back() > tolerance_sec) {
      throw std::domain_error("interpolate_vector3_stamped failed! Query time is too large.");
    }
    return vec_list.back().vector;
  } else {
    const int prev_idx = next_idx - 1;
    const double ratio = (time - time_list[prev_idx]) / (time_list[next_idx] - time_list[prev_idx]);
    geometry_msgs::msg::Point interpolated_vec = tier4_autoware_utils::calcInterpolatedPoint(
      vec_list[prev_idx].vector, vec_list[next_idx].vector, ratio);

    return tier4_autoware_utils::createVector3(
      interpolated_vec.x, interpolated_vec.y, interpolated_vec.z);
  }
}

geometry_msgs::msg::Point integrate_position(
  const std::vector<tier4_debug_msgs::msg::Float64Stamped> & vx_list,
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list, const double coef_vx,
  const double yaw_init)
{
  double t_prev = rclcpp::Time(vx_list.front().stamp).seconds();
  double yaw = yaw_init;
  geometry_msgs::msg::Point d_pos = tier4_autoware_utils::createPoint(0.0, 0.0, 0.0);
  for (std::size_t i = 0; i < vx_list.size() - 1; ++i) {
    const double t_cur = rclcpp::Time(vx_list[i + 1].stamp).seconds();
    const geometry_msgs::msg::Vector3 gyro_interpolated =
      interpolate_vector3_stamped(gyro_list, rclcpp::Time(vx_list[i + 1].stamp).seconds());
    yaw += gyro_interpolated.z * (t_cur - t_prev);

    d_pos.x += (t_cur - t_prev) * vx_list[i].data * std::cos(yaw) * coef_vx;
    d_pos.y += (t_cur - t_prev) * vx_list[i].data * std::sin(yaw) * coef_vx;

    t_prev = t_cur;
  }
  return d_pos;
}

geometry_msgs::msg::Vector3 calculate_error_rpy(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
  const geometry_msgs::msg::Vector3 & gyro_bias)
{
  const geometry_msgs::msg::Vector3 rpy_0 =
    tier4_autoware_utils::getRPY(pose_list.front().pose.orientation);
  const geometry_msgs::msg::Vector3 rpy_1 =
    tier4_autoware_utils::getRPY(pose_list.back().pose.orientation);
  const geometry_msgs::msg::Vector3 d_rpy = integrate_orientation(gyro_list, gyro_bias);

  geometry_msgs::msg::Vector3 error_rpy = tier4_autoware_utils::createVector3(
    clip_radian(-rpy_1.x + rpy_0.x + d_rpy.x), clip_radian(-rpy_1.y + rpy_0.y + d_rpy.y),
    clip_radian(-rpy_1.z + rpy_0.z + d_rpy.z));
  return error_rpy;
}

geometry_msgs::msg::Vector3 integrate_orientation(
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
  const geometry_msgs::msg::Vector3 & gyro_bias)
{
  geometry_msgs::msg::Vector3 d_rpy = tier4_autoware_utils::createVector3(0.0, 0.0, 0.0);
  double t_prev = rclcpp::Time(gyro_list.front().header.stamp).seconds();
  for (std::size_t i = 0; i < gyro_list.size() - 1; ++i) {
    double t_cur = rclcpp::Time(gyro_list[i + 1].header.stamp).seconds();

    d_rpy.x += (t_cur - t_prev) * (gyro_list[i].vector.x - gyro_bias.x);
    d_rpy.y += (t_cur - t_prev) * (gyro_list[i].vector.y - gyro_bias.y);
    d_rpy.z += (t_cur - t_prev) * (gyro_list[i].vector.z - gyro_bias.z);

    t_prev = t_cur;
  }
  return d_rpy;
}

double get_mean_abs_vx(const std::vector<tier4_debug_msgs::msg::Float64Stamped> & vx_list)
{
  double mean_abs_vx = 0;
  for (const auto & msg : vx_list) {
    mean_abs_vx += abs(msg.data);
  }
  mean_abs_vx /= vx_list.size();
  return mean_abs_vx;
}

double get_mean_abs_wz(const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list)
{
  double mean_abs_wz = 0;
  for (auto & msg : gyro_list) {
    mean_abs_wz += abs(msg.vector.z);
  }
  mean_abs_wz /= gyro_list.size();
  return mean_abs_wz;
}

geometry_msgs::msg::Vector3 transform_vector3(
  const geometry_msgs::msg::Vector3 & vec, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Vector3Stamped vec_stamped;
  vec_stamped.vector = vec;

  geometry_msgs::msg::Vector3Stamped vec_stamped_transformed;
  tf2::doTransform(vec_stamped, vec_stamped_transformed, transform);
  return vec_stamped_transformed.vector;
}

/* This function is copied from tf2_geometry_msgs package, which should ideally be avoided.
   However, at the time of implementation, we got the "undefined reference" error for fromMsg, 
   and thus TEMPORARILY ported the function here. */
inline
void myFromMsg(const geometry_msgs::msg::Transform & in, tf2::Transform & out)
{
  out.setOrigin(tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
  // w at the end in the constructor
  out.setRotation(tf2::Quaternion(in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));
}

geometry_msgs::msg::TransformStamped inverse_transform(
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Transform tf;
  myFromMsg(transform.transform, tf);
  geometry_msgs::msg::TransformStamped transform_inv;
  transform_inv.header = transform.header;
  transform_inv.transform = tf2::toMsg(tf.inverse());
  return transform_inv;
}
