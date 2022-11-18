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

#ifndef DEVIATION_ESTIMATOR__UTILS_HPP_
#define DEVIATION_ESTIMATOR__UTILS_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <tf2/transform_datatypes.h>

#include <fstream>
#include <vector>

double double_round(const double x, const int n);

template <typename T>
double calculateMean(const std::vector<T> & v)
{
  if (v.size() == 0) {
    return 0;
  }

  double mean = 0;
  for (const T & t : v) {
    mean += t;
  }
  mean /= v.size();
  return mean;
}

template <typename T>
double calculateStd(const std::vector<T> & v)
{
  if (v.size() == 0) {
    return 0;
  }

  const double mean = calculateMean(v);
  double error = 0;
  for (const T & t : v) {
    error += pow(t - mean, 2);
  }
  return std::sqrt(error / v.size());
}

template <typename T>
double calculateStdMeanConst(const std::vector<T> & v, const double mean)
{
  if (v.size() == 0) {
    return 0;
  }

  double error = 0;
  for (const T & t : v) {
    error += pow(t - mean, 2);
  }
  return std::sqrt(error / v.size());
}

struct CompareMsgTimestamp
{
  template <typename T1>
  bool operator()(T1 const & t1, double const & t2) const
  {
    return rclcpp::Time(t1.header.stamp).seconds() < t2;
  }

  template <typename T2>
  bool operator()(double const & t1, T2 const & t2) const
  {
    return t1 < rclcpp::Time(t2.header.stamp).seconds();
  }

  template <typename T1, typename T2>
  bool operator()(T1 const & t1, T2 const & t2) const
  {
    return rclcpp::Time(t1.header.stamp).seconds() < rclcpp::Time(t2.header.stamp).seconds();
  }

  template <typename T1>
  bool operator()(T1 const & t1, rclcpp::Time const & t2) const
  {
    return rclcpp::Time(t1.header.stamp).seconds() < t2.seconds();
  }

  template <typename T2>
  bool operator()(rclcpp::Time const & t1, T2 const & t2) const
  {
    return t1.seconds() < rclcpp::Time(t2.header.stamp).seconds();
  }
};

template <typename T>
std::vector<T> extractSubTrajectory(
  const std::vector<T> & msg_list, const rclcpp::Time & t0, const rclcpp::Time & t1)
{
  const auto start_iter =
    std::lower_bound(msg_list.begin(), msg_list.end(), t0, CompareMsgTimestamp());
  const auto end_iter =
    std::lower_bound(msg_list.begin(), msg_list.end(), t1, CompareMsgTimestamp());
  std::vector<T> msg_list_sub(start_iter, end_iter);
  return msg_list_sub;
}

template <typename T, typename U>
double norm_xy(const T p1, const U p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double clipRadian(const double rad);

void saveEstimatedParameters(
  const std::string output_path, const double stddev_vx, const double stddev_wz,
  const double coef_vx, const double bias_wz,
  const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
  const geometry_msgs::msg::Vector3 & angular_velocity_offset);

geometry_msgs::msg::Point calculateErrorPos(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_list);

geometry_msgs::msg::Vector3 calculateErrorRPY(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_list);

#endif  // DEVIATION_ESTIMATOR__UTILS_HPP_
