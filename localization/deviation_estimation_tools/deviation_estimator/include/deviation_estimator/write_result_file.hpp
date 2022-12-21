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

#ifndef DEVIATION_ESTIMATOR__WRITE_RESULT_FILE_HPP_
#define DEVIATION_ESTIMATOR__WRITE_RESULT_FILE_HPP_

#include "geometry_msgs/msg/vector3.hpp"

#include <fmt/core.h>

#include <fstream>
#include <string>

void save_estimated_result(
  const std::string output_path, const double stddev_vx, const double stddev_wz,
  const double coef_vx, const double bias_wz,
  const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
  const geometry_msgs::msg::Vector3 & angular_velocity_offset);

void save_estimated_result(
  const std::string output_path, const double stddev_vx, const double stddev_wz,
  const double coef_vx, const double bias_wz,
  const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
  const geometry_msgs::msg::Vector3 & angular_velocity_offset, const bool is_distance_valid);

std::string generate_estimation_section(
  const double stddev_vx, const double stddev_wz, const double coef_vx, const double bias_wz,
  const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
  const geometry_msgs::msg::Vector3 & angular_velocity_offset);

#endif  // DEVIATION_ESTIMATOR__WRITE_RESULT_FILE_HPP_
