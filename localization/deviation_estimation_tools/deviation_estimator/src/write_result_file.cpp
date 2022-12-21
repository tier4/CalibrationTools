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

#include "deviation_estimator/write_result_file.hpp"

#include <cmath>

double double_round(const double x, const int n)
{
  return std::round(x * std::pow(10, n)) / std::pow(10, n);
}

void save_estimated_result(
  const std::string output_path, const double stddev_vx, const double stddev_wz,
  const double coef_vx, const double bias_wz,
  const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
  const geometry_msgs::msg::Vector3 & angular_velocity_offset)
{
  std::ofstream file(output_path);
  file << generate_estimation_section(
    stddev_vx, stddev_wz, coef_vx, bias_wz, angular_velocity_stddev, angular_velocity_offset);
  file.close();
}

void save_estimated_result(
  const std::string output_path, const double stddev_vx, const double stddev_wz,
  const double coef_vx, const double bias_wz,
  const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
  const geometry_msgs::msg::Vector3 & angular_velocity_offset, const bool is_distance_valid)
{
  std::ofstream file(output_path);
  file << generate_validation_section(is_distance_valid);
  file << "\n";
  file << generate_estimation_section(
    stddev_vx, stddev_wz, coef_vx, bias_wz, angular_velocity_stddev, angular_velocity_offset);
  file.close();
}

std::string generate_estimation_section(
  const double stddev_vx, const double stddev_wz, const double coef_vx, const double bias_wz,
  const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
  const geometry_msgs::msg::Vector3 & angular_velocity_offset)
{
  std::string output_string;
  output_string += "# Results expressed in base_link\n";
  output_string += "# Copy the following to deviation_evaluator.param.yaml\n";
  output_string += fmt::format("stddev_vx: {}\n", double_round(stddev_vx, 5));
  output_string += fmt::format("stddev_wz: {}\n", double_round(stddev_wz, 5));
  output_string += fmt::format("coef_vx: {}\n", double_round(coef_vx, 5));
  output_string += fmt::format("bias_wz: {}\n", double_round(bias_wz, 5));
  output_string += "\n";
  output_string += "# Results expressed in imu_link\n";
  output_string += "# Copy the following to imu_corrector.param.yaml\n";
  output_string +=
    fmt::format("angular_velocity_stddev_xx: {}\n", double_round(angular_velocity_stddev.x, 5));
  output_string +=
    fmt::format("angular_velocity_stddev_yy: {}\n", double_round(angular_velocity_stddev.y, 5));
  output_string +=
    fmt::format("angular_velocity_stddev_zz: {}\n", double_round(angular_velocity_stddev.z, 5));
  output_string +=
    fmt::format("angular_velocity_offset_x: {}\n", double_round(angular_velocity_offset.x, 5));
  output_string +=
    fmt::format("angular_velocity_offset_y: {}\n", double_round(angular_velocity_offset.y, 5));
  output_string +=
    fmt::format("angular_velocity_offset_z: {}\n", double_round(angular_velocity_offset.z, 5));
  return output_string;
}
