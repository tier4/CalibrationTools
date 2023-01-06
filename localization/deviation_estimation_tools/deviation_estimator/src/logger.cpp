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

#include "deviation_estimator/logger.hpp"

#include <cmath>

Logger::Logger(const std::string output_path) : output_path_(output_path)
{
  std::ofstream file(output_path_);
  file.close();
}

void Logger::log_estimated_result_section(
  const double stddev_vx, const double stddev_wz, const double coef_vx, const double bias_wz,
  const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
  const geometry_msgs::msg::Vector3 & angular_velocity_offset) const
{
  std::ofstream file(output_path_, std::ios::app);
  file << "# Results expressed in base_link\n";
  file << "# Copy the following to deviation_evaluator.param.yaml\n";
  file << fmt::format("stddev_vx: {}\n", double_round(stddev_vx, 5));
  file << fmt::format("stddev_wz: {}\n", double_round(stddev_wz, 5));
  file << fmt::format("coef_vx: {}\n", double_round(coef_vx, 5));
  file << fmt::format("bias_wz: {}\n", double_round(bias_wz, 5));
  file << "\n";
  file << "# Results expressed in imu_link\n";
  file << "# Copy the following to imu_corrector.param.yaml\n";
  file << fmt::format(
    "angular_velocity_stddev_xx: {}\n", double_round(angular_velocity_stddev.x, 5));
  file << fmt::format(
    "angular_velocity_stddev_yy: {}\n", double_round(angular_velocity_stddev.y, 5));
  file << fmt::format(
    "angular_velocity_stddev_zz: {}\n", double_round(angular_velocity_stddev.z, 5));
  file << fmt::format(
    "angular_velocity_offset_x: {}\n", double_round(angular_velocity_offset.x, 5));
  file << fmt::format(
    "angular_velocity_offset_y: {}\n", double_round(angular_velocity_offset.y, 5));
  file << fmt::format(
    "angular_velocity_offset_z: {}\n", double_round(angular_velocity_offset.z, 5));
  file.close();
}

void Logger::log_validation_result_section(const ValidationModule & validation_module) const
{
  std::ofstream file(output_path_, std::ios::app);
  file << "\n# Validation results\n";
  file << "# value: [min, max]\n";

  std::vector<std::string> keys{
    "coef_vx",
    "stddev_vx",
    "angular_velocity_offset_x",
    "angular_velocity_offset_y",
    "angular_velocity_offset_z",
    "angular_velocity_stddev_xx",
    "angular_velocity_stddev_yy",
    "angular_velocity_stddev_zz"};

  for (const std::string & key : keys) {
    try {
      const auto min_max = validation_module.get_min_max(key);
      if (validation_module.is_valid(key)) {
        file << "[OK] ";
      } else {
        file << "[NG] ";
      }
      file << fmt::format(
        "{}: [{}, {}]\n", key, double_round(min_max.first, 5), double_round(min_max.second, 5));
    } catch (std::domain_error & e) {  // if the data is not enough
      file << fmt::format("[NG] {}: Not enough data provided yet\n", key);
    }
  }
  file.close();
}
