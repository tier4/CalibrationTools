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

Logger::Logger(
  const std::string & output_dir,
  const std::string & imu_topic)
: output_log_path_(output_dir + "/output.txt"),
  output_imu_param_path_(output_dir + "/imu_corrector.param.yaml"),
  imu_topic_(imu_topic)
{
  std::ofstream file_log(output_log_path_);
  file_log.close();

  std::ofstream file_imu_param(output_imu_param_path_);
  file_imu_param.close();
}

void Logger::log_estimated_result_section(
  const double stddev_vx, const double coef_vx,
  const geometry_msgs::msg::Vector3 & angular_velocity_stddev,
  const geometry_msgs::msg::Vector3 & angular_velocity_offset) const
{
  std::ofstream file_log(output_log_path_, std::ios::app);
  file_log << "# Results expressed in base_link\n";
  file_log << "# Copy the following to deviation_evaluator.param.yaml\n";
  file_log << fmt::format("stddev_vx: {:.5f}\n", stddev_vx);
  file_log << fmt::format("coef_vx: {:.5f}\n", coef_vx);
  file_log << fmt::format("imu_topic: {}\n", imu_topic_);
  file_log.close();

  std::ofstream file_imu_param(output_imu_param_path_, std::ios::app);
  file_imu_param << "# Estimated by deviation_estimator\n";
  file_imu_param << "/**:\n";
  file_imu_param << "  ros__parameters:\n";
  file_imu_param << fmt::format(
    "    angular_velocity_offset_x: {:.5f}\n", angular_velocity_offset.x);
  file_imu_param << fmt::format(
    "    angular_velocity_offset_y: {:.5f}\n", angular_velocity_offset.y);
  file_imu_param << fmt::format(
    "    angular_velocity_offset_z: {:.5f}\n", angular_velocity_offset.z);
  file_imu_param << fmt::format(
    "    angular_velocity_stddev_xx: {:.5f}\n", angular_velocity_stddev.x);
  file_imu_param << fmt::format(
    "    angular_velocity_stddev_yy: {:.5f}\n", angular_velocity_stddev.y);
  file_imu_param << fmt::format(
    "    angular_velocity_stddev_zz: {:.5f}\n", angular_velocity_stddev.z);
}

void Logger::log_validation_result_section(const ValidationModule & validation_module) const
{
  std::ofstream file(output_log_path_, std::ios::app);
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
