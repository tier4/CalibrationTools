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


#include "deviation_estimator/gyro_bias_module.hpp"
#include "deviation_estimator/utils.hpp"

GyroBiasModule::GyroBiasModule(){}

void GyroBiasModule::update_bias(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_list,
  const double dt)
{
  const auto error_rpy = calculateErrorRPY(pose_list, twist_list);
  gyro_bias_pair_.first.x += dt * error_rpy.x;
  gyro_bias_pair_.first.y += dt * error_rpy.y;
  gyro_bias_pair_.first.z += dt * error_rpy.z;
  gyro_bias_pair_.second.x += dt * dt;
  gyro_bias_pair_.second.y += dt * dt;
  gyro_bias_pair_.second.z += dt * dt;

  geometry_msgs::msg::Vector3 gyro_bias;
  gyro_bias.x = error_rpy.x / dt;
  gyro_bias.y = error_rpy.y / dt;
  gyro_bias.z = error_rpy.z / dt;
  gyro_bias_list.push_back(gyro_bias);
}

geometry_msgs::msg::Vector3 GyroBiasModule::get_bias_base_link() const
{
  geometry_msgs::msg::Vector3 gyro_bias_base;
  gyro_bias_base.x = gyro_bias_pair_.first.x / gyro_bias_pair_.second.z;
  gyro_bias_base.y = gyro_bias_pair_.first.y / gyro_bias_pair_.second.y;
  gyro_bias_base.z = gyro_bias_pair_.first.z / gyro_bias_pair_.second.z;
  return gyro_bias_base;
}

geometry_msgs::msg::Vector3 GyroBiasModule::get_bias_std() const
{
  std::vector<double> stddev_bias_list_x, stddev_bias_list_y, stddev_bias_list_z;
  for (const auto & e: gyro_bias_list) {
    stddev_bias_list_x.push_back(e.x);
    stddev_bias_list_y.push_back(e.y);
    stddev_bias_list_z.push_back(e.z);
  }
  geometry_msgs::msg::Vector3 stddev_bias;
  stddev_bias.x = calculateStdMeanConst(
    stddev_bias_list_x, gyro_bias_pair_.first.x / gyro_bias_pair_.second.x);
  stddev_bias.y = calculateStdMeanConst(
    stddev_bias_list_y, gyro_bias_pair_.first.x / gyro_bias_pair_.second.x);
  stddev_bias.z = calculateStdMeanConst(
    stddev_bias_list_z, gyro_bias_pair_.first.x / gyro_bias_pair_.second.x);
  return stddev_bias;
}

bool GyroBiasModule::empty() const
{
  return gyro_bias_list.empty();
}
