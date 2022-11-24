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

#include "deviation_estimator/velocity_coef_module.hpp"

#include "deviation_estimator/utils.hpp"

void VelocityCoefModule::update_coef(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_list, const double dt)
{
  const auto d_pos = calculate_error_pos(pose_list, twist_list, 1.0);
  double dx = pose_list.back().pose.position.x - pose_list.front().pose.position.x;
  double dy = pose_list.back().pose.position.y - pose_list.front().pose.position.y;
  if (d_pos.x * d_pos.x + d_pos.y * d_pos.y == 0) return;

  double d_coef_vx = (d_pos.x * dx + d_pos.y * dy) / (d_pos.x * d_pos.x + d_pos.y * d_pos.y);

  double time_factor = (rclcpp::Time(twist_list.back().header.stamp).seconds() -
                        rclcpp::Time(twist_list.front().header.stamp).seconds()) /
                       dt;
  coef_vx_.first += d_coef_vx * time_factor;
  coef_vx_.second += 1;
  coef_vx_list_.push_back(d_coef_vx);
}

double VelocityCoefModule::get_coef() const
{
  if (coef_vx_.second == 0) return 1.0;
  return coef_vx_.first / coef_vx_.second;
}

double VelocityCoefModule::get_coef_std() const { return calculate_std(coef_vx_list_); }

bool VelocityCoefModule::empty() const { return coef_vx_.second == 0; }
