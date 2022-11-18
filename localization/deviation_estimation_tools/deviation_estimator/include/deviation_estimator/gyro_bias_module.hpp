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

#ifndef DEVIATION_ESTIMATOR__GYRO_BIAS_MODULE_HPP_
#define DEVIATION_ESTIMATOR__GYRO_BIAS_MODULE_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <vector>

class GyroBiasModule
{
public:
  GyroBiasModule();
  void update_bias(
    const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
    const std::vector<geometry_msgs::msg::TwistStamped> & twist_list, const double dt);
  geometry_msgs::msg::Vector3 get_bias_base_link() const;
  geometry_msgs::msg::Vector3 get_bias_std() const;
  bool empty() const;

private:
  std::vector<geometry_msgs::msg::Vector3> gyro_bias_list;
  std::pair<geometry_msgs::msg::Vector3, geometry_msgs::msg::Vector3> gyro_bias_pair_;
};

#endif  // DEVIATION_ESTIMATOR__GYRO_BIAS_MODULE_HPP_
