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

#ifndef DEVIATION_ESTIMATOR__VELOCITY_COEF_MODULE_HPP_
#define DEVIATION_ESTIMATOR__VELOCITY_COEF_MODULE_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tier4_debug_msgs/msg/float64_stamped.hpp"

#include <vector>

class VelocityCoefModule
{
public:
  VelocityCoefModule() = default;
  void update_coef(
    const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
    const std::vector<tier4_debug_msgs::msg::Float64Stamped> & vx_list,
    const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
    const double dt);
  double get_coef() const;
  double get_coef_std() const;
  bool empty() const;

private:
  std::vector<double> coef_vx_list_;
  std::pair<double, double> coef_vx_;
};

#endif  // DEVIATION_ESTIMATOR__VELOCITY_COEF_MODULE_HPP_
