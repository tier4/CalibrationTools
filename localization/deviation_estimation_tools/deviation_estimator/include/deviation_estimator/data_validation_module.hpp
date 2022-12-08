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

#ifndef DEVIATION_ESTIMATOR__DATA_VALIDATION_MODULE_HPP_
#define DEVIATION_ESTIMATOR__DATA_VALIDATION_MODULE_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

class DataValidationModule
{
public:
  DataValidationModule(rclcpp::Node * node);
  void update_pose(const geometry_msgs::msg::PoseStamped & pose);
  bool is_data_valid();

private:
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_ptr_;
  double distance_travelled_;

  const double distance_travelled_threshold_;
};

#endif  // DEVIATION_ESTIMATOR__DATA_VALIDATION_MODULE_HPP_
