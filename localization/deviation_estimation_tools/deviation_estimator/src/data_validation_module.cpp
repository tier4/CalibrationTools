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

#include "deviation_estimator/data_validation_module.hpp"

#include "deviation_estimator/utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

DataValidationModule::DataValidationModule(rclcpp::Node * node)
: distance_travelled_(0.0),
  distance_travelled_threshold_(node->declare_parameter<double>("distance_travelled_threshold"))
{
}

void DataValidationModule::update_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  distance_travelled_ += norm_xy(pose.pose.position, latest_pose_ptr_->pose.position);
  latest_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(pose);
}

bool DataValidationModule::is_data_valid()
{
  bool is_travelled_distance_valid = distance_travelled_threshold_ < distance_travelled_;

  return is_travelled_distance_valid;
}
