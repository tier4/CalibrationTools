// Copyright 2020 Tier IV, Inc.
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
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TUNABLE_STATIC_TF_BROADCASTER__TUNABLE_STATIC_TF_BROADCASTER_NODE_HPP_
#define TUNABLE_STATIC_TF_BROADCASTER__TUNABLE_STATIC_TF_BROADCASTER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace tunable_static_tf_broadcaster
{
using tier4_autoware_utils::createQuaternionFromRPY;
using geometry_msgs::msg::TransformStamped;
using rcl_interfaces::msg::SetParametersResult;
using tf2_ros::TransformBroadcaster;

class TunableStaticTfBroadcasterNode : public rclcpp::Node
{
public:
  explicit TunableStaticTfBroadcasterNode(const rclcpp::NodeOptions & node_options);

private:
  TransformBroadcaster broadcaster_{*this};
  TransformStamped transform_;
  double publish_rate_;

  // Utility
  double declare_parameter_with_min_max(
    const std::string & name, const double default_value, const double min_value,
    const double max_value,
    const std::string & description_name = "", const std::string & description = "");

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();

  // Parameter reconfigure
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  SetParametersResult onParameter(const std::vector<rclcpp::Parameter> & parameters);
};
}  // namespace tunable_static_tf_broadcaster

#endif  // TUNABLE_STATIC_TF_BROADCASTER__TUNABLE_STATIC_TF_BROADCASTER_NODE_HPP_
