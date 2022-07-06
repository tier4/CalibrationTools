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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tunable_static_tf_broadcaster/tunable_static_tf_broadcaster_node.hpp"


namespace tunable_static_tf_broadcaster
{
TunableStaticTfBroadcasterNode::TunableStaticTfBroadcasterNode(
  const rclcpp::NodeOptions & node_options)
: Node("tunable_static_tf_broadcaster_node", node_options)
{
  using std::placeholders::_1;

  // Initialize transform
  {
    transform_.transform.translation.x = this->declare_parameter("tf_x", 0.0);
    transform_.transform.translation.y = this->declare_parameter("tf_y", 0.0);
    transform_.transform.translation.z = this->declare_parameter("tf_z", 0.0);
    transform_.transform.rotation = createQuaternionFromRPY(
      this->declare_parameter_with_min_max("tf_roll", 0.0, -6.3, 6.3),
      this->declare_parameter_with_min_max("tf_pitch", 0.0, -6.3, 6.3),
      this->declare_parameter_with_min_max("tf_yaw", 0.0, -6.3, 6.3));
  }

  // Get configuration from param
  publish_rate_ = this->declare_parameter("rate", 10.0);
  transform_.header.frame_id = this->declare_parameter("header_frame", "world");
  transform_.child_frame_id = this->declare_parameter("child_frame", "base_link");

  RCLCPP_DEBUG(
    this->get_logger(), "Setting parameters... {rate: %f, header_frame: %s, child_frame: %s}",
    publish_rate_, transform_.header.frame_id.c_str(), transform_.child_frame_id.c_str());

  // Timer
  {
    double dt = 1.0 / publish_rate_;
    auto on_timer = std::bind(&TunableStaticTfBroadcasterNode::onTimer, this);
    const auto period_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt));
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer)>>(
      this->get_clock(), period_ns, std::move(on_timer),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);
  }

  // Parameter reconfigure
  set_param_res_ =
    add_on_set_parameters_callback(
    std::bind(&TunableStaticTfBroadcasterNode::onParameter, this, _1));
}

double TunableStaticTfBroadcasterNode::declare_parameter_with_min_max(
  const std::string & name, const double default_value, const double min_value,
  const double max_value,
  const std::string & description_name, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.name = description_name;
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = description;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = min_value;
  range.to_value = max_value;
  desc.floating_point_range.push_back(range);
  return this->declare_parameter(name, default_value, desc);
}

void TunableStaticTfBroadcasterNode::onTimer()
{
  // Send transform
  transform_.header.stamp =
    this->now() + rclcpp::Duration::from_seconds(1 / publish_rate_);    // Set future time to allow
  // slower sending w/o other node's timeout.
  broadcaster_.sendTransform(transform_);
}

SetParametersResult TunableStaticTfBroadcasterNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto update_param = [&parameters](const std::string & name, double & v) {
      const auto it = std::find_if(
        parameters.cbegin(), parameters.cend(),
        [&name](const rclcpp::Parameter & parameter) {return parameter.get_name() == name;});
      if (it != parameters.cend()) {
        v = it->as_double();
      }
    };

  SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    auto & t = transform_;
    update_param("tf_x", t.transform.translation.x);
    update_param("tf_y", t.transform.translation.y);
    update_param("tf_z", t.transform.translation.z);
    tf2::Quaternion quaternion{};
    tf2::fromMsg(t.transform.rotation, quaternion);
    double roll{};
    double pitch{};
    double yaw{};
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    update_param("tf_roll", roll);
    update_param("tf_pitch", pitch);
    update_param("tf_yaw", yaw);
    t.transform.rotation = createQuaternionFromRPY(roll, pitch, yaw);
    RCLCPP_DEBUG(
      this->get_logger(),
      "Setting parameters by params... {x: %lf y: %lf z: %lf roll: %lf pitch: %lf yaw: %lf}",
      t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
      roll, pitch, yaw);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

}  // namespace tunable_static_tf_broadcaster

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tunable_static_tf_broadcaster::TunableStaticTfBroadcasterNode)
