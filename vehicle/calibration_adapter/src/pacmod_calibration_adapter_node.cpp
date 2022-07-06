//
// Copyright 2020 Tier IV, Inc. All rights reserved.
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
//

#include <tf2/utils.h>
#include <memory>
#include "calibration_adapter/pacmod_calibration_adapter_node.hpp"

PacmodCalibrationAdapterNode::PacmodCalibrationAdapterNode()
{
  using std::placeholders::_1;

  // QoS setup
  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);
  durable_qos.transient_local();  // option for latching

  pub_handle_status_ =
    create_publisher<Float32Stamped>(
    "~/output/handle_status", durable_qos);
  sub_handle_status_ = create_subscription<SteeringWheelStatusStamped>(
    "~/input/handle_status", queue_size,
    std::bind(&PacmodCalibrationAdapterNode::callbackSteeringWheelStatus, this, _1));
}

void PacmodCalibrationAdapterNode::callbackSteeringWheelStatus(
  const SteeringWheelStatusStamped::ConstSharedPtr msg)
{
  tier4_calibration_msgs::msg::Float32Stamped steer_msgs;
  steer_msgs.header.stamp = msg->stamp;
  steer_msgs.header.frame_id ="base_link";
  steer_msgs.data = msg->data;
  pub_handle_status_->publish(steer_msgs);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PacmodCalibrationAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
