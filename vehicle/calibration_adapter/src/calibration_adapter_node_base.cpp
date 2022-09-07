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

#include "calibration_adapter/calibration_adapter_node_base.hpp"

#include <tf2/utils.h>

#include <memory>

CalibrationAdapterNodeBase::CalibrationAdapterNodeBase() : Node("calibration_adapter")
{
  using std::placeholders::_1;

  // QoS setup
  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);
  durable_qos.transient_local();  // option for latching

  pub_accel_status_ = create_publisher<tier4_calibration_msgs::msg::Float32Stamped>(
    "~/output/accel_status", durable_qos);
  pub_brake_status_ = create_publisher<tier4_calibration_msgs::msg::Float32Stamped>(
    "~/output/brake_status", durable_qos);
  pub_steer_status_ = create_publisher<tier4_calibration_msgs::msg::Float32Stamped>(
    "~/output/steer_status", durable_qos);
  pub_accel_cmd_ = create_publisher<tier4_calibration_msgs::msg::Float32Stamped>(
    "~/output/accel_cmd", durable_qos);
  pub_brake_cmd_ = create_publisher<tier4_calibration_msgs::msg::Float32Stamped>(
    "~/output/brake_cmd", durable_qos);
  pub_steer_cmd_ = create_publisher<tier4_calibration_msgs::msg::Float32Stamped>(
    "~/output/steer_cmd", durable_qos);

  // steering angle
  pub_steering_angle_status_ =
    create_publisher<Float32Stamped>("~/output/steering_angle_status", durable_qos);
  sub_steering_angle_status_ = create_subscription<SteeringAngleStatus>(
    "~/input/steering_angle_status", queue_size,
    std::bind(&CalibrationAdapterNodeBase::callbackSteeringAngleStatus, this, _1));

  pub_is_engage_ =
    create_publisher<tier4_calibration_msgs::msg::BoolStamped>("~/output/is_engage", durable_qos);

  sub_engage_status_ = create_subscription<autoware_auto_vehicle_msgs::msg::Engage>(
    "~/input/is_engage", queue_size,
    std::bind(&CalibrationAdapterNodeBase::onEngageStatus, this, _1));
  sub_actuation_status_ = create_subscription<ActuationStatusStamped>(
    "~/input/actuation_status", queue_size,
    std::bind(&CalibrationAdapterNodeBase::onActuationStatus, this, _1));
  sub_actuation_command_ = create_subscription<ActuationCommandStamped>(
    "~/input/actuation_command", queue_size,
    std::bind(&CalibrationAdapterNodeBase::onActuationCmd, this, _1));
}

void CalibrationAdapterNodeBase::callbackSteeringAngleStatus(
  const SteeringAngleStatus::ConstSharedPtr msg)
{
  Float32Stamped steer_status_msg;
  steer_status_msg.header.stamp = msg->stamp;
  steer_status_msg.data = msg->steering_tire_angle;
  pub_steering_angle_status_->publish(steer_status_msg);
}

void CalibrationAdapterNodeBase::onActuationCmd(const ActuationCommandStamped::ConstSharedPtr msg)
{
  Float32Stamped brake_msgs;
  brake_msgs.header = msg->header;
  brake_msgs.data = msg->actuation.brake_cmd;
  pub_brake_cmd_->publish(brake_msgs);

  Float32Stamped accel_msgs;
  accel_msgs.header = msg->header;
  accel_msgs.data = msg->actuation.accel_cmd;
  pub_accel_cmd_->publish(accel_msgs);

  Float32Stamped steer_msgs;
  steer_msgs.header = msg->header;
  steer_msgs.data = msg->actuation.steer_cmd;
  pub_steer_cmd_->publish(steer_msgs);
}

void CalibrationAdapterNodeBase::onActuationStatus(const ActuationStatusStamped::ConstSharedPtr msg)
{
  Float32Stamped accel_msgs;
  accel_msgs.header = msg->header;
  accel_msgs.data = msg->status.accel_status;
  pub_accel_status_->publish(accel_msgs);

  Float32Stamped brake_msgs;
  brake_msgs.header = msg->header;
  brake_msgs.data = msg->status.brake_status;
  pub_brake_status_->publish(brake_msgs);

  Float32Stamped steer_msgs;
  steer_msgs.header = msg->header;
  steer_msgs.data = msg->status.steer_status;
  pub_steer_status_->publish(steer_msgs);
}

void CalibrationAdapterNodeBase::onEngageStatus(const EngageStatus::SharedPtr msg)
{
  BoolStamped engage_msgs;
  engage_msgs.data = msg->engage;
  pub_is_engage_->publish(engage_msgs);
}
