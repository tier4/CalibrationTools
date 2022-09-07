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

#include "calibration_adapter/calibration_adapter_node.hpp"

#include <tf2/utils.h>

#include <limits>
#include <memory>
#include <vector>

CalibrationAdapterNode::CalibrationAdapterNode()
{
  using std::placeholders::_1;

  // QoS setup
  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);
  durable_qos.transient_local();  // option for latching
  lowpass_cutoff_value_ = this->declare_parameter<double>("lowpass_cutoff_value", 0.033);

  pub_steering_angle_cmd_ =
    create_publisher<Float32Stamped>("~/output/steering_angle_cmd", durable_qos);
  pub_acceleration_status_ =
    create_publisher<Float32Stamped>("~/output/acceleration_status", durable_qos);
  pub_acceleration_cmd_ =
    create_publisher<Float32Stamped>("~/output/acceleration_cmd", durable_qos);

  sub_control_cmd_ = create_subscription<ControlCommandStamped>(
    "~/input/control_cmd", queue_size,
    std::bind(&CalibrationAdapterNode::callbackControlCmd, this, _1));
  sub_twist_ = create_subscription<Velocity>(
    "~/input/twist_status", queue_size,
    std::bind(&CalibrationAdapterNode::callbackTwistStatus, this, _1));
}

template <class T>
T CalibrationAdapterNode::getNearestTimeDataFromVec(
  const T base_data, const double back_time, const std::vector<T> & vec)
{
  double nearest_time = std::numeric_limits<double>::max();
  const double target_time = rclcpp::Time(base_data.header.stamp).seconds() - back_time;
  T nearest_time_data;
  for (const auto & data : vec) {
    const double data_time = rclcpp::Time(data.header.stamp).seconds();
    const auto delta_time = std::abs(target_time - data_time);
    if (nearest_time > delta_time) {
      nearest_time_data = data;
      nearest_time = delta_time;
    }
  }
  return nearest_time_data;
}

double CalibrationAdapterNode::getAccel(
  const TwistStamped & prev_twist, const TwistStamped & current_twist, const double dt)
{
  if (dt < 1e-03) {
    // invalid twist. return prev acceleration
    return acceleration_;
  }
  const double dv = current_twist.twist.linear.x - prev_twist.twist.linear.x;
  return dv / dt;
}

template <class T>
void CalibrationAdapterNode::pushDataToVec(
  const T data, const std::size_t max_size, std::vector<T> * vec)
{
  vec->emplace_back(data);
  while (vec->size() > max_size) {
    vec->erase(vec->begin());
  }
}

void CalibrationAdapterNode::callbackControlCmd(const ControlCommandStamped::ConstSharedPtr msg)
{
  Float32Stamped steer_angle_msg;
  steer_angle_msg.header.stamp = msg->stamp;
  steer_angle_msg.header.frame_id = "base_link";
  steer_angle_msg.data = msg->lateral.steering_tire_angle;
  pub_steering_angle_cmd_->publish(steer_angle_msg);

  Float32Stamped accel_msg;
  accel_msg.header.stamp = msg->stamp;
  accel_msg.header.frame_id = "base_link";
  accel_msg.data = msg->longitudinal.acceleration;
  pub_acceleration_cmd_->publish(accel_msg);
}

void CalibrationAdapterNode::callbackTwistStatus(const Velocity::ConstSharedPtr msg)
{
  TwistStamped twist;
  twist.header = msg->header;
  twist.twist.linear.x = msg->longitudinal_velocity;
  if (!twist_vec_.empty()) {
    const auto past_msg = getNearestTimeDataFromVec(twist, dif_twist_time_, twist_vec_);
    const double dt =
      (rclcpp::Time(msg->header.stamp) - rclcpp::Time(past_msg.header.stamp)).seconds();
    const double raw_acceleration = getAccel(past_msg, twist, dt);
    acceleration_ =
      math_utils::lowpassFilter(acceleration_, raw_acceleration, lowpass_cutoff_value_, dt);
  }
  pushDataToVec(twist, twist_vec_max_size_, &twist_vec_);

  Float32Stamped accel_status_msg;
  accel_status_msg.header.stamp = msg->header.stamp;
  accel_status_msg.data = acceleration_;
  pub_acceleration_status_->publish(accel_status_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibrationAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
