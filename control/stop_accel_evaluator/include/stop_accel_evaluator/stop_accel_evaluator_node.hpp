// Copyright 2021 Tier IV, Inc.
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

#ifndef STOP_ACCEL_EVALUATOR__STOP_ACCEL_EVALUATOR_NODE_HPP_
#define STOP_ACCEL_EVALUATOR__STOP_ACCEL_EVALUATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "signal_processing/lowpass_filter_1d.hpp"
#include "tier4_autoware_utils/ros/self_pose_listener.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"

#include <deque>
#include <memory>

namespace stop_accel_evaluator
{
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using tier4_autoware_utils::SelfPoseListener;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using tier4_debug_msgs::msg::Float32Stamped;

class StopAccelEvaluatorNode : public rclcpp::Node
{
public:
  explicit StopAccelEvaluatorNode(const rclcpp::NodeOptions & node_options);

private:
  std::unique_ptr<TwistStamped> current_vel_ptr_{nullptr};
  std::unique_ptr<TwistStamped> prev_vel_ptr_{nullptr};

  double stop_accel_{0.0};
  double stop_accel_with_gravity_{0.0};

  rclcpp::Subscription<Odometry>::SharedPtr sub_twist_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<Float32MultiArrayStamped>::SharedPtr sub_debug_values_;

  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_stop_accel_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_stop_accel_with_gravity_;

  std::deque<double> imu_deque_;

  size_t imu_deque_size_;
  double not_running_acc_;
  double not_running_vel_;
  int stop_valid_imu_accel_num_;

  double current_acc_{0.0};
  std::shared_ptr<LowpassFilter1d> lpf_acc_{nullptr};
  std::shared_ptr<LowpassFilter1d> lpf_pitch_{nullptr};

  SelfPoseListener self_pose_listener_{this};

  void onTwist(const Odometry::ConstSharedPtr msg);
  void onImu(const Imu::ConstSharedPtr msg);

  void calculateStopAccel();
  void publishStopAccel() const;
};
}  // namespace stop_accel_evaluator

#endif  // STOP_ACCEL_EVALUATOR__STOP_ACCEL_EVALUATOR_NODE_HPP_
