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

#include "stop_accel_evaluator/stop_accel_evaluator_node.hpp"

#include "tier4_autoware_utils/math/constants.hpp"

#include <algorithm>
#include <memory>
#include <utility>

namespace
{
double getPitchByQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  const Eigen::Vector3d v = q.toRotationMatrix() * Eigen::Vector3d::UnitX();
  const double den = std::max(std::hypot(v.x(), v.y()), 1e-8);
  const double pitch = -1.0 * std::atan2(v.z(), den);
  return pitch;
}
}  // namespace

namespace stop_accel_evaluator
{
StopAccelEvaluatorNode::StopAccelEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("stop_accel_evaluator", node_options)
{
  using std::placeholders::_1;

  imu_deque_size_ = static_cast<size_t>(this->declare_parameter("imu_deque_size", 30));
  not_running_acc_ = this->declare_parameter("not_running_acc", 0.005);
  not_running_vel_ = this->declare_parameter("not_running_vel", 0.1);
  stop_valid_imu_accel_num_ = this->declare_parameter("stop_valid_imu_accel_num", 4);

  {  // lowpass filter
    lpf_pitch_ = std::make_shared<LowpassFilter1d>(this->declare_parameter("lpf_pitch_gain", 0.95));
    lpf_acc_ = std::make_shared<LowpassFilter1d>(this->declare_parameter("lpf_acc_gain", 0.2));
  }

  sub_twist_ = this->create_subscription<Odometry>(
    "~/twist", 1, std::bind(&StopAccelEvaluatorNode::onTwist, this, _1));
  sub_imu_ =
    this->create_subscription<Imu>("~/imu", 1, std::bind(&StopAccelEvaluatorNode::onImu, this, _1));

  pub_stop_accel_ = this->create_publisher<Float32Stamped>("~/stop_accel", 1);
  pub_stop_accel_with_gravity_ =
    this->create_publisher<Float32Stamped>("~/stop_accel_with_gravity", 1);

  self_pose_listener_.waitForFirstPose();
}

void StopAccelEvaluatorNode::onTwist(const Odometry::ConstSharedPtr msg)
{
  if (current_vel_ptr_) {
    prev_vel_ptr_ = std::move(current_vel_ptr_);
  }
  current_vel_ptr_ = std::unique_ptr<TwistStamped>();
  current_vel_ptr_->header = msg->header;
  current_vel_ptr_->twist = msg->twist.twist;
}

void StopAccelEvaluatorNode::onImu(const Imu::ConstSharedPtr msg)
{
  if (!current_vel_ptr_ || !prev_vel_ptr_) {
    return;
  }

  // save x acceleration of imu
  imu_deque_.push_back(-msg->linear_acceleration.x);
  while (imu_deque_.size() > imu_deque_size_) {
    imu_deque_.pop_front();
  }

  calculateStopAccel();
}

void StopAccelEvaluatorNode::calculateStopAccel()
{
  // calculate accel
  const double prev_acc = current_acc_;
  const double dv = current_vel_ptr_->twist.linear.x - prev_vel_ptr_->twist.linear.x;
  const double dt = std::max(
    (rclcpp::Time(current_vel_ptr_->header.stamp) - rclcpp::Time(prev_vel_ptr_->header.stamp))
      .seconds(),
    1e-03);
  const double accel = dv / dt;
  current_acc_ = lpf_acc_->filter(accel);

  // calculate when the car has just stopped
  const bool just_stop_flag = (std::abs(current_acc_) < not_running_acc_) &&
                              (std::abs(prev_acc) > not_running_acc_) &&
                              (std::abs(prev_vel_ptr_->twist.linear.x) < not_running_vel_);
  if (just_stop_flag) {
    int accel_num = 0;
    double accel_sum = 0;
    bool minus_accel_flag = false;
    for (size_t t = 0; t < imu_deque_.size(); ++t) {
      const double acc = imu_deque_.at(imu_deque_.size() - 1 - t);
      if (acc < 0) {
        minus_accel_flag = true;
      }

      if (minus_accel_flag) {
        accel_sum += acc;
        accel_num += 1;

        // calculate stop accel with/without gravity
        if (accel_num == stop_valid_imu_accel_num_) {
          const auto current_pose_ptr = self_pose_listener_.getCurrentPose();

          const double pitch =
            lpf_pitch_->filter(getPitchByQuaternion(current_pose_ptr->pose.orientation));

          stop_accel_ = accel_sum / accel_num + tier4_autoware_utils::gravity *
                                                  std::sin(pitch);  // consider removing gravity
          stop_accel_with_gravity_ = accel_sum / accel_num;         // not consider removing gravity

          publishStopAccel();
          return;
        }
      }
    }
  }
}

void StopAccelEvaluatorNode::publishStopAccel() const
{
  auto msg_with_gravity = std::make_unique<Float32Stamped>();
  msg_with_gravity->stamp = this->now();
  msg_with_gravity->data = stop_accel_with_gravity_;
  pub_stop_accel_with_gravity_->publish(std::move(msg_with_gravity));

  auto msg = std::make_unique<Float32Stamped>();
  msg->stamp = this->now();
  msg->data = stop_accel_;
  pub_stop_accel_->publish(std::move(msg));
}
}  // namespace stop_accel_evaluator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stop_accel_evaluator::StopAccelEvaluatorNode)
