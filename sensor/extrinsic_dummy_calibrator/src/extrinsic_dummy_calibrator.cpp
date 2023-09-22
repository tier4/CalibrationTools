// Copyright 2023 Tier IV, Inc.
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

#include "extrinsic_dummy_calibrator/extrinsic_dummy_calibrator.hpp"

#include <limits>
#include <memory>

namespace extrinsic_dummy_calibrator
{
ExtrinsicDummyCalibrator::ExtrinsicDummyCalibrator(const rclcpp::NodeOptions & node_options)
: Node("extrinsic_dummy_calibrator", node_options)
{
  // set launch param
  parent_frame_ = this->declare_parameter("parent_frame", "");
  child_frame_ = this->declare_parameter("child_frame", "");

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscription_option = rclcpp::SubscriptionOptions();
  subscription_option.callback_group = callback_group_;

  server_ = this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
    "extrinsic_calibration", std::bind(
                               &ExtrinsicDummyCalibrator::requestReceivedCallback, this,
                               std::placeholders::_1, std::placeholders::_2));
}

void ExtrinsicDummyCalibrator::requestReceivedCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  response->success = true;
  response->result_pose = request->initial_pose;
  response->score = 1.0;
}

}  // namespace extrinsic_dummy_calibrator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<extrinsic_dummy_calibrator::ExtrinsicDummyCalibrator>(node_options);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
