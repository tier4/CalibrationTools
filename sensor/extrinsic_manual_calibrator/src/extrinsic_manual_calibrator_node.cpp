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

#include "extrinsic_manual_calibrator/extrinsic_manual_calibrator_node.hpp"

#include <memory>

using namespace std::chrono_literals;

ExtrinsicManualCalibratorNode::ExtrinsicManualCalibratorNode(
  const rclcpp::NodeOptions & node_options)
: Node("extrinsic_manual_calibrator_node", node_options), done_(false)
{
  using namespace std::placeholders;

  // rosparam
  tf_parameter_ns_ =
    this->declare_parameter("tf_parameter_ns", "tf_broadcaster");

  // calibration request callback
  server_ = this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
    "extrinsic_calibration",
    std::bind(&ExtrinsicManualCalibratorNode::calibrationRequestCallback, this, _1, _2));

  // callback group
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // calibration done callback
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/done", 1, std::bind(&ExtrinsicManualCalibratorNode::calibrationDoneCallback, this, _1),
    sub_opt);

  // parameters client
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    this, tf_parameter_ns_, rmw_qos_profile_default, callback_group_);
}

void ExtrinsicManualCalibratorNode::calibrationRequestCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  while (!parameters_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Waiting for service...");
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Setting ros parameters...");
  setTfParameters(parameters_client_, request->initial_pose);

  while (rclcpp::ok() && !done_) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Waiting for calibration to complete...");
    rclcpp::sleep_for(1s);
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Calibration is completed, getting parameters...");
  response->result_pose = getTfParameters(parameters_client_);
  response->success = true;
  response->score = 1.0;

  rclcpp::shutdown();
}

void ExtrinsicManualCalibratorNode::calibrationDoneCallback(
  std_msgs::msg::Bool::ConstSharedPtr done)
{
  done_ = done->data;
}

geometry_msgs::msg::Pose ExtrinsicManualCalibratorNode::getTfParameters(
  const rclcpp::AsyncParametersClient::SharedPtr & client)
{
  geometry_msgs::msg::Pose pose;
  auto params = client->get_parameters({"tf_x", "tf_y", "tf_z", "tf_roll", "tf_pitch", "tf_yaw"});
  pose.position.x = params.get().at(0).as_double();
  pose.position.y = params.get().at(1).as_double();
  pose.position.z = params.get().at(2).as_double();
  pose.orientation = tier4_autoware_utils::createQuaternionFromRPY(
    params.get().at(3).as_double(), params.get().at(4).as_double(),
    params.get().at(5).as_double());
  return pose;
}
void ExtrinsicManualCalibratorNode::setTfParameters(
  const rclcpp::AsyncParametersClient::SharedPtr & client, const geometry_msgs::msg::Pose & pose)
{
  const auto xyz = pose.position;
  const auto rpy = tier4_autoware_utils::getRPY(pose.orientation);
  auto results = client->set_parameters(
    {rclcpp::Parameter("tf_x", xyz.x), rclcpp::Parameter("tf_y", xyz.y),
      rclcpp::Parameter("tf_z", xyz.z), rclcpp::Parameter("tf_roll", rpy.x),
      rclcpp::Parameter("tf_pitch", rpy.y), rclcpp::Parameter("tf_yaw", rpy.z)});
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<ExtrinsicManualCalibratorNode>(node_options);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
