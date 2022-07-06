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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tier4_calibration_msgs/srv/extrinsic_calibration_manager.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("extrinsic_calibration_client");

  auto client = node->create_client<tier4_calibration_msgs::srv::ExtrinsicCalibrationManager>(
    "extrinsic_calibration_manager");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for service...");
  }

  auto request =
    std::make_shared<tier4_calibration_msgs::srv::ExtrinsicCalibrationManager::Request>();
  request->src_path = node->declare_parameter("src_path", "");
  request->dst_path = node->declare_parameter("dst_path", "");
  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Received service message. success " << result.get()->success << " score " <<
        result.get()->score);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Problem while waiting for response.");
  }

  rclcpp::shutdown();
  return 0;
}
