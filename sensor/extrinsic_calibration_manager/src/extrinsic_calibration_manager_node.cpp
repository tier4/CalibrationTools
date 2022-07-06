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

#include "extrinsic_calibration_manager/extrinsic_calibration_manager_node.hpp"

#include <iomanip>
#include <vector>
#include <memory>
#include <string>
#include <sstream>

using namespace std::chrono_literals;

ExtrinsicCalibrationManagerNode::ExtrinsicCalibrationManagerNode(
  const rclcpp::NodeOptions & node_options)
: Node("extrinsic_calibration_manager_node", node_options)
{
  using namespace std::placeholders;

  server_ = this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrationManager>(
    "extrinsic_calibration_manager",
    std::bind(&ExtrinsicCalibrationManagerNode::calibrationRequestCallback, this, _1, _2));

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  parent_frame_ = this->declare_parameter("parent_frame", "");
  child_frames_ = this->declare_parameter("child_frames", std::vector<std::string>());
  client_ns_ = this->declare_parameter("client_ns", "extrinsic_calibration");
}

void ExtrinsicCalibrationManagerNode::calibrationRequestCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrationManager::Request>
  request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrationManager::Response>
  response)
{
  // open yaml file
  auto yaml_node = YAML::LoadFile(request->src_path);
  if (yaml_node.IsNull()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Reading yaml file failed: " << request->src_path);
    return;
  }

  // create clients
  for (const auto & child_frame : child_frames_) {
    TargetClient target_client;
    if (!createTargetClient(
        yaml_node, parent_frame_, child_frame, client_ns_, callback_group_,
        target_client))
    {
      rclcpp::shutdown();
    }
    target_clients_.push_back(target_client);
  }

  // wait for client services
  for (auto & target_client : target_clients_) {
    while (!target_client.client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for service: " << target_client.child_frame);
    }
  }

  // call client services
  for (auto & target_client : target_clients_) {
    auto req = std::make_shared<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request>();
    *req = target_client.request;

    auto cb = [&](rclcpp::Client<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedFuture
        response_client) {
        auto res = response_client.get();
        target_client.response = *res;
        target_client.estimated = true;
        RCLCPP_INFO_STREAM(
          this->get_logger(), "Received service message: " << target_client.child_frame <<
            "(success = " << res->success <<
            " score = " << res->score << ")");
      };

    RCLCPP_INFO_STREAM(this->get_logger(), "Call service: " << target_client.child_frame);
    target_client.client->async_send_request(req, cb);
  }

  // wait for responses
  while (rclcpp::ok()) {
    bool done = std::all_of(
      target_clients_.begin(), target_clients_.end(), [](auto target_client) {
        return target_client.estimated;
      });
    if (done) {break;}
    rclcpp::sleep_for(5s);
    RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for responses...");
  }

  // dump yaml file
  dumpCalibrationConfig(request->dst_path, target_clients_);

  // TODO(Akihito OHSATO): handling results of success/score
  response->success = true;
  response->score = 0.0;
}

bool ExtrinsicCalibrationManagerNode::createTargetClient(
  const YAML::Node & yaml_node, const std::string & parent_frame,
  const std::string & child_frame, const std::string & client_ns,
  const rclcpp::CallbackGroup::SharedPtr & callback_group, TargetClient & target_client)
{
  target_client.parent_frame = parent_frame;
  target_client.child_frame = child_frame;

  target_client.client = this->create_client<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
    target_client.parent_frame + "/" + target_client.child_frame + "/" + client_ns,
    rmw_qos_profile_default, callback_group);

  target_client.estimated = false;

  try {
    target_client.request.initial_pose = getPoseFromYaml(yaml_node, parent_frame, child_frame);
  } catch (const std::runtime_error & exception) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(), "Loading parameter from yaml failed: " << exception.what());
    return false;
  }

  return true;
}

geometry_msgs::msg::Pose ExtrinsicCalibrationManagerNode::getPoseFromYaml(
  const YAML::Node & yaml_node, const std::string & parent_frame,
  const std::string & child_frame)
{
  tf2::Vector3 pos(
    yaml_node[parent_frame][child_frame]["x"].as<double>(),
    yaml_node[parent_frame][child_frame]["y"].as<double>(),
    yaml_node[parent_frame][child_frame]["z"].as<double>());

  tf2::Quaternion quat;
  tf2::fromMsg(
    tier4_autoware_utils::createQuaternionFromRPY(
      yaml_node[parent_frame][child_frame]["roll"].as<double>(),
      yaml_node[parent_frame][child_frame]["pitch"].as<double>(),
      yaml_node[parent_frame][child_frame]["yaw"].as<double>()), quat);
  return tier4_autoware_utils::transform2pose(toMsg(tf2::Transform(quat, pos)));
}

bool ExtrinsicCalibrationManagerNode::dumpCalibrationConfig(
  const std::string & path, const std::vector<TargetClient> & target_clients)
{
  std::ofstream file_out(path.c_str());
  if (file_out.fail()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Open yaml file failed: " << path);
    return false;
  }

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << parent_frame_ << YAML::Value << YAML::BeginMap;
  for (const auto & target_client : target_clients) {
    const auto xyz = target_client.response.result_pose.position;
    const auto rpy = tier4_autoware_utils::getRPY(target_client.response.result_pose.orientation);

    std::ostringstream xyz_x, xyz_y, xyz_z, rpy_x, rpy_y, rpy_z;

    xyz_x << std::setw(yaml_precision_) << std::fixed << xyz.x;
    xyz_y << std::setw(yaml_precision_) << std::fixed << xyz.y;
    xyz_z << std::setw(yaml_precision_) << std::fixed << xyz.z;
    rpy_x << std::setw(yaml_precision_) << std::fixed << rpy.x;
    rpy_y << std::setw(yaml_precision_) << std::fixed << rpy.y;
    rpy_z << std::setw(yaml_precision_) << std::fixed << rpy.z;

    out << YAML::Key << target_client.child_frame << YAML::Value << YAML::BeginMap << YAML::Key <<
      "x" << YAML::Value << xyz_x.str() << YAML::Key << "y" << YAML::Value << xyz_y.str() <<
      YAML::Key << "z" << YAML::Value << xyz_z.str() << YAML::Key << "roll" << YAML::Value <<
      rpy_x.str() << YAML::Key << "pitch" << YAML::Value << rpy_y.str() << YAML::Key << "yaw" <<
      YAML::Value << rpy_z.str() << YAML::EndMap;
  }

  out << YAML::EndMap << YAML::EndMap;
  file_out << out.c_str();
  file_out.close();
  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<ExtrinsicCalibrationManagerNode>(node_options);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
