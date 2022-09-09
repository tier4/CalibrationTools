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

#include "pitch_checker/pitch_checker.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

PitchChecker::PitchChecker(const rclcpp::NodeOptions & node_options)
: Node("pitch_checker", node_options)
{
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);
  using namespace std::placeholders;
  update_hz_ = this->declare_parameter<double>("update_hz", 10.0);
  output_file_ = this->declare_parameter<std::string>("output_file", "pitch.csv");
  save_flag_server_ = this->create_service<std_srvs::srv::Trigger>(
    "/pitch_checker/save_flag", std::bind(&PitchChecker::onSaveService, this, _1, _2, _3));
  initTimer(1.0 / update_hz_);
}

void PitchChecker::initTimer(double period_s)
{
  auto timer_callback = std::bind(&PitchChecker::timerCallback, this);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

bool PitchChecker::onSaveService(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> req_header,
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  pitchInfoToMap();
  if (writeMap()) {
    res->success = true;
    res->message = "Data has been successfully saved on " + output_file_;
  } else {
    res->success = false;
    res->message = "Failed to save data. Maybe wrong path?";
  }

  return true;
}

void PitchChecker::timerCallback()
{
  getTf();
  if (get_tf_count_ >= MAX_TF_COUNT_) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("pitch_compare"),
      "Save data and stop recording due to capacity limitation.");
    pitchInfoToMap();
    writeMap();
  }
}

bool PitchChecker::getTf()
{
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform;
  try {
    transform = transform_listener_->getTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    auto & clk = *this->get_clock();
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      rclcpp::get_logger("pitch_checker"), clk, 5000, "cannot get map to base_link transform. %s",
      ex.what());
    return false;
  }
  double roll, pitch, yaw;
  tf2::getEulerYPR(transform->transform.rotation, roll, pitch, yaw);
  TfInfo tf_info;
  tf_info.pitch = pitch;
  tf_info.yaw = yaw;
  tf_info.x = transform->transform.translation.x;
  tf_info.y = transform->transform.translation.y;
  tf_info.z = transform->transform.translation.z;
  tf_info_vec_.emplace_back(tf_info);
  get_tf_count_++;
  return true;
}

void PitchChecker::pitchInfoToMap()
{
  tf_map_.clear();
  tf_map_unified_.clear();
  for (const auto tf_info : tf_info_vec_) {
    const int x_i = static_cast<int>(std::nearbyint(tf_info.x));
    const int y_i = static_cast<int>(std::nearbyint(tf_info.y));
    const auto xy_i = std::pair<int, int>(x_i, y_i);
    // create new key
    if (tf_map_.count(xy_i) == 0) {
      tf_map_[xy_i] = std::vector<TfInfo>();
    }
    tf_map_[xy_i].emplace_back(tf_info);
  }

  for (const auto & map_val : tf_map_) {
    const auto key = map_val.first;
    const auto tf_info_vec = map_val.second;
    if (tf_info_vec.empty()) {
      continue;
    }
    std::vector<TfInfo> vec1;
    std::vector<TfInfo> vec2;
    // separate vec by yaw
    for (const auto & tf_info : tf_info_vec) {
      const double dyaw = std::fabs(tf_info.yaw - tf_info_vec.front().yaw);
      if (dyaw < M_PI_2) {
        vec1.emplace_back(tf_info);
      } else {
        vec2.emplace_back(tf_info);
      }
      tf_map_unified_[key] = std::vector<TfInfo>();
      if (!vec1.empty()) {
        tf_map_unified_[key].emplace_back(getMedianPitchTfInfo(vec1));
      }
      if (!vec2.empty()) {
        tf_map_unified_[key].emplace_back(getMedianPitchTfInfo(vec2));
      }
    }
  }
}

TfInfo PitchChecker::getMedianPitchTfInfo(const std::vector<TfInfo> & tf_info_vec)
{
  auto tf_info_vec_local = tf_info_vec;
  std::sort(
    tf_info_vec_local.begin(), tf_info_vec_local.end(),
    [](const TfInfo & x, const TfInfo & y) { return x.pitch < y.pitch; });

  if (tf_info_vec_local.size() % 2 == 0) {
    const int mid_idx_1 = (tf_info_vec_local.size()) / 2;
    const int mid_idx_2 = (tf_info_vec_local.size()) / 2 - 1;
    TfInfo tf_info;
    tf_info.x = (tf_info_vec_local.at(mid_idx_1).x + tf_info_vec_local.at(mid_idx_2).x) / 2.0;
    tf_info.y = (tf_info_vec_local.at(mid_idx_1).y + tf_info_vec_local.at(mid_idx_2).y) / 2.0;
    tf_info.z = (tf_info_vec_local.at(mid_idx_1).z + tf_info_vec_local.at(mid_idx_2).z) / 2.0;
    tf_info.pitch =
      (tf_info_vec_local.at(mid_idx_1).pitch + tf_info_vec_local.at(mid_idx_2).pitch) / 2.0;
    tf_info.yaw = (tf_info_vec_local.at(mid_idx_1).yaw + tf_info_vec_local.at(mid_idx_2).yaw) / 2.0;
    return tf_info;
  } else {
    const int mid_idx = (tf_info_vec_local.size() - 1) / 2;
    return tf_info_vec_local.at(mid_idx);
  }
}

bool PitchChecker::writeMap()
{
  std::ofstream of(output_file_);
  if (!of.is_open()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("pitch_checker"), "cannot open the file: " << output_file_);
    return false;
  }

  of << "x,y,z,yaw,pitch" << std::endl;
  for (const auto & map_val : tf_map_unified_) {
    const auto key = map_val.first;
    const int x = key.first;
    const int y = key.second;
    for (const auto tf_info : map_val.second) {
      of << x << "," << y << "," << tf_info.z << "," << tf_info.yaw << "," << tf_info.pitch
         << std::endl;
    }
  }
  return true;
}
