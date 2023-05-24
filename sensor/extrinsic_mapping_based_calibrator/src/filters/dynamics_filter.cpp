// Copyright 2022 Tier IV, Inc.
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

#include <extrinsic_mapping_based_calibrator/filters/dynamics_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#define UNUSED(x) (void)x;

void DynamicsFilter::setName(const std::string & name) { name_ = name + " (DynamicsFilter)"; }

std::vector<CalibrationFrame> DynamicsFilter::filter(
  const std::vector<CalibrationFrame> & calibration_frames, MappingData::Ptr & mapping_data)
{
  UNUSED(mapping_data);
  std::vector<CalibrationFrame> filtered_frames;

  std::stringstream ss;
  ss << "Accepted kyframes due to dynamics & interpolation: ";

  for (auto & frame : calibration_frames) {
    RCLCPP_INFO(
      rclcpp::get_logger(name_), "Attempting to add keyframe id=%d to the calibration list",
      frame.target_frame_->keyframe_id_);
    RCLCPP_INFO(rclcpp::get_logger(name_), "\t - stopped: %s", frame.stopped_ ? " true" : "false");
    RCLCPP_INFO(
      rclcpp::get_logger(name_), "\t - interpolated time: %.4f s (%s)", frame.interpolated_time_,
      frame.interpolated_time_ < parameters_->max_allowed_interpolated_time_ ? "accepted"
                                                                             : "rejected");
    RCLCPP_INFO(
      rclcpp::get_logger(name_), "\t - interpolated distance: %.4f m (%s)",
      frame.interpolated_distance_,
      frame.interpolated_distance_ < parameters_->max_allowed_interpolated_distance_ ? "accepted"
                                                                                     : "rejected");
    RCLCPP_INFO(
      rclcpp::get_logger(name_), "\t - interpolated angle: %.4f deg (%s)",
      frame.interpolated_angle_,
      frame.interpolated_angle_ < parameters_->max_allowed_interpolated_angle_ ? "accepted"
                                                                               : "rejected");
    RCLCPP_INFO(
      rclcpp::get_logger(name_), "\t - interpolated speed: %.4f m/s (%s)", frame.estimated_speed_,
      frame.estimated_speed_ < parameters_->max_allowed_interpolated_speed_ ? "accepted"
                                                                            : "rejected");
    RCLCPP_INFO(
      rclcpp::get_logger(name_), "\t - interpolated accel: %.4f m/s2 (%s)", frame.estimated_accel_,
      frame.estimated_accel_ < parameters_->max_allowed_interpolated_accel_ ? "accepted"
                                                                            : "rejected");

    bool standard_criteria =
      frame.interpolated_time_ < parameters_->max_allowed_interpolated_time_ &&
      frame.interpolated_distance_ < parameters_->max_allowed_interpolated_distance_ &&
      frame.interpolated_angle_ < parameters_->max_allowed_interpolated_angle_ &&
      frame.estimated_speed_ < parameters_->max_allowed_interpolated_speed_ &&
      frame.estimated_accel_ < parameters_->max_allowed_interpolated_accel_;

    bool straight_criteria =
      frame.interpolated_time_ < parameters_->max_allowed_interpolated_time_ &&
      frame.interpolated_distance_ < parameters_->max_allowed_interpolated_distance_straight_ &&
      frame.interpolated_angle_ < parameters_->max_allowed_interpolated_angle_straight_ &&
      frame.estimated_speed_ < parameters_->max_allowed_interpolated_speed_straight_ &&
      frame.estimated_accel_ < parameters_->max_allowed_interpolated_accel_straight_;

    RCLCPP_INFO(
      rclcpp::get_logger(name_), "\t - standard criteria: %s",
      standard_criteria ? "accepted" : "rejected");
    RCLCPP_INFO(
      rclcpp::get_logger(name_), "\t - straight criteria: %s",
      straight_criteria ? "accepted" : "rejected");

    if (
      (standard_criteria || straight_criteria) &&
      (frame.stopped_ || !parameters_->calibration_use_only_stopped_)) {
      filtered_frames.emplace_back(frame);
      ss << frame.target_frame_->frame_id_ << "/" << frame.target_frame_->keyframe_id_ << " ";
    }
  }

  if (filtered_frames.size() > 0) {
    RCLCPP_INFO(rclcpp::get_logger(name_), "%s\n", ss.str().c_str());
  }

  return filtered_frames;
}
