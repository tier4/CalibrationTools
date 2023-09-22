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

#include <extrinsic_mapping_based_calibrator/filters/lost_state_filter.hpp>
#include <rclcpp/rclcpp.hpp>

void LostStateFilter::setName(const std::string & name) { name_ = name + " (LostStateFilter)"; }

std::vector<CalibrationFrame> LostStateFilter::filter(
  const std::vector<CalibrationFrame> & calibration_frames, MappingData::Ptr & data)
{
  std::vector<CalibrationFrame> filtered_frames;
  std::vector<int> deleted_keyframe_ids;

  std::vector<int> invalid_keyframe_ids;
  std::unordered_set<int> invalid_keyframe_ids_map;

  bool lost = false;

  // Find all keyframes that are either "lost" or have a frame "lost" nearby
  for (const auto & frame : data->processed_frames_) {
    lost |= frame->lost_;

    if (frame->is_key_frame_ && lost) {
      if (invalid_keyframe_ids_map.find(frame->keyframe_id_) == invalid_keyframe_ids_map.end()) {
        invalid_keyframe_ids.push_back(frame->keyframe_id_);
        invalid_keyframe_ids_map.insert(frame->keyframe_id_);
      }

      lost = false;
    }
  }

  if (invalid_keyframe_ids.size() == 0) {
    return calibration_frames;
  }

  int left_invalid_keyframe_id = 0;

  // Find and separate frames that close enough to "lost" keyframes
  for (const auto & frame : calibration_frames) {
    // Check closest keyframe
    int keyframe_id = frame.target_frame_->keyframe_id_;

    if (!frame.target_frame_->is_key_frame_) {
      for (int i = 0;; i++) {
        int left_frame_id = frame.target_frame_->frame_id_ - i;
        int right_frame_id = frame.target_frame_->frame_id_ + i;
        if (left_frame_id >= 0 && data->processed_frames_[left_frame_id]->is_key_frame_) {
          keyframe_id = data->processed_frames_[left_frame_id]->keyframe_id_;
          break;
        }
        if (
          right_frame_id < static_cast<int>(data->processed_frames_.size()) &&
          data->processed_frames_[right_frame_id]->is_key_frame_) {
          keyframe_id = data->processed_frames_[right_frame_id]->keyframe_id_;
          break;
        }
      }
    }

    while (left_invalid_keyframe_id < static_cast<int>(invalid_keyframe_ids.size()) - 1 &&
           invalid_keyframe_ids[left_invalid_keyframe_id + 1] < keyframe_id) {
      left_invalid_keyframe_id++;
    }

    if (
      std::abs(invalid_keyframe_ids[left_invalid_keyframe_id] - keyframe_id) <=
        parameters_->dense_pointcloud_num_keyframes_ ||
      (left_invalid_keyframe_id < static_cast<int>(invalid_keyframe_ids.size()) - 1 &&
       std::abs(invalid_keyframe_ids[left_invalid_keyframe_id + 1] - keyframe_id) <=
         parameters_->dense_pointcloud_num_keyframes_)) {
      deleted_keyframe_ids.push_back(keyframe_id);
    } else {
      filtered_frames.push_back(frame);
    }
  }

  std::stringstream ss;
  ss << "Invalid keyframes due to being associated with 'lost' frames: ";

  for (const auto & id : deleted_keyframe_ids) {
    ss << id << " ";
  }

  if (deleted_keyframe_ids.size() > 0) {
    RCLCPP_WARN(rclcpp::get_logger(name_), "%s\n", ss.str().c_str());
  }

  return filtered_frames;
}
