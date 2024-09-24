// Copyright 2024 TIER IV, Inc.
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

#include <mapping_based_calibrator/filters/best_frames_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/common/pca.h>

#define UNUSED(x) (void)x;

void BestFramesFilter::setName(const std::string & name) { name_ = name + " (BestFramesFilter)"; }

std::vector<CalibrationFrame> BestFramesFilter::filter(
  const std::vector<CalibrationFrame> & input_calibration_frames, MappingData::Ptr & mapping_data)
{
  UNUSED(mapping_data);

  int max_keyframe = -1;
  int use_last_frames_only_min_frame = -1;

  // Find the frame of the last keyframe
  for (auto it = mapping_data->keyframes_and_stopped_.rbegin();
       it != mapping_data->keyframes_and_stopped_.rend(); ++it) {
    if ((*it)->is_key_frame_ && max_keyframe == -1) {
      max_keyframe = (*it)->keyframe_id_;
    }
    if (
      (*it)->is_key_frame_ && max_keyframe != -1 &&
      (*it)->keyframe_id_ < max_keyframe - parameters_->lidar_calibration_max_frames_) {
      use_last_frames_only_min_frame = (*it)->frame_id_;
      break;
    }
  }

  std::vector<CalibrationFrame> calibration_frames;

  if (parameters_->calibration_use_only_last_frames_) {
    RCLCPP_INFO(rclcpp::get_logger(name_), "Only choosing the last ones\n");
    std::copy_if(
      input_calibration_frames.begin(), input_calibration_frames.end(),
      std::back_inserter(calibration_frames),
      [&use_last_frames_only_min_frame](const auto & calibration_frame) {
        return calibration_frame.target_frame_->frame_id_ >= use_last_frames_only_min_frame;
      });
  } else {
    calibration_frames = input_calibration_frames;
  }

  std::vector<CalibrationFrame> filtered_frames;
  const int & calibration_max_frames = filter_type_ == Filter::FilterType::CameraFilter
                                         ? parameters_->camera_calibration_max_frames_
                                         : parameters_->lidar_calibration_max_frames_;

  std::vector<std::pair<float, std::size_t>> pca_coeff_calibration_id_pairs;

  for (std::size_t i = 0; i < calibration_frames.size(); i++) {
    auto & frame = calibration_frames[i];

    if (!frame.source_pointcloud_) {
      pca_coeff_calibration_id_pairs.push_back(std::make_pair(0.f, i));
      continue;
    }

    pcl::PCA<PointType> pca;
    pca.setInputCloud(frame.source_pointcloud_);

    float pca_coefficient = std::sqrt(std::abs(pca.getEigenValues().z()));

    RCLCPP_INFO(
      rclcpp::get_logger(name_), "\t - pca coeff: %.4f (%s)", pca_coefficient,
      pca_coefficient >= parameters_->calibration_min_pca_eigenvalue_ ? "accepted" : "rejected");

    if (pca_coefficient >= parameters_->calibration_min_pca_eigenvalue_) {
      pca_coeff_calibration_id_pairs.push_back(std::make_pair(pca_coefficient, i));
    }
  }

  std::sort(
    pca_coeff_calibration_id_pairs.begin(), pca_coeff_calibration_id_pairs.end(),
    [](auto & lhs, auto & rhs) { return lhs.first > rhs.first; });

  std::stringstream ss;
  ss << "Final selected keyframes: ";

  for (auto & pair : pca_coeff_calibration_id_pairs) {
    bool accepted = true;
    for (auto & accepted_frame : filtered_frames) {
      if (
        std::abs(
          calibration_frames[pair.second].target_frame_->distance_ -
          accepted_frame.target_frame_->distance_) <
        parameters_->calibration_min_distance_between_frames_) {
        accepted = false;
        break;
      }
    }

    if (accepted) {
      auto & accepted_frame = calibration_frames[pair.second];
      filtered_frames.push_back(accepted_frame);
      ss << accepted_frame.target_frame_->frame_id_ << "/"
         << accepted_frame.target_frame_->keyframe_id_ << " ";
    }

    if (static_cast<int>(filtered_frames.size()) == calibration_max_frames) {
      break;
    }
  }

  if (filtered_frames.size() > 0) {
    RCLCPP_INFO(rclcpp::get_logger(name_), "%s\n", ss.str().c_str());
  }

  return filtered_frames;
}
