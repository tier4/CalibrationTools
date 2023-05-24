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

#include <extrinsic_mapping_based_calibrator/sensor_calibrator.hpp>
#include <extrinsic_mapping_based_calibrator/utils.hpp>
#include <tier4_calibration_pcl_extensions/voxel_grid_triplets_impl.hpp>

SensorCalibrator::SensorCalibrator(
  const std::string calibrator_sensor_frame, const std::string calibrator_name,
  CalibrationParameters::Ptr & parameters, MappingData::Ptr & mapping_data,
  std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
: calibrator_sensor_frame_(calibrator_sensor_frame),
  calibrator_name_(calibrator_name),
  parameters_(parameters),
  data_(mapping_data),
  tf_buffer_(tf_buffer)
{
}

PointcloudType::Ptr SensorCalibrator::getDensePointcloudFromMap(
  const Eigen::Matrix4f & pose, const Frame::Ptr & frame, double resolution, double max_range)
{
  int frame_id = frame->frame_id_;

  // Find the closest keyframe to the requested keyframe
  Frame::Ptr keyframe_left, keyframe_right, keyframe;

  for (auto it = data_->processed_frames_.begin() + frame_id;
       it != data_->processed_frames_.begin(); it--) {
    if ((*it)->is_key_frame_) {
      keyframe_left = *it;
      break;
    }
  }

  for (auto it = data_->processed_frames_.begin() + frame_id; it != data_->processed_frames_.end();
       it++) {
    if ((*it)->is_key_frame_) {
      keyframe_right = *it;
      break;
    }
  }

  if (keyframe_left && keyframe_right) {
    keyframe =
      (keyframe_right->frame_id_ - frame->frame_id_ < frame->frame_id_ - keyframe_left->frame_id_)
        ? keyframe_right
        : keyframe_left;
  } else if (keyframe_left) {
    keyframe = keyframe_left;
  } else if (keyframe_right) {
    keyframe = keyframe_right;
  } else {
    assert(false);
  }

  int min_keyframe_id =
    std::max<int>(0, keyframe->keyframe_id_ - parameters_->dense_pointcloud_num_keyframes_);
  int max_keyframe_id = std::min<int>(
    data_->keyframes_.size() - 1,
    keyframe->keyframe_id_ + parameters_->dense_pointcloud_num_keyframes_);

  int min_frame_id = data_->keyframes_[min_keyframe_id]->frame_id_;
  int max_frame_id = data_->keyframes_[max_keyframe_id]->frame_id_;

  auto target_map_pose = pose.inverse();

  // Sum all frames in the target coordinate system (tcs)
  PointcloudType::Ptr tmp_tcs_ptr(new PointcloudType());
  PointcloudType::Ptr subsampled_tcs_ptr(new PointcloudType());

  for (int i = min_frame_id; i <= max_frame_id; i++) {
    Frame::Ptr frame = data_->processed_frames_[i];
    PointcloudType::Ptr frame_tcs_ptr(new PointcloudType());

    auto map_frame_pose = frame->pose_;
    auto target_frame_pose = target_map_pose * map_frame_pose;

    pcl::transformPointCloud(*frame->pointcloud_raw_, *frame_tcs_ptr, target_frame_pose);
    *tmp_tcs_ptr += *frame_tcs_ptr;
  }

  PointcloudType::Ptr cropped_tcd_ptr = cropPointCloud<PointcloudType>(tmp_tcs_ptr, max_range);

  pcl::VoxelGridTriplets<PointType> voxel_grid;
  voxel_grid.setLeafSize(resolution, resolution, resolution);
  voxel_grid.setInputCloud(cropped_tcd_ptr);
  voxel_grid.filter(*subsampled_tcs_ptr);

  return subsampled_tcs_ptr;
}
