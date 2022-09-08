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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR_TYPES_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR_TYPES_HPP_

#include <Eigen/Dense>

#include <std_msgs/msg/header.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

using PointType = pcl::PointXYZ;
using PointcloudType = pcl::PointCloud<PointType>;

struct Frame
{
  using Ptr = std::shared_ptr<Frame>;
  float distance_;
  float delta_distance_;
  std_msgs::msg::Header header_;
  PointcloudType::Ptr pointcloud_raw_;
  PointcloudType::Ptr pointcloud_subsampled_;
  int frame_id_;
  int keyframe_id_;
  bool processed_;
  bool is_key_frame_;
  bool stopped_;
  int frames_since_stop_;
  Eigen::Matrix4f pose_;  // map->lidar
};

struct CalibrationFrame
{
  PointcloudType::Ptr source_pointcloud_;
  // PointcloudType::Ptr target_pointcloud_;  // we may not be able to store the pointcloud since it
  // is
  std_msgs::msg::Header source_header_;

  Frame::Ptr target_frame_;
  Eigen::Matrix4f local_map_pose_;  // pose in the map from the target lidar

  double interpolated_distance_;
  double interpolated_angle_;  // det(rot * inv rot) o algo asi
  double interpolated_time_;
  double estimated_speed_;
  double estimated_accel_;
  bool stopped_;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR_TYPES_HPP_
