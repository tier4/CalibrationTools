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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <list>
#include <map>
#include <mutex>

using PointType = pcl::PointXYZ;
using PointcloudType = pcl::PointCloud<PointType>;

struct ObjectBB
{
  using Ptr = std::shared_ptr<ObjectBB>;
  using ConstPtr = std::shared_ptr<const ObjectBB>;

  Eigen::Matrix4f pose_;
  Eigen::Vector3f size_;
};

struct ObjectsBB
{
  using Ptr = std::shared_ptr<ObjectsBB>;
  using ConstPtr = std::shared_ptr<const ObjectsBB>;

  std_msgs::msg::Header header_;
  std::vector<ObjectBB> objects_;
};

struct Frame
{
  using Ptr = std::shared_ptr<Frame>;
  using ConstPtr = std::shared_ptr<const Frame>;
  float distance_{0.f};
  float delta_distance_{0.f};
  float rough_speed_{0.f};
  std_msgs::msg::Header header_;
  PointcloudType::Ptr pointcloud_raw_;
  PointcloudType::Ptr pointcloud_subsampled_;
  int frame_id_{-1};
  int keyframe_id_{-1};
  bool processed_{false};
  bool is_key_frame_{false};
  bool stopped_{false};
  bool lost_{false};
  int frames_since_stop_{0};
  Eigen::Matrix4f pose_;  // map->lidar
};

struct CalibrationFrame
{
  using Ptr = std::shared_ptr<CalibrationFrame>;
  using ConstPtr = std::shared_ptr<const CalibrationFrame>;

  PointcloudType::Ptr source_pointcloud_;
  std_msgs::msg::Header source_header_;

  Frame::Ptr target_frame_;
  Eigen::Matrix4f local_map_pose_;

  double interpolated_distance_;
  double interpolated_angle_;
  double interpolated_time_;
  double estimated_speed_;
  double estimated_accel_;
  bool stopped_;
};

struct MappingData
{
  using Ptr = std::shared_ptr<MappingData>;
  using ConstPtr = std::shared_ptr<const MappingData>;

  std::string map_frame_;
  std::string mapping_lidar_frame_;
  std::vector<std::string> calibration_lidar_frame_names_;

  std::mutex mutex_;
  int n_processed_frames_{0};
  std::list<Frame::Ptr> unprocessed_frames_;
  std::vector<Frame::Ptr> processed_frames_;
  std::vector<Frame::Ptr> keyframes_;
  std::vector<Frame::Ptr> keyframes_and_stopped_;
  pcl::PointCloud<PointType>::Ptr local_map_ptr_;
  std::vector<geometry_msgs::msg::PoseStamped> trajectory_;

  // Calibration matching data
  std::map<std::string, std::list<sensor_msgs::msg::PointCloud2::SharedPtr>>
    calibration_pointclouds_list_map_;
  std::map<std::string, std_msgs::msg::Header::SharedPtr> calibration_lidar_header_map_;
  std::map<std::string, int> last_unmatched_keyframe_map_;
  std::map<std::string, std::vector<CalibrationFrame>> calibration_frames_map_;

  // Object recognition results
  std::vector<ObjectsBB> detected_objects_;
};

struct MappingParameters
{
  using Ptr = std::shared_ptr<MappingParameters>;
  using ConstPtr = std::shared_ptr<const MappingParameters>;

  bool mapping_verbose_;
  bool use_rosbag_;
  int mapping_max_frames_;
  int local_map_num_keyframes_;
  double mapping_max_range_;
  double viz_max_range_;
  double mapping_viz_leaf_size_;

  double ndt_resolution_;
  double ndt_step_size_;
  int ndt_max_iterations_;
  double ndt_epsilon_;
  int ndt_num_threads_;

  double leaf_size_input_;
  double leaf_size_local_map_;

  double new_keyframe_min_distance_;
  double new_frame_min_distance_;
  double frame_stopped_distance_;
  int frames_since_stop_force_frame_;

  int calibration_skip_keyframes_;
  double lost_frame_max_angle_diff_;
  double lost_frame_interpolation_error_;
  double lost_frame_max_acceleration_;
};

struct LidarCalibrationParameters
{
  using Ptr = std::shared_ptr<LidarCalibrationParameters>;
  using ConstPtr = std::shared_ptr<const LidarCalibrationParameters>;

  bool calibration_verbose_;
  double leaf_size_dense_map_;
  int dense_pointcloud_num_keyframes_;
  int calibration_min_frames_;
  int calibration_max_frames_;
  double calibration_viz_leaf_size_;

  // Calibration preprocessing
  double max_allowed_interpolated_time_;
  double max_allowed_interpolated_distance_;
  double max_allowed_interpolated_angle_;
  double max_allowed_interpolated_speed_;
  double max_allowed_interpolated_accel_;

  double max_allowed_interpolated_distance_straight_;
  double max_allowed_interpolated_angle_straight_;
  double max_allowed_interpolated_speed_straight_;
  double max_allowed_interpolated_accel_straight_;

  // Calibration parameters
  int solver_iterations_;
  double max_corr_dist_coarse_;
  double max_corr_dist_fine_;
  double max_corr_dist_ultrafine_;

  bool calibration_use_only_stopped_;
  double max_calibration_range_;
  double calibration_min_pca_eigenvalue_;
  double calibration_min_distance_between_frames_;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR_TYPES_HPP_
