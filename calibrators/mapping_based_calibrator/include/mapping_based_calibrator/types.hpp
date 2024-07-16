// Copyright 2024 Tier IV, Inc.
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

#ifndef MAPPING_BASED_CALIBRATOR__TYPES_HPP_
#define MAPPING_BASED_CALIBRATOR__TYPES_HPP_

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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
  float distance_{0.f};  // line integration of the trajectory starting from the the origin
  Eigen::Vector3f delta_translation_{
    Eigen::Vector3f::Zero()};  // translation component of the difference between the last pose and
                               // this
  Eigen::Vector3f rough_speed_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f aux_delta_translation_{
    Eigen::Vector3f::Zero()};  // delta translation from  the current frame to the latest non
                               // accepted frame
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
  Eigen::Matrix4f pose_;            // map->lidar
  Eigen::Matrix4f predicted_pose_;  // map->lidar
  float dt_{-1};
};

struct CalibrationFrame
{
  using Ptr = std::shared_ptr<CalibrationFrame>;
  using ConstPtr = std::shared_ptr<const CalibrationFrame>;

  sensor_msgs::msg::CameraInfo::SharedPtr source_camera_info_;
  sensor_msgs::msg::CompressedImage::SharedPtr source_image_;
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
  std::vector<std::string> calibration_camera_optical_link_frame_names;
  std::vector<std::string> calibration_lidar_frame_names_;

  std::recursive_mutex mutex_;
  int n_processed_frames_{0};
  std::list<Frame::Ptr> unprocessed_frames_;
  std::vector<Frame::Ptr> processed_frames_;
  std::vector<Frame::Ptr> keyframes_;
  std::vector<Frame::Ptr> keyframes_and_stopped_;
  pcl::PointCloud<PointType>::Ptr local_map_ptr_;
  std::vector<geometry_msgs::msg::PoseStamped> trajectory_;

  // Calibration matching data
  std::map<std::string, std_msgs::msg::Header::SharedPtr> calibration_lidar_header_map_;
  std::map<std::string, int> last_unmatched_keyframe_map_;
  std::map<std::string, std::vector<CalibrationFrame>> camera_calibration_frames_map_;
  std::map<std::string, std::vector<CalibrationFrame>> lidar_calibration_frames_map_;

  // Object recognition results
  std::vector<ObjectsBB> detected_objects_;
};

struct MappingParameters
{
  using Ptr = std::shared_ptr<MappingParameters>;
  using ConstPtr = std::shared_ptr<const MappingParameters>;

  std::string registrator_name_;
  bool mapping_verbose_;
  bool use_rosbag_;
  int mapping_max_frames_;
  int local_map_num_keyframes_;
  double mapping_min_range_;
  double mapping_max_range_;
  int min_mapping_pointcloud_size_;
  int min_calibration_pointcloud_size_;
  double mapping_lost_timeout_;
  double viz_max_range_;
  double mapping_viz_leaf_size_;

  double mapper_resolution_;
  double mapper_step_size_;
  int mapper_max_iterations_;
  double mapper_epsilon_;
  double mapper_max_correspondence_distance_;
  int mapper_num_threads_;

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

  bool crop_z_calibration_pointclouds_;
  double crop_z_calibration_pointclouds_value_;
};

struct CalibrationParameters
{
  using Ptr = std::shared_ptr<CalibrationParameters>;
  using ConstPtr = std::shared_ptr<const CalibrationParameters>;

  bool calibration_verbose_;
  double leaf_size_dense_map_;
  int dense_pointcloud_num_keyframes_;
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

  bool filter_detections_;
  double detection_max_time_tolerance_;
  double detection_size_tolerance_;

  // Calibration parameters
  int solver_iterations_;
  double max_corr_dist_coarse_;
  double max_corr_dist_fine_;
  double max_corr_dist_ultra_fine_;

  bool calibration_use_only_stopped_;
  bool calibration_use_only_last_frames_;
  double min_calibration_range_;
  double max_calibration_range_;
  double calibration_min_pca_eigenvalue_;
  double calibration_min_distance_between_frames_;
  double calibration_eval_max_corr_distance_;

  // Lidar calibration parameters
  int lidar_calibration_min_frames_;
  int lidar_calibration_max_frames_;

  // Camera calibration parameters
  double pc_features_min_distance_;
  double pc_features_max_distance_;
  int camera_calibration_min_frames_;
  int camera_calibration_max_frames_;

  // Base lidar calibration parameters;
  bool calibrate_base_frame_;
  std::string base_frame_;

  double base_lidar_crop_box_min_x_;
  double base_lidar_crop_box_min_y_;
  double base_lidar_crop_box_min_z_;
  double base_lidar_crop_box_max_x_;
  double base_lidar_crop_box_max_y_;
  double base_lidar_crop_box_max_z_;

  double base_lidar_max_inlier_distance_;
  int base_lidar_max_iterations_;
  int base_lidar_min_plane_points_;
  double base_lidar_min_plane_points_percentage_;
  double base_lidar_max_cos_distance_;
  bool base_lidar_overwrite_xy_yaw_;
};

#endif  // MAPPING_BASED_CALIBRATOR__TYPES_HPP_
