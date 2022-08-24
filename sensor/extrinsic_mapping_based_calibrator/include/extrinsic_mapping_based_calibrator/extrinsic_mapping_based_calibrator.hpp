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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR_EXTRINSIC_MAPPING_BASED_CALIBRATOR_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR_EXTRINSIC_MAPPING_BASED_CALIBRATOR_HPP_

#include <Eigen/Dense>
#include <kalman_filter/kalman_filter.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tier4_calibration_msgs/srv/extrinsic_calibrator.hpp>
#include <tier4_calibration_msgs/srv/key_frame_map.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
//#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <unordered_map>
#include <mutex>
#include <string>
#include <vector>
#include <queue>

using PointType = pcl::PointXYZ;
using PointcloudType = pcl::PointCloud<PointType>;


struct Frame {
  float distance_;
  std_msgs::msg::Header header_;
  PointcloudType::Ptr pointcloud_raw_;
  PointcloudType::Ptr pointcloud_subsampled_;
  int frame_id_;
  int keyframe_id_;
  bool processed_;
  bool is_key_frame_;
  bool stopped_;
  Eigen::Matrix4f pose_; // map->lidar
};

struct CalibrationFrame {
  PointcloudType::Ptr source_pointcloud_;
  PointcloudType::Ptr target_pointcloud_; // we may not be able to store the pointcloud since it is
  std_msgs::msg::Header source_header_;

  Eigen::Matrix4f local_map_pose_; // pose in the map from the target lidar

  double interpolated_distance_;
  double interpolated_angle_; // det(rot * inv rot) o algo asi
  double interpolated_time_;
  double estimated_speed_;

};

class ExtrinsicMappingBasedCalibrator : public rclcpp::Node
{
public:
  ExtrinsicMappingBasedCalibrator(const rclcpp::NodeOptions & options);

protected:
  //void requestReceivedCallback(
  //  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
  //  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);

  void keyFrameCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::KeyFrameMap::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::KeyFrameMap::Response> response);

  void calibrationPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);
  void mappingPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);
  void timerCallback();

  void mappingThreadWorker();
  //void publishTf();

  void subsampleFrame(std::shared_ptr<Frame>);

  void initLocalMap(std::shared_ptr<Frame> frame);
  void checkKeyframe(std::shared_ptr<Frame> frame);
  void createKeyFrame();
  void recalculateLocalMap();
  PointcloudType::Ptr cropPointCloud(PointcloudType::Ptr & pointcloud, double max_range);

  // Parameters
  std::string base_frame_;
  std::string sensor_kit_frame_;
  std::string lidar_base_frame_;
  std::string map_frame_;
  int max_frames_;
  int local_map_num_keyframes_;
  int calibration_num_keyframes_;
  double max_pointcloud_range_;
  double ndt_resolution_;
  double ndt_step_size_;
  int ndt_max_iterations_;
  int ndt_num_threads_;
  double leaf_size_input_;
  double leaf_size_local_map_;
  double leaf_size_dense_map_;
  double new_keyframe_min_distance_;
  double new_frame_min_distance_;
  double frame_stopped_distance_;
  double frame_nonstopped_distance_;
  int calibration_skip_keyframes_;

  // ROS Interface
  tf2_ros::StaticTransformBroadcaster tf_broascaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr srv_callback_group_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frame_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr keyframe_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keyframe_markers_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr calibration_pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapping_pointcloud_sub_;

  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr service_server_;
  rclcpp::Service<tier4_calibration_msgs::srv::KeyFrameMap>::SharedPtr keyframe_map_server_;

  rclcpp::TimerBase::SharedPtr timer_;

  pclomp::NormalDistributionsTransform<PointType, PointType> ndt;

  // Threading, sync, and result
  std::mutex mutex_;

  // ROS Data
  std_msgs::msg::Header::SharedPtr lidar_header_;
  std::string lidar_frame_;

  // ROS Publishers buffers
  PointcloudType::Ptr published_map_pointcloud_ptr_;
  nav_msgs::msg::Path published_frames_path_;
  nav_msgs::msg::Path published_keyframes_path_;
  visualization_msgs::msg::MarkerArray published_keyframes_markers_;

  // Calibration matching data
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> calibration_pointclouds_queue_;
  int last_unmatched_keyframe_;

  // Mapping data
  int selected_keyframe_;
  int n_processed_frames_;
  std::queue<std::shared_ptr<Frame>> unprocessed_frames_;
  std::vector<std::shared_ptr<Frame>> processed_frames_;
  std::vector<std::shared_ptr<Frame>> keyframes_;
  pcl::PointCloud<PointType>::Ptr local_map_ptr_;
  std::vector<geometry_msgs::msg::PoseStamped> trajectory_;

};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR_EXTRINSIC_MAPPING_BASED_CALIBRATOR_HPP_
