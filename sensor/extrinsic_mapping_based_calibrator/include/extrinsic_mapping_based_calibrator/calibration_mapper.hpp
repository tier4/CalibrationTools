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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR_CALIBRATION_MAPPER_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR_CALIBRATION_MAPPER_HPP_

#include <Eigen/Dense>
#include <extrinsic_mapping_based_calibrator/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_interfaces/srv/pause.hpp>
#include <rosbag2_interfaces/srv/resume.hpp>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_calibration_msgs/srv/frame.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_types.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <tf2_ros/buffer.h>

class CalibrationMapper
{
public:
  using Ptr = std::shared_ptr<CalibrationMapper>;
  using PointPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
  using PointSubscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>;
  using FrameService = rclcpp::Service<tier4_calibration_msgs::srv::Frame>;

  CalibrationMapper(
    MappingParameters::Ptr & parameters, MappingData::Ptr & mapping_data,
    PointPublisher::SharedPtr & map_pub,
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & frame_path_pub,
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & keyframe_path_pub,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & keyframe_markers_pub,
    rclcpp::Client<rosbag2_interfaces::srv::Pause>::SharedPtr & rosbag2_pause_client,
    rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedPtr & rosbag2_resume_client,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /*!
   * Message callback for calibration pointclouds (pointclouds in the frame to calibrate)
   * @param[in] pc Calibration pointcloud msg
   */
  void calibrationPointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr pc, const std::string & frame_name);

  /*!
   * Message callback for mapping pointclouds (pointclouds used for the map used as a target during
   * calibration)
   * @param[in] pc Calibration pointcloud msg
   */
  void mappingPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);

  /*!
   * Timer callback. Publishes a 'sparse' map and the trajectory of the mapping
   */
  void publisherTimerCallback();

  /*!
   * Timer callback. Matches mapping keyframes with calibration lidars
   */
  void dataMatchingTimerCallback();

  /*!
   * Whether or not map has stopped
   */
  bool stopped();

  /*!
   * Stop the mapper
   */
  void stop();

protected:
  /*!
   * Matches mapping keyframes with a singular calibration lidar
   */
  void mappingCalibrationDatamatching(const std::string & calibration_frame);

  // Mapping methods

  /*!
   * Dedicated thread to do mapping
   */
  void mappingThreadWorker();

  /*!
   * Initialize a map using a Frame
   * @param[in] frame Source pointcloud
   */
  void initLocalMap(Frame::Ptr frame);

  /*!
   * Check whether a frame is considered a Keyframe
   * @param[in] frame Keyframe candidate
   */
  void checkKeyframe(Frame::Ptr frame);

  /*!
   * Check whether a keyframe is lost
   * @param[in] frame Keyframe candidate
   */
  void checkKeyframeLost(Frame::Ptr frame);

  /*!
   * Check whether a frame is lost
   * @param[in] frame frame candidate
   */
  bool checkFrameLost(const Frame::Ptr & prev_frame, Frame::Ptr & frame, float dt);

  /*!
   * Check whether a frame should be dropped
   * @param[in] frame frame candidate
   */
  bool shouldDropFrame(const Frame::Ptr & prev_frame, Frame::Ptr & frame, float delta_distance);

  /*!
   * Recalculate the mapping local map based on the latest keyframes
   */
  void recalculateLocalMap();

  MappingParameters::Ptr parameters_;
  MappingData::Ptr data_;

  PointPublisher::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frame_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr keyframe_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keyframe_markers_pub_;

  rclcpp::Client<rosbag2_interfaces::srv::Pause>::SharedPtr rosbag2_pause_client_;
  rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedPtr rosbag2_resume_client_;

  pclomp::NormalDistributionsTransform<PointType, PointType> ndt_;

  // ROS Data
  std_msgs::msg::Header::SharedPtr mapping_lidar_header_;

  // ROS Publishers buffers
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  PointcloudType::Ptr published_map_pointcloud_ptr_;
  nav_msgs::msg::Path published_frames_path_;
  nav_msgs::msg::Path published_keyframes_path_;
  visualization_msgs::msg::MarkerArray published_keyframes_markers_;

  // Rosbag interface
  bool bag_paused_;

  // External interface
  bool stopped_;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR_CALIBRATION_MAPPER_HPP_