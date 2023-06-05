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

#ifndef EXTRINSIC_REFLECTOR_BASED_CALIBRATOR__EXTRINSIC_REFLECTOR_BASED_CALIBRATOR_HPP_
#define EXTRINSIC_REFLECTOR_BASED_CALIBRATOR__EXTRINSIC_REFLECTOR_BASED_CALIBRATOR_HPP_

#include <Eigen/Dense>
#include <extrinsic_reflector_based_calibrator/types.hpp>
#include <kalman_filter/kalman_filter.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tier4_calibration_msgs/srv/extrinsic_calibrator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class ExtrinsicReflectorBasedCalibrator : public rclcpp::Node
{
public:
  using PointType = pcl::PointXYZ;
  using TreeType = pcl::octree::OctreePointCloudSearch<PointType>;
  using index_t = std::uint32_t;

  explicit ExtrinsicReflectorBasedCalibrator(const rclcpp::NodeOptions & options);

protected:
  void requestReceivedCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);

  void backgroundModelRequestCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);
  void radarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);

  void extractBackgroundModel(
    const pcl::PointCloud<PointType>::Ptr & sensor_pointcloud,
    const std_msgs::msg::Header & current_header, std_msgs::msg::Header & last_updated_header,
    BackgroundModel & background_model);

  pcl::PointCloud<PointType>::Ptr extractForegroundPoints(
    const pcl::PointCloud<PointType>::Ptr & sensor_pointcloud,
    const BackgroundModel & background_model);

  // Parameters
  std::string base_frame_;
  std::string sensor_kit_frame_;  // the parent for this calibration method must be a sensor kit
  std::string lidar_base_frame_;  // the child for this calibration method must be a the base of a
                                  // lidar (probably different from the actual lidar tf)
  double lidar_background_model_leaf_size_;
  double radar_background_model_leaf_size_;
  double background_model_margin_;
  double background_model_timeout_;
  double max_match_yaw_distance_;
  double min_foreground_distance_;  // needs to be about at least double the leaf size

  // ROS Interface
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr srv_callback_group_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_background_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_foreground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_background_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_foreground_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_sub_;

  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr
    calibration_request_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr background_model_server_;

  // Threading, sync, and result
  std::mutex mutex_;

  // ROS Data
  std_msgs::msg::Header lidar_header_, radar_header_;
  std::string lidar_frame_, radar_frame_;

  // Initial tfs comparable with the one with our method
  geometry_msgs::msg::Transform initial_base_to_lidar_msg_;
  tf2::Transform initial_base_to_lidar_tf2_;
  Eigen::Isometry3d initial_base_to_lidar_eigen_;

  // Other tfs to calculate the complete chain. There are constant for our purposes
  geometry_msgs::msg::Transform base_to_sensor_kit_msg_;
  tf2::Transform base_to_sensor_kit_tf2_;
  Eigen::Isometry3d base_to_sensor_kit_eigen_;

  geometry_msgs::msg::Transform lidar_base_to_lidar_msg_;
  tf2::Transform lidar_base_to_lidar_tf2_;
  Eigen::Isometry3d lidar_base_to_lidar_eigen_;

  geometry_msgs::msg::Pose output_calibration_msg_;

  bool got_initial_transform_;
  bool broadcast_tf_;
  bool calibration_done_;

  // Filtering
  KalmanFilter lidar_filter_, radar_filter_;
  bool first_observation_;
  double initial_lidar_cov_;
  double initial_radar_cov_;
  double lidar_measurement_cov_;
  double lidar_process_cov_;
  double radar_measurement_cov_;
  double radar_process_cov_;
  double lidar_convergence_threshold_;
  double radar_convergence_threshold_;

  // Background model
  bool extract_lidar_background_model_;
  bool extract_radar_background_model_;
  std_msgs::msg::Header latest_updated_lidar_frame_;
  std_msgs::msg::Header latest_updated_radar_frame_;
  BackgroundModel lidar_background_model_;
  BackgroundModel radar_background_model_;
};

#endif  // EXTRINSIC_REFLECTOR_BASED_CALIBRATOR__EXTRINSIC_REFLECTOR_BASED_CALIBRATOR_HPP_
