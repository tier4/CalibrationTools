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
#include <extrinsic_reflector_based_calibrator/track.hpp>
#include <extrinsic_reflector_based_calibrator/types.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_srvs/srv/empty.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
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
#include <utility>
#include <vector>

class ExtrinsicReflectorBasedCalibrator : public rclcpp::Node
{
public:
  using PointType = pcl::PointXYZ;
  using index_t = std::uint32_t;

  explicit ExtrinsicReflectorBasedCalibrator(const rclcpp::NodeOptions & options);

protected:
  void requestReceivedCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);

  void timerCallback();

  void backgroundModelRequestCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void trackingRequestCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void sendCalibrationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void radarCallback(const radar_msgs::msg::RadarTracks::SharedPtr msg);

  std::vector<Eigen::Vector3d> extractReflectors(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  std::vector<Eigen::Vector3d> extractReflectors(const radar_msgs::msg::RadarTracks::SharedPtr msg);

  void extractBackgroundModel(
    const pcl::PointCloud<PointType>::Ptr & sensor_pointcloud,
    const std_msgs::msg::Header & current_header, std_msgs::msg::Header & last_updated_header,
    std_msgs::msg::Header & first_header, BackgroundModel & background_model);

  void extractForegroundPoints(
    const pcl::PointCloud<PointType>::Ptr & sensor_pointcloud,
    const BackgroundModel & background_model, bool use_ransac,
    pcl::PointCloud<PointType>::Ptr & foreground_points, Eigen::Vector4f & ground_model);

  std::vector<pcl::PointCloud<PointType>::Ptr> extractClusters(
    const pcl::PointCloud<PointType>::Ptr & foreground_pointcloud,
    const double cluster_max_tolerance, const int cluster_min_points, const int cluster_max_points);

  std::vector<Eigen::Vector3d> findReflectorsFromClusters(
    const std::vector<pcl::PointCloud<PointType>::Ptr> & clusters,
    const Eigen::Vector4f & ground_model);

  bool checkInitialTransforms();

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matchDetections(
    const std::vector<Eigen::Vector3d> & lidar_detections,
    const std::vector<Eigen::Vector3d> & radar_detections);

  void trackMatches(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> & matches,
    builtin_interfaces::msg::Time & time);
  void calibrateSensors();
  void visualizationMarkers(
    const std::vector<Eigen::Vector3d> & lidar_detections,
    const std::vector<Eigen::Vector3d> & radar_detections,
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> & matched_detections);

  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  struct Parameters
  {
    std::string parent_frame;
    bool use_lidar_initial_crop_box_filter;
    double lidar_initial_crop_box_min_x;
    double lidar_initial_crop_box_min_y;
    double lidar_initial_crop_box_min_z;
    double lidar_initial_crop_box_max_x;
    double lidar_initial_crop_box_max_y;
    double lidar_initial_crop_box_max_z;
    bool use_radar_initial_crop_box_filter;
    double radar_initial_crop_box_min_x;
    double radar_initial_crop_box_min_y;
    double radar_initial_crop_box_min_z;
    double radar_initial_crop_box_max_x;
    double radar_initial_crop_box_max_y;
    double radar_initial_crop_box_max_z;
    double lidar_background_model_leaf_size;
    double radar_background_model_leaf_size;
    double max_calibration_range;
    double background_model_timeout;
    double max_match_yaw_distance;
    double min_foreground_distance;  // needs to be about at least double the leaf size
    double background_extraction_timeout;
    double ransac_threshold;
    int ransac_max_iterations;
    double lidar_cluster_max_tolerance;
    int lidar_cluster_min_points;
    int lidar_cluster_max_points;
    double radar_cluster_max_tolerance;
    int radar_cluster_min_points;
    int radar_cluster_max_points;

    double reflector_radius;
    double reflector_max_height;
    double max_matching_distance;
    double max_initial_calibration_translation_error;
    double max_initial_calibration_rotation_error;
  } parameters_;

  // ROS Interface
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  rclcpp::CallbackGroup::SharedPtr calibration_api_srv_callback_group_;
  rclcpp::CallbackGroup::SharedPtr calibration_ui_srv_callback_group_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_background_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_foreground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_colored_clusters_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lidar_detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_background_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_foreground_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr radar_detections_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr matches_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracking_markers_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<radar_msgs::msg::RadarTracks>::SharedPtr radar_sub_;

  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr
    calibration_request_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr background_model_service_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr tracking_service_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr send_calibration_service_server_;

  // Threading, sync, and result
  std::mutex mutex_;

  // ROS Data
  std_msgs::msg::Header lidar_header_, radar_header_;
  std::string lidar_frame_, radar_frame_;

  // Initial tfs comparable with the one with our method
  geometry_msgs::msg::Transform initial_radar_to_lidar_msg_;
  tf2::Transform initial_radar_to_lidar_tf2_;
  Eigen::Isometry3d initial_radar_to_lidar_eigen_;
  Eigen::Isometry3d calibrated_radar_to_lidar_eigen_;

  geometry_msgs::msg::Transform parent_to_lidar_msg_;
  tf2::Transform parent_to_lidar_tf2_;
  Eigen::Isometry3d parent_to_lidar_eigen_;

  bool got_initial_transform_;
  bool broadcast_tf_;
  bool calibration_valid_;
  bool send_calibration_;

  // Background model
  bool extract_lidar_background_model_;
  bool extract_radar_background_model_;
  std_msgs::msg::Header latest_updated_lidar_header_;
  std_msgs::msg::Header latest_updated_radar_header_;
  std_msgs::msg::Header first_lidar_header_;
  std_msgs::msg::Header first_radar_header_;
  std_msgs::msg::Header latest_updated_radar_frame_;
  BackgroundModel lidar_background_model_;
  BackgroundModel radar_background_model_;

  radar_msgs::msg::RadarTracks::SharedPtr latest_radar_msgs_;

  // Tracking
  bool tracking_active_;
  int current_new_tracks_;
  TrackFactory::Ptr factory_ptr_;
  std::vector<Track> active_tracks_;
  std::vector<Track> converged_tracks_;
};

#endif  // EXTRINSIC_REFLECTOR_BASED_CALIBRATOR__EXTRINSIC_REFLECTOR_BASED_CALIBRATOR_HPP_
