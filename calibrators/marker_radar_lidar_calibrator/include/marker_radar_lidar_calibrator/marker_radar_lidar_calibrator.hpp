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

#ifndef MARKER_RADAR_LIDAR_CALIBRATOR__MARKER_RADAR_LIDAR_CALIBRATOR_HPP_
#define MARKER_RADAR_LIDAR_CALIBRATOR__MARKER_RADAR_LIDAR_CALIBRATOR_HPP_

#include <Eigen/Dense>
#include <marker_radar_lidar_calibrator/track.hpp>
#include <marker_radar_lidar_calibrator/types.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_srvs/srv/empty.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tier4_calibration_msgs/srv/extrinsic_calibrator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace marker_radar_lidar_calibrator
{

class ExtrinsicReflectorBasedCalibrator : public rclcpp::Node
{
public:
  using PointType = pcl::PointXYZ;
  using index_t = std::uint32_t;
  enum class TransformationType { svd_2d, yaw_only_rotation_2d, svd_3d, zero_roll_3d };

  enum class MsgType { radar_tracks, radar_scan, radar_cloud };

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

  void deleteTrackRequestCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void radarTracksCallback(const radar_msgs::msg::RadarTracks::SharedPtr msg);

  void radarScanCallback(const radar_msgs::msg::RadarScan::SharedPtr msg);

  void radarCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  template <typename RadarMsgType>
  pcl::PointCloud<PointType>::Ptr extractRadarPointcloud(const std::shared_ptr<RadarMsgType> & msg);

  std::vector<Eigen::Vector3d> extractLidarReflectors(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  std::vector<Eigen::Vector3d> extractRadarReflectors(
    pcl::PointCloud<PointType>::Ptr radar_pointcloud_ptr);

  void extractBackgroundModel(
    const pcl::PointCloud<PointType>::Ptr & sensor_pointcloud,
    const std_msgs::msg::Header & current_header, std_msgs::msg::Header & last_updated_header,
    std_msgs::msg::Header & first_header, BackgroundModel & background_model);

  void extractForegroundPoints(
    const pcl::PointCloud<PointType>::Ptr & sensor_pointcloud,
    const BackgroundModel & background_model, bool use_ransac,
    pcl::PointCloud<PointType>::Ptr & foreground_points, Eigen::Vector4d & ground_model);

  std::vector<pcl::PointCloud<PointType>::Ptr> extractClusters(
    const pcl::PointCloud<PointType>::Ptr & foreground_pointcloud,
    const double cluster_max_tolerance, const int cluster_min_points, const int cluster_max_points);

  std::vector<Eigen::Vector3d> findReflectorsFromClusters(
    const std::vector<pcl::PointCloud<PointType>::Ptr> & clusters,
    const Eigen::Vector4d & ground_model);

  bool checkInitialTransforms();

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matchDetections(
    const std::vector<Eigen::Vector3d> & lidar_detections,
    const std::vector<Eigen::Vector3d> & radar_detections);

  bool trackMatches(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> & matches,
    builtin_interfaces::msg::Time & time);

  std::tuple<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr> getPointsSet();
  std::tuple<double, double> get2DRotationDelta(
    std::vector<Track> converged_tracks, bool is_crossval);

  std::pair<double, double> computeCalibrationError(
    const Eigen::Isometry3d & radar_to_lidar_isometry);
  void estimateTransformation();
  void calculateCalibrationError(Eigen::Isometry3d calibrated_radar_to_lidar_transformation);
  void crossValEvaluation();
  void findCombinations(
    int n, int k, std::vector<int> & curr, int first_num,
    std::vector<std::vector<int>> & combinations);
  void selectCombinations(
    int tracks_size, int num_of_samples, std::vector<std::vector<int>> & combinations);
  void evaluateCombinations(std::vector<std::vector<int>> & combinations, int num_of_samples);

  void publishMetrics();
  void calibrateSensors();
  void visualizationMarkers(
    const std::vector<Eigen::Vector3d> & lidar_detections,
    const std::vector<Eigen::Vector3d> & radar_detections,
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> & matched_detections);
  void visualizeTrackMarkers();
  void deleteTrackMarkers();
  void drawCalibrationStatusText();
  geometry_msgs::msg::Point eigenToPointMsg(const Eigen::Vector3d & p_eigen);
  double getYawError(const Eigen::Vector3d & v1, const Eigen::Vector3d & v2);

  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  struct Parameters
  {
    std::string radar_optimization_frame;  // frame that is assumed to be parallel to the radar
                                           // if estimating the transformation by 2d algorithms
                                           // (needed for radars that do not provide elevation)
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
    int max_number_of_combination_samples;
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_plane_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_foreground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_colored_clusters_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lidar_detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_background_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_foreground_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr radar_detections_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr matches_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracking_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr text_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr metrics_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<radar_msgs::msg::RadarTracks>::SharedPtr radar_tracks_sub_;
  rclcpp::Subscription<radar_msgs::msg::RadarScan>::SharedPtr radar_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_cloud_sub_;

  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr
    calibration_request_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr background_model_service_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr tracking_service_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr send_calibration_service_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr delete_track_service_server_;

  // Threading, sync, and result
  std::mutex mutex_;

  // ROS Data
  std_msgs::msg::Header lidar_header_, radar_header_;
  std::string lidar_frame_, radar_frame_;

  // Initial tfs comparable with the one with our method
  geometry_msgs::msg::Transform initial_radar_to_lidar_msg_;
  Eigen::Isometry3d initial_radar_to_lidar_eigen_;
  Eigen::Isometry3d calibrated_radar_to_lidar_eigen_;

  // radar optimization is the frame that radar optimize the transformation to.
  geometry_msgs::msg::Transform radar_optimization_to_lidar_msg_;
  Eigen::Isometry3d radar_optimization_to_lidar_eigen_;

  geometry_msgs::msg::Transform init_radar_optimization_to_radar_msg_;
  Eigen::Isometry3d initial_radar_optimization_to_radar_eigen_;

  bool got_initial_transform_{false};
  bool broadcast_tf_{false};
  bool calibration_valid_{false};
  double calibration_distance_score_{std::numeric_limits<double>::max()};
  double calibration_yaw_score_{std::numeric_limits<double>::max()};
  bool send_calibration_{false};

  // Background model
  bool extract_lidar_background_model_{false};
  bool extract_radar_background_model_{false};
  std_msgs::msg::Header latest_updated_lidar_header_;
  std_msgs::msg::Header latest_updated_radar_header_;
  std_msgs::msg::Header first_lidar_header_;
  std_msgs::msg::Header first_radar_header_;
  std_msgs::msg::Header latest_updated_radar_frame_;
  BackgroundModel lidar_background_model_;
  BackgroundModel radar_background_model_;

  radar_msgs::msg::RadarTracks::SharedPtr latest_radar_tracks_msgs_;
  radar_msgs::msg::RadarScan::SharedPtr latest_radar_scan_msgs_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_radar_cloud_msgs_;

  // Tracking
  bool tracking_active_{false};
  int current_new_tracks_{false};
  TrackFactory::Ptr factory_ptr_;
  std::vector<Track> active_tracks_;
  std::vector<Track> converged_tracks_;

  // Converged points
  pcl::PointCloud<PointType>::Ptr lidar_points_ocs_;
  pcl::PointCloud<PointType>::Ptr radar_points_rcs_;

  // Metrics
  std::vector<float> output_metrics_;

  MsgType msg_type_;
  TransformationType transformation_type_;
  static constexpr int MARKER_SIZE_PER_TRACK = 8;

  bool first_time_{true};
  Eigen::Vector4d ground_model_;
};

}  // namespace marker_radar_lidar_calibrator

#endif  // MARKER_RADAR_LIDAR_CALIBRATOR__MARKER_RADAR_LIDAR_CALIBRATOR_HPP_
