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

#include <extrinsic_reflector_based_calibrator/extrinsic_reflector_based_calibrator.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <chrono>
#include <iostream>
#include <limits>

#define UPDATE_PARAM(PARAM_STRUCT, NAME) update_param(parameters, #NAME, PARAM_STRUCT.NAME)

namespace
{
template <typename T>
void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
  if (it != parameters.cend()) {
    value = it->template get_value<T>();
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("extrinsic_reflector_based_calibrator"),
      "Setting parameter [" << name << "] to " << value);
  }
}
}  // namespace

rcl_interfaces::msg::SetParametersResult ExtrinsicReflectorBasedCalibrator::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  Parameters p = parameters_;

  try {
    UPDATE_PARAM(p, parent_frame);
    UPDATE_PARAM(p, use_lidar_initial_crop_box_filter);
    UPDATE_PARAM(p, lidar_initial_crop_box_min_x);
    UPDATE_PARAM(p, lidar_initial_crop_box_min_y);
    UPDATE_PARAM(p, lidar_initial_crop_box_min_z);
    UPDATE_PARAM(p, lidar_initial_crop_box_max_x);
    UPDATE_PARAM(p, lidar_initial_crop_box_max_y);
    UPDATE_PARAM(p, lidar_initial_crop_box_max_z);
    UPDATE_PARAM(p, use_radar_initial_crop_box_filter);
    UPDATE_PARAM(p, radar_initial_crop_box_min_x);
    UPDATE_PARAM(p, radar_initial_crop_box_min_y);
    UPDATE_PARAM(p, radar_initial_crop_box_min_z);
    UPDATE_PARAM(p, radar_initial_crop_box_max_x);
    UPDATE_PARAM(p, radar_initial_crop_box_max_y);
    UPDATE_PARAM(p, radar_initial_crop_box_max_z);
    UPDATE_PARAM(p, lidar_background_model_leaf_size);
    UPDATE_PARAM(p, radar_background_model_leaf_size);

    UPDATE_PARAM(p, max_calibration_range);
    UPDATE_PARAM(p, background_model_timeout);
    UPDATE_PARAM(p, max_match_yaw_distance);
    UPDATE_PARAM(p, min_foreground_distance);
    UPDATE_PARAM(p, background_extraction_timeout);
    UPDATE_PARAM(p, ransac_threshold);
    UPDATE_PARAM(p, ransac_max_iterations);
    UPDATE_PARAM(p, lidar_cluster_max_tolerance);
    UPDATE_PARAM(p, lidar_cluster_min_points);
    UPDATE_PARAM(p, lidar_cluster_max_points);
    UPDATE_PARAM(p, radar_cluster_max_tolerance);
    UPDATE_PARAM(p, radar_cluster_min_points);
    UPDATE_PARAM(p, radar_cluster_max_points);
    UPDATE_PARAM(p, reflector_radius);
    UPDATE_PARAM(p, reflector_max_height);
    UPDATE_PARAM(p, max_matching_distance);
    UPDATE_PARAM(p, max_initial_calibration_translation_error);
    UPDATE_PARAM(p, max_initial_calibration_rotation_error);

    // transaction succeeds, now assign values
    parameters_ = p;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

ExtrinsicReflectorBasedCalibrator::ExtrinsicReflectorBasedCalibrator(
  const rclcpp::NodeOptions & options)
: Node("extrinsic_reflector_based_calibrator_node", options),
  tf_broadcaster_(this),
  got_initial_transform_(false),
  calibration_valid_(false),
  send_calibration_(false),
  extract_lidar_background_model_(false),
  extract_radar_background_model_(false),
  tracking_active_(false),
  current_new_tracks_(0)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  parameters_.parent_frame = this->declare_parameter<std::string>("parent_frame");

  parameters_.use_lidar_initial_crop_box_filter =
    this->declare_parameter<bool>("use_lidar_initial_crop_box_filter", true);
  parameters_.lidar_initial_crop_box_min_x =
    this->declare_parameter<double>("lidar_initial_crop_box_min_x", -50.0);
  parameters_.lidar_initial_crop_box_min_y =
    this->declare_parameter<double>("lidar_initial_crop_box_min_y", -50.0);
  parameters_.lidar_initial_crop_box_min_z =
    this->declare_parameter<double>("lidar_initial_crop_box_min_z", -50.0);
  parameters_.lidar_initial_crop_box_max_x =
    this->declare_parameter<double>("lidar_initial_crop_box_max_x", 50.0);
  parameters_.lidar_initial_crop_box_max_y =
    this->declare_parameter<double>("lidar_initial_crop_box_max_y", 50.0);
  parameters_.lidar_initial_crop_box_max_z =
    this->declare_parameter<double>("lidar_initial_crop_box_max_z", 50.0);

  parameters_.use_radar_initial_crop_box_filter =
    this->declare_parameter<bool>("use_radar_initial_crop_box_filter", true);
  parameters_.radar_initial_crop_box_min_x =
    this->declare_parameter<double>("radar_initial_crop_box_min_x", -50.0);
  parameters_.radar_initial_crop_box_min_y =
    this->declare_parameter<double>("radar_initial_crop_box_min_y", -50.0);
  parameters_.radar_initial_crop_box_min_z =
    this->declare_parameter<double>("radar_initial_crop_box_min_z", -50.0);
  parameters_.radar_initial_crop_box_max_x =
    this->declare_parameter<double>("radar_initial_crop_box_max_x", 50.0);
  parameters_.radar_initial_crop_box_max_y =
    this->declare_parameter<double>("radar_initial_crop_box_max_y", 50.0);
  parameters_.radar_initial_crop_box_max_z =
    this->declare_parameter<double>("radar_initial_crop_box_max_z", 50.0);

  double calibration_max_range = this->declare_parameter<double>("calibration_max_range", 60.0);
  lidar_background_model_.min_point_ =
    Eigen::Vector4f(-calibration_max_range, -calibration_max_range, -calibration_max_range, 1.f);
  lidar_background_model_.max_point_ =
    Eigen::Vector4f(calibration_max_range, calibration_max_range, calibration_max_range, 1.f);
  radar_background_model_.min_point_ =
    Eigen::Vector4f(-calibration_max_range, -calibration_max_range, -calibration_max_range, 1.f);
  radar_background_model_.max_point_ =
    Eigen::Vector4f(calibration_max_range, calibration_max_range, calibration_max_range, 1.f);

  lidar_background_model_.leaf_size_ =
    this->declare_parameter<double>("lidar_background_model_leaf_size", 0.1);
  radar_background_model_.leaf_size_ =
    this->declare_parameter<double>("radar_background_model_leaf_size", 0.1);
  parameters_.max_calibration_range =
    this->declare_parameter<double>("max_calibration_range", 50.0);
  parameters_.background_model_timeout =
    this->declare_parameter<double>("background_model_timeout", 5.0);
  parameters_.min_foreground_distance =
    this->declare_parameter<double>("min_foreground_distance", 0.4);
  parameters_.background_extraction_timeout =
    this->declare_parameter<double>("background_extraction_timeout", 15.0);
  parameters_.ransac_threshold = this->declare_parameter<double>("ransac_threshold", 0.2);
  parameters_.ransac_max_iterations = this->declare_parameter<int>("ransac_max_iterations", 100);
  parameters_.lidar_cluster_max_tolerance =
    this->declare_parameter<double>("lidar_cluster_max_tolerance", 0.5);
  parameters_.lidar_cluster_min_points =
    this->declare_parameter<int>("lidar_cluster_min_points", 3);
  parameters_.lidar_cluster_max_points =
    this->declare_parameter<int>("lidar_cluster_max_points", 2000);
  parameters_.radar_cluster_max_tolerance =
    this->declare_parameter<double>("radar_cluster_max_tolerance", 0.5);
  parameters_.radar_cluster_min_points =
    this->declare_parameter<int>("radar_cluster_min_points", 1);
  parameters_.radar_cluster_max_points =
    this->declare_parameter<int>("radar_cluster_max_points", 10);
  parameters_.reflector_radius = this->declare_parameter<double>("reflector_radius", 0.1);
  parameters_.reflector_max_height = this->declare_parameter<double>("reflector_max_height", 1.2);
  parameters_.max_matching_distance = this->declare_parameter<double>("max_matching_distance", 1.0);

  double initial_lidar_cov = this->declare_parameter<double>("initial_lidar_cov", 0.5);
  double initial_radar_cov = this->declare_parameter<double>("initial_radar_cov", 2.0);
  double lidar_measurement_cov = this->declare_parameter<double>("lidar_measurement_cov", 0.03);
  double radar_measurement_cov = this->declare_parameter<double>("radar_measurement_cov", 0.05);
  double lidar_process_cov = this->declare_parameter<double>("lidar_process_cov", 0.01);
  double radar_process_cov = this->declare_parameter<double>("radar_process_cov", 0.01);
  double lidar_convergence_thresh =
    this->declare_parameter<double>("lidar_convergence_thresh", 0.03);
  double radar_convergence_thresh =
    this->declare_parameter<double>("radar_convergence_thresh", 0.03);
  double timeout_thresh = this->declare_parameter<double>("timeout_thresh", 3.0);

  parameters_.max_initial_calibration_translation_error =
    this->declare_parameter<double>("max_initial_calibration_translation_error", 1.0);
  parameters_.max_initial_calibration_rotation_error =
    this->declare_parameter<double>("max_initial_calibration_rotation_error", 45.0);

  factory_ptr_ = std::make_shared<TrackFactory>(
    initial_lidar_cov, initial_radar_cov, lidar_measurement_cov, radar_measurement_cov,
    lidar_process_cov, radar_process_cov, lidar_convergence_thresh, radar_convergence_thresh,
    timeout_thresh, parameters_.max_matching_distance);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);

  lidar_background_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_background_pointcloud", 10);
  lidar_foreground_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_foreground_pointcloud", 10);
  lidar_colored_clusters_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_colored_clusters", 10);
  lidar_detections_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("lidar_detection_markers", 10);

  radar_background_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("radar_background_pointcloud", 10);
  radar_foreground_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("radar_foreground_pointcloud", 10);
  radar_detections_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("radar_detection_markers", 10);
  matches_markers_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("matches_markers", 10);
  tracking_markers_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("tracking_markers", 10);
  text_markers_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("text_markers", 10);
  metrics_pub_ =
    this->create_publisher<std_msgs::msg::Float32MultiArray>("cross_validation_metrics", 10);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_lidar_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ExtrinsicReflectorBasedCalibrator::lidarCallback, this, std::placeholders::_1));

  radar_sub_ = this->create_subscription<radar_msgs::msg::RadarTracks>(
    "input_radar_objects", rclcpp::SensorDataQoS(),
    std::bind(&ExtrinsicReflectorBasedCalibrator::radarCallback, this, std::placeholders::_1));

  // The service server runs in a dedicated thread
  calibration_api_srv_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  calibration_ui_srv_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&ExtrinsicReflectorBasedCalibrator::paramCallback, this, std::placeholders::_1));

  calibration_request_server_ =
    this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
      "extrinsic_calibration",
      std::bind(
        &ExtrinsicReflectorBasedCalibrator::requestReceivedCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, calibration_api_srv_callback_group_);

  background_model_service_server_ = this->create_service<std_srvs::srv::Empty>(
    "extract_background_model",
    std::bind(
      &ExtrinsicReflectorBasedCalibrator::backgroundModelRequestCallback, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, calibration_ui_srv_callback_group_);

  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(1),
    std::bind(&ExtrinsicReflectorBasedCalibrator::timerCallback, this));
}

void ExtrinsicReflectorBasedCalibrator::requestReceivedCallback(
  __attribute__((unused))
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request>
    request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  using std::chrono_literals::operator""s;

  // Loop until the calibration finishes
  while (rclcpp::ok()) {
    rclcpp::sleep_for(1s);
    std::unique_lock<std::mutex> lock(mutex_);

    if (send_calibration_) {
      break;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 60000, "Waiting for the calibration to end");
  }

  std::unique_lock<std::mutex> lock(mutex_);

  Eigen::Isometry3d calibrated_parent_to_radar_transform =
    parent_to_lidar_eigen_ * calibrated_radar_to_lidar_eigen_.inverse();
  geometry_msgs::msg::Pose calibrated_parent_to_radar_pose =
    toMsg(calibrated_parent_to_radar_transform);
  response->result_pose = calibrated_parent_to_radar_pose;
  response->success = true;
}

void ExtrinsicReflectorBasedCalibrator::timerCallback()
{
  if (
    lidar_background_model_.valid_ && radar_background_model_.valid_ && !tracking_service_server_) {
    tracking_service_server_ = this->create_service<std_srvs::srv::Empty>(
      "add_lidar_radar_pair",
      std::bind(
        &ExtrinsicReflectorBasedCalibrator::trackingRequestCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, calibration_ui_srv_callback_group_);
  }

  if (calibration_valid_ && !send_calibration_service_server_) {
    send_calibration_service_server_ = this->create_service<std_srvs::srv::Empty>(
      "send_calibration",
      std::bind(
        &ExtrinsicReflectorBasedCalibrator::sendCalibrationCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, calibration_ui_srv_callback_group_);
  }

  if (converged_tracks_.size() > 0 && !delete_track_service_server_) {
    delete_track_service_server_ = this->create_service<std_srvs::srv::Empty>(
      "delete_lidar_radar_pair",
      std::bind(
        &ExtrinsicReflectorBasedCalibrator::deleteTrackRequestCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, calibration_ui_srv_callback_group_);
  }

}

void ExtrinsicReflectorBasedCalibrator::backgroundModelRequestCallback(
  __attribute__((unused)) const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  __attribute__((unused)) const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  using std::chrono_literals::operator""s;

  {
    std::unique_lock<std::mutex> lock(mutex_);
    extract_lidar_background_model_ = true;
    extract_radar_background_model_ = true;
  }

  while (rclcpp::ok()) {
    rclcpp::sleep_for(1s);
    std::unique_lock<std::mutex> lock(mutex_);

    if (lidar_background_model_.valid_ && radar_background_model_.valid_) {
      break;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Waiting for the calibration to end");
  }

  RCLCPP_INFO(this->get_logger(), "Background model estimated");
}

void ExtrinsicReflectorBasedCalibrator::trackingRequestCallback(
  __attribute__((unused)) const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  __attribute__((unused)) const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  using std::chrono_literals::operator""s;

  {
    std::unique_lock<std::mutex> lock(mutex_);
    tracking_active_ = true;
    current_new_tracks_ = 0;
  }

  while (rclcpp::ok()) {
    rclcpp::sleep_for(1s);
    std::unique_lock<std::mutex> lock(mutex_);

    if (!tracking_active_ && current_new_tracks_) {
      break;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for detections to converge. Active tracks=%lu. New tracks=%d", active_tracks_.size(),
      current_new_tracks_);
  }

  RCLCPP_INFO(this->get_logger(), "New converged detections: %d", current_new_tracks_);
}

void ExtrinsicReflectorBasedCalibrator::deleteTrackRequestCallback(
  __attribute__((unused)) const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  __attribute__((unused)) const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  using std::chrono_literals::operator""s;

  if (converged_tracks_.size() > 0) {
    converged_tracks_.pop_back();
    calibrateSensors();
    visualizeTrackMarkers();
    drawCalibrationStatusText();
      RCLCPP_INFO(this->get_logger(), "You delete one previous track, there are %d converged tracks remain", 
        static_cast<int>(converged_tracks_.size()));
  }
  else {
    RCLCPP_INFO(this->get_logger(), "You cannot delete previous track, because there are no track");
  }
}

void ExtrinsicReflectorBasedCalibrator::sendCalibrationCallback(
  __attribute__((unused)) const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  __attribute__((unused)) const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_);
  send_calibration_ = true;
}

void ExtrinsicReflectorBasedCalibrator::lidarCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "lidarCallback");
  if (!latest_radar_msgs_ || latest_radar_msgs_->tracks.size() == 0) {
    RCLCPP_INFO(this->get_logger(), "There were no tracks");
    return;
  }

  auto lidar_detections = extractReflectors(msg);
  auto radar_detections = extractReflectors(latest_radar_msgs_);
  latest_radar_msgs_->tracks.clear();

  auto matches = matchDetections(lidar_detections, radar_detections);

  bool is_converged = false;
  trackMatches(matches, msg->header.stamp, is_converged);
  if(is_converged)
    calibrateSensors();
  visualizationMarkers(lidar_detections, radar_detections, matches);
  visualizeTrackMarkers();
  drawCalibrationStatusText();
  

  RCLCPP_INFO(
    this->get_logger(),
    "Lidar detections: %lu Radar detections: %lu Matches: %lu Active tracks; %lu Converged tracks: "
    "%lu",
    lidar_detections.size(), radar_detections.size(), matches.size(), active_tracks_.size(),
    converged_tracks_.size());
}

void ExtrinsicReflectorBasedCalibrator::radarCallback(
  const radar_msgs::msg::RadarTracks::SharedPtr msg)
{
  if (!latest_radar_msgs_) {
    latest_radar_msgs_ = msg;
  } else {
    latest_radar_msgs_->header = msg->header;
    latest_radar_msgs_->tracks.insert(
      latest_radar_msgs_->tracks.end(), msg->tracks.begin(), msg->tracks.end());
  }
}

std::vector<Eigen::Vector3d> ExtrinsicReflectorBasedCalibrator::extractReflectors(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  lidar_frame_ = msg->header.frame_id;
  lidar_header_ = msg->header;
  bool extract_background_model;
  bool valid_background_model;
  std::vector<Eigen::Vector3d> detections;

  {
    std::unique_lock<std::mutex> lock(mutex_);
    extract_background_model = extract_lidar_background_model_;
    valid_background_model = lidar_background_model_.valid_;
  }

  pcl::PointCloud<PointType>::Ptr lidar_pointcloud_ptr(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*msg, *lidar_pointcloud_ptr);

  if (parameters_.use_lidar_initial_crop_box_filter) {
    pcl::CropBox<PointType> box_filter;
    pcl::PointCloud<PointType>::Ptr tmp_lidar_pointcloud_ptr(new pcl::PointCloud<PointType>);
    RCLCPP_INFO(this->get_logger(), "pre lidar_pointcloud_ptr=%lu", lidar_pointcloud_ptr->size());
    RCLCPP_WARN(
      this->get_logger(), "cropbox parameters=%f | %f | %f",
      parameters_.lidar_initial_crop_box_min_x, parameters_.lidar_initial_crop_box_min_y,
      parameters_.lidar_initial_crop_box_min_z);
    RCLCPP_WARN(
      this->get_logger(), "cropbox parameters=%f | %f | %f",
      parameters_.lidar_initial_crop_box_max_x, parameters_.lidar_initial_crop_box_max_y,
      parameters_.lidar_initial_crop_box_max_z);
    box_filter.setMin(Eigen::Vector4f(
      parameters_.lidar_initial_crop_box_min_x, parameters_.lidar_initial_crop_box_min_y,
      parameters_.lidar_initial_crop_box_min_z, 1.0));
    box_filter.setMax(Eigen::Vector4f(
      parameters_.lidar_initial_crop_box_max_x, parameters_.lidar_initial_crop_box_max_y,
      parameters_.lidar_initial_crop_box_max_z, 1.0));
    box_filter.setInputCloud(lidar_pointcloud_ptr);
    box_filter.filter(*tmp_lidar_pointcloud_ptr);
    lidar_pointcloud_ptr.swap(tmp_lidar_pointcloud_ptr);
    RCLCPP_INFO(this->get_logger(), "lidar_pointcloud_ptr=%lu", lidar_pointcloud_ptr->size());
  }

  if (extract_background_model && !valid_background_model) {
    extractBackgroundModel(
      lidar_pointcloud_ptr, msg->header, latest_updated_lidar_header_, first_lidar_header_,
      lidar_background_model_);
    return detections;
  }

  if (!valid_background_model) {
    return detections;
  }

  pcl::PointCloud<PointType>::Ptr foreground_pointcloud_ptr;
  Eigen::Vector4f ground_model;
  extractForegroundPoints(
    lidar_pointcloud_ptr, lidar_background_model_, true, foreground_pointcloud_ptr, ground_model);

  auto clusters = extractClusters(
    foreground_pointcloud_ptr, parameters_.lidar_cluster_max_tolerance,
    parameters_.lidar_cluster_min_points, parameters_.lidar_cluster_max_points);
  detections = findReflectorsFromClusters(clusters, ground_model);

  // Visualization
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clusters_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZRGB>());

  Eigen::Vector3i colors[7] = {{0, 0, 255},   {0, 128, 255}, {0, 200, 200}, {0, 255, 0},
                               {200, 200, 0}, {255, 0, 0},   {255, 0, 255}};

  std::size_t colored_clusters_pointcloud_size = 0;

  for (const auto & cluster : clusters) {
    colored_clusters_pointcloud_size += cluster->size();
  }

  colored_clusters_pointcloud_ptr->reserve(colored_clusters_pointcloud_size);

  for (std::size_t i = 0; i < clusters.size(); i++) {
    const auto & cluster = clusters[i];
    pcl::PointXYZRGB colored_p;
    auto & color = colors[i % 7];

    for (const auto & p : cluster->points) {
      colored_p.getArray3fMap() = p.getArray3fMap();
      colored_p.r = color.x();
      colored_p.b = color.y();
      colored_p.g = color.z();
      colored_clusters_pointcloud_ptr->push_back(colored_p);
    }
  }

  colored_clusters_pointcloud_ptr->width = colored_clusters_pointcloud_ptr->size();
  colored_clusters_pointcloud_ptr->height = 1;

  RCLCPP_INFO(
    this->get_logger(), "Colored clusters size=%lu", colored_clusters_pointcloud_ptr->size());

  sensor_msgs::msg::PointCloud2 background_msg;
  pcl::toROSMsg(*lidar_background_model_.pointcloud_, background_msg);
  background_msg.header = lidar_header_;
  lidar_background_pub_->publish(background_msg);

  sensor_msgs::msg::PointCloud2 foreground_msg;
  pcl::toROSMsg(*foreground_pointcloud_ptr, foreground_msg);
  foreground_msg.header = lidar_header_;
  lidar_foreground_pub_->publish(foreground_msg);

  sensor_msgs::msg::PointCloud2 colored_clusters_msg;
  pcl::toROSMsg(*colored_clusters_pointcloud_ptr, colored_clusters_msg);
  colored_clusters_msg.header = lidar_header_;
  lidar_colored_clusters_pub_->publish(colored_clusters_msg);

  return detections;
}

std::vector<Eigen::Vector3d> ExtrinsicReflectorBasedCalibrator::extractReflectors(
  const radar_msgs::msg::RadarTracks::SharedPtr msg)
{
  radar_frame_ = msg->header.frame_id;
  radar_header_ = msg->header;
  bool extract_background_model;
  bool valid_background_model;
  std::vector<Eigen::Vector3d> detections;

  {
    std::unique_lock<std::mutex> lock(mutex_);
    extract_background_model = extract_radar_background_model_;
    valid_background_model = radar_background_model_.valid_;
  }

  pcl::PointCloud<PointType>::Ptr radar_pointcloud_ptr(new pcl::PointCloud<PointType>);
  radar_pointcloud_ptr->reserve(msg->tracks.size());

  for (const auto & track : msg->tracks) {
    radar_pointcloud_ptr->emplace_back(track.position.x, track.position.y, track.position.z);
  }

  if (parameters_.use_radar_initial_crop_box_filter) {
    pcl::CropBox<PointType> box_filter;
    pcl::PointCloud<PointType>::Ptr tmp_radar_pointcloud_ptr(new pcl::PointCloud<PointType>);
    box_filter.setMin(Eigen::Vector4f(
      parameters_.radar_initial_crop_box_min_x, parameters_.radar_initial_crop_box_min_y,
      parameters_.radar_initial_crop_box_min_z, 1.0));
    box_filter.setMax(Eigen::Vector4f(
      parameters_.radar_initial_crop_box_max_x, parameters_.radar_initial_crop_box_max_y,
      parameters_.radar_initial_crop_box_max_z, 1.0));
    box_filter.setInputCloud(radar_pointcloud_ptr);
    box_filter.filter(*tmp_radar_pointcloud_ptr);
    radar_pointcloud_ptr.swap(tmp_radar_pointcloud_ptr);
  }

  if (extract_background_model && !valid_background_model) {
    extractBackgroundModel(
      radar_pointcloud_ptr, msg->header, latest_updated_radar_header_, first_radar_header_,
      radar_background_model_);
    return detections;
  }

  if (!valid_background_model) {
    return detections;
  }

  pcl::PointCloud<PointType>::Ptr foreground_pointcloud_ptr;
  Eigen::Vector4f ground_model;
  extractForegroundPoints(
    radar_pointcloud_ptr, radar_background_model_, false, foreground_pointcloud_ptr, ground_model);
  auto clusters = extractClusters(
    foreground_pointcloud_ptr, parameters_.radar_cluster_max_tolerance,
    parameters_.radar_cluster_min_points, parameters_.radar_cluster_max_points);

  detections.reserve(clusters.size());

  RCLCPP_INFO(this->get_logger(), "Extracting radar reflectors from clusters");

  for (const auto & cluster : clusters) {
    Eigen::Vector3d p_avg = Eigen::Vector3d::Zero();

    for (const auto & p : cluster->points) {
      p_avg += Eigen::Vector3d(p.x, p.y, p.z);
    }

    p_avg /= cluster->points.size();
    RCLCPP_INFO(
      this->get_logger(), "\t Radar reflector id=%lu size=%lu center: x=%.2f y=%.2f z=%.2f",
      detections.size(), cluster->points.size(), p_avg.x(), p_avg.y(), p_avg.z());

    detections.emplace_back(p_avg);
  }

  sensor_msgs::msg::PointCloud2 background_msg;
  pcl::toROSMsg(*radar_background_model_.pointcloud_, background_msg);
  background_msg.header = radar_header_;
  radar_background_pub_->publish(background_msg);

  return detections;
}

void ExtrinsicReflectorBasedCalibrator::extractBackgroundModel(
  const pcl::PointCloud<PointType>::Ptr & sensor_pointcloud_ptr,
  const std_msgs::msg::Header & current_header, std_msgs::msg::Header & last_updated_header,
  std_msgs::msg::Header & first_header, BackgroundModel & background_model)
{
  // Initialize background model in the first iteration
  if (background_model.set_.size() == 0) {
    background_model.min_point_ = Eigen::Vector4f(
      -parameters_.max_calibration_range, -parameters_.max_calibration_range,
      -parameters_.max_calibration_range, 1.0);
    background_model.max_point_ = Eigen::Vector4f(
      parameters_.max_calibration_range, parameters_.max_calibration_range,
      parameters_.max_calibration_range, 1.0);
    last_updated_header = current_header;
    first_header = current_header;

    RCLCPP_INFO(this->get_logger(), "Background model estimation: first iteration");
    RCLCPP_INFO(
      this->get_logger(), "\t min_point: x=%.2f y=%.2f z=%.2f w=%.2f",
      background_model.min_point_.x(), background_model.min_point_.y(),
      background_model.min_point_.z(), background_model.min_point_.w());
    RCLCPP_INFO(
      this->get_logger(), "\t max_point: x=%.2f y=%.2f z=%.2f w=%.2f",
      background_model.max_point_.x(), background_model.max_point_.y(),
      background_model.max_point_.z(), background_model.max_point_.w());
  }

  index_t x_cells = (background_model.max_point_.x() - background_model.min_point_.x()) /
                    background_model.leaf_size_;
  index_t y_cells = (background_model.max_point_.y() - background_model.min_point_.y()) /
                    background_model.leaf_size_;
  index_t z_cells = (background_model.max_point_.z() - background_model.min_point_.z()) /
                    background_model.leaf_size_;
  background_model.pointcloud_->points.reserve(x_cells * y_cells * z_cells);
  index_t prev_num_points = background_model.set_.size();

  for (const auto & p : sensor_pointcloud_ptr->points) {
    index_t x_index =
      static_cast<index_t>((p.x - background_model.min_point_.x()) / background_model.leaf_size_);
    index_t y_index =
      static_cast<index_t>((p.y - background_model.min_point_.y()) / background_model.leaf_size_);
    index_t z_index =
      static_cast<index_t>((p.z - background_model.min_point_.z()) / background_model.leaf_size_);
    index_t index = z_index * y_cells * x_cells + y_index * x_cells + x_index;
    const auto & it = background_model.set_.emplace(index);

    if (it.second) {
      PointType p_center;
      p_center.x = background_model.min_point_.x() + background_model.leaf_size_ * (x_index + 0.5f);
      p_center.y = background_model.min_point_.y() + background_model.leaf_size_ * (y_index + 0.5f);
      p_center.z = background_model.min_point_.z() + background_model.leaf_size_ * (z_index + 0.5f);
      background_model.pointcloud_->push_back(p_center);
    }
  }

  double time_since_last_start =
    (rclcpp::Time(current_header.stamp) - rclcpp::Time(first_header.stamp)).seconds();

  if (
    background_model.set_.size() > prev_num_points &&
    time_since_last_start < parameters_.background_extraction_timeout) {
    RCLCPP_INFO(
      this->get_logger(), "Current points in the background model: %lu",
      background_model.set_.size());
    last_updated_header = current_header;
    return;
  }

  double time_since_last_update =
    (rclcpp::Time(current_header.stamp) - rclcpp::Time(last_updated_header.stamp)).seconds();
  if (
    time_since_last_update < parameters_.background_model_timeout &&
    time_since_last_update >= 0.0 &&
    time_since_last_start < parameters_.background_extraction_timeout) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Waiting for timeout (%.2f)",
      time_since_last_update);
    return;
  }

  background_model.tree_.setInputCloud(background_model.pointcloud_);
  background_model.pointcloud_->points.shrink_to_fit();

  RCLCPP_INFO(this->get_logger(), "Finished background model initialization");

  {
    std::unique_lock<std::mutex> lock(mutex_);
    background_model.valid_ = true;
  }
}

void ExtrinsicReflectorBasedCalibrator::extractForegroundPoints(
  const pcl::PointCloud<PointType>::Ptr & sensor_pointcloud_ptr,
  const BackgroundModel & background_model, bool use_ransac,
  pcl::PointCloud<PointType>::Ptr & foreground_pointcloud_ptr, Eigen::Vector4f & ground_model)
{
  RCLCPP_INFO(this->get_logger(), "Extracting foreground");
  RCLCPP_INFO(this->get_logger(), "\t initial points: %lu", sensor_pointcloud_ptr->size());

  // Crop box
  pcl::PointCloud<PointType>::Ptr cropped_pointcloud_ptr(new pcl::PointCloud<PointType>);
  pcl::CropBox<PointType> crop_filter;
  crop_filter.setMin(background_model.min_point_);
  crop_filter.setMax(background_model.max_point_);
  crop_filter.setInputCloud(sensor_pointcloud_ptr);
  crop_filter.filter(*cropped_pointcloud_ptr);
  RCLCPP_INFO(this->get_logger(), "\t cropped points: %lu", cropped_pointcloud_ptr->size());

  // Fast hash
  pcl::PointCloud<PointType>::Ptr voxel_filtered_pointcloud_ptr(new pcl::PointCloud<PointType>);
  voxel_filtered_pointcloud_ptr->reserve(cropped_pointcloud_ptr->size());

  index_t x_cells = (background_model.max_point_.x() - background_model.min_point_.x()) /
                    background_model.leaf_size_;
  index_t y_cells = (background_model.max_point_.y() - background_model.min_point_.y()) /
                    background_model.leaf_size_;

  for (const auto & p : cropped_pointcloud_ptr->points) {
    index_t x_index =
      static_cast<index_t>((p.x - background_model.min_point_.x()) / background_model.leaf_size_);
    index_t y_index =
      static_cast<index_t>((p.y - background_model.min_point_.y()) / background_model.leaf_size_);
    index_t z_index =
      static_cast<index_t>((p.z - background_model.min_point_.z()) / background_model.leaf_size_);
    index_t index = z_index * y_cells * x_cells + y_index * x_cells + x_index;

    if (background_model.set_.count(index) == 0) {
      voxel_filtered_pointcloud_ptr->emplace_back(p);
    }
  }
  RCLCPP_INFO(
    this->get_logger(), "\t voxel filtered points: %lu", voxel_filtered_pointcloud_ptr->size());

  // K-search
  pcl::PointCloud<PointType>::Ptr tree_filtered_pointcloud_ptr(new pcl::PointCloud<PointType>);
  tree_filtered_pointcloud_ptr->reserve(voxel_filtered_pointcloud_ptr->size());
  float min_foreground_square_distance =
    parameters_.min_foreground_distance * parameters_.min_foreground_distance;

  for (const auto & p : voxel_filtered_pointcloud_ptr->points) {
    std::vector<int> indexes;
    std::vector<float> square_distances;

    if (background_model.tree_.nearestKSearch(p, 1, indexes, square_distances) > 0) {
      if (square_distances.size() == 0 || square_distances[0] >= min_foreground_square_distance) {
        tree_filtered_pointcloud_ptr->emplace_back(p);
      }
    }
  }

  RCLCPP_INFO(
    this->get_logger(), "\t tree filtered points: %lu", tree_filtered_pointcloud_ptr->size());

  if (!use_ransac) {
    foreground_pointcloud_ptr = tree_filtered_pointcloud_ptr;
    return;
  }

  // Plane ransac (since the orientation changes slightly between data, this one does not use the
  // background model)
  pcl::ModelCoefficients::Ptr coefficients_ptr(new pcl::ModelCoefficients);
  pcl::PointCloud<PointType>::Ptr ransac_filtered_pointcloud_ptr(new pcl::PointCloud<PointType>);
  pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);
  pcl::SACSegmentation<PointType> seg;
  pcl::ExtractIndices<PointType> extract;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(parameters_.ransac_threshold);
  seg.setMaxIterations(parameters_.ransac_max_iterations);
  seg.setInputCloud(sensor_pointcloud_ptr);
  seg.segment(*inliers_ptr, *coefficients_ptr);

  ransac_filtered_pointcloud_ptr->reserve(tree_filtered_pointcloud_ptr->size());

  for (const auto & p : tree_filtered_pointcloud_ptr->points) {
    if (
      p.x * coefficients_ptr->values[0] + p.y * coefficients_ptr->values[1] +
        p.z * coefficients_ptr->values[2] + coefficients_ptr->values[3] >
      parameters_.ransac_threshold) {
      ransac_filtered_pointcloud_ptr->emplace_back(p);
    }
  }

  RCLCPP_INFO(
    this->get_logger(), "\t ransac filtered points: %lu", ransac_filtered_pointcloud_ptr->size());

  foreground_pointcloud_ptr = ransac_filtered_pointcloud_ptr;
  ground_model = Eigen::Vector4f(
    coefficients_ptr->values[0], coefficients_ptr->values[1], coefficients_ptr->values[2],
    coefficients_ptr->values[3]);
}

std::vector<pcl::PointCloud<ExtrinsicReflectorBasedCalibrator::PointType>::Ptr>
ExtrinsicReflectorBasedCalibrator::extractClusters(
  const pcl::PointCloud<PointType>::Ptr & foreground_pointcloud_ptr,
  const double cluster_max_tolerance, const int cluster_min_points, const int cluster_max_points)
{
  pcl::search::KdTree<PointType>::Ptr tree_ptr(new pcl::search::KdTree<PointType>);
  tree_ptr->setInputCloud(foreground_pointcloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> cluster_extractor;
  cluster_extractor.setClusterTolerance(cluster_max_tolerance);
  cluster_extractor.setMinClusterSize(cluster_min_points);
  cluster_extractor.setMaxClusterSize(cluster_max_points);
  cluster_extractor.setSearchMethod(tree_ptr);
  cluster_extractor.setInputCloud(foreground_pointcloud_ptr);
  cluster_extractor.extract(cluster_indices);

  RCLCPP_INFO(
    this->get_logger(), "Cluster extraction input size: %lu", foreground_pointcloud_ptr->size());

  std::vector<pcl::PointCloud<PointType>::Ptr> cluster_vector;

  for (const auto & cluster : cluster_indices) {
    pcl::PointCloud<PointType>::Ptr cluster_pointcloud_ptr(new pcl::PointCloud<PointType>);
    cluster_pointcloud_ptr->reserve(cluster.indices.size());

    for (const auto & idx : cluster.indices) {
      cluster_pointcloud_ptr->push_back((*foreground_pointcloud_ptr)[idx]);
    }

    cluster_pointcloud_ptr->width = cluster_pointcloud_ptr->size();
    cluster_pointcloud_ptr->height = 1;
    cluster_pointcloud_ptr->is_dense = true;
    RCLCPP_INFO(
      this->get_logger(), "\t found cluster of size: %lu", cluster_pointcloud_ptr->size());

    cluster_vector.push_back(cluster_pointcloud_ptr);
  }

  return cluster_vector;
}

std::vector<Eigen::Vector3d> ExtrinsicReflectorBasedCalibrator::findReflectorsFromClusters(
  const std::vector<pcl::PointCloud<PointType>::Ptr> & clusters,
  const Eigen::Vector4f & ground_model)
{
  std::vector<Eigen::Vector3d> reflector_centers;
  RCLCPP_INFO(this->get_logger(), "Extracting lidar reflectors from clusters");

  for (const auto & cluster_pointcloud_ptr : clusters) {
    float max_h = -std::numeric_limits<float>::max();
    PointType highest_point;

    for (const auto & p : cluster_pointcloud_ptr->points) {
      float height =
        p.x * ground_model.x() + p.y * ground_model.y() + p.z * ground_model.z() + ground_model.w();
      if (height > max_h) {
        max_h = height;
        highest_point = p;
      }
    }

    if (max_h > parameters_.reflector_max_height) {
      continue;
    }

    pcl::search::KdTree<PointType>::Ptr tree_ptr(new pcl::search::KdTree<PointType>);
    tree_ptr->setInputCloud(cluster_pointcloud_ptr);

    std::vector<int> indexes;
    std::vector<float> squared_distances;

    if (
      tree_ptr->radiusSearch(
        highest_point, parameters_.reflector_radius, indexes, squared_distances) > 0) {
      Eigen::Vector3d center = Eigen::Vector3d::Zero();

      for (const auto & index : indexes) {
        const auto & p = cluster_pointcloud_ptr->points[index];
        center += Eigen::Vector3d(p.x, p.y, p.z);
      }

      center /= indexes.size();
      RCLCPP_INFO(
        this->get_logger(), "\t Lidar reflector id=%lu size=%lu center: x=%.2f y=%.2f z=%.2f",
        reflector_centers.size(), indexes.size(), center.x(), center.y(), center.z());
      reflector_centers.push_back(center);
    }
  }

  return reflector_centers;
}

bool ExtrinsicReflectorBasedCalibrator::checkInitialTransforms()
{
  if (lidar_frame_ == "" || radar_frame_ == "") {
    return false;
  }

  if (got_initial_transform_) {
    return true;
  }

  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    initial_radar_to_lidar_msg_ =
      tf_buffer_->lookupTransform(radar_frame_, lidar_frame_, t, timeout).transform;

    fromMsg(initial_radar_to_lidar_msg_, initial_radar_to_lidar_tf2_);
    initial_radar_to_lidar_eigen_ = tf2::transformToEigen(initial_radar_to_lidar_msg_);
    calibrated_radar_to_lidar_eigen_ = initial_radar_to_lidar_eigen_;

    parent_to_lidar_msg_ =
      tf_buffer_->lookupTransform(parameters_.parent_frame, lidar_frame_, t, timeout).transform;

    fromMsg(parent_to_lidar_msg_, parent_to_lidar_tf2_);
    parent_to_lidar_eigen_ = tf2::transformToEigen(parent_to_lidar_msg_);

    got_initial_transform_ = true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "could not get initial tf. %s", ex.what());
    return false;
  }

  return true;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
ExtrinsicReflectorBasedCalibrator::matchDetections(
  const std::vector<Eigen::Vector3d> & lidar_detections,
  const std::vector<Eigen::Vector3d> & radar_detections)
{
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matched_detections;

  if (lidar_detections.size() == 0 || radar_detections.size() == 0 || !checkInitialTransforms()) {
    return matched_detections;
  }

  // Lidar transformed detections
  std::vector<Eigen::Vector3d> lidar_detections_transformed;
  const auto radar_to_lidar_transorm = initial_radar_to_lidar_eigen_;

  std::transform(
    lidar_detections.cbegin(), lidar_detections.cend(),
    std::back_inserter(lidar_detections_transformed),
    [&radar_to_lidar_transorm](const auto & lidar_detection) {
      auto transformed_point = radar_to_lidar_transorm * lidar_detection;
      transformed_point.z() = 0.f;
      return transformed_point;
    });

  std::vector<std::size_t> lidar_to_radar_closest_idx, radar_to_lidar_closest_idx;
  lidar_to_radar_closest_idx.resize(lidar_detections.size());
  radar_to_lidar_closest_idx.resize(radar_detections.size());

  RCLCPP_INFO(this->get_logger(), "Matching each lidar detections to its closest radar detection");

  for (std::size_t lidar_index = 0; lidar_index < lidar_detections.size(); lidar_index++) {
    float closest_distance = std::numeric_limits<float>::max();
    std::size_t closest_index = 0;

    for (std::size_t radar_index = 0; radar_index < radar_detections.size(); radar_index++) {
      float distance =
        (lidar_detections_transformed[lidar_index] - radar_detections[radar_index]).norm();

      if (distance < closest_distance) {
        closest_distance = distance;
        closest_index = radar_index;
      }
    }

    RCLCPP_INFO(
      this->get_logger(), "\tClosest radar to lidar=%lu is %lu with distance %f", lidar_index,
      closest_index, closest_distance);
    lidar_to_radar_closest_idx[lidar_index] = closest_index;
  }

  RCLCPP_INFO(this->get_logger(), "Matching each radar detections to its closest lidar detection");

  for (std::size_t radar_index = 0; radar_index < radar_detections.size(); radar_index++) {
    float closest_distance = std::numeric_limits<float>::max();
    std::size_t closest_index = 0;

    for (std::size_t lidar_index = 0; lidar_index < lidar_detections.size(); lidar_index++) {
      float distance =
        (lidar_detections_transformed[lidar_index] - radar_detections[radar_index]).norm();

      if (distance < closest_distance) {
        closest_distance = distance;
        closest_index = lidar_index;
      }
    }

    RCLCPP_INFO(
      this->get_logger(), "\tClosest lidar to radar=%lu is %lu with distance %f", radar_index,
      closest_index, closest_distance);

    radar_to_lidar_closest_idx[radar_index] = closest_index;
  }

  for (std::size_t lidar_index = 0; lidar_index < lidar_detections.size(); lidar_index++) {
    std::size_t closest_radar_index = lidar_to_radar_closest_idx[lidar_index];
    RCLCPP_INFO(
      this->get_logger(), "lidar_index = %lu / %lu", lidar_index, lidar_detections.size());
    RCLCPP_INFO(
      this->get_logger(), "closest_radar_index = %lu / %lu", closest_radar_index,
      radar_detections.size());
    float distance =
      (lidar_detections_transformed[lidar_index] - radar_detections[closest_radar_index]).norm();
    if (
      radar_to_lidar_closest_idx[closest_radar_index] == lidar_index &&
      distance < parameters_.max_matching_distance) {
      matched_detections.emplace_back(
        lidar_detections[lidar_index], radar_detections[closest_radar_index]);
    }
  }

  return matched_detections;
}

void ExtrinsicReflectorBasedCalibrator::trackMatches(
  const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> & matches,
  builtin_interfaces::msg::Time & current_time, bool & is_converged)
{
  std::unique_lock<std::mutex> lock(mutex_);

  if (!tracking_active_) {
    return;
  }

  // Check if active tracks expired
  auto timed_out_end = std::remove_if(
    active_tracks_.begin(), active_tracks_.end(),
    [&current_time](const auto & track) { return track.timedOut(current_time); });

  active_tracks_.erase(timed_out_end, active_tracks_.end());

  bool new_track = false;
  // Update tracks
  for (const auto & [lidar_detection, radar_detection] : matches) {
    if (std::any_of(
          converged_tracks_.begin(), converged_tracks_.end(),
          [&lidar_detection, &radar_detection](auto & track) {
            return track.partialMatch(lidar_detection, radar_detection);
          })) {
      continue;
    }

    new_track = std::all_of(
      active_tracks_.begin(), active_tracks_.end(),
      [&lidar_detection, &radar_detection](auto & track) {
        return !track.match(lidar_detection, radar_detection);
      });

    if (new_track) {
      active_tracks_.push_back(
        factory_ptr_->makeTrack(lidar_detection, radar_detection, current_time));
    } else {
      std::for_each(
        active_tracks_.begin(), active_tracks_.end(),
        [&lidar_detection, &radar_detection](auto & track) {
          track.updateIfMatch(lidar_detection, radar_detection);
        });
    }
  }

  // Move converged tracks
  std::copy_if(
    active_tracks_.begin(), active_tracks_.end(), std::back_inserter(converged_tracks_),
    [](auto & track) { return track.converged(); });

  current_new_tracks_ += std::transform_reduce(
    active_tracks_.begin(), active_tracks_.end(), 0, std::plus{},
    [](auto & track) { return track.converged(); });

  is_converged = std::any_of(active_tracks_.begin(), active_tracks_.end(),
    [](auto &track) { return track.converged(); });

  auto converged_end = std::remove_if(
    active_tracks_.begin(), active_tracks_.end(),
    [&current_time](auto & track) { return track.converged(); });

  active_tracks_.erase(converged_end, active_tracks_.end());

  if (current_new_tracks_ > 0 && active_tracks_.size() == 0) {
    tracking_active_ = false;
  }
}

void ExtrinsicReflectorBasedCalibrator::calibrateSensors()
{
  if (!checkInitialTransforms() || converged_tracks_.size() == 0) {
    if(converged_tracks_.size() == 0) {
      std_msgs::msg::Float32MultiArray cv_metrics_msg = std_msgs::msg::Float32MultiArray();
      cv_metrics_msg.data = {0, 0, 0, 0, 0}; 
      metrics_pub_->publish(cv_metrics_msg);
    }
    return;
  }

  // Define two sets of 2D points (just 3D points with z=0)
  pcl::PointCloud<PointType>::Ptr lidar_points_pcs(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr radar_points_rcs(new pcl::PointCloud<PointType>);
  lidar_points_pcs->reserve(converged_tracks_.size());
  radar_points_rcs->reserve(converged_tracks_.size());

  double delta_cos_sum = 0.0;
  double delta_sin_sum = 0.0;

  auto eigen_to_pcl_2d = [](const auto & p) { return PointType(p.x(), p.y(), 0.0); };

  for (std::size_t track_index = 0; track_index < converged_tracks_.size(); track_index++) {
    auto track = converged_tracks_[track_index];
    // lidar coordinates
    const auto & lidar_estimation = track.getLidarEstimation();
    // to parent coordinates
    const auto & lidar_estimation_pcs = parent_to_lidar_eigen_ * lidar_estimation;
    // to radar coordinates
    const auto & lidar_transformed_estimation = initial_radar_to_lidar_eigen_ * lidar_estimation;
    const auto & radar_estimation_rcs = track.getRadarEstimation();
    lidar_points_pcs->emplace_back(eigen_to_pcl_2d(lidar_estimation_pcs));
    radar_points_rcs->emplace_back(eigen_to_pcl_2d(radar_estimation_rcs));

    const double lidar_transformed_norm = lidar_transformed_estimation.norm();
    const double lidar_transformed_cos = lidar_transformed_estimation.x() / lidar_transformed_norm;
    const double lidar_transformed_sin = lidar_transformed_estimation.y() / lidar_transformed_norm;

    const double radar_norm = radar_estimation_rcs.norm();
    const double radar_cos = radar_estimation_rcs.x() / radar_norm;
    const double radar_sin = radar_estimation_rcs.y() / radar_norm;

    // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
    // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
    // a = lidar
    // b = radar
    double delta_angle_sin = lidar_transformed_sin * radar_cos - lidar_transformed_cos * radar_sin;
    double delta_angle_cos = lidar_transformed_cos * radar_cos + lidar_transformed_sin * radar_sin;
    delta_sin_sum += delta_angle_sin;
    delta_cos_sum += delta_angle_cos;

    // logging
    RCLCPP_INFO_STREAM(
    this->get_logger(), "lidar_estimation:\n"
                          << lidar_estimation.matrix());
    RCLCPP_INFO_STREAM(
    this->get_logger(), "lidar_transformed_estimation:\n"
                          << lidar_transformed_estimation.matrix());
    RCLCPP_INFO_STREAM(
    this->get_logger(), "radar_estimation_rcs:\n"
                          << radar_estimation_rcs.matrix());
  }


  // Estimate full transformation using SVD
  pcl::registration::TransformationEstimationSVD<PointType, PointType> estimator;
  Eigen::Matrix4f full_radar_to_parent_transformation;
  estimator.estimateRigidTransformation(
    *lidar_points_pcs, *radar_points_rcs, full_radar_to_parent_transformation);
  Eigen::Isometry3d calibrated_2d_radar_to_parent_transformation(
    full_radar_to_parent_transformation.cast<double>());

  calibrated_2d_radar_to_parent_transformation.translation().z() =
    (initial_radar_to_lidar_eigen_ * parent_to_lidar_eigen_.inverse()).translation().z();
  Eigen::Isometry3d calibrated_2d_radar_to_lidar_transformation =
    calibrated_2d_radar_to_parent_transformation * parent_to_lidar_eigen_;

  // Estimate the 2D transformation estimating only yaw
  double delta_cos = delta_cos_sum / converged_tracks_.size();
  double delta_sin = -delta_sin_sum / converged_tracks_.size();

  Eigen::Matrix3d delta_rotation;
  delta_rotation << delta_cos, -delta_sin, 0.0, delta_sin, delta_cos, 0.0, 0.0, 0.0, 1.0;
  Eigen::Isometry3d delta_transformation = Eigen::Isometry3d::Identity();
  delta_transformation.linear() = delta_rotation;
  Eigen::Isometry3d calibrated_rotation_radar_to_lidar_transformation =
    delta_transformation * initial_radar_to_lidar_eigen_;

  // Estimate the pre & post calibration error
  auto get_yaw_error = [](const Eigen::Vector3d & v1, const Eigen::Vector3d & v2) -> double {
    return std::abs(std::acos(v1.dot(v2) / (v1.norm() * v2.norm())));
  };

  auto compute_calibration_error =
    [&](const Eigen::Isometry3d & radar_to_lidar_isometry) -> std::pair<double, double> {
    double distance_error = 0.0;
    double yaw_error = 0.0;

    for (auto & track : converged_tracks_) {
      auto lidar_estimation = track.getLidarEstimation();
      auto radar_estimation = track.getRadarEstimation();
      auto lidar_estimation_transformed = radar_to_lidar_isometry * lidar_estimation;
      lidar_estimation_transformed.z() = 0.0;
      radar_estimation.z() = 0.0;

      distance_error += (lidar_estimation_transformed - radar_estimation).norm();
      yaw_error += get_yaw_error(lidar_estimation_transformed, radar_estimation);
    }

    distance_error /= static_cast<double>(converged_tracks_.size());
    yaw_error *= 180.0 / (M_PI * static_cast<double>(converged_tracks_.size()));

    return std::make_pair(distance_error, yaw_error);
  };

  auto [initial_distance_error, initial_yaw_error] =
    compute_calibration_error(initial_radar_to_lidar_eigen_);
  auto [calibrated_2d_distance_error, calibrated_2d_yaw_error] =
    compute_calibration_error(calibrated_2d_radar_to_lidar_transformation);
  auto [calibrated_rotation_distance_error, calibrated_rotation_yaw_error] =
    compute_calibration_error(calibrated_rotation_radar_to_lidar_transformation);

  RCLCPP_INFO_STREAM(
    this->get_logger(), "Initial radar->lidar transform:\n"
                          << initial_radar_to_lidar_eigen_.matrix());
  RCLCPP_INFO_STREAM(
    this->get_logger(), "2D calibration radar->lidar transform:\n"
                          << calibrated_2d_radar_to_lidar_transformation.matrix());
  RCLCPP_INFO_STREAM(
    this->get_logger(), "Pure rotation calibration radar->lidar transform:\n"
                          << calibrated_rotation_radar_to_lidar_transformation.matrix());

  // Evaluate the cross validation metric using SVD
  double cv_distance_error = 0.0;
  double cv_yaw_error = 0.0;
  if(converged_tracks_.size() > 1) {
    std::vector<int> indices_vec(converged_tracks_.size());
    std::iota(indices_vec.begin(), indices_vec.end(), 0);
    for (std::size_t rotate = 0; rotate < indices_vec.size(); ++rotate) {
      // Rotate the vector so that it starts from the current index
      std::rotate(indices_vec.begin(), indices_vec.begin() + 1, indices_vec.end());

      pcl::PointCloud<PointType>::Ptr cv_lidar_points_pcs(new pcl::PointCloud<PointType>);
      pcl::PointCloud<PointType>::Ptr cv_radar_points_rcs(new pcl::PointCloud<PointType>);
      cv_lidar_points_pcs->reserve(converged_tracks_.size()-1);
      cv_radar_points_rcs->reserve(converged_tracks_.size()-1);

      for (std::size_t i = 0; i < converged_tracks_.size()-1; i++) {
        cv_lidar_points_pcs->emplace_back(lidar_points_pcs->points[indices_vec[i]]);
        cv_radar_points_rcs->emplace_back(radar_points_rcs->points[indices_vec[i]]);
      }
      // random select n-1 points to compute new calibration matrix and 1 to evaluate. 
      // Estimate parital transformation using SVD
      pcl::registration::TransformationEstimationSVD<PointType, PointType> cv_estimator;
      Eigen::Matrix4f cv_radar_to_parent_transformation;
      cv_estimator.estimateRigidTransformation(
        *cv_lidar_points_pcs, *cv_radar_points_rcs, cv_radar_to_parent_transformation);
      Eigen::Isometry3d cv_calibrated_2d_radar_to_parent_transformation(
        cv_radar_to_parent_transformation.cast<double>());
      cv_calibrated_2d_radar_to_parent_transformation.translation().z() =
        (initial_radar_to_lidar_eigen_ * parent_to_lidar_eigen_.inverse()).translation().z();
      Eigen::Isometry3d cv_calibrated_2d_radar_to_lidar_transformation =
        cv_calibrated_2d_radar_to_parent_transformation * parent_to_lidar_eigen_;
      // Estimate the pre & post calibration error

      auto compute_calibration_error_with_one_validation =
        [&](const Eigen::Isometry3d & radar_to_lidar_isometry) -> void {
        auto track = converged_tracks_[indices_vec.back()];
        auto lidar_estimation = track.getLidarEstimation();
        auto radar_estimation = track.getRadarEstimation();
        auto lidar_estimation_transformed = radar_to_lidar_isometry * lidar_estimation;
        lidar_estimation_transformed.z() = 0.0;
        radar_estimation.z() = 0.0;

        cv_distance_error += (lidar_estimation_transformed - radar_estimation).norm();
        cv_yaw_error += get_yaw_error(lidar_estimation_transformed, radar_estimation);
      };
      compute_calibration_error_with_one_validation(cv_calibrated_2d_radar_to_lidar_transformation);
    }
    cv_distance_error /= static_cast<double>(converged_tracks_.size());
    cv_yaw_error *= 180.0 / (M_PI * static_cast<double>(converged_tracks_.size()));
  }

  RCLCPP_INFO(
    this->get_logger(),
    "track size=%f, cross validation calibration error: detection2detection.distance=%.4fm, yaw=%.4f degrees",
    static_cast<float>(converged_tracks_.size()), cv_distance_error, cv_yaw_error);
  output_cv_distance_error = static_cast<float>(cv_distance_error);
  output_cv_yaw_error = static_cast<float>(cv_yaw_error);
  output_calibration_distance_error = static_cast<float>(calibrated_2d_distance_error);
  output_calibration_yaw_error = static_cast<float>(calibrated_2d_yaw_error);

  
  // publish metrics
  std_msgs::msg::Float32MultiArray cv_metrics_msg = std_msgs::msg::Float32MultiArray();
  cv_metrics_msg.data = {static_cast<float>(converged_tracks_.size()), output_cv_distance_error,
                          output_cv_yaw_error, output_calibration_distance_error, output_calibration_yaw_error}; 
  metrics_pub_->publish(cv_metrics_msg);


  // Evaluate the different calibrations and decide on an output
  auto compute_transformation_difference =
    [](const Eigen::Isometry3d & t1, const Eigen::Isometry3d & t2) -> std::pair<double, double> {
    double translation_difference = (t2.inverse() * t1).translation().norm();
    double rotation_difference =
      std::acos(std::min(1.0, 0.5 * ((t2.rotation().inverse() * t1.rotation()).trace() - 1.0)));

    return std::make_pair(translation_difference, rotation_difference);
  };
  RCLCPP_INFO(
    this->get_logger(),
    "Initial calibration error: detection2detection.distance=%.4fm yaw=%.4f degrees",
    initial_distance_error, initial_yaw_error);
  RCLCPP_INFO(
    this->get_logger(),
    "Final calibration error: detection2detection.distance=%.4fm yaw=%.4f degrees",
    calibrated_2d_distance_error, calibrated_2d_yaw_error);
  RCLCPP_INFO(
    this->get_logger(),
    "Final calibration error (rotation only): detection2detection.distance=%.4fm yaw=%.4f degrees",
    calibrated_rotation_distance_error, calibrated_rotation_yaw_error);

  auto [calibrated_2d_translation_difference, calibrated_2d_rotation_difference] =
    compute_transformation_difference(
      initial_radar_to_lidar_eigen_, calibrated_2d_radar_to_lidar_transformation);
  auto [calibrated_rotation_translation_difference, calibrated_rotation_rotation_difference] =
    compute_transformation_difference(
      initial_radar_to_lidar_eigen_, calibrated_rotation_radar_to_lidar_transformation);

  std::unique_lock<std::mutex> lock(mutex_);
  if (
    calibrated_2d_translation_difference < parameters_.max_initial_calibration_translation_error &&
    calibrated_2d_rotation_difference < parameters_.max_initial_calibration_rotation_error) {
    RCLCPP_INFO(
      this->get_logger(), "The 2D calibration pose was chosen as the output calibration pose");
    calibrated_radar_to_lidar_eigen_ = calibrated_2d_radar_to_lidar_transformation;
    calibration_valid_ = true;
  } else if (
    calibrated_rotation_translation_difference <
      parameters_.max_initial_calibration_translation_error &&
    calibrated_rotation_rotation_difference < parameters_.max_initial_calibration_rotation_error) {
    RCLCPP_WARN(
      this->get_logger(),
      "The pure rotation calibration pose was chosen as the output calibration pose. This may mean "
      "you need to collect more points");
    calibrated_radar_to_lidar_eigen_ = calibrated_rotation_radar_to_lidar_transformation;
    calibration_valid_ = true;
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "The calibrated poses differ considerably with the initial calibration. This may be either a "
      "fault of the algorithm or a bad calibration initialization");
  }
}

void ExtrinsicReflectorBasedCalibrator::visualizationMarkers(
  const std::vector<Eigen::Vector3d> & lidar_detections,
  const std::vector<Eigen::Vector3d> & radar_detections,
  const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> & matched_detections)
{
  auto eigen_to_point_msg = [](const Eigen::Vector3d & p_eigen) {
    geometry_msgs::msg::Point p;
    p.x = p_eigen.x();
    p.y = p_eigen.y();
    p.z = p_eigen.z();
    return p;
  };

  visualization_msgs::msg::MarkerArray lidar_detections_marker_array;

  for (std::size_t detection_index = 0; detection_index < lidar_detections.size();
       detection_index++) {
    const auto & detection_center = lidar_detections[detection_index];
    visualization_msgs::msg::Marker marker;
    marker.header = lidar_header_;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.id = detection_index;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = detection_center.x();
    marker.pose.position.y = detection_center.y();
    marker.pose.position.z = detection_center.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = parameters_.reflector_radius;
    marker.scale.y = parameters_.reflector_radius;
    marker.scale.z = parameters_.reflector_radius;
    marker.color.a = 0.6;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    lidar_detections_marker_array.markers.push_back(marker);
  }

  lidar_detections_pub_->publish(lidar_detections_marker_array);

  visualization_msgs::msg::MarkerArray radar_detections_marker_array;

  for (std::size_t detection_index = 0; detection_index < radar_detections.size();
       detection_index++) {
    const auto & detection_center = radar_detections[detection_index];
    visualization_msgs::msg::Marker marker;
    marker.header = radar_header_;
    marker.id = detection_index;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.ns = "center";
    marker.pose.position.x = detection_center.x();
    marker.pose.position.y = detection_center.y();
    marker.pose.position.z = detection_center.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = parameters_.reflector_radius;
    marker.scale.y = parameters_.reflector_radius;
    marker.scale.z = parameters_.reflector_radius;
    marker.color.a = 0.6;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    radar_detections_marker_array.markers.push_back(marker);

    geometry_msgs::msg::Point p1, p2;
    p1.z -= 0.5;
    p2.z += 0.5;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.ns = "line";
    marker.scale.x = 0.2 * parameters_.reflector_radius;
    marker.scale.y = 0.2 * parameters_.reflector_radius;
    marker.scale.z = 0.2 * parameters_.reflector_radius;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    radar_detections_marker_array.markers.push_back(marker);
  }

  radar_detections_pub_->publish(radar_detections_marker_array);

  visualization_msgs::msg::MarkerArray matches_marker_array;

  for (std::size_t match_index = 0; match_index < matched_detections.size(); match_index++) {
    const auto & [lidar_detection, radar_detection] = matched_detections[match_index];
    const auto lidar_detection_transformed = initial_radar_to_lidar_eigen_ * lidar_detection;

    visualization_msgs::msg::Marker marker;
    marker.header = radar_header_;
    marker.id = match_index;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.ns = "match";
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 0.6;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.points.push_back(eigen_to_point_msg(lidar_detection_transformed));
    marker.points.push_back(eigen_to_point_msg(radar_detection));
    matches_marker_array.markers.push_back(marker);
  }

  matches_markers_pub_->publish(matches_marker_array);
}


void ExtrinsicReflectorBasedCalibrator::visualizeTrackMarkers() 
{
  auto eigen_to_point_msg = [](const Eigen::Vector3d & p_eigen) {
    geometry_msgs::msg::Point p;
    p.x = p_eigen.x();
    p.y = p_eigen.y();
    p.z = p_eigen.z();
    return p;
  };
  auto add_track_markers = [&](
                             const Eigen::Vector3d & lidar_estimation,
                             const Eigen::Vector3d & radar_estimation_transformed,
                             const std::string ns, const std_msgs::msg::ColorRGBA & color,
                             std::vector<visualization_msgs::msg::Marker> & markers) {
    visualization_msgs::msg::Marker marker;

    marker.header = lidar_header_;
    marker.id = markers.size();
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = ns;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2 * parameters_.reflector_radius;
    marker.scale.y = 0.2 * parameters_.reflector_radius;
    marker.scale.z = 0.2 * parameters_.reflector_radius;
    marker.color = color;
    marker.points.push_back(eigen_to_point_msg(radar_estimation_transformed));
    marker.points.push_back(eigen_to_point_msg(lidar_estimation));
    markers.push_back(marker);

    marker.id = markers.size();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position = eigen_to_point_msg(radar_estimation_transformed);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = parameters_.reflector_radius;
    marker.scale.y = parameters_.reflector_radius;
    marker.scale.z = parameters_.reflector_radius;
    marker.points.clear();
    markers.push_back(marker);

    marker.id = markers.size();
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.2 * parameters_.reflector_radius;
    marker.scale.y = 0.2 * parameters_.reflector_radius;
    marker.scale.z = 0.2 * parameters_.reflector_radius;
    marker.points.push_back(eigen_to_point_msg(Eigen::Vector3d(0.0, 0.0, -0.5)));
    marker.points.push_back(eigen_to_point_msg(Eigen::Vector3d(0.0, 0.0, 0.5)));
    markers.push_back(marker);

    marker.id = markers.size();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position = eigen_to_point_msg(lidar_estimation);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = parameters_.reflector_radius;
    marker.scale.y = parameters_.reflector_radius;
    marker.scale.z = parameters_.reflector_radius;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.points.clear();
    markers.push_back(marker);
  };

  // Visualization
  visualization_msgs::msg::MarkerArray tracking_marker_array;
  std_msgs::msg::ColorRGBA initial_color;
  initial_color.r = 1.0;
  initial_color.a = 1.0;

  std_msgs::msg::ColorRGBA calibrated_color;
  calibrated_color.g = 1.0;
  calibrated_color.a = 1.0;

  for (std::size_t track_index = 0; track_index < converged_tracks_.size(); track_index++) {
    auto & track = converged_tracks_[track_index];
    const auto & lidar_estimation = track.getLidarEstimation();
    const auto & radar_estimation = track.getRadarEstimation();

    const auto initial_radar_estimation_transformed =
      initial_radar_to_lidar_eigen_.inverse() * radar_estimation;
    const auto calibrated_radar_estimation_transformed =
      calibrated_radar_to_lidar_eigen_.inverse() * radar_estimation;

    add_track_markers(
      lidar_estimation, initial_radar_estimation_transformed, "initial", initial_color,
      tracking_marker_array.markers);
    add_track_markers(
      lidar_estimation, calibrated_radar_estimation_transformed, "calibrated", calibrated_color,
      tracking_marker_array.markers);
  }

  tracking_markers_pub_->publish(tracking_marker_array);
}


void ExtrinsicReflectorBasedCalibrator::drawCalibrationStatusText()
{
  auto to_string_with_precision = [](const double value, const int n = 3) -> std::string {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << value;
    return out.str();
  };
  
  visualization_msgs::msg::Marker text_marker;

  text_marker.id = 0;
  text_marker.header = lidar_header_;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.ns = "calibration_status";
  text_marker.scale.z = 0.3;

  text_marker.text =
    "pairs=" + std::to_string(converged_tracks_.size()) +
    "\ncrossval_distance_error(m)=" + to_string_with_precision(output_cv_distance_error) +
    "\ncrossval_yaw_error(deg)=" + to_string_with_precision(output_cv_yaw_error) +
    "\n2d_distance_error(m)=" + to_string_with_precision(output_calibration_distance_error) +
    "\n2d_yaw_error(deg)=" + to_string_with_precision(output_calibration_yaw_error);

  text_marker.pose.position.x = 1;
  text_marker.pose.position.y = 1;
  text_marker.pose.position.z = 1;
  text_marker.pose.orientation.x = 0.0;
  text_marker.pose.orientation.y = 0.0;
  text_marker.pose.orientation.z = 0.0;
  text_marker.pose.orientation.w = 1.0;  

  text_markers_pub_->publish(text_marker);
}