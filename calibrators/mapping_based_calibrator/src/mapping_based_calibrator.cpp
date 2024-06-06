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

#include <mapping_based_calibrator/mapping_based_calibrator.hpp>
#include <mapping_based_calibrator/serialization.hpp>
#include <mapping_based_calibrator/utils.hpp>
#include <rosbag2_interfaces/srv/pause.hpp>
#include <rosbag2_interfaces/srv/resume.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tier4_calibration_pcl_extensions/voxel_grid_triplets.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <pcl/io/pcd_io.h>

#include <chrono>
#include <fstream>
#include <iostream>

#define UPDATE_PARAM(PARAM_STRUCT, NAME) update_param(p, #NAME, PARAM_STRUCT.NAME##_)

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
      rclcpp::get_logger("mapping_based_calibrator"),
      "Setting parameter [" << name << "] to " << value);
  }
}
}  // namespace

ExtrinsicMappingBasedCalibrator::ExtrinsicMappingBasedCalibrator(
  const rclcpp::NodeOptions & options)
: Node("mapping_based_calibrator_node", options),
  tf_broadcaster_(this),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  transform_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
  mapping_parameters_(std::make_shared<MappingParameters>()),
  calibration_parameters_(std::make_shared<CalibrationParameters>()),
  mapping_data_(std::make_shared<MappingData>())
{
  using std::chrono_literals::operator""s;

  calibration_parameters_->calibrate_base_frame_ =
    this->declare_parameter<bool>("calibrate_base_frame");
  calibration_parameters_->base_frame_ = this->declare_parameter<std::string>("base_frame");
  mapping_data_->map_frame_ = this->declare_parameter<std::string>("map_frame");

  mapping_data_->calibration_camera_optical_link_frame_names =
    this->declare_parameter<std::vector<std::string>>("calibration_camera_optical_link_frames");

  mapping_data_->calibration_lidar_frame_names_ =
    this->declare_parameter<std::vector<std::string>>("calibration_lidar_frames");

  std::vector<std::string> calibration_camera_info_topics =
    this->declare_parameter<std::vector<std::string>>("calibration_camera_info_topics");

  std::vector<std::string> calibration_image_topics =
    this->declare_parameter<std::vector<std::string>>("calibration_image_topics");

  std::vector<std::string> calibration_pointcloud_topics =
    this->declare_parameter<std::vector<std::string>>("calibration_pointcloud_topics");

  auto remove_empty_strings = [](std::vector<std::string> & v1) {
    std::vector<std::string> v2;
    std::copy_if(v1.begin(), v1.end(), std::back_inserter(v2), [](const std::string & s) {
      return s.size() > 0;
    });

    v1 = v2;
  };

  remove_empty_strings(mapping_data_->calibration_camera_optical_link_frame_names);
  remove_empty_strings(mapping_data_->calibration_lidar_frame_names_);
  remove_empty_strings(calibration_camera_info_topics);
  remove_empty_strings(calibration_image_topics);
  remove_empty_strings(calibration_pointcloud_topics);

  mapping_data_->mapping_lidar_frame_ = this->declare_parameter<std::string>("mapping_lidar_frame");

  mapping_parameters_->registrator_name_ =
    this->declare_parameter<std::string>("mapping_registrator");
  mapping_parameters_->mapping_verbose_ = this->declare_parameter<bool>("mapping_verbose", false);
  calibration_parameters_->calibration_verbose_ =
    this->declare_parameter<bool>("calibration_verbose", false);
  mapping_parameters_->use_rosbag_ = this->declare_parameter<bool>("use_rosbag", true);
  mapping_parameters_->mapping_max_frames_ =
    this->declare_parameter<int>("mapping_max_frames", 500);
  mapping_parameters_->local_map_num_keyframes_ =
    this->declare_parameter<int>("local_map_num_keyframes", 15);
  calibration_parameters_->dense_pointcloud_num_keyframes_ =
    this->declare_parameter<int>("dense_pointcloud_num_keyframes", 10);
  mapping_parameters_->mapping_min_range_ =
    this->declare_parameter<double>("mapping_min_range", 0.5);
  mapping_parameters_->mapping_max_range_ =
    this->declare_parameter<double>("mapping_max_range", 60.0);
  mapping_parameters_->min_mapping_pointcloud_size_ =
    this->declare_parameter<int>("min_mapping_pointcloud_size", 10000);
  mapping_parameters_->min_calibration_pointcloud_size_ =
    this->declare_parameter<int>("min_calibration_pointcloud_size", 500);
  mapping_parameters_->mapping_lost_timeout_ =
    this->declare_parameter<double>("mapping_lost_timeout", 1.0);

  // Mapping parameters
  mapping_parameters_->mapper_resolution_ =
    this->declare_parameter<double>("mapper_resolution", 5.0);
  mapping_parameters_->mapper_step_size_ = this->declare_parameter<double>("mapper_step_size", 0.1);
  mapping_parameters_->mapper_max_iterations_ =
    this->declare_parameter<int>("mapper_max_iterations", 35);
  mapping_parameters_->mapper_epsilon_ = this->declare_parameter<double>("mapper_epsilon", 0.01);
  mapping_parameters_->mapper_num_threads_ = this->declare_parameter<int>("mapper_num_threads", 8);
  mapping_parameters_->mapper_max_correspondence_distance_ =
    this->declare_parameter<double>("mapper_max_correspondence_distance", 0.1);

  mapping_parameters_->mapping_viz_leaf_size_ =
    this->declare_parameter<double>("mapping_viz_leaf_size", 0.15);
  calibration_parameters_->calibration_viz_leaf_size_ =
    this->declare_parameter<double>("calibration_viz_leaf_size", 0.15);
  mapping_parameters_->leaf_size_input_ = this->declare_parameter<double>("leaf_size_input", 0.1);
  mapping_parameters_->leaf_size_local_map_ =
    this->declare_parameter<double>("leaf_size_local_map", 0.1);
  calibration_parameters_->leaf_size_dense_map_ =
    this->declare_parameter<double>("leaf_size_dense_map", 0.05);
  mapping_parameters_->new_keyframe_min_distance_ =
    this->declare_parameter<double>("new_keyframe_min_distance", 1.0);
  mapping_parameters_->new_frame_min_distance_ =
    this->declare_parameter<double>("new_frame_min_distance", 0.05);
  mapping_parameters_->frame_stopped_distance_ =
    this->declare_parameter<double>("frame_stopped_distance", 0.02);
  mapping_parameters_->frames_since_stop_force_frame_ =
    this->declare_parameter<int>("frames_since_stoped_force_frame", 5);
  mapping_parameters_->calibration_skip_keyframes_ =
    this->declare_parameter<int>("calibration_skip_keyframes", 5);

  // Calibration frames selection criteria and preprocessing parameters
  calibration_parameters_->max_allowed_interpolated_time_ =
    this->declare_parameter<double>("max_allowed_interpolated_time", 0.05);
  calibration_parameters_->max_allowed_interpolated_distance_ =
    this->declare_parameter<double>("max_allowed_interpolated_distance", 0.05);
  calibration_parameters_->max_allowed_interpolated_angle_ =
    this->declare_parameter<double>("max_allowed_interpolated_angle", 1.0);
  calibration_parameters_->max_allowed_interpolated_speed_ =
    this->declare_parameter<double>("max_allowed_interpolated_speed", 3.0);
  calibration_parameters_->max_allowed_interpolated_accel_ =
    this->declare_parameter<double>("max_allowed_interpolated_accel", 0.4);

  calibration_parameters_->max_allowed_interpolated_distance_straight_ =
    this->declare_parameter<double>("max_allowed_interpolated_distance_straight", 0.08);
  calibration_parameters_->max_allowed_interpolated_angle_straight_ =
    this->declare_parameter<double>("max_allowed_interpolated_angle_straight", 0.5);  // deg
  calibration_parameters_->max_allowed_interpolated_speed_straight_ =
    this->declare_parameter<double>("max_allowed_interpolated_speed_straight", 5.0);
  calibration_parameters_->max_allowed_interpolated_accel_straight_ =
    this->declare_parameter<double>("max_allowed_interpolated_accel_straight", 0.1);

  calibration_parameters_->filter_detections_ =
    this->declare_parameter<bool>("filter_detections", true);
  calibration_parameters_->detection_max_time_tolerance_ =
    this->declare_parameter<double>("detection_max_time_tolerance", 0.5);
  calibration_parameters_->detection_size_tolerance_ =
    this->declare_parameter<double>("detection_size_tolerance", 0.4);

  mapping_parameters_->lost_frame_max_angle_diff_ =
    this->declare_parameter<double>("lost_frame_max_angle_diff", 25.0);  // def
  mapping_parameters_->lost_frame_interpolation_error_ =
    this->declare_parameter<double>("lost_frame_interpolation_error", 0.05);
  mapping_parameters_->lost_frame_max_acceleration_ =
    this->declare_parameter<double>("lost_frame_max_acceleration", 8.0);
  mapping_parameters_->viz_max_range_ = this->declare_parameter<double>("viz_max_range", 80.0);
  mapping_parameters_->crop_z_calibration_pointclouds_ =
    this->declare_parameter<bool>("crop_z_calibration_pointclouds", true);
  mapping_parameters_->crop_z_calibration_pointclouds_value_ =
    this->declare_parameter<double>("crop_z_calibration_pointclouds_value", 2.0);

  calibration_parameters_->calibration_use_only_stopped_ =
    this->declare_parameter<bool>("calibration_use_only_stopped", false);
  calibration_parameters_->calibration_use_only_last_frames_ =
    this->declare_parameter<bool>("calibration_use_only_last_frames", false);
  calibration_parameters_->max_calibration_range_ =
    this->declare_parameter<double>("max_calibration_range", 80.0);
  calibration_parameters_->min_calibration_range_ =
    this->declare_parameter<double>("min_calibration_range", 1.5);
  calibration_parameters_->calibration_min_pca_eigenvalue_ =
    this->declare_parameter<double>("calibration_min_pca_eigenvalue", 0.25);
  calibration_parameters_->calibration_min_distance_between_frames_ =
    this->declare_parameter<double>("calibration_min_distance_between_frames", 5.0);
  calibration_parameters_->calibration_eval_max_corr_distance_ =
    this->declare_parameter<double>("calibration_eval_max_corr_distance", 0.1);

  // Calibration parameters
  calibration_parameters_->solver_iterations_ =
    this->declare_parameter<int>("solver_iterations", 200);
  calibration_parameters_->max_corr_dist_coarse_ =
    this->declare_parameter<double>("max_corr_dist_coarse", 0.5);
  calibration_parameters_->max_corr_dist_fine_ =
    this->declare_parameter<double>("max_corr_dist_fine", 0.1);
  calibration_parameters_->max_corr_dist_ultra_fine_ =
    this->declare_parameter<double>("max_corr_dist_ultra_fine", 0.05);

  // Lidar calibration-only parameters
  calibration_parameters_->lidar_calibration_min_frames_ =
    this->declare_parameter<int>("lidar_calibration_min_frames", 2);
  calibration_parameters_->lidar_calibration_max_frames_ =
    this->declare_parameter<int>("lidar_calibration_max_frames", 10);

  // Camera calibration-only parameters
  calibration_parameters_->camera_calibration_min_frames_ =
    this->declare_parameter<int>("camera_calibration_min_frames", 1);
  calibration_parameters_->camera_calibration_max_frames_ =
    this->declare_parameter<int>("camera_calibration_max_frames", 10);
  calibration_parameters_->pc_features_min_distance_ =
    this->declare_parameter<double>("pc_features_min_distance", 0.2);
  calibration_parameters_->pc_features_max_distance_ =
    this->declare_parameter<double>("pc_features_max_distance", 40.0);

  // Base lidar calibration parameters
  calibration_parameters_->base_lidar_crop_box_min_x_ =
    this->declare_parameter<double>("base_lidar_crop_box_min_x", -20.0);
  calibration_parameters_->base_lidar_crop_box_min_y_ =
    this->declare_parameter<double>("base_lidar_crop_box_min_y", -20.0);
  calibration_parameters_->base_lidar_crop_box_min_z_ =
    this->declare_parameter<double>("base_lidar_crop_box_min_z", -20.0);
  calibration_parameters_->base_lidar_crop_box_max_x_ =
    this->declare_parameter<double>("base_lidar_crop_box_max_x", 20.0);
  calibration_parameters_->base_lidar_crop_box_max_y_ =
    this->declare_parameter<double>("base_lidar_crop_box_max_y", 20.0);
  calibration_parameters_->base_lidar_crop_box_max_z_ =
    this->declare_parameter<double>("base_lidar_crop_box_max_z", 20.0);
  calibration_parameters_->base_lidar_max_inlier_distance_ =
    this->declare_parameter<double>("base_lidar_max_inlier_distance", 0.01);
  calibration_parameters_->base_lidar_max_iterations_ =
    this->declare_parameter<int>("base_lidar_max_iterations", 1000);
  calibration_parameters_->base_lidar_min_plane_points_ =
    this->declare_parameter<int>("base_lidar_min_plane_points", 1000);
  calibration_parameters_->base_lidar_min_plane_points_percentage_ =
    this->declare_parameter<double>("base_lidar_min_plane_points_percentage", 10.0);
  calibration_parameters_->base_lidar_max_cos_distance_ =
    this->declare_parameter<double>("base_lidar_max_cos_distance", 0.5);
  calibration_parameters_->base_lidar_overwrite_xy_yaw_ =
    this->declare_parameter<bool>("base_lidar_overwrite_xy_yaw", false);

  auto map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_map", 10);

  auto frame_path_pub = this->create_publisher<nav_msgs::msg::Path>("frame_path", 10);
  auto frame_predicted_path_pub =
    this->create_publisher<nav_msgs::msg::Path>("frame_predicted_path", 10);
  auto keyframe_path_pub = this->create_publisher<nav_msgs::msg::Path>("keyframe_path", 10);
  auto keyframe_markers_pub =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("keyframe_markers", 10);

  auto rosbag2_pause_client_ = this->create_client<rosbag2_interfaces::srv::Pause>(
    "/rosbag2_player/pause", rmw_qos_profile_services_default);
  auto rosbag2_resume_client_ = this->create_client<rosbag2_interfaces::srv::Resume>(
    "/rosbag2_player/resume", rmw_qos_profile_services_default);

  // Set up mapper
  mapper_ = std::make_shared<CalibrationMapper>(
    mapping_parameters_, mapping_data_, map_pub, frame_path_pub, frame_predicted_path_pub,
    keyframe_path_pub, keyframe_markers_pub, rosbag2_pause_client_, rosbag2_resume_client_,
    tf_buffer_);

  // Set up camera calibrators
  for (std::size_t camera_id = 0;
       camera_id < mapping_data_->calibration_camera_optical_link_frame_names.size(); camera_id++) {
    const auto & frame_name = mapping_data_->calibration_camera_optical_link_frame_names[camera_id];
    const std::string calibration_lidar_namespace =
      "calibration_lidar_" + std::to_string(camera_id);

    auto target_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      calibration_lidar_namespace + "/target_map", 10);
    auto target_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      frame_name + "/target_markers", 10);

    camera_calibrators_[frame_name] = std::make_shared<CameraCalibrator>(
      frame_name, calibration_parameters_, mapping_data_, tf_buffer_, target_map_pub,
      target_markers_pub);
  }

  // Set up lidar calibrators
  for (std::size_t lidar_id = 0; lidar_id < mapping_data_->calibration_lidar_frame_names_.size();
       lidar_id++) {
    const auto & frame_name = mapping_data_->calibration_lidar_frame_names_[lidar_id];
    const std::string calibration_lidar_namespace = "calibration_lidar_" + std::to_string(lidar_id);
    auto initial_source_aligned_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      calibration_lidar_namespace + "/initial_source_aligned_map", 10);
    auto calibrated_source_aligned_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      calibration_lidar_namespace + "/calibrated_source_aligned_map", 10);
    auto target_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      calibration_lidar_namespace + "/target_map", 10);

    lidar_calibrators_[frame_name] = std::make_shared<LidarCalibrator>(
      frame_name, calibration_parameters_, mapping_data_, tf_buffer_,
      initial_source_aligned_map_pub, calibrated_source_aligned_map_pub, target_map_pub);
  }

  // Set up base calibrators
  auto base_lidar_augmented_pointcloud_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("base_lidar_augmented_pointcloud", 10);
  auto base_lidar_augmented_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_pointcloud", 10);

  base_lidar_calibrator_ = std::make_shared<BaseLidarCalibrator>(
    calibration_parameters_, mapping_data_, tf_buffer_, tf_broadcaster_,
    base_lidar_augmented_pointcloud_pub, base_lidar_augmented_pub);

  srv_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  service_server_ = this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
    "extrinsic_calibration",
    std::bind(
      &ExtrinsicMappingBasedCalibrator::requestReceivedCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, srv_callback_group_);

  // Set up sensor callbacks
  assert(
    mapping_data_->calibration_camera_optical_link_frame_names.size() ==
    calibration_image_topics.size());
  assert(calibration_camera_info_topics.size() == calibration_image_topics.size());
  for (std::size_t i = 0; i < mapping_data_->calibration_camera_optical_link_frame_names.size();
       i++) {
    const std::string & calibration_camera_info_topic = calibration_camera_info_topics[i];
    const std::string & calibration_image_topic = calibration_image_topics[i];
    const std::string & calibration_frame_name =
      mapping_data_->calibration_camera_optical_link_frame_names[i];

    calibration_camera_info_subs_[calibration_frame_name] =
      this->create_subscription<sensor_msgs::msg::CameraInfo>(
        calibration_camera_info_topic, rclcpp::SensorDataQoS().keep_all(),
        [&](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          mapper_->calibrationCameraInfoCallback(msg, calibration_frame_name);
        });

    calibration_image_subs_[calibration_frame_name] =
      this->create_subscription<sensor_msgs::msg::CompressedImage>(
        calibration_image_topic, rclcpp::SensorDataQoS().keep_all(),
        [&](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
          mapper_->calibrationImageCallback(msg, calibration_frame_name);
        });
  }

  assert(
    mapping_data_->calibration_lidar_frame_names_.size() == calibration_pointcloud_topics.size());
  for (std::size_t i = 0; i < mapping_data_->calibration_lidar_frame_names_.size(); i++) {
    const std::string & calibration_pointcloud_topic = calibration_pointcloud_topics[i];
    const std::string & calibration_frame_name = mapping_data_->calibration_lidar_frame_names_[i];

    calibration_pointcloud_subs_[calibration_frame_name] =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        calibration_pointcloud_topic, rclcpp::SensorDataQoS().keep_all(),
        [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          mapper_->calibrationPointCloudCallback(msg, calibration_frame_name);
        });
  }

  mapping_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "mapping_pointcloud", rclcpp::SensorDataQoS().keep_all(),
    std::bind(&CalibrationMapper::mappingPointCloudCallback, mapper_, std::placeholders::_1));

  detected_objects_sub_ = this->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "detected_objects", rclcpp::SensorDataQoS().keep_all(),
    std::bind(
      &ExtrinsicMappingBasedCalibrator::detectedObjectsCallback, this, std::placeholders::_1));

  predicted_objects_sub_ =
    this->create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
      "predicted_objects", rclcpp::SensorDataQoS().keep_all(),
      std::bind(
        &ExtrinsicMappingBasedCalibrator::predictedObjectsCallback, this, std::placeholders::_1));

  stop_mapping_server_ = this->create_service<std_srvs::srv::Empty>(
    "stop_mapping",
    [&](
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response> response) {
      mapper_->stop();
      RCLCPP_INFO_STREAM(this->get_logger(), "Mapper stopped through service");
    },
    rmw_qos_profile_services_default);

  load_database_server_ = this->create_service<tier4_calibration_msgs::srv::CalibrationDatabase>(
    "load_database",
    std::bind(
      &ExtrinsicMappingBasedCalibrator::loadDatabaseCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default);

  save_database_server_ = this->create_service<tier4_calibration_msgs::srv::CalibrationDatabase>(
    "save_database",
    std::bind(
      &ExtrinsicMappingBasedCalibrator::saveDatabaseCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default);

  publisher_timer_ = rclcpp::create_timer(
    this, this->get_clock(), 5s, std::bind(&CalibrationMapper::publisherTimerCallback, mapper_));

  data_matching_timer_ = rclcpp::create_timer(
    this, this->get_clock(), 1s, std::bind(&CalibrationMapper::dataMatchingTimerCallback, mapper_));
}

rcl_interfaces::msg::SetParametersResult ExtrinsicMappingBasedCalibrator::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  std::unique_lock<std::mutex> service_lock(service_mutex_);
  std::unique_lock<std::recursive_mutex> mapping_lock(mapping_data_->mutex_);

  if (mapper_->getState() == CalibrationMapper::MAPPING || calibration_pending_) {
    RCLCPP_WARN(this->get_logger(), "Can not modify the parameters while mapping or calibrating");
    result.successful = false;
    result.reason = "Attempted to modify the parameters while mapping / calibrating";
    return result;
  }

  MappingParameters mapping_parameters = *mapping_parameters_;
  CalibrationParameters calibration_parameters = *calibration_parameters_;

  try {
    UPDATE_PARAM(mapping_parameters, use_rosbag);
    UPDATE_PARAM(mapping_parameters, mapping_verbose);
    UPDATE_PARAM(mapping_parameters, mapping_max_frames);
    UPDATE_PARAM(mapping_parameters, local_map_num_keyframes);
    UPDATE_PARAM(mapping_parameters, mapping_max_range);
    UPDATE_PARAM(mapping_parameters, min_mapping_pointcloud_size);
    UPDATE_PARAM(mapping_parameters, min_calibration_pointcloud_size);
    UPDATE_PARAM(mapping_parameters, mapping_lost_timeout);
    UPDATE_PARAM(mapping_parameters, mapper_resolution);
    UPDATE_PARAM(mapping_parameters, mapper_step_size);
    UPDATE_PARAM(mapping_parameters, mapper_max_iterations);
    UPDATE_PARAM(mapping_parameters, mapper_num_threads);
    UPDATE_PARAM(mapping_parameters, mapping_viz_leaf_size);
    UPDATE_PARAM(calibration_parameters, calibration_viz_leaf_size);
    UPDATE_PARAM(mapping_parameters, leaf_size_input);
    UPDATE_PARAM(mapping_parameters, leaf_size_local_map);
    UPDATE_PARAM(calibration_parameters, leaf_size_dense_map);
    UPDATE_PARAM(mapping_parameters, new_keyframe_min_distance);
    UPDATE_PARAM(mapping_parameters, new_frame_min_distance);
    UPDATE_PARAM(mapping_parameters, frame_stopped_distance);
    UPDATE_PARAM(mapping_parameters, frames_since_stop_force_frame);

    UPDATE_PARAM(mapping_parameters, calibration_skip_keyframes);

    UPDATE_PARAM(calibration_parameters, max_allowed_interpolated_time);
    UPDATE_PARAM(calibration_parameters, max_allowed_interpolated_distance);
    UPDATE_PARAM(calibration_parameters, max_allowed_interpolated_angle);
    UPDATE_PARAM(calibration_parameters, max_allowed_interpolated_speed);
    UPDATE_PARAM(calibration_parameters, max_allowed_interpolated_accel);
    UPDATE_PARAM(calibration_parameters, max_allowed_interpolated_distance_straight);
    UPDATE_PARAM(calibration_parameters, max_allowed_interpolated_angle_straight);
    UPDATE_PARAM(calibration_parameters, max_allowed_interpolated_speed_straight);
    UPDATE_PARAM(calibration_parameters, max_allowed_interpolated_accel_straight);

    UPDATE_PARAM(mapping_parameters, lost_frame_max_angle_diff);
    UPDATE_PARAM(mapping_parameters, lost_frame_interpolation_error);
    UPDATE_PARAM(mapping_parameters, lost_frame_max_acceleration);
    UPDATE_PARAM(calibration_parameters, min_calibration_range);
    UPDATE_PARAM(calibration_parameters, max_calibration_range);
    UPDATE_PARAM(calibration_parameters, calibration_min_pca_eigenvalue);
    UPDATE_PARAM(calibration_parameters, calibration_min_distance_between_frames);

    // Lidar calibration parameters
    UPDATE_PARAM(calibration_parameters, lidar_calibration_min_frames);
    UPDATE_PARAM(calibration_parameters, lidar_calibration_max_frames);

    // Camera calibration parameters
    UPDATE_PARAM(calibration_parameters, pc_features_min_distance);
    UPDATE_PARAM(calibration_parameters, pc_features_max_distance);
    UPDATE_PARAM(calibration_parameters, camera_calibration_min_frames);
    UPDATE_PARAM(calibration_parameters, camera_calibration_max_frames);

    // transaction succeeds, now assign values
    *mapping_parameters_ = mapping_parameters;
    *calibration_parameters_ = calibration_parameters;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  for (auto it = lidar_calibrators_.begin(); it != lidar_calibrators_.end(); it++) {
    it->second->configureCalibrators();
  }

  return result;
}

void ExtrinsicMappingBasedCalibrator::requestReceivedCallback(
  [[maybe_unused]] const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request>
    request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  using std::chrono_literals::operator""s;

  if (mapper_->getState() == CalibrationMapper::INITIAL) {
    mapper_->start();
  } else {
    RCLCPP_WARN(this->get_logger(), "The mapper had already started / finished !");
  }

  // Wait until map finishes
  while (mapper_->getState() != CalibrationMapper::FINISHED && rclcpp::ok()) {
    rclcpp::sleep_for(1s);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 30000, "Waiting for the mapper to finish");
  }

  {
    std::unique_lock<std::mutex> lock(service_mutex_);
    if (calibration_pending_) {
      RCLCPP_WARN(this->get_logger(), "There is a calibration pending !. Aborting this request");
      return;
    }

    calibration_pending_ = true;
  }

  std::vector<std::thread> calibrator_thread_pool;

  for (const std::string & calibration_frame_name : mapping_data_->calibration_lidar_frame_names_) {
    calibrator_thread_pool.emplace_back([&]() {
      RCLCPP_INFO(
        this->get_logger(), "Beginning lidar calibration for %s", calibration_frame_name.c_str());

      auto [status, transform, score] = lidar_calibrators_[calibration_frame_name]->calibrate();

      RCLCPP_INFO(
        this->get_logger(), "Lidar calibration for %s finished", calibration_frame_name.c_str());

      std::unique_lock<std::mutex> lock(service_mutex_);
      calibration_pending_map_[calibration_frame_name] = false;

      tier4_calibration_msgs::msg::CalibrationResult result;
      result.transform_stamped = tf2::eigenToTransform(Eigen::Isometry3d(transform));
      result.transform_stamped.header.frame_id = mapping_data_->mapping_lidar_frame_;
      result.transform_stamped.child_frame_id = calibration_frame_name;
      result.score = score;
      result.success = status;
      result.message.data = "Score corresponds to the source->target distance error";
      response->results.emplace_back(result);
    });
  }

  for (const std::string & calibration_frame_name :
       mapping_data_->calibration_camera_optical_link_frame_names) {
    calibrator_thread_pool.emplace_back([&]() {
      RCLCPP_INFO(
        this->get_logger(), "Beginning camera calibration for %s", calibration_frame_name.c_str());

      auto [status, transform, score] = camera_calibrators_[calibration_frame_name]->calibrate();

      RCLCPP_INFO(
        this->get_logger(), "Camera calibration for %s finished", calibration_frame_name.c_str());

      std::unique_lock<std::mutex> lock(service_mutex_);
      calibration_pending_map_[calibration_frame_name] = false;
      tier4_calibration_msgs::msg::CalibrationResult result;
      result.transform_stamped = tf2::eigenToTransform(Eigen::Isometry3d(transform));
      result.transform_stamped.header.frame_id = mapping_data_->mapping_lidar_frame_;
      result.transform_stamped.child_frame_id = calibration_frame_name;
      result.score = score;
      result.success = status;
      result.message.data = "Not implemented";
      response->results.emplace_back(result);
    });
  }

  if (calibration_parameters_->calibrate_base_frame_) {
    calibrator_thread_pool.emplace_back([&]() {
      const std::string & base_frame = calibration_parameters_->base_frame_;
      if (base_frame == "") {
        RCLCPP_INFO(this->get_logger(), "Base frame can not be empty");
        return;
      }

      RCLCPP_INFO(
        this->get_logger(), "Beginning ground plane calibration for %s", base_frame.c_str());

      auto [status, transform, score] = base_lidar_calibrator_->calibrate();

      RCLCPP_INFO(
        this->get_logger(), "Ground plane calibration for %s finished", base_frame.c_str());

      std::unique_lock<std::mutex> lock(service_mutex_);
      calibration_pending_map_[base_frame] = false;
      tier4_calibration_msgs::msg::CalibrationResult result;
      result.transform_stamped = tf2::eigenToTransform(Eigen::Isometry3d(transform));
      result.transform_stamped.header.frame_id = mapping_data_->mapping_lidar_frame_;
      result.transform_stamped.child_frame_id = base_frame;
      result.score = score;
      result.success = status;
      result.message.data = "Base calibration provides no score";
      response->results.emplace_back(result);
    });
  }

  // Wait until all calibrators finish
  std::for_each(calibrator_thread_pool.begin(), calibrator_thread_pool.end(), [](std::thread & t) {
    t.join();
  });

  RCLCPP_INFO_STREAM(this->get_logger(), "Sending the results to the calibrator manager");

  std::unique_lock<std::mutex> lock(service_mutex_);
  calibration_pending_ = false;
}

void ExtrinsicMappingBasedCalibrator::detectedObjectsCallback(
  const autoware_perception_msgs::msg::DetectedObjects::SharedPtr objects)
{
  // Convert objects into ObjectBB
  ObjectsBB new_objects;
  new_objects.header_ = objects->header;

  for (auto & object : objects->objects) {
    ObjectBB new_object;
    Eigen::Affine3d pose_affine;
    tf2::fromMsg(object.kinematics.pose_with_covariance.pose, pose_affine);
    new_object.pose_ = pose_affine.matrix().cast<float>();
    new_object.size_ = Eigen::Vector3f(
      object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z);

    new_objects.objects_.push_back(new_object);
  }

  RCLCPP_INFO(this->get_logger(), "Adding %ld detections", new_objects.objects_.size());

  // Add them to the data
  std::unique_lock<std::recursive_mutex> lock(mapping_data_->mutex_);
  mapping_data_->detected_objects_.push_back(new_objects);
}

void ExtrinsicMappingBasedCalibrator::predictedObjectsCallback(
  const autoware_perception_msgs::msg::PredictedObjects::SharedPtr objects)
{
  // Convert objects into ObjectBB
  ObjectsBB new_objects;
  new_objects.header_ = objects->header;

  for (auto & object : objects->objects) {
    ObjectBB new_object;
    Eigen::Affine3d pose_affine;
    tf2::fromMsg(object.kinematics.initial_pose_with_covariance.pose, pose_affine);
    new_object.pose_ = pose_affine.matrix().cast<float>();
    new_object.size_ = Eigen::Vector3f(
      object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z);

    new_objects.objects_.push_back(new_object);
  }

  RCLCPP_INFO(this->get_logger(), "Adding %ld detections", new_objects.objects_.size());

  // Add them to the data
  std::unique_lock<std::recursive_mutex> lock(mapping_data_->mutex_);
  mapping_data_->detected_objects_.push_back(new_objects);
}

void ExtrinsicMappingBasedCalibrator::loadDatabaseCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Response> response)
{
  // cSpell:ignore iarchive
  std::ifstream ifs(request->path);
  boost::archive::binary_iarchive ia(ifs);

  std::unique_lock<std::recursive_mutex> lock(mapping_data_->mutex_);

  mapping_data_->camera_calibration_frames_map_.clear();
  mapping_data_->lidar_calibration_frames_map_.clear();
  mapping_data_->processed_frames_.clear();
  mapping_data_->keyframes_.clear();
  mapping_data_->keyframes_and_stopped_.clear();

  ia >> mapping_data_->camera_calibration_frames_map_;
  for (auto it = mapping_data_->camera_calibration_frames_map_.begin();
       it != mapping_data_->camera_calibration_frames_map_.end(); it++) {
    RCLCPP_INFO(
      this->get_logger(), "Loaded %ld camera calibration frames (%s)...", it->second.size(),
      it->first.c_str());
  }

  ia >> mapping_data_->lidar_calibration_frames_map_;
  for (auto it = mapping_data_->lidar_calibration_frames_map_.begin();
       it != mapping_data_->lidar_calibration_frames_map_.end(); it++) {
    RCLCPP_INFO(
      this->get_logger(), "Loaded %ld lidar calibration frames (%s)...", it->second.size(),
      it->first.c_str());
  }

  ia >> mapping_data_->processed_frames_;
  RCLCPP_INFO(this->get_logger(), "Loaded %ld frames...", mapping_data_->processed_frames_.size());

  ia >> mapping_data_->keyframes_;
  ia >> mapping_data_->keyframes_and_stopped_;
  RCLCPP_INFO(this->get_logger(), "Loaded %ld keyframes...", mapping_data_->keyframes_.size());

  ia >> mapping_data_->detected_objects_;
  RCLCPP_INFO(this->get_logger(), "Loaded %ld objects...", mapping_data_->detected_objects_.size());

  mapper_->stop();

  RCLCPP_INFO(this->get_logger(), "Finished");

  response->success = true;
}

void ExtrinsicMappingBasedCalibrator::saveDatabaseCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Response> response)
{
  // cSpell:ignore oarchive
  std::ofstream ofs(request->path);
  boost::archive::binary_oarchive oa(ofs);

  std::unique_lock<std::recursive_mutex> lock(mapping_data_->mutex_);

  for (auto it = mapping_data_->camera_calibration_frames_map_.begin();
       it != mapping_data_->camera_calibration_frames_map_.end(); it++) {
    RCLCPP_INFO(
      this->get_logger(), "Saving %ld camera calibration frames (%s)...", it->second.size(),
      it->first.c_str());
  }
  oa << mapping_data_->camera_calibration_frames_map_;

  for (auto it = mapping_data_->lidar_calibration_frames_map_.begin();
       it != mapping_data_->lidar_calibration_frames_map_.end(); it++) {
    RCLCPP_INFO(
      this->get_logger(), "Saving %ld lidar calibration frames (%s)...", it->second.size(),
      it->first.c_str());
  }
  oa << mapping_data_->lidar_calibration_frames_map_;

  RCLCPP_INFO(this->get_logger(), "Saving %ld frames...", mapping_data_->processed_frames_.size());
  oa << mapping_data_->processed_frames_;

  RCLCPP_INFO(this->get_logger(), "Saving %ld keyframes...", mapping_data_->keyframes_.size());
  oa << mapping_data_->keyframes_;
  oa << mapping_data_->keyframes_and_stopped_;

  RCLCPP_INFO(this->get_logger(), "Saving %ld objects...", mapping_data_->detected_objects_.size());
  oa << mapping_data_->detected_objects_;

  RCLCPP_INFO(this->get_logger(), "Saving dense map cloud...");

  PointcloudType::Ptr map_cloud_ptr(new PointcloudType());
  PointcloudType::Ptr map_subsampled_cloud_ptr(new PointcloudType());

  for (const auto & frame : mapping_data_->processed_frames_) {
    PointcloudType::Ptr tmp_cloud_ptr(new PointcloudType());

    pcl::transformPointCloud(*frame->pointcloud_raw_, *tmp_cloud_ptr, frame->pose_);
    *map_cloud_ptr += *tmp_cloud_ptr;
  }

  PointcloudType::Ptr map_cropped_cloud_ptr = cropPointCloud<PointcloudType>(
    map_cloud_ptr, mapping_parameters_->mapping_min_range_,
    mapping_parameters_->mapping_max_range_);

  pcl::VoxelGridTriplets<PointType> voxel_grid;
  voxel_grid.setLeafSize(
    calibration_parameters_->leaf_size_dense_map_, calibration_parameters_->leaf_size_dense_map_,
    calibration_parameters_->leaf_size_dense_map_);
  voxel_grid.setInputCloud(map_cropped_cloud_ptr);
  voxel_grid.filter(*map_subsampled_cloud_ptr);
  pcl::io::savePCDFileASCII("dense_map.pcd", *map_subsampled_cloud_ptr);

  mapper_->stop();

  RCLCPP_INFO(this->get_logger(), "Finished");

  response->success = true;
}
