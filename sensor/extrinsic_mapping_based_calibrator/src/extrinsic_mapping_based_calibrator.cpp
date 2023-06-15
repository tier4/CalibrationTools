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

#include <extrinsic_mapping_based_calibrator/extrinsic_mapping_based_calibrator.hpp>
#include <extrinsic_mapping_based_calibrator/serialization.hpp>
#include <extrinsic_mapping_based_calibrator/utils.hpp>
#include <rosbag2_interfaces/srv/pause.hpp>
#include <rosbag2_interfaces/srv/resume.hpp>
#include <tier4_calibration_pcl_extensions/voxel_grid_triplets.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <pcl/io/pcd_io.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <chrono>
#include <fstream>
#include <iostream>

#define UNUSED(x) (void)x;

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
      rclcpp::get_logger("extrinsic_mapping_based_calibrator"),
      "Setting parameter [" << name << "] to " << value);
  }
}
}  // namespace

ExtrinsicMappingBasedCalibrator::ExtrinsicMappingBasedCalibrator(
  const rclcpp::NodeOptions & options)
: Node("extrinsic_mapping_based_calibrator_node", options),
  tf_broadcaster_(this),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  transform_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
  mapping_parameters_(std::make_shared<MappingParameters>()),
  calibration_parameters_(std::make_shared<CalibrationParameters>()),
  mapping_data_(std::make_shared<MappingData>())
{
  using std::chrono_literals::operator""s;

  std::vector<std::string> camera_calibration_service_names =
    this->declare_parameter<std::vector<std::string>>("camera_calibration_service_names");
  std::vector<std::string> lidar_calibration_service_names =
    this->declare_parameter<std::vector<std::string>>("lidar_calibration_service_names");

  std::vector<std::string> camera_calibration_sensor_kit_frames =
    this->declare_parameter<std::vector<std::string>>("camera_calibration_sensor_kit_frames");
  std::vector<std::string> lidar_calibration_sensor_kit_frames =
    this->declare_parameter<std::vector<std::string>>("lidar_calibration_sensor_kit_frames");

  std::vector<std::string> calibration_camera_frames =
    this->declare_parameter<std::vector<std::string>>("calibration_camera_frames");

  std::vector<std::string> calibration_lidar_base_frames =
    this->declare_parameter<std::vector<std::string>>("calibration_lidar_base_frames");

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

  remove_empty_strings(camera_calibration_service_names);
  remove_empty_strings(lidar_calibration_service_names);
  remove_empty_strings(lidar_calibration_sensor_kit_frames);
  remove_empty_strings(calibration_camera_frames);
  remove_empty_strings(calibration_lidar_base_frames);
  remove_empty_strings(mapping_data_->calibration_camera_optical_link_frame_names);
  remove_empty_strings(mapping_data_->calibration_lidar_frame_names_);
  remove_empty_strings(calibration_camera_info_topics);
  remove_empty_strings(calibration_image_topics);
  remove_empty_strings(calibration_pointcloud_topics);

  mapping_data_->mapping_lidar_frame_ = this->declare_parameter<std::string>("mapping_lidar_frame");

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
  mapping_parameters_->leaf_size_input_ = this->declare_parameter<double>("leaf_size_iput", 0.1);
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
  calibration_parameters_->max_corr_dist_ultrafine_ =
    this->declare_parameter<double>("max_corr_dist_ultrafine", 0.05);

  // Lidar calibration-only parameters
  calibration_parameters_->lidar_calibration_min_frames_ =
    this->declare_parameter<int>("lidar_calibration_min_frames", 2);
  calibration_parameters_->lidar_calibration_max_frames_ =
    this->declare_parameter<int>("lidar_calibration_max_frames", 10);

  // Camera calibration-only parameters TODO(knzo25): sort the parameters
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

  // Set up lidar calibrators
  for (const auto & frame_name : mapping_data_->calibration_camera_optical_link_frame_names) {
    auto target_map_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(frame_name + "/target_map", 10);
    auto target_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      frame_name + "/target_markers", 10);

    camera_calibrators_[frame_name] = std::make_shared<CameraCalibrator>(
      frame_name, calibration_parameters_, mapping_data_, tf_buffer_, target_map_pub,
      target_markers_pub);
  }

  for (const auto & frame_name : mapping_data_->calibration_lidar_frame_names_) {
    auto initial_source_aligned_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      frame_name + "/initial_source_aligned_map", 10);
    auto calibrated_source_aligned_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      frame_name + "/calibrated_source_aligned_map", 10);
    auto target_map_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(frame_name + "/target_map", 10);

    lidar_calibrators_[frame_name] = std::make_shared<LidarCalibrator>(
      frame_name, calibration_parameters_, mapping_data_, tf_buffer_,
      initial_source_aligned_map_pub, calibrated_source_aligned_map_pub, target_map_pub);
  }

  auto base_lidar_augmented_pointcloud_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("base_lidar_augmented_pointcloud", 10);
  auto base_lidar_augmented_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_pointcloud", 10);

  base_lidar_calibrator_ = std::make_shared<BaseLidarCalibrator>(
    calibration_parameters_, mapping_data_, tf_buffer_, tf_broadcaster_,
    base_lidar_augmented_pointcloud_pub, base_lidar_augmented_pub);

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

    sensor_kit_frame_map_[calibration_frame_name] = camera_calibration_sensor_kit_frames[i];
    calibration_camera_frame_map_[calibration_frame_name] = calibration_camera_frames[i];

    srv_callback_groups_map_[calibration_frame_name] =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    calibration_api_server_map_[calibration_frame_name] =
      this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
        camera_calibration_service_names[i] + "/extrinsic_calibration",
        [&](
          const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
          const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response>
            response) {
          cameraCalibrationRequestReceivedCallback(
            sensor_kit_frame_map_[calibration_frame_name],
            calibration_camera_frame_map_[calibration_frame_name], calibration_frame_name, request,
            response);
        },
        rmw_qos_profile_services_default, srv_callback_groups_map_[calibration_frame_name]);

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

    sensor_kit_frame_map_[calibration_frame_name] = lidar_calibration_sensor_kit_frames[i];
    calibration_lidar_base_frame_map_[calibration_frame_name] = calibration_lidar_base_frames[i];

    srv_callback_groups_map_[calibration_frame_name] =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    calibration_api_server_map_[calibration_frame_name] =
      this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
        lidar_calibration_service_names[i] + "/extrinsic_calibration",
        [&](
          const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
          const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response>
            response) {
          lidarCalibrationRequestReceivedCallback(
            sensor_kit_frame_map_[calibration_frame_name],
            calibration_lidar_base_frame_map_[calibration_frame_name], calibration_frame_name,
            request, response);
        },
        rmw_qos_profile_services_default, srv_callback_groups_map_[calibration_frame_name]);

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

  detected_objects_sub_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
      "detected_objects", rclcpp::SensorDataQoS().keep_all(),
      std::bind(
        &ExtrinsicMappingBasedCalibrator::detectedObjectsCallback, this, std::placeholders::_1));

  predicted_objects_sub_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "predicted_objects", rclcpp::SensorDataQoS().keep_all(),
      std::bind(
        &ExtrinsicMappingBasedCalibrator::predictedObjectsCallback, this, std::placeholders::_1));

  for (auto & calibration_frame_name : mapping_data_->calibration_lidar_frame_names_) {
    single_lidar_calibration_server_map_[calibration_frame_name] =
      this->create_service<tier4_calibration_msgs::srv::Frame>(
        calibration_frame_name + "/single_lidar_calibration",
        std::bind(
          &LidarCalibrator::singleSensorCalibrationCallback,
          lidar_calibrators_[calibration_frame_name], std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default);

    multiple_lidar_calibration_server_map_[calibration_frame_name] =
      this->create_service<tier4_calibration_msgs::srv::Frame>(
        calibration_frame_name + "/multiple_lidar_calibration",
        std::bind(
          &LidarCalibrator::multipleSensorCalibrationCallback,
          lidar_calibrators_[calibration_frame_name], std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default);
  }

  base_link_calibration_server_ = this->create_service<std_srvs::srv::Empty>(
    "base_link_calibration",
    [&](
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      const std::shared_ptr<std_srvs::srv::Empty::Response> response) {
      UNUSED(request);
      UNUSED(response);
      Eigen::Matrix4f transform;
      float score;
      std::unique_lock<std::mutex> data_lock(mapping_data_->mutex_);
      RCLCPP_INFO_STREAM(this->get_logger(), "Starting base lidar calibration");
      base_lidar_calibrator_->calibrate(transform, score);
    },
    rmw_qos_profile_services_default);

  stop_mapping_server_ = this->create_service<std_srvs::srv::Empty>(
    "stop_mapping",
    [&](
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      const std::shared_ptr<std_srvs::srv::Empty::Response> response) {
      UNUSED(request);
      UNUSED(response);
      std::unique_lock<std::mutex> data_lock(mapping_data_->mutex_);
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
    this, get_clock(), 5s, std::bind(&CalibrationMapper::publisherTimerCallback, mapper_));

  data_matching_timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&CalibrationMapper::dataMatchingTimerCallback, mapper_));
}

rcl_interfaces::msg::SetParametersResult ExtrinsicMappingBasedCalibrator::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  MappingParameters mapping_parameters = *mapping_parameters_;
  CalibrationParameters calibration_parameters = *calibration_parameters_;
  std::unique_lock<std::mutex> lock(mapping_data_->mutex_);

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

void ExtrinsicMappingBasedCalibrator::cameraCalibrationRequestReceivedCallback(
  const std::string & parent_frame, const std::string & calibration_camera_frame,
  const std::string & calibration_camera_optical_link_frame,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  (void)request;
  using std::chrono_literals::operator""ms;

  Eigen::Isometry3d parent_to_mapping_lidar_eigen;
  Eigen::Isometry3d camera_to_camera_optical_link_eigen;

  {
    std::unique_lock<std::mutex> service_lock(service_mutex_);
    std::unique_lock<std::mutex> data_lock(mapping_data_->mutex_);

    calibration_pending_map_[calibration_camera_optical_link_frame] = true;
    calibration_status_map_[calibration_camera_optical_link_frame] = false;
    calibration_results_map_[calibration_camera_optical_link_frame] = Eigen::Matrix4f::Identity();

    RCLCPP_INFO_STREAM(this->get_logger(), "Calibration request received");
    RCLCPP_INFO_STREAM(this->get_logger(), "\t\tparent_frame = " << parent_frame);
    RCLCPP_INFO_STREAM(
      this->get_logger(), "\t\tcalibration_camera_frame = " << calibration_camera_frame);
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "\t\tcalibration_camera_optical_link_frame = " << calibration_camera_optical_link_frame);

    try {
      rclcpp::Time t = rclcpp::Time(0);
      rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

      geometry_msgs::msg::Transform parent_to_mapping_lidar_msg =
        tf_buffer_->lookupTransform(parent_frame, mapping_data_->mapping_lidar_frame_, t, timeout)
          .transform;

      parent_to_mapping_lidar_eigen = tf2::transformToEigen(parent_to_mapping_lidar_msg);

      geometry_msgs::msg::Transform camera_to_camera_optical_link_msg =
        tf_buffer_
          ->lookupTransform(
            calibration_camera_frame, calibration_camera_optical_link_frame, t, timeout)
          .transform;

      camera_to_camera_optical_link_eigen =
        tf2::transformToEigen(camera_to_camera_optical_link_msg);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "could not get initial tfs. Aborting calibration. %s", ex.what());
      response->success = false;
      return;
    }
  }

  // Start monitoring and calibration frame
  std::thread t([&]() {
    while (!mapper_->stopped() && rclcpp::ok()) {
      rclcpp::sleep_for(1000ms);
    }

    Eigen::Matrix4f calibration_result;
    float calibration_score;
    bool calibration_status = camera_calibrators_[calibration_camera_optical_link_frame]->calibrate(
      calibration_result, calibration_score);

    std::unique_lock<std::mutex> lock(service_mutex_);
    calibration_pending_map_[calibration_camera_optical_link_frame] = false;
    calibration_status_map_[calibration_camera_optical_link_frame] = calibration_status;
    calibration_results_map_[calibration_camera_optical_link_frame] = calibration_result;
    calibration_scores_map_[calibration_camera_optical_link_frame] = calibration_score;
  });

  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(service_mutex_);

      if (!calibration_pending_map_[calibration_camera_optical_link_frame]) {
        break;
      }
    }

    rclcpp::sleep_for(1000ms);
  }

  std::unique_lock<std::mutex> lock(service_mutex_);
  t.join();

  Eigen::Isometry3d mapping_to_calibration_lidar_lidar_eigen = Eigen::Isometry3d(
    calibration_results_map_[calibration_camera_optical_link_frame].cast<double>());
  Eigen::Isometry3d parent_to_lidar_base_eigen = parent_to_mapping_lidar_eigen *
                                                 mapping_to_calibration_lidar_lidar_eigen *
                                                 camera_to_camera_optical_link_eigen.inverse();

  response->result_pose = tf2::toMsg(parent_to_lidar_base_eigen);
  response->score = calibration_scores_map_[calibration_camera_optical_link_frame];
  response->success = calibration_status_map_[calibration_camera_optical_link_frame];

  // Convert  raw calibration to the output tf
  RCLCPP_INFO_STREAM(this->get_logger(), "Sending the tesults to the calibrator manager");
}

void ExtrinsicMappingBasedCalibrator::lidarCalibrationRequestReceivedCallback(
  const std::string & parent_frame, const std::string & calibration_lidar_base_frame,
  const std::string & calibration_lidar_frame,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  (void)request;
  using std::chrono_literals::operator""ms;

  Eigen::Isometry3d parent_to_mapping_lidar_eigen;
  Eigen::Isometry3d lidar_base_to_lidar_eigen;

  {
    std::unique_lock<std::mutex> service_lock(service_mutex_);
    std::unique_lock<std::mutex> data_lock(mapping_data_->mutex_);

    calibration_pending_map_[calibration_lidar_frame] = true;
    calibration_status_map_[calibration_lidar_frame] = false;
    calibration_results_map_[calibration_lidar_frame] = Eigen::Matrix4f::Identity();

    RCLCPP_INFO_STREAM(this->get_logger(), "Calibration request received");
    RCLCPP_INFO_STREAM(this->get_logger(), "\t\tparent_frame = " << parent_frame);
    RCLCPP_INFO_STREAM(
      this->get_logger(), "\t\tcalibration_lidar_base_frame = " << calibration_lidar_base_frame);
    RCLCPP_INFO_STREAM(
      this->get_logger(), "\t\tcalibration_lidar_frame = " << calibration_lidar_frame);

    try {
      rclcpp::Time t = rclcpp::Time(0);
      rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

      geometry_msgs::msg::Transform parent_to_mapping_lidar_msg =
        tf_buffer_->lookupTransform(parent_frame, mapping_data_->mapping_lidar_frame_, t, timeout)
          .transform;

      parent_to_mapping_lidar_eigen = tf2::transformToEigen(parent_to_mapping_lidar_msg);

      geometry_msgs::msg::Transform lidar_base_to_lidar_msg =
        tf_buffer_
          ->lookupTransform(calibration_lidar_base_frame, calibration_lidar_frame, t, timeout)
          .transform;

      lidar_base_to_lidar_eigen = tf2::transformToEigen(lidar_base_to_lidar_msg);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "could not get initial tfs. Aborting calibration. %s", ex.what());
      response->success = false;
      return;
    }
  }

  // Start monitoring and calibration frame
  std::thread t([&]() {
    while (!mapper_->stopped() && rclcpp::ok()) {
      rclcpp::sleep_for(1000ms);
    }

    Eigen::Matrix4f calibration_result;
    float calibration_score;
    bool calibration_status =
      lidar_calibrators_[calibration_lidar_frame]->calibrate(calibration_result, calibration_score);

    std::unique_lock<std::mutex> lock(service_mutex_);
    calibration_pending_map_[calibration_lidar_frame] = false;
    calibration_status_map_[calibration_lidar_frame] = calibration_status;
    calibration_results_map_[calibration_lidar_frame] = calibration_result;
    calibration_scores_map_[calibration_lidar_frame] = calibration_score;
  });

  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(service_mutex_);

      if (!calibration_pending_map_[calibration_lidar_frame]) {
        break;
      }
    }

    rclcpp::sleep_for(1000ms);
  }

  std::unique_lock<std::mutex> lock(service_mutex_);
  t.join();

  Eigen::Isometry3d mapping_to_calibration_lidar_lidar_eigen =
    Eigen::Isometry3d(calibration_results_map_[calibration_lidar_frame].cast<double>());
  Eigen::Isometry3d parent_to_lidar_base_eigen = parent_to_mapping_lidar_eigen *
                                                 mapping_to_calibration_lidar_lidar_eigen *
                                                 lidar_base_to_lidar_eigen.inverse();

  response->result_pose = tf2::toMsg(parent_to_lidar_base_eigen);
  response->score = calibration_scores_map_[calibration_lidar_frame];
  response->success = calibration_status_map_[calibration_lidar_frame];

  // Convert  raw calibration to the output tf
  RCLCPP_INFO_STREAM(this->get_logger(), "Sending the tesults to the calibrator manager");
}

void ExtrinsicMappingBasedCalibrator::detectedObjectsCallback(
  const autoware_auto_perception_msgs::msg::DetectedObjects::SharedPtr objects)
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
  std::unique_lock<std::mutex> lock(mapping_data_->mutex_);
  mapping_data_->detected_objects_.push_back(new_objects);
}

void ExtrinsicMappingBasedCalibrator::predictedObjectsCallback(
  const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr objects)
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
  std::unique_lock<std::mutex> lock(mapping_data_->mutex_);
  mapping_data_->detected_objects_.push_back(new_objects);
}

void ExtrinsicMappingBasedCalibrator::loadDatabaseCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Response> response)
{
  std::ifstream ifs(request->path);
  boost::archive::binary_iarchive ia(ifs);

  std::unique_lock<std::mutex> lock(mapping_data_->mutex_);

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
  std::ofstream ofs(request->path);
  boost::archive::binary_oarchive oa(ofs);

  std::unique_lock<std::mutex> lock(mapping_data_->mutex_);

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
