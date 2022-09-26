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
#include <rosbag2_interfaces/srv/pause.hpp>
#include <rosbag2_interfaces/srv/resume.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <tf2_eigen/tf2_eigen.h>

#include <chrono>
#include <fstream>
#include <iostream>

using namespace std::chrono_literals;

#define UNUSED(x) (void)x;

#define UPDATE_MAPPING_CALIBRATOR_PARAM(PARAM_STRUCT, NAME) \
  update_param(p, #NAME, PARAM_STRUCT.NAME##_)

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
  tf_broascaster_(this),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  transform_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
  mapping_parameters_(std::make_shared<MappingParameters>()),
  calibration_parameters_(std::make_shared<LidarCalibrationParameters>()),
  mapping_data_(std::make_shared<MappingData>())
{
  std::vector<std::string> calibration_service_names =
    this->declare_parameter<std::vector<std::string>>("calibration_service_names");
  std::vector<std::string> sensor_kit_frames =
    this->declare_parameter<std::vector<std::string>>("sensor_kit_frames");
  std::vector<std::string> calibration_lidar_base_frames =
    this->declare_parameter<std::vector<std::string>>("calibration_lidar_base_frames");
  mapping_data_->map_frame_ = this->declare_parameter<std::string>("map_frame");
  mapping_data_->calibration_lidar_frame_names_ =
    this->declare_parameter<std::vector<std::string>>("calibration_lidar_frames");
  std::vector<std::string> calibration_pointcloud_topics =
    this->declare_parameter<std::vector<std::string>>("calibration_pointcloud_topics");

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
    this->declare_parameter<int>("dense_pointcloud_num_keyframes_", 10);
  mapping_parameters_->mapping_max_range_ =
    this->declare_parameter<double>("mapping_max_range", 60.0);

  // Mapping parameters
  mapping_parameters_->ndt_resolution_ = this->declare_parameter<double>("ndt_resolution", 5.0);
  mapping_parameters_->ndt_step_size_ = this->declare_parameter<double>("ndt_step_size", 0.1);
  mapping_parameters_->ndt_max_iterations_ = this->declare_parameter<int>("ndt_max_iterations", 35);
  mapping_parameters_->ndt_epsilon_ = this->declare_parameter<double>("ndt_epsilon", 0.01);
  mapping_parameters_->ndt_num_threads_ = this->declare_parameter<int>("ndt_num_threads", 8);

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
  calibration_parameters_->calibration_min_frames_ =
    this->declare_parameter<int>("calibration_min_frames", 2);
  calibration_parameters_->calibration_max_frames_ =
    this->declare_parameter<int>("calibration_max_frames", 10);

  // Calibration frames selection criteria and preprocessing parameters
  calibration_parameters_->max_allowed_interpolated_time_ =
    this->declare_parameter<double>("max_allowed_interpolated_time", 0.03);
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
  calibration_parameters_->calibration_use_only_stopped_ =
    this->declare_parameter<bool>("calibration_use_only_stopped", false);
  calibration_parameters_->max_calibration_range_ =
    this->declare_parameter<double>("max_calibration_range", 80.0);
  calibration_parameters_->calibration_min_pca_eigenvalue_ =
    this->declare_parameter<double>("calibration_min_pca_eigenvalue", 0.25);
  calibration_parameters_->calibration_min_distance_between_frames_ =
    this->declare_parameter<double>("calibration_min_distance_between_frames", 5.0);

  // Calibration parameters
  calibration_parameters_->solver_iterations_ =
    this->declare_parameter<int>("solver_iterations", 200);
  calibration_parameters_->max_corr_dist_coarse_ =
    this->declare_parameter<double>("max_corr_dist_coarse", 0.5);
  calibration_parameters_->max_corr_dist_fine_ =
    this->declare_parameter<double>("max_corr_dist_fine", 0.1);
  calibration_parameters_->max_corr_dist_ultrafine_ =
    this->declare_parameter<double>("max_corr_dist_ultrafine", 0.05);

  auto map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_map", 10);

  auto frame_path_pub = this->create_publisher<nav_msgs::msg::Path>("frame_path", 10);
  auto keyframe_path_pub = this->create_publisher<nav_msgs::msg::Path>("keyframe_path", 10);
  auto keyframe_markers_pub =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("keyframe_markers", 10);

  auto rosbag2_pause_client_ = this->create_client<rosbag2_interfaces::srv::Pause>(
    "/rosbag2_player/pause", rmw_qos_profile_services_default);
  auto rosbag2_resume_client_ = this->create_client<rosbag2_interfaces::srv::Resume>(
    "/rosbag2_player/resume", rmw_qos_profile_services_default);

  // Set up mapper
  mapper_ = std::make_shared<CalibrationMapper>(
    mapping_parameters_, mapping_data_, map_pub, frame_path_pub, keyframe_path_pub,
    keyframe_markers_pub, rosbag2_pause_client_, rosbag2_resume_client_, tf_buffer_);

  // Set up lidar calibrators
  for (const auto & frame_name : mapping_data_->calibration_lidar_frame_names_) {
    auto initial_source_aligned_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      frame_name + "/initial_source_aligned_map", 10);
    auto calibrated_source_aligned_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      frame_name + "/calibrated_source_aligned_map", 10);
    auto target_map_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(frame_name + "/target_map", 10);

    lidar_calibrators_[frame_name] = std::make_shared<LidarCalibrator>(
      frame_name, calibration_parameters_, mapping_data_, tf_buffer_,
      initial_source_aligned_map_pub_, calibrated_source_aligned_map_pub_, target_map_pub_);
  }

  // Set up sensor callbacks
  assert(
    mapping_data_->calibration_lidar_frame_names_.size() == calibration_pointcloud_topics.size());
  for (std::size_t i = 0; i < mapping_data_->calibration_lidar_frame_names_.size(); i++) {
    const std::string & calibration_pointcloud_topic = calibration_pointcloud_topics[i];
    const std::string & calibration_frame_name = mapping_data_->calibration_lidar_frame_names_[i];

    sensor_kit_frame_map_[calibration_frame_name] = sensor_kit_frames[i];
    calibration_lidar_base_frame_map_[calibration_frame_name] = calibration_lidar_base_frames[i];

    srv_callback_groups_map_[calibration_frame_name] =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    calibration_api_server_map_[calibration_frame_name] =
      this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
        calibration_service_names[i] + "/extrinsic_calibration",
        [&](
          const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
          const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response>
            response) {
          requestReceivedCallback(
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
          &LidarCalibrator::singleLidarCalibrationCallback,
          lidar_calibrators_[calibration_frame_name], std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default);

    multiple_lidar_calibration_server_map_[calibration_frame_name] =
      this->create_service<tier4_calibration_msgs::srv::Frame>(
        calibration_frame_name + "/multiple_lidar_calibration",
        std::bind(
          &LidarCalibrator::multipleLidarCalibrationCallback,
          lidar_calibrators_[calibration_frame_name], std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default);
  }

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
    this, get_clock(), 10s, std::bind(&CalibrationMapper::publisherTimerCallback, mapper_));

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
  LidarCalibrationParameters calibration_parameters = *calibration_parameters_;
  std::unique_lock<std::mutex> lock(mapping_data_->mutex_);

  try {
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, use_rosbag);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, mapping_verbose);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, mapping_max_frames);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, local_map_num_keyframes);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, mapping_max_range);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, ndt_resolution);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, ndt_step_size);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, ndt_max_iterations);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, ndt_num_threads);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, mapping_viz_leaf_size);
    UPDATE_MAPPING_CALIBRATOR_PARAM(calibration_parameters, calibration_viz_leaf_size);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, leaf_size_input);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, leaf_size_local_map);
    UPDATE_MAPPING_CALIBRATOR_PARAM(calibration_parameters, leaf_size_dense_map);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, new_keyframe_min_distance);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, new_frame_min_distance);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, frame_stopped_distance);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, frames_since_stop_force_frame);

    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, calibration_skip_keyframes);
    UPDATE_MAPPING_CALIBRATOR_PARAM(calibration_parameters, calibration_min_frames);
    UPDATE_MAPPING_CALIBRATOR_PARAM(calibration_parameters, calibration_max_frames);

    UPDATE_MAPPING_CALIBRATOR_PARAM(calibration_parameters, max_allowed_interpolated_time);
    UPDATE_MAPPING_CALIBRATOR_PARAM(calibration_parameters, max_allowed_interpolated_distance);
    UPDATE_MAPPING_CALIBRATOR_PARAM(calibration_parameters, max_allowed_interpolated_angle);
    UPDATE_MAPPING_CALIBRATOR_PARAM(calibration_parameters, max_allowed_interpolated_speed);
    UPDATE_MAPPING_CALIBRATOR_PARAM(calibration_parameters, max_allowed_interpolated_accel);
    UPDATE_MAPPING_CALIBRATOR_PARAM(
      calibration_parameters, max_allowed_interpolated_distance_straight);
    UPDATE_MAPPING_CALIBRATOR_PARAM(
      calibration_parameters, max_allowed_interpolated_angle_straight);
    UPDATE_MAPPING_CALIBRATOR_PARAM(
      calibration_parameters, max_allowed_interpolated_speed_straight);
    UPDATE_MAPPING_CALIBRATOR_PARAM(
      calibration_parameters, max_allowed_interpolated_accel_straight);

    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, lost_frame_max_angle_diff);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, lost_frame_interpolation_error);
    UPDATE_MAPPING_CALIBRATOR_PARAM(mapping_parameters, lost_frame_max_acceleration);
    UPDATE_MAPPING_CALIBRATOR_PARAM(calibration_parameters, max_calibration_range);
    UPDATE_MAPPING_CALIBRATOR_PARAM(calibration_parameters, calibration_min_pca_eigenvalue);
    UPDATE_MAPPING_CALIBRATOR_PARAM(
      calibration_parameters, calibration_min_distance_between_frames);

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
  const std::string & parent_frame, const std::string & calibration_lidar_base_frame,
  const std::string & calibration_lidar_frame,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  (void)request;

  Eigen::Isometry3d parent_to_mapping_lidar_eigen_;
  Eigen::Isometry3d lidar_base_to_lidar_eigen_;

  {
    std::unique_lock<std::mutex> service_lock(service_mutex_);
    std::unique_lock<std::mutex> data_lock(mapping_data_->mutex_);

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

      geometry_msgs::msg::Transform parent_to_mapping_lidar_msg_ =
        tf_buffer_->lookupTransform(parent_frame, mapping_data_->mapping_lidar_frame_, t, timeout)
          .transform;

      parent_to_mapping_lidar_eigen_ = tf2::transformToEigen(parent_to_mapping_lidar_msg_);

      geometry_msgs::msg::Transform lidar_base_to_lidar_msg_ =
        tf_buffer_
          ->lookupTransform(calibration_lidar_base_frame, calibration_lidar_frame, t, timeout)
          .transform;

      lidar_base_to_lidar_eigen_ = tf2::transformToEigen(lidar_base_to_lidar_msg_);
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
    calibration_status_map_[calibration_lidar_frame] = calibration_status;
    calibration_results_map_[calibration_lidar_frame] = calibration_result;
    calibration_scores_map_[calibration_lidar_frame] = calibration_score;
  });

  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(service_mutex_);

      if (calibration_status_map_[calibration_lidar_frame]) {
        break;
      }
    }

    rclcpp::sleep_for(1000ms);
  }

  std::unique_lock<std::mutex> lock(service_mutex_);
  t.join();

  Eigen::Isometry3d mapping_to_calibration_lidar_lidar_eigen =
    Eigen::Isometry3d(calibration_results_map_[calibration_lidar_frame].cast<double>());
  Eigen::Isometry3d parent_to_lidar_base_eigen =
    parent_to_mapping_lidar_eigen_ * mapping_to_calibration_lidar_lidar_eigen.inverse() *
    lidar_base_to_lidar_eigen_.inverse();

  response->result_pose = tf2::toMsg(parent_to_lidar_base_eigen);
  response->score = calibration_scores_map_[calibration_lidar_frame];
  response->success = calibration_status_map_[calibration_lidar_frame];

  // Convert  raw calibration to the output tf
  RCLCPP_INFO_STREAM(this->get_logger(), "Sending the tesults to the calibrator manager");

  std::this_thread::sleep_for(std::chrono::seconds(1));
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

  mapping_data_->calibration_frames_map_.clear();
  mapping_data_->processed_frames_.clear();
  mapping_data_->keyframes_.clear();

  ia >> mapping_data_->calibration_frames_map_;
  for (auto it = mapping_data_->calibration_frames_map_.begin();
       it != mapping_data_->calibration_frames_map_.end(); it++) {
    RCLCPP_INFO(
      this->get_logger(), "Loaded %ld calibration frames (%s)...", it->second.size(),
      it->first.c_str());
  }

  ia >> mapping_data_->processed_frames_;
  RCLCPP_INFO(this->get_logger(), "Loaded %ld frames...", mapping_data_->processed_frames_.size());

  ia >> mapping_data_->keyframes_;
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

  for (auto it = mapping_data_->calibration_frames_map_.begin();
       it != mapping_data_->calibration_frames_map_.end(); it++) {
    RCLCPP_INFO(
      this->get_logger(), "Saving %ld calibration frames (%s)...", it->second.size(),
      it->first.c_str());
  }
  oa << mapping_data_->calibration_frames_map_;

  RCLCPP_INFO(this->get_logger(), "Saving %ld frames...", mapping_data_->processed_frames_.size());
  oa << mapping_data_->processed_frames_;

  RCLCPP_INFO(this->get_logger(), "Saving %ld keyframes...", mapping_data_->keyframes_.size());
  oa << mapping_data_->keyframes_;

  RCLCPP_INFO(this->get_logger(), "Saving %ld objects...", mapping_data_->detected_objects_.size());
  oa << mapping_data_->detected_objects_;

  mapper_->stop();

  RCLCPP_INFO(this->get_logger(), "Finished");

  response->success = true;
}
