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
#include <extrinsic_mapping_based_calibrator/voxel_grid_filter_wrapper.hpp>
#include <pcl_ros/transforms.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <tier4_pcl_extensions/voxel_grid_triplets.hpp>

#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/joint_icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <thread>

#define UNUSED(x) (void)x;

using namespace std::chrono_literals;

#define UPDATE_MAPPING_CALIBRATOR_PARAM(PARAM_STRUCT, NAME) \
  update_param(parameters, #NAME, PARAM_STRUCT.NAME##_)

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
  delta_pose_(Eigen::Matrix4f::Identity()),
  selected_keyframe_(-1),
  n_processed_frames_(0)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  sensor_kit_frame_ = this->declare_parameter<std::string>("parent_frame");
  lidar_base_frame_ = this->declare_parameter<std::string>("child_frame");
  map_frame_ = this->declare_parameter<std::string>("map_frame");
  calibration_frame_names_ =
    this->declare_parameter<std::vector<std::string>>("calibration_lidar_frames");
  calibration_pointcloud_topics_ =
    this->declare_parameter<std::vector<std::string>>("calibration_pointcloud_topics");

  mapping_lidar_frame_ = this->declare_parameter<std::string>("mapping_lidar_frame");

  params_.verbose_ = this->declare_parameter<bool>("verbose", false);
  params_.use_rosbag_ = this->declare_parameter<bool>("use_rosbag", true);
  params_.max_frames_ = this->declare_parameter<int>("max_frames", 500);
  params_.local_map_num_keyframes_ = this->declare_parameter<int>("local_map_num_keyframes", 15);
  params_.calibration_num_keyframes_ =
    this->declare_parameter<int>("calibration_num_keyframes_", 10);
  params_.max_pointcloud_range_ = this->declare_parameter<double>(
    "max_pointcloud_range", 60.0);  // maximum range of pointclouds during mapping

  // Mapping parameters
  params_.ndt_resolution_ = this->declare_parameter<double>("ndt_resolution", 5.0);
  params_.ndt_step_size_ = this->declare_parameter<double>("ndt_step_size", 0.1);
  params_.ndt_max_iterations_ = this->declare_parameter<int>("ndt_max_iterations", 35);
  params_.ndt_num_threads_ = this->declare_parameter<int>("ndt_num_threads", 8);

  params_.leaf_size_viz_ = this->declare_parameter<double>("leaf_size_viz", 0.15);
  params_.leaf_size_input_ = this->declare_parameter<double>("leaf_size_iput", 0.1);
  params_.leaf_size_local_map_ = this->declare_parameter<double>("leaf_size_local_map", 0.1);
  params_.leaf_size_dense_map_ = this->declare_parameter<double>("leaf_size_dense_map", 0.05);
  params_.new_keyframe_min_distance_ =
    this->declare_parameter<double>("new_keyframe_min_distance", 1.0);
  params_.new_frame_min_distance_ = this->declare_parameter<double>("new_frame_min_distance", 0.05);
  params_.frame_stopped_distance_ = this->declare_parameter<double>(
    "frame_stopped_distance",
    0.02);  // distance between frames to consider the car to be stopped. uses hysteresis
  params_.frame_nonstopped_distance_ = this->declare_parameter<double>(
    "frame_nonstopped_distance",
    0.05);  // distance between frames to consider the car to be moving. uses hysteresis
  params_.frames_since_stop_force_frame_ =
    this->declare_parameter<int>("frames_since_stoped_force_frame", 5);
  params_.calibration_skip_keyframes_ = this->declare_parameter<int>(
    "calibration_skip_keyframes", 5);  // Skip the first frames for calibration
  params_.calibration_max_frames_ = this->declare_parameter<int>("calibration_max_frames", 10);

  // Calibration frames selection criteria and preprocessing parameters
  params_.calibration_max_interpolated_time_ =
    this->declare_parameter<double>("calibration_max_interpolated_time", 0.03);
  params_.calibration_max_interpolated_distance_ =
    this->declare_parameter<double>("calibration_max_interpolated_distance", 0.05);
  params_.calibration_max_interpolated_angle_ =
    this->declare_parameter<double>("calibration_max_interpolated_angle", 1.0);
  params_.calibration_max_interpolated_speed_ =
    this->declare_parameter<double>("calibration_max_interpolated_speed", 3.0);
  params_.calibration_max_interpolated_accel_ =
    this->declare_parameter<double>("calibration_max_interpolated_accel", 0.4);

  params_.calibration_max_interpolated_distance_straight_ =
    this->declare_parameter<double>("calibration_max_interpolated_distance_straight", 0.08);
  params_.calibration_max_interpolated_angle_straight_ =
    this->declare_parameter<double>("calibration_max_interpolated_angle_straight", 0.01);
  params_.calibration_max_interpolated_speed_straight_ =
    this->declare_parameter<double>("calibration_max_interpolated_speed_straight", 5.0);
  params_.calibration_max_interpolated_accel_straight_ =
    this->declare_parameter<double>("calibration_max_interpolated_accel_straight", 0.1);

  params_.lost_frame_max_angle_diff_ =
    this->declare_parameter<double>("lost_frame_max_angle_diff", 25.0);  // def
  params_.lost_frame_interpolation_error_ =
    this->declare_parameter<double>("lost_frame_interpolation_error", 0.05);
  params_.lost_frame_skip_frames_ = this->declare_parameter<int>("lost_frame_skip_frames", 10);
  params_.calibration_use_only_stopped_ =
    this->declare_parameter<bool>("calibration_use_only_stopped", false);
  params_.max_calibration_range_ = this->declare_parameter<double>("max_calibration_range", 80.0);
  params_.calibration_min_pca_eigenvalue_ =
    this->declare_parameter<double>("calibration_min_pca_eigenvalue", 0.25);

  // Calibration parameters
  params_.calibration_solver_iterations_ =
    this->declare_parameter<double>("calibration_solver_iterations_", 200);
  params_.calibration_max_corr_dist_coarse_ =
    this->declare_parameter<double>("calibration_max_corr_dist_coarse_", 0.5);
  params_.calibration_max_corr_dist_fine_ =
    this->declare_parameter<double>("calibration_max_corr_dist_fine_", 0.1);
  params_.calibration_max_corr_dist_ultrafine_ =
    this->declare_parameter<double>("calibration_max_corr_dist_ultrafine_", 0.05);

  map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_map", 10);
  keyframe_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("keyframe_map", 10);
  keyframe_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("keyframe", 10);

  for (const auto & calibration_frame_name : calibration_frame_names_) {
    initial_source_aligned_map_pub_map_[calibration_frame_name] =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        calibration_frame_name + "/initial_source_aligned_map", 10);
    calibrated_source_aligned_map_pub_map_[calibration_frame_name] =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        calibration_frame_name + "/calibrated_source_aligned_map", 10);
    target_map_pub_map_[calibration_frame_name] =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        calibration_frame_name + "/target_map_pub", 10);
  }

  frame_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("frame_path", 10);
  keyframe_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("keyframe_path", 10);
  keyframe_markers_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("keyframe_markers", 10);

  assert(calibration_frame_names_.size() == calibration_pointcloud_topics_.size());
  for (std::size_t i = 0; i < calibration_frame_names_.size(); i++) {
    const std::string & calibration_pointcloud_topic = calibration_pointcloud_topics_[i];
    const std::string & calibration_frame_name = calibration_frame_names_[i];

    calibration_pointcloud_subs_[calibration_frame_name] =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        calibration_pointcloud_topic, rclcpp::SensorDataQoS(),
        [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          // RCLCPP_WARN(
          //   get_logger(), "Pre pointcloud callback = %s", calibration_frame_name.c_str());
          calibrationPointCloudCallback(msg, calibration_frame_name);
        });
  }

  mapping_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "mapping_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(
      &ExtrinsicMappingBasedCalibrator::mappingPointCloudCallback, this, std::placeholders::_1));

  // The service server runs in a dedicated thread
  srv_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  keyframe_map_server_ = this->create_service<tier4_calibration_msgs::srv::Frame>(
    "keyframe_map",
    std::bind(
      &ExtrinsicMappingBasedCalibrator::keyFrameCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default);

  for (auto & calibration_frame_name : calibration_frame_names_) {
    single_lidar_calibration_server_map_[calibration_frame_name] =
      this->create_service<tier4_calibration_msgs::srv::Frame>(
        calibration_frame_name + "/single_lidar_calibration",
        [&](
          const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
          const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response) {
          singleLidarCalibrationCallback(calibration_frame_name, request, response);
        },
        rmw_qos_profile_services_default);

    multiple_lidar_calibration_server_map_[calibration_frame_name] =
      this->create_service<tier4_calibration_msgs::srv::Frame>(
        calibration_frame_name + "/multiple_lidar_calibration",
        [&](
          const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
          const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response) {
          multipleLidarCalibrationCallback(calibration_frame_name, request, response);
        },
        rmw_qos_profile_services_default);
  }

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

  rosbag2_pause_client_ = this->create_client<rosbag2_interfaces::srv::Pause>(
    "/rosbag2_player/pause", rmw_qos_profile_services_default);
  rosbag2_resume_client_ = this->create_client<rosbag2_interfaces::srv::Resume>(
    "/rosbag2_player/resume", rmw_qos_profile_services_default);

  timer_ = rclcpp::create_timer(
    this, get_clock(), 10s, std::bind(&ExtrinsicMappingBasedCalibrator::timerCallback, this));

  published_map_pointcloud_ptr_.reset(new PointcloudType());

  // Mapping configuration
  ndt_.setResolution(params_.ndt_resolution_);             // 1.0
  ndt_.setStepSize(params_.ndt_step_size_);                // 0.1
  ndt_.setMaximumIterations(params_.ndt_max_iterations_);  // 35
  ndt_.setTransformationEpsilon(0.01);
  ndt_.setNeighborhoodSearchMethod(pclomp::DIRECT7);

  if (params_.ndt_num_threads_ > 0) {
    ndt_.setNumThreads(params_.ndt_num_threads_);
  }

  for (auto & calibration_frame_name : calibration_frame_names_) {
    last_unmatched_keyframe_map_[calibration_frame_name] = params_.calibration_skip_keyframes_;
  }

  std::thread thread = std::thread(&ExtrinsicMappingBasedCalibrator::mappingThreadWorker, this);
  thread.detach();

  // Calibration configuration
  correspondence_estimator_ =
    pcl::make_shared<pcl::registration::CorrespondenceEstimation<PointType, PointType>>();

  calibration_ndt_ = pcl::make_shared<pclomp::NormalDistributionsTransform<PointType, PointType>>();
  calibration_gicp_ =
    pcl::make_shared<pcl::GeneralizedIterativeClosestPoint<PointType, PointType>>();
  calibration_icp_coarse_ = pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();
  calibration_icp_fine_ = pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();
  calibration_icp_ultrafine_ = pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();

  calibration_registrators_ = {
    calibration_ndt_, calibration_gicp_, calibration_icp_coarse_, calibration_icp_fine_,
    calibration_icp_ultrafine_};

  calibration_batch_icp_coarse_ =
    pcl::make_shared<pcl::JointIterativeClosestPointExtended<PointType, PointType>>();
  calibration_batch_icp_fine_ =
    pcl::make_shared<pcl::JointIterativeClosestPointExtended<PointType, PointType>>();
  calibration_batch_icp_ultrafine_ =
    pcl::make_shared<pcl::JointIterativeClosestPointExtended<PointType, PointType>>();
  calibration_batch_registrators_ = {
    calibration_batch_icp_coarse_, calibration_batch_icp_fine_, calibration_batch_icp_ultrafine_};

  configureCalibrators();
}

rcl_interfaces::msg::SetParametersResult ExtrinsicMappingBasedCalibrator::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  Params params = params_;

  try {
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, use_rosbag);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, verbose);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, max_frames);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, local_map_num_keyframes);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_num_keyframes);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, max_pointcloud_range);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, ndt_resolution);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, ndt_step_size);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, ndt_max_iterations);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, ndt_num_threads);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, leaf_size_viz);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, leaf_size_input);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, leaf_size_local_map);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, leaf_size_dense_map);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, new_keyframe_min_distance);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, new_frame_min_distance);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, frame_stopped_distance);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, frame_nonstopped_distance);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, frames_since_stop_force_frame);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_skip_keyframes);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_max_frames);

    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_max_interpolated_time);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_max_interpolated_distance);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_max_interpolated_angle);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_max_interpolated_speed);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_max_interpolated_accel);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_max_interpolated_distance_straight);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_max_interpolated_angle_straight);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_max_interpolated_speed_straight);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_max_interpolated_accel_straight);

    UPDATE_MAPPING_CALIBRATOR_PARAM(params, lost_frame_max_angle_diff);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, lost_frame_interpolation_error);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, lost_frame_skip_frames);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, max_calibration_range);
    UPDATE_MAPPING_CALIBRATOR_PARAM(params, calibration_min_pca_eigenvalue);

    // transaction succeeds, now assign values
    params_ = params;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  configureCalibrators();

  return result;
}

void ExtrinsicMappingBasedCalibrator::configureCalibrators()
{
  calibration_gicp_->setMaxCorrespondenceDistance(params_.calibration_max_corr_dist_coarse_);
  calibration_icp_coarse_->setMaxCorrespondenceDistance(params_.calibration_max_corr_dist_coarse_);
  calibration_icp_fine_->setMaxCorrespondenceDistance(params_.calibration_max_corr_dist_fine_);
  calibration_icp_ultrafine_->setMaxCorrespondenceDistance(
    params_.calibration_max_corr_dist_ultrafine_);

  for (auto & calibrator : calibration_registrators_) {
    calibrator->setMaximumIterations(params_.calibration_solver_iterations_);
  }

  calibration_batch_icp_coarse_->setMaxCorrespondenceDistance(
    params_.calibration_max_corr_dist_coarse_);
  calibration_batch_icp_fine_->setMaxCorrespondenceDistance(
    params_.calibration_max_corr_dist_fine_);
  calibration_batch_icp_ultrafine_->setMaxCorrespondenceDistance(
    params_.calibration_max_corr_dist_ultrafine_);

  for (auto & calibrator : calibration_batch_registrators_) {
    calibrator->setMaximumIterations(params_.calibration_solver_iterations_);
  }
}

void ExtrinsicMappingBasedCalibrator::setUpCalibrators(
  PointcloudType::Ptr & source_pointcloud_ptr, PointcloudType::Ptr & target_pointcloud_ptr)
{
  for (auto & calibrator : calibration_registrators_) {
    calibrator->setInputSource(source_pointcloud_ptr);
    calibrator->setInputTarget(target_pointcloud_ptr);
  }
}

void ExtrinsicMappingBasedCalibrator::calibrationPointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string & frame_name)
{
  assert(calibration_pointcloud_subs_.find(frame_name) != calibration_pointcloud_subs_.end());

  // auto & calibration_lidar_header = calibration_lidar_header_map_[frame_name];
  auto & last_unmatched_keyframe = last_unmatched_keyframe_map_[frame_name];
  auto & calibration_frames = calibration_frames_map_[frame_name];
  auto & calibration_pointclouds_queue = calibration_pointclouds_queue_map_[frame_name];

  // if (!calibration_lidar_header) {
  //   calibration_lidar_header = std::make_shared<std_msgs::msg::Header>(msg->header);
  // }

  calibration_pointclouds_queue.push(msg);

  std::unique_lock<std::mutex> lock(mutex_);

  while (last_unmatched_keyframe < static_cast<int>(keyframes_and_stopped_.size()) - 1) {
    Frame::Ptr keyframe = keyframes_and_stopped_[last_unmatched_keyframe];
    auto keyframe_stamp = rclcpp::Time(keyframe->header_.stamp);

    if (
      calibration_pointclouds_queue.size() < 3 ||
      rclcpp::Time(calibration_pointclouds_queue.back()->header.stamp) < keyframe_stamp) {
      return;
    }

    // We need there to be a frame after the keyframe since we may wanto to interpolate in that
    // direction
    if (keyframe->frame_id_ + 1 >= static_cast<int>(processed_frames_.size())) {
      return;
    }

    // Iterate for all frames between the last keyframe and this one looking for a stopped frame,
    // which we want to use as a calibration frame
    auto pc_msg_left = calibration_pointclouds_queue.front();
    calibration_pointclouds_queue.pop();
    auto pc_msg_right = calibration_pointclouds_queue.front();

    while (rclcpp::Time(pc_msg_right->header.stamp) < keyframe_stamp) {
      pc_msg_left = pc_msg_right;
      calibration_pointclouds_queue.pop();
      pc_msg_right = calibration_pointclouds_queue.front();
    }

    double dt_left = (keyframe_stamp - rclcpp::Time(pc_msg_left->header.stamp)).seconds();
    double dt_right = (rclcpp::Time(pc_msg_right->header.stamp) - keyframe_stamp).seconds();

    // Drop all the pointcloud messages until we
    sensor_msgs::msg::PointCloud2::SharedPtr pc_msg;
    Frame::Ptr frame_left, frame_right, frame_aux;
    double interpolated_distance;
    double interpolated_time;
    double interpolated_speed;
    double interpolated_accel;

    // Compute first and second order derivates with finite differences
    if (dt_left < dt_right) {
      pc_msg = pc_msg_left;

      frame_left = processed_frames_[keyframe->frame_id_ - 1];
      frame_right = keyframe;
      frame_aux = processed_frames_[keyframe->frame_id_ + 1];

      double dt1 =
        (rclcpp::Time(frame_right->header_.stamp) - rclcpp::Time(frame_left->header_.stamp))
          .seconds();
      double dt2 =
        (rclcpp::Time(frame_aux->header_.stamp) - rclcpp::Time(frame_right->header_.stamp))
          .seconds();
      interpolated_speed = std::abs((frame_aux->distance_ - frame_left->distance_) / (dt1 + dt2));
      interpolated_accel = 2 * std::abs(
                                 ((frame_aux->distance_ - frame_right->distance_) / dt2 -
                                  (frame_right->distance_ - frame_left->distance_) / dt1) /
                                 (dt1 + dt2));

      interpolated_time = dt_left;
    } else {
      pc_msg = pc_msg_right;

      frame_aux = processed_frames_[keyframe->frame_id_ - 1];
      frame_left = keyframe;
      frame_right = processed_frames_[keyframe->frame_id_ + 1];

      double dt1 =
        (rclcpp::Time(frame_left->header_.stamp) - rclcpp::Time(frame_aux->header_.stamp))
          .seconds();
      double dt2 =
        (rclcpp::Time(frame_right->header_.stamp) - rclcpp::Time(frame_left->header_.stamp))
          .seconds();
      interpolated_speed = std::abs((frame_right->distance_ - frame_aux->distance_) / (dt1 + dt2));
      interpolated_accel = std::abs(
        ((frame_right->distance_ - frame_left->distance_) / dt2 -
         (frame_left->distance_ - frame_aux->distance_) / dt1) /
        (dt1 + dt2));

      interpolated_time = dt_right;
    }

    interpolated_distance =
      (frame_right->distance_ - frame_left->distance_) * interpolated_time /
      (rclcpp::Time(frame_right->header_.stamp) - rclcpp::Time(frame_left->header_.stamp))
        .seconds();

    // Interpolate the pose
    Eigen::Matrix4f interpolated_pose = poseInterpolation(
      (rclcpp::Time(pc_msg->header.stamp) - rclcpp::Time(frame_left->header_.stamp)).seconds(), 0.f,
      (rclcpp::Time(frame_right->header_.stamp) - rclcpp::Time(frame_left->header_.stamp))
        .seconds(),
      frame_left->pose_, frame_right->pose_);

    // Compute interpolation angle
    // trR = 1 + 2cos(theta)
    Eigen::Matrix4f aux_pose = keyframe->pose_.inverse() * interpolated_pose;
    Eigen::Affine3f aux_affine(aux_pose);
    double interpolated_angle =
      180.0 *
      std::abs(std::acos(std::min(std::max(0.5 * aux_affine.linear().trace() - 0.5, -1.0), 1.0))) /
      3.1416;

    PointcloudType::Ptr pc_ptr(new PointcloudType());
    pcl::fromROSMsg(*pc_msg, *pc_ptr);
    transformPointcloud<PointcloudType>(pc_msg->header.frame_id, frame_name, pc_ptr, *tf_buffer_);

    CalibrationFrame calibration_frame;

    calibration_frame.source_pointcloud_ = pc_ptr;
    calibration_frame.source_header_ = pc_msg->header;

    calibration_frame.target_frame_ = keyframe;
    calibration_frame.local_map_pose_ = interpolated_pose;

    calibration_frame.interpolated_distance_ = interpolated_distance;
    calibration_frame.interpolated_angle_ = interpolated_angle;
    calibration_frame.interpolated_time_ = interpolated_time;
    calibration_frame.estimated_speed_ = interpolated_speed;
    calibration_frame.estimated_accel_ = interpolated_accel;
    calibration_frame.stopped_ = keyframe->stopped_;

    calibration_frames.emplace_back(calibration_frame);

    last_unmatched_keyframe += 1;
  }
}

void ExtrinsicMappingBasedCalibrator::mappingPointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!mapping_lidar_header_) {
    mapping_lidar_header_ = std::make_shared<std_msgs::msg::Header>(msg->header);
  }

  PointcloudType::Ptr pc_ptr(new PointcloudType());
  pcl::fromROSMsg(*msg, *pc_ptr);
  transformPointcloud<PointcloudType>(
    msg->header.frame_id, mapping_lidar_frame_, pc_ptr, *tf_buffer_);

  std::unique_lock<std::mutex> lock(mutex_);
  auto frame = std::make_shared<Frame>();
  frame->header_ = msg->header;
  frame->pointcloud_raw_ = pc_ptr;

  if (
    rclcpp::Time(msg->header.stamp) < rclcpp::Time(mapping_lidar_header_->stamp) ||
    static_cast<int>(processed_frames_.size()) >= params_.max_frames_) {
    return;
  }

  if (params_.use_rosbag_ && !bag_paused_ && unprocessed_frames_.size() > 0) {
    auto cb = [&](rclcpp::Client<rosbag2_interfaces::srv::Pause>::SharedFuture response_client) {
      auto res = response_client.get();
      (void)res;
      std::unique_lock<std::mutex> lock(mutex_);
      bag_paused_ = true;
      RCLCPP_WARN_STREAM(this->get_logger(), "Received pause response:");
    };
    RCLCPP_WARN_STREAM(this->get_logger(), "Send pause call:");
    auto request = std::make_shared<rosbag2_interfaces::srv::Pause::Request>();
    rosbag2_pause_client_->async_send_request(request, cb);
  }

  unprocessed_frames_.push(frame);
  RCLCPP_INFO(
    get_logger(), "ROS: New pointcloud. Unprocessed=%lu Frames=%lu Keyframes=%lu",
    unprocessed_frames_.size(), processed_frames_.size(), keyframes_.size());
}

void ExtrinsicMappingBasedCalibrator::mappingThreadWorker()
{
  bool first_iteration = true;

  while (rclcpp::ok()) {
    Frame::Ptr frame, prev_frame;
    Eigen::Matrix4f prev_pose = Eigen::Matrix4f::Identity();
    float prev_distance = 0.f;

    // Locked section
    {
      std::unique_lock<std::mutex> lock(mutex_);

      if (unprocessed_frames_.size() == 0) {
        if (params_.use_rosbag_ && bag_paused_) {
          auto cb =
            [&](rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedFuture response_client) {
              auto res = response_client.get();
              (void)res;
              std::unique_lock<std::mutex> lock(mutex_);
              bag_paused_ = false;
              RCLCPP_WARN_STREAM(this->get_logger(), "Received resume response:");
            };

          RCLCPP_WARN_STREAM(this->get_logger(), "Sending resume call:");
          auto request = std::make_shared<rosbag2_interfaces::srv::Resume::Request>();
          rosbag2_resume_client_->async_send_request(request, cb);
        }

        lock.unlock();
        rclcpp::sleep_for(10ms);
        continue;
      }

      frame = unprocessed_frames_.back();
      unprocessed_frames_.pop();

      if (processed_frames_.size() > 0) {
        prev_frame = processed_frames_.back();
        prev_pose = prev_frame->pose_;
        prev_distance = prev_frame->distance_;
      }
    }

    // If there are no keyframes make the first and recalculate the local map
    if (first_iteration) {
      RCLCPP_INFO(get_logger(), "Thread. Init local map");
      initLocalMap(frame);
    }

    // Subsample the frame
    // pcl::VoxelGrid<PointType> voxel_grid;
    VoxelGridWrapper<PointType> voxel_grid;
    frame->pointcloud_subsampled_ = PointcloudType::Ptr(new PointcloudType());
    PointcloudType::Ptr aligned_cloud_ptr(new PointcloudType());
    PointcloudType::Ptr cropped_cloud_ptr =
      cropPointCloud(frame->pointcloud_raw_, params_.max_pointcloud_range_);

    voxel_grid.setLeafSize(
      params_.leaf_size_input_, params_.leaf_size_input_, params_.leaf_size_input_);
    voxel_grid.setInputCloud(cropped_cloud_ptr);
    voxel_grid.filter(*frame->pointcloud_subsampled_);

    // Register the frame to the map
    ndt_.setInputTarget(local_map_ptr_);
    ndt_.setInputSource(frame->pointcloud_subsampled_);

    if (first_iteration) {
      frame->pose_ = Eigen::Matrix4f::Identity();
    } else {
      Eigen::Matrix4f guess = prev_pose * delta_pose_;
      ndt_.align(*aligned_cloud_ptr, guess);
      frame->pose_ = ndt_.getFinalTransformation();
      delta_pose_ = prev_pose.inverse() * frame->pose_;

      std::cout << "NDT score: " << ndt_.getFitnessScore() << std::endl;
    }

    std::unique_lock<std::mutex> lock(mutex_);

    // We record the whole trajectory
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = frame->header_;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = tf2::toMsg(Eigen::Affine3d(frame->pose_.cast<double>()));
    trajectory_.push_back(pose_msg);

    // Fill the frame information
    Eigen::Vector3f translation = Eigen::Affine3f(frame->pose_.inverse() * prev_pose).translation();
    float delta_distance = translation.norm();
    frame->distance_ = prev_distance + delta_distance;
    frame->frame_id_ = processed_frames_.size();
    frame->processed_ = true;

    if (!first_iteration && delta_distance < params_.new_frame_min_distance_) {
      if (
        std::abs(delta_distance - prev_frame->delta_distance_) < params_.frame_stopped_distance_) {
        // When the vehicle is stopped, we may either skip the frame, record it as a normal frame,
        // or even save it for calibration

        prev_frame->frames_since_stop_ += 1;
        prev_frame->delta_distance_ = delta_distance;

        frame->frames_since_stop_ = prev_frame->frames_since_stop_;
        frame->stopped_ = true;

        if (prev_frame->frames_since_stop_ == params_.frames_since_stop_force_frame_) {
          RCLCPP_WARN(get_logger(), "Added a keyframe_and_stopped frame");
          keyframes_and_stopped_.push_back(frame);
        } else if (
          prev_frame->stopped_ &&
          std::abs(prev_frame->frames_since_stop_ - params_.frames_since_stop_force_frame_) > 1) {
          RCLCPP_INFO(get_logger(), "Dropped frame due to stopped");
          continue;
        }
      } else {
        // If the vehicle moved to little we drop the frame
        RCLCPP_INFO(
          get_logger(), "Dropped frame. Unprocessed=%lu Frames=%lu Keyframes=%lu",
          unprocessed_frames_.size(), processed_frames_.size(), keyframes_.size());
        continue;
      }
    }

    processed_frames_.push_back(frame);
    checkKeyframe(frame);

    RCLCPP_INFO(
      get_logger(), "New frame. Distance=%.2f Unprocessed=%lu Frames=%lu Keyframes=%lu",
      frame->distance_, unprocessed_frames_.size(), processed_frames_.size(), keyframes_.size());
    // Publish tf
    first_iteration = false;
  }
}

void ExtrinsicMappingBasedCalibrator::initLocalMap(Frame::Ptr frame)
{
  local_map_ptr_.reset(new PointcloudType());
  PointcloudType::Ptr cropped_cloud_ptr =
    cropPointCloud(frame->pointcloud_raw_, params_.max_pointcloud_range_);
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setLeafSize(
    params_.leaf_size_local_map_, params_.leaf_size_local_map_, params_.leaf_size_local_map_);
  voxel_grid.setInputCloud(cropped_cloud_ptr);
  voxel_grid.filter(*local_map_ptr_);
}

void ExtrinsicMappingBasedCalibrator::checkKeyframe(Frame::Ptr frame)
{
  if (
    keyframes_.size() == 0 ||
    frame->distance_ >= keyframes_.back()->distance_ + params_.new_keyframe_min_distance_) {
    keyframes_.push_back(frame);
    keyframes_and_stopped_.push_back(frame);
    frame->is_key_frame_ = true;
    frame->keyframe_id_ = keyframes_.size();
    checkKeyframeLost(frame);
    recalculateLocalMap();
  }
}

void ExtrinsicMappingBasedCalibrator::checkKeyframeLost(Frame::Ptr keyframe)
{
  assert(keyframe->is_key_frame_);

  if (keyframe->keyframe_id_ <= 1) {
    return;
  }

  const Frame::Ptr & two_left_frame = keyframes_[keyframe->keyframe_id_ - 2];
  const Frame::Ptr & left_frame = keyframes_[keyframe->keyframe_id_ - 1];
  Frame::Ptr & right_frame = keyframe;

  Eigen::Affine3f dpose1(two_left_frame->pose_.inverse() * left_frame->pose_);
  Eigen::Affine3f dpose2(left_frame->pose_.inverse() * right_frame->pose_);
  Eigen::Affine3f left_pose(left_frame->pose_);
  Eigen::Affine3f right_pose(right_frame->pose_);

  auto d1 = dpose1.translation().normalized();
  auto d2 = dpose2.translation().normalized();

  float trans_angle_diff = (180.0 / M_PI) * std::acos(d1.dot(d2));
  trans_angle_diff =
    dpose2.translation().norm() > params_.new_keyframe_min_distance_ ? trans_angle_diff : 0.0;

  float rot_angle_diff =
    (180.0 / M_PI) * std::acos(std::min(
                       1.0, 0.5 * ((dpose1.rotation().inverse() * dpose2.rotation()).trace() -
                                   1.0)));  // Tr(R) = 1 + 2*cos(theta)

  if (
    std::abs(trans_angle_diff) > params_.lost_frame_max_angle_diff_ ||
    std::abs(trans_angle_diff) > params_.lost_frame_max_angle_diff_) {
    keyframe->lost_ = true;

    RCLCPP_WARN(
      get_logger(),
      "Mapping failed. Angle between keyframes is too high. a1=%.2f (deg) a2=%.2f (deg) "
      "theshold=%.2f (deg)",
      trans_angle_diff, rot_angle_diff, params_.lost_frame_max_angle_diff_);

    return;
  }

  for (int i = left_frame->frame_id_; i < right_frame->frame_id_; i++) {
    Frame::Ptr & mid_frame = processed_frames_[i];

    // Interpolate the pose
    Eigen::Matrix4f interpolated_pose_matrix = poseInterpolation(
      (rclcpp::Time(mid_frame->header_.stamp) - rclcpp::Time(left_frame->header_.stamp)).seconds(),
      0.f,
      (rclcpp::Time(right_frame->header_.stamp) - rclcpp::Time(left_frame->header_.stamp))
        .seconds(),
      left_frame->pose_, right_frame->pose_);

    Eigen::Affine3f interpolated_pose(interpolated_pose_matrix);
    Eigen::Affine3f frame_pose(mid_frame->pose_);

    float trans_diff = (interpolated_pose.inverse() * frame_pose).translation().norm();
    float rot_angle_diff =
      (180.0 / M_PI) *
      std::acos(std::min(
        1.0, 0.5 * ((interpolated_pose.rotation().inverse() * frame_pose.rotation()).trace() -
                    1.0)));  // Tr(R) = 1 + 2*cos(theta)

    if (
      (!left_frame->stopped_ && !right_frame->stopped_) &&
      (trans_diff > params_.lost_frame_interpolation_error_ ||
       std::abs(rot_angle_diff) > params_.lost_frame_max_angle_diff_)) {
      keyframe->lost_ = true;

      RCLCPP_WARN(
        get_logger(), "Mapping failed. Interpolation error is too high. d=%.2f (m) a=%.2f (deg)",
        trans_diff, rot_angle_diff);

      return;
    }

    float frame_left_distance = (frame_pose.translation() - left_pose.translation()).norm();
    float interpolated_left_distance =
      (interpolated_pose.translation() - left_pose.translation()).norm();
    float line_dist_error = std::sqrt(
      frame_left_distance * frame_left_distance +
      interpolated_left_distance * interpolated_left_distance);

    if (line_dist_error > params_.lost_frame_interpolation_error_) {
      keyframe->lost_ = true;

      RCLCPP_WARN(
        get_logger(), "Mapping failed. Interpolation error is too high. line_dist_error=%.2f (m)",
        line_dist_error);

      return;
    }
  }
}

void ExtrinsicMappingBasedCalibrator::recalculateLocalMap()
{
  local_map_ptr_->clear();

  PointcloudType::Ptr tmp_mcs_ptr(new PointcloudType());

  for (int i = 0; i < params_.local_map_num_keyframes_ && i < static_cast<int>(keyframes_.size());
       i++) {
    const auto & keyframe = keyframes_[keyframes_.size() - i - 1];

    if (keyframe->lost_ && i != 0) {
      break;
    }

    PointcloudType::Ptr keyframe_mcs_ptr(new PointcloudType());
    pcl::transformPointCloud(*keyframe->pointcloud_subsampled_, *keyframe_mcs_ptr, keyframe->pose_);
    *tmp_mcs_ptr += *keyframe_mcs_ptr;
  }

  VoxelGridWrapper<PointType> voxel_grid;
  voxel_grid.setLeafSize(
    params_.leaf_size_local_map_, params_.leaf_size_local_map_, params_.leaf_size_local_map_);
  voxel_grid.setInputCloud(tmp_mcs_ptr);
  voxel_grid.filter(*local_map_ptr_);
}

void ExtrinsicMappingBasedCalibrator::timerCallback()
{
  static int published_frames = 0;
  static int published_keyframes = 0;

  std::unique_lock<std::mutex> lock(mutex_);

  if (static_cast<int>(keyframes_.size()) == published_keyframes) {
    return;
  }

  // Process the new keyframes into the map
  PointcloudType::Ptr tmp_mcs_ptr(new PointcloudType());
  PointcloudType::Ptr subsampled_mcs_ptr(new PointcloudType());
  *tmp_mcs_ptr += *published_map_pointcloud_ptr_;

  for (auto it = processed_frames_.begin() + published_keyframes; it != processed_frames_.end();
       ++it) {
    Frame::Ptr frame = *it;
    PointcloudType::Ptr frame_mcs_ptr(new PointcloudType());
    pcl::transformPointCloud(*frame->pointcloud_subsampled_, *frame_mcs_ptr, frame->pose_);
    *tmp_mcs_ptr += *frame_mcs_ptr;
  }

  pcl::transformPointCloud(*tmp_mcs_ptr, *tmp_mcs_ptr, processed_frames_.back()->pose_.inverse());
  tmp_mcs_ptr = cropPointCloud(tmp_mcs_ptr, params_.max_calibration_range_);
  pcl::transformPointCloud(*tmp_mcs_ptr, *tmp_mcs_ptr, processed_frames_.back()->pose_);

  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setLeafSize(params_.leaf_size_viz_, params_.leaf_size_viz_, params_.leaf_size_viz_);
  voxel_grid.setInputCloud(tmp_mcs_ptr);
  voxel_grid.filter(*subsampled_mcs_ptr);

  published_map_pointcloud_ptr_.swap(subsampled_mcs_ptr);

  // Process the new keyframes and frames into the paths
  for (auto it = keyframes_.begin() + published_keyframes; it != keyframes_.end(); ++it) {
    Frame::Ptr keyframe = *it;
    Eigen::Matrix4d pose_matrix = keyframe->pose_.cast<double>();
    Eigen::Isometry3d pose_isometry(pose_matrix);
    geometry_msgs::msg::PoseStamped pose_msg;
    visualization_msgs::msg::Marker keyframe_marker;
    pose_msg.header = keyframe->header_;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = tf2::toMsg(pose_isometry);
    published_keyframes_path_.poses.push_back(pose_msg);

    keyframe_marker.header = keyframe->header_;
    keyframe_marker.header.frame_id = map_frame_;
    keyframe_marker.ns = "keyframe_id";
    keyframe_marker.id = keyframe->keyframe_id_;
    keyframe_marker.color.r = 1.0;
    keyframe_marker.color.g = 1.0;
    keyframe_marker.color.b = 1.0;
    keyframe_marker.color.a = 1.0;
    keyframe_marker.scale.x = 0.03;

    keyframe_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    keyframe_marker.text = std::to_string(keyframe->keyframe_id_);
    keyframe_marker.scale.z = 0.3;
    keyframe_marker.pose = pose_msg.pose;
    keyframe_marker.pose.position.z += 0.1;
    published_keyframes_markers_.markers.push_back(keyframe_marker);
  }

  for (auto it = processed_frames_.begin() + published_frames; it != processed_frames_.end();
       ++it) {
    Frame::Ptr frame = *it;
    Eigen::Matrix4d pose_matrix = frame->pose_.cast<double>();
    Eigen::Isometry3d pose_isometry(pose_matrix);
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = frame->header_;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = tf2::toMsg(pose_isometry);
    published_frames_path_.poses.push_back(pose_msg);
  }

  published_frames = processed_frames_.size();
  published_keyframes = keyframes_.size();

  // Publish the data
  sensor_msgs::msg::PointCloud2 published_map_msg;
  pcl::toROSMsg(*published_map_pointcloud_ptr_, published_map_msg);

  published_map_msg.header.stamp = mapping_lidar_header_->stamp;
  published_keyframes_path_.header.stamp = mapping_lidar_header_->stamp;
  published_frames_path_.header.stamp = mapping_lidar_header_->stamp;

  published_map_msg.header.frame_id = map_frame_;
  published_keyframes_path_.header.frame_id = map_frame_;
  published_frames_path_.header.frame_id = map_frame_;

  map_pub_->publish(published_map_msg);
  keyframe_path_pub_->publish(published_keyframes_path_);
  frame_path_pub_->publish(published_frames_path_);
  keyframe_markers_pub_->publish(published_keyframes_markers_);

  return;
}

PointcloudType::Ptr ExtrinsicMappingBasedCalibrator::getDensePointcloudFromMap(
  const Eigen::Matrix4f & pose, Frame::Ptr & frame, double resolution, double max_range)
{
  int frame_id = frame->frame_id_;

  // Find the closest keyframe to the requested keyframe
  Frame::Ptr keyframe_left, keyframe_right, keyframe;

  for (auto it = processed_frames_.begin() + frame_id; it != processed_frames_.begin(); it--) {
    if ((*it)->is_key_frame_) {
      keyframe_left = *it;
      break;
    }
  }

  for (auto it = processed_frames_.begin() + frame_id; it != processed_frames_.end(); it++) {
    if ((*it)->is_key_frame_) {
      keyframe_right = *it;
      break;
    }
  }

  if (keyframe_left && keyframe_right) {
    keyframe =
      (keyframe_right->frame_id_ - frame->frame_id_ < frame->frame_id_ - keyframe_left->frame_id_)
        ? keyframe_right
        : keyframe_left;
  } else if (keyframe_left) {
    keyframe = keyframe_left;
  } else if (keyframe_right) {
    keyframe = keyframe_right;
  } else {
    assert(false);
  }

  int min_keyframe_id =
    std::max<int>(0, keyframe->keyframe_id_ - params_.calibration_num_keyframes_);
  int max_keyframe_id = std::min<int>(
    keyframes_.size() - 1, keyframe->keyframe_id_ + params_.calibration_num_keyframes_);

  int min_frame_id = keyframes_[min_keyframe_id]->frame_id_;
  int max_frame_id = keyframes_[max_keyframe_id]->frame_id_;

  auto target_map_pose = pose.inverse();

  // Sum all frames in the target coordinate system (tcs)
  PointcloudType::Ptr tmp_tcs_ptr(new PointcloudType());
  PointcloudType::Ptr subsampled_tcs_ptr(new PointcloudType());

  for (int i = min_frame_id; i <= max_frame_id; i++) {
    Frame::Ptr frame = processed_frames_[i];
    PointcloudType::Ptr frame_tcs_ptr(new PointcloudType());

    auto map_frame_pose = frame->pose_;
    auto target_frame_pose = target_map_pose * map_frame_pose;

    pcl::transformPointCloud(*frame->pointcloud_raw_, *frame_tcs_ptr, target_frame_pose);
    *tmp_tcs_ptr += *frame_tcs_ptr;
  }

  PointcloudType::Ptr cropped_tcd_ptr = cropPointCloud(tmp_tcs_ptr, max_range);

  pcl::VoxelGridTriplets<PointType> voxel_grid;
  voxel_grid.setLeafSize(resolution, resolution, resolution);
  voxel_grid.setInputCloud(cropped_tcd_ptr);
  voxel_grid.filter(*subsampled_tcs_ptr);

  return subsampled_tcs_ptr;
}

std::vector<CalibrationFrame>
ExtrinsicMappingBasedCalibrator::filterCalibrationFramesByInterpolationError(
  const std::vector<CalibrationFrame> & calibration_frames)
{
  std::vector<CalibrationFrame> filtered_frames;

  // vector with invalid frame ids

  for (auto & frame : calibration_frames) {
    (void)frame;
    // if a keyframe, look for the next, and do the in between frame check using the interpolation
    // formula is not a keyframe. look for the left keyframe and the right, and do the same if is a
    // stopped frame, skip these checks
  }

  return filtered_frames;

  // Using the invalid frame ids, filter all calibration frames based on the ids if they are close
  // enough to ignore
}

std::vector<CalibrationFrame> ExtrinsicMappingBasedCalibrator::filterCalibrationFramesByDynamics(
  const std::vector<CalibrationFrame> & calibration_frames)
{
  std::vector<CalibrationFrame> filtered_frames;

  for (auto & frame : calibration_frames) {
    RCLCPP_INFO(
      get_logger(), "Attempting to add keyframe id=%d to the calibration list",
      frame.target_frame_->keyframe_id_);
    RCLCPP_INFO(get_logger(), "\t - stopped: %s", frame.stopped_ ? " true" : "false");
    RCLCPP_INFO(
      get_logger(), "\t - interpolated time: %.4f s (%s)", frame.interpolated_time_,
      frame.interpolated_time_ < params_.calibration_max_interpolated_time_ ? "accepted"
                                                                            : "rejected");
    RCLCPP_INFO(
      get_logger(), "\t - interpolated distance: %.4f m (%s)", frame.interpolated_distance_,
      frame.interpolated_distance_ < params_.calibration_max_interpolated_distance_ ? "accepted"
                                                                                    : "rejected");
    RCLCPP_INFO(
      get_logger(), "\t - interpolated angle: %.4f deg (%s)", frame.interpolated_angle_,
      frame.interpolated_angle_ < params_.calibration_max_interpolated_angle_ ? "accepted"
                                                                              : "rejected");
    RCLCPP_INFO(
      get_logger(), "\t - interpolated speed: %.4f m/s (%s)", frame.estimated_speed_,
      frame.estimated_speed_ < params_.calibration_max_interpolated_speed_ ? "accepted"
                                                                           : "rejected");
    RCLCPP_INFO(
      get_logger(), "\t - interpolated accel: %.4f m/s2 (%s)", frame.estimated_accel_,
      frame.estimated_accel_ < params_.calibration_max_interpolated_accel_ ? "accepted"
                                                                           : "rejected");

    bool standard_criteria =
      frame.interpolated_time_ < params_.calibration_max_interpolated_time_ &&
      frame.interpolated_distance_ < params_.calibration_max_interpolated_distance_ &&
      frame.interpolated_angle_ < params_.calibration_max_interpolated_angle_ &&
      frame.estimated_speed_ < params_.calibration_max_interpolated_speed_ &&
      frame.estimated_accel_ < params_.calibration_max_interpolated_accel_;

    bool straight_criteria =
      frame.interpolated_time_ < params_.calibration_max_interpolated_time_ &&
      frame.interpolated_distance_ < params_.calibration_max_interpolated_distance_straight_ &&
      frame.interpolated_angle_ < params_.calibration_max_interpolated_angle_straight_ &&
      frame.estimated_speed_ < params_.calibration_max_interpolated_speed_straight_ &&
      frame.estimated_accel_ < params_.calibration_max_interpolated_accel_straight_;

    RCLCPP_INFO(
      get_logger(), "\t - standard criteria: %s", standard_criteria ? "accepted" : "rejected");
    RCLCPP_INFO(
      get_logger(), "\t - straight criteria: %s", straight_criteria ? "accepted" : "rejected");

    if (
      (standard_criteria || straight_criteria) &&
      (frame.stopped_ || !params_.calibration_use_only_stopped_)) {
      filtered_frames.emplace_back(frame);
    }
  }

  return filtered_frames;
}

std::vector<CalibrationFrame> ExtrinsicMappingBasedCalibrator::selectBestKCalibrationFrames(
  const std::vector<CalibrationFrame> & calibration_frames, int num_frames)
{
  std::vector<CalibrationFrame> filtered_frames;

  for (auto & frame : calibration_frames) {
    pcl::PCA<PointType> pca;
    pca.setInputCloud(frame.source_pointcloud_);

    float pca_coefficient = std::sqrt(std::abs(pca.getEigenValues().z()));

    RCLCPP_INFO(
      get_logger(), "\t - pca coeff: %.4f (%s)", pca_coefficient,
      pca_coefficient >= params_.calibration_min_pca_eigenvalue_ ? "accepted" : "rejected");

    if (pca_coefficient >= params_.calibration_min_pca_eigenvalue_) {
      filtered_frames.push_back(frame);
    }
  }

  if (int(filtered_frames.size()) <= num_frames) {
    return filtered_frames;
  }

  std::vector<bool> mask;
  std::vector<CalibrationFrame> accepted_frames;

  mask.resize(filtered_frames.size());
  std::fill(mask.begin(), mask.end(), false);

  // Insert a random element from filtered to accepted
  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_int_distribution<std::mt19937::result_type> dist(0, filtered_frames.size() - 1);
  accepted_frames.push_back(filtered_frames[dist(rng)]);

  for (int k = 1; k < num_frames; k++) {
    int best_candidate_frame_id = -1;
    float best_candidate_frame_distance = 0.f;
    for (std::size_t i = 0; i < filtered_frames.size(); i++) {
      if (mask[i]) {
        continue;
      }

      auto & candidate_frame = filtered_frames[i];
      float min_distance = std::numeric_limits<float>::max();

      for (auto & accepted_frame : accepted_frames) {
        float distance = std::abs(
          candidate_frame.target_frame_->distance_ - accepted_frame.target_frame_->distance_);
        min_distance = std::min(min_distance, distance);
      }

      if (min_distance > best_candidate_frame_distance) {
        best_candidate_frame_id = i;
        best_candidate_frame_distance = min_distance;
      }
    }

    accepted_frames.push_back(filtered_frames[best_candidate_frame_id]);
    mask[best_candidate_frame_id] = true;
  }

  assert(int(accepted_frames.size()) == num_frames);
  return accepted_frames;
}

void ExtrinsicMappingBasedCalibrator::keyFrameCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_);

  int requested_keyframe_id = request->id;

  if (requested_keyframe_id < 0 || requested_keyframe_id >= static_cast<int>(keyframes_.size())) {
    RCLCPP_ERROR(get_logger(), "Invalid keyframe id");
    return;
  }

  Frame::Ptr & keyframe = keyframes_[requested_keyframe_id];

  RCLCPP_INFO(
    get_logger(), "keyFrameCallback callback. Processing keyframe=%d", requested_keyframe_id);

  PointcloudType::Ptr subsampled_kcs_ptr =
    getDensePointcloudFromMap(keyframe->pose_, keyframe, params_.leaf_size_dense_map_, 5.0);
  PointcloudType::Ptr cropped_scan_kcd_ptr = cropPointCloud(keyframe->pointcloud_raw_, 50.0);

  RCLCPP_INFO(
    get_logger(), "keyFrameCallback callback. map points=%lu", subsampled_kcs_ptr->size());

  response->success = true;
}

#pragma GCC push_options
#pragma GCC optimize("O0")

void ExtrinsicMappingBasedCalibrator::singleLidarCalibrationCallback(
  const std::string & calibration_frame_name,
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_);

  Eigen::Matrix4f initial_calibration_transform;
  float initial_distance;

  // Get the tf between frames
  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    geometry_msgs::msg::Transform initial_target_to_source_msg;
    Eigen::Affine3d initial_target_to_source_affine;

    initial_target_to_source_msg =
      tf_buffer_->lookupTransform(mapping_lidar_frame_, calibration_frame_name, t, timeout)
        .transform;

    initial_target_to_source_affine = tf2::transformToEigen(initial_target_to_source_msg);
    initial_distance = initial_target_to_source_affine.translation().norm();
    initial_calibration_transform = initial_target_to_source_affine.matrix().cast<float>();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "could not get initial tf. %s", ex.what());
    return;
  }

  auto & calibration_frames = calibration_frames_map_[calibration_frame_name];
  auto & initial_source_aligned_map_pub =
    initial_source_aligned_map_pub_map_[calibration_frame_name];
  auto & calibrated_source_aligned_map_pub =
    calibrated_source_aligned_map_pub_map_[calibration_frame_name];
  auto & target_map_pub = target_map_pub_map_[calibration_frame_name];

  std::vector<CalibrationFrame> filtered_calibration_frames =
    filterCalibrationFramesByDynamics(calibration_frames);

  if (request->id >= static_cast<int>(filtered_calibration_frames.size())) {
    RCLCPP_WARN(
      this->get_logger(), "Invalid requested calibration frame. size=%lu",
      filtered_calibration_frames.size());
    return;
  }

  CalibrationFrame & calibration_frame = filtered_calibration_frames[request->id];
  PointcloudType::Ptr source_pc_ptr =
    cropPointCloud(calibration_frame.source_pointcloud_, params_.max_calibration_range_);

  PointcloudType::Ptr target_dense_pc_ptr = getDensePointcloudFromMap(
    calibration_frame.target_frame_->pose_, calibration_frame.target_frame_,
    params_.leaf_size_dense_map_, params_.max_calibration_range_ + initial_distance);
  PointcloudType::Ptr target_thin_pc_ptr = getDensePointcloudFromMap(
    calibration_frame.target_frame_->pose_, calibration_frame.target_frame_,
    params_.leaf_size_local_map_, params_.max_calibration_range_ + initial_distance);

  PointcloudType::Ptr initial_source_aligned_pc_ptr(new PointcloudType());
  pcl::transformPointCloud(
    *source_pc_ptr, *initial_source_aligned_pc_ptr, initial_calibration_transform);

  // Evaluate the initial calibration
  setUpCalibrators(source_pc_ptr, target_dense_pc_ptr);

  // Crop unused areas of the target pointcloud to save processing time
  cropTargetPointcloud<PointType>(
    initial_source_aligned_pc_ptr, target_dense_pc_ptr, initial_distance);
  cropTargetPointcloud<PointType>(
    initial_source_aligned_pc_ptr, target_thin_pc_ptr, initial_distance);

  correspondence_estimator_->setInputSource(initial_source_aligned_pc_ptr);
  correspondence_estimator_->setInputTarget(target_dense_pc_ptr);
  double initial_score = sourceTargetDistance(*correspondence_estimator_);

  RCLCPP_WARN(
    this->get_logger(),
    "Initial calibration score = %.4f (avg.squared.dist) | sqrt.score = %.4f m | discretization = "
    "%.4f m",
    initial_score, std::sqrt(initial_score), params_.leaf_size_dense_map_);

  // Find best calibration using an "ensemble" of calibrators
  std::vector<Eigen::Matrix4f> candidate_transforms = {initial_calibration_transform};
  Eigen::Matrix4f best_transform;
  float best_score;

  findBestTransform<pcl::Registration<PointType, PointType>, PointType>(
    candidate_transforms, calibration_registrators_, params_.verbose_, best_transform, best_score);

  PointcloudType::Ptr calibrated_source_aligned_pc_ptr(new PointcloudType());
  pcl::transformPointCloud(*source_pc_ptr, *calibrated_source_aligned_pc_ptr, best_transform);

  RCLCPP_WARN(
    this->get_logger(),
    "Best calibration score = %.4f (avg.squared.dist) | sqrt.score = %.4f m | discretization = "
    "%.4f m",
    best_score, std::sqrt(best_score), params_.leaf_size_dense_map_);

  PointcloudType::Ptr test_aligned_pc_ptr(new PointcloudType());
  pcl::transformPointCloud(*source_pc_ptr, *test_aligned_pc_ptr, best_transform);

  correspondence_estimator_->setInputSource(test_aligned_pc_ptr);
  correspondence_estimator_->setInputTarget(target_dense_pc_ptr);
  double test_score = sourceTargetDistance(*correspondence_estimator_);

  RCLCPP_WARN(
    this->get_logger(),
    "Test calibration score = %.4f (avg.squared.dist) | sqrt.score = %.4f m | discretization = "
    "%.4f m",
    test_score, std::sqrt(test_score), params_.leaf_size_dense_map_);

  // Output ROS data
  PointcloudType::Ptr initial_source_aligned_map_ptr(new PointcloudType());
  PointcloudType::Ptr calibrated_source_aligned_map_ptr(new PointcloudType());
  PointcloudType::Ptr target_thin_map_ptr(new PointcloudType());
  pcl::transformPointCloud(
    *initial_source_aligned_pc_ptr, *initial_source_aligned_map_ptr,
    calibration_frame.target_frame_->pose_);
  pcl::transformPointCloud(
    *calibrated_source_aligned_pc_ptr, *calibrated_source_aligned_map_ptr,
    calibration_frame.target_frame_->pose_);
  pcl::transformPointCloud(
    *target_thin_pc_ptr, *target_thin_map_ptr, calibration_frame.target_frame_->pose_);

  sensor_msgs::msg::PointCloud2 initial_source_aligned_map_msg, calibrated_source_aligned_map_msg,
    target_map_msg;
  initial_source_aligned_pc_ptr->width = initial_source_aligned_pc_ptr->points.size();
  initial_source_aligned_pc_ptr->height = 1;
  calibrated_source_aligned_pc_ptr->width = calibrated_source_aligned_pc_ptr->points.size();
  calibrated_source_aligned_pc_ptr->height = 1;
  target_thin_pc_ptr->width = target_thin_pc_ptr->points.size();
  target_thin_pc_ptr->height = 1;

  pcl::toROSMsg(*initial_source_aligned_map_ptr, initial_source_aligned_map_msg);
  pcl::toROSMsg(*calibrated_source_aligned_map_ptr, calibrated_source_aligned_map_msg);
  pcl::toROSMsg(*target_thin_map_ptr, target_map_msg);

  initial_source_aligned_map_msg.header = calibration_frame.source_header_;
  initial_source_aligned_map_msg.header.frame_id = map_frame_;
  calibrated_source_aligned_map_msg.header = calibration_frame.source_header_;
  calibrated_source_aligned_map_msg.header.frame_id = map_frame_;
  target_map_msg.header = calibration_frame.target_frame_->header_;
  target_map_msg.header.frame_id = map_frame_;

  initial_source_aligned_map_pub->publish(initial_source_aligned_map_msg);
  calibrated_source_aligned_map_pub->publish(calibrated_source_aligned_map_msg);
  target_map_pub->publish(target_map_msg);

  response->success = true;
}

void ExtrinsicMappingBasedCalibrator::multipleLidarCalibrationCallback(
  const std::string & calibration_frame_name,
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response)
{
  UNUSED(request);
  std::unique_lock<std::mutex> lock(mutex_);

  Eigen::Matrix4f initial_calibration_transform;
  float initial_distance;

  RCLCPP_INFO(this->get_logger(), "Calibrating frame: %s", calibration_frame_name.c_str());
  RCLCPP_INFO(this->get_logger(), "Obtaining initial calibration...");

  // Get the tf between frames
  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    geometry_msgs::msg::Transform initial_target_to_source_msg;
    Eigen::Affine3d initial_target_to_source_affine;

    initial_target_to_source_msg =
      tf_buffer_->lookupTransform(mapping_lidar_frame_, calibration_frame_name, t, timeout)
        .transform;

    initial_target_to_source_affine = tf2::transformToEigen(initial_target_to_source_msg);
    initial_distance = initial_target_to_source_affine.translation().norm();
    initial_calibration_transform = initial_target_to_source_affine.matrix().cast<float>();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "could not get initial tf. %s", ex.what());
    response->success = false;
    return;
  }

  auto & calibration_frames = calibration_frames_map_[calibration_frame_name];
  auto & initial_source_aligned_map_pub =
    initial_source_aligned_map_pub_map_[calibration_frame_name];
  auto & calibrated_source_aligned_map_pub =
    calibrated_source_aligned_map_pub_map_[calibration_frame_name];
  // auto & target_map_pub = target_map_pub_map_[calibration_frame_name];

  RCLCPP_INFO(this->get_logger(), "Original calibration frames: %ld", calibration_frames.size());

  std::vector<CalibrationFrame> filtered_calibration_frames_by_interpolation =
    filterCalibrationFramesByInterpolationError(calibration_frames);

  RCLCPP_INFO(
    this->get_logger(), "Filtered calibration frames by dynamics: %ld",
    filtered_calibration_frames_by_interpolation.size());

  std::vector<CalibrationFrame> filtered_calibration_frames_by_dynamics =
    filterCalibrationFramesByDynamics(filtered_calibration_frames_by_interpolation);

  RCLCPP_INFO(
    this->get_logger(), "Filtered calibration frames by dynamics: %ld",
    filtered_calibration_frames_by_dynamics.size());

  std::vector<CalibrationFrame> accepted_calibration_frames = selectBestKCalibrationFrames(
    filtered_calibration_frames_by_dynamics, params_.calibration_max_frames_);

  RCLCPP_INFO(
    this->get_logger(), "Accepted calibration frames: %ld", accepted_calibration_frames.size());

  std::vector<pcl::PointCloud<PointType>::Ptr> sources, targets;
  RCLCPP_INFO(this->get_logger(), "Preparing dense calibration pointclouds from the map...");

  // Prepare pointclouds for calibration
  for (auto & calibration_frame : accepted_calibration_frames) {
    PointcloudType::Ptr source_pc_ptr =
      cropPointCloud(calibration_frame.source_pointcloud_, params_.max_calibration_range_);

    PointcloudType::Ptr target_pc_ptr = getDensePointcloudFromMap(
      calibration_frame.target_frame_->pose_, calibration_frame.target_frame_,
      params_.leaf_size_dense_map_, params_.max_calibration_range_ + initial_distance);

    // Transfor the source to target frame to crop it later
    PointcloudType::Ptr initial_source_aligned_pc_ptr(new PointcloudType());
    pcl::transformPointCloud(
      *source_pc_ptr, *initial_source_aligned_pc_ptr, initial_calibration_transform);

    // Crop unused areas of the target pointcloud to save processing time
    cropTargetPointcloud<PointType>(initial_source_aligned_pc_ptr, target_pc_ptr, initial_distance);

    sources.push_back(source_pc_ptr);
    targets.push_back(target_pc_ptr);
  }

  // Set all the registrators with the pointclouds
  for (auto & calibrator : calibration_batch_registrators_) {
    calibrator->clearInputSources();
    calibrator->clearInputTargets();

    for (std::size_t i = 0; i < sources.size(); i++) {
      calibrator->addInputSource(sources[i]);
      calibrator->addInputTarget(targets[i]);
    }
  }

  // Perform single-frame calibration for all frames
  std::vector<float> initial_calibration_single_frame_score(accepted_calibration_frames.size());
  std::vector<float> single_frame_calibration_multi_frame_score(accepted_calibration_frames.size());
  std::vector<float> single_frame_calibration_single_frame_score(
    accepted_calibration_frames.size());
  std::vector<Eigen::Matrix4f> best_single_frame_transforms(accepted_calibration_frames.size());

  RCLCPP_INFO(this->get_logger(), "Calibration using single frames...");
  for (std::size_t i = 0; i < accepted_calibration_frames.size(); i++) {
    // start

    PointcloudType::Ptr initial_source_aligned_pc_ptr(new PointcloudType());
    pcl::transformPointCloud(
      *sources[i], *initial_source_aligned_pc_ptr, initial_calibration_transform);

    // Evaluate the initial calibration
    // setUpCalibrators(sources[i], targets[i]);
    for (auto & calibrator : calibration_registrators_) {
      calibrator->setInputSource(sources[i]);
      calibrator->setInputTarget(targets[i]);
    }

    correspondence_estimator_->setInputSource(initial_source_aligned_pc_ptr);
    correspondence_estimator_->setInputTarget(targets[i]);
    double initial_score = sourceTargetDistance(*correspondence_estimator_);

    // Find best calibration using an "ensemble" of calibrators
    std::vector<Eigen::Matrix4f> candidate_transforms = {initial_calibration_transform};
    Eigen::Matrix4f best_single_frame_transform;
    float best_single_frame_transform_score;

    findBestTransform<pcl::Registration<PointType, PointType>, PointType>(
      candidate_transforms, calibration_registrators_, params_.verbose_,
      best_single_frame_transform, best_single_frame_transform_score);

    float best_single_frame_transform_multi_frame_score =
      sourceTargetDistance<PointType>(sources, targets, best_single_frame_transform);

    initial_calibration_single_frame_score[i] = initial_score;
    single_frame_calibration_multi_frame_score[i] = best_single_frame_transform_multi_frame_score;
    single_frame_calibration_single_frame_score[i] = best_single_frame_transform_score;
    best_single_frame_transforms[i] = best_single_frame_transform;

    RCLCPP_INFO(
      this->get_logger(),
      "Calibration frame=%ld Keyframe=%d Initial single-frame score= %.4f Single-frame score=%.4f "
      "Multi-frame score=%.4f",
      i, accepted_calibration_frames[i].target_frame_->keyframe_id_, initial_score,
      best_single_frame_transform_score, best_single_frame_transform_multi_frame_score);
    // finish
  }

  // Choose the best sigle-frame calibration
  std::vector<float>::iterator best_single_frame_calibration_multi_frame_score_it =
    std::min_element(
      std::begin(single_frame_calibration_multi_frame_score),
      std::end(single_frame_calibration_multi_frame_score));
  int best_single_frame_calibration_multi_frame_score_index = std::distance(
    std::begin(single_frame_calibration_multi_frame_score),
    best_single_frame_calibration_multi_frame_score_it);

  RCLCPP_INFO(
    this->get_logger(),
    "Single-frame calibration results:\n  Best Calibration frame=%d Keyframe=%d Multi-frame "
    "score=%.4f",
    best_single_frame_calibration_multi_frame_score_index,
    accepted_calibration_frames[best_single_frame_calibration_multi_frame_score_index]
      .target_frame_->keyframe_id_,
    *best_single_frame_calibration_multi_frame_score_it);

  RCLCPP_INFO(this->get_logger(), "Calibration using multiple frames...");
  std::vector<Eigen::Matrix4f> candidate_transforms = {
    initial_calibration_transform,
    best_single_frame_transforms[best_single_frame_calibration_multi_frame_score_index]};
  Eigen::Matrix4f best_multi_frame_calibration_transform;
  float best_multi_frame_calibration_multi_frame_score;

  findBestTransform<pcl::JointIterativeClosestPointExtended<PointType, PointType>, PointType>(
    candidate_transforms, calibration_batch_registrators_, params_.verbose_,
    best_multi_frame_calibration_transform, best_multi_frame_calibration_multi_frame_score);

  float initial_score =
    sourceTargetDistance<PointType>(sources, targets, initial_calibration_transform);
  float final_score =
    sourceTargetDistance<PointType>(sources, targets, best_multi_frame_calibration_transform);

  RCLCPP_WARN(
    this->get_logger(),
    "Initial calibration score = %.4f (avg.squared.dist) | sqrt.score = %.4f m | discretization = "
    "%.4f m",
    initial_score, std::sqrt(initial_score), params_.leaf_size_dense_map_);
  RCLCPP_WARN(
    this->get_logger(),
    "Best calibration score = %.4f (avg.squared.dist) | sqrt.score = %.4f m | discretization = "
    "%.4f m",
    final_score, std::sqrt(final_score), params_.leaf_size_dense_map_);
  RCLCPP_WARN(
    this->get_logger(),
    "Test calibration score = %.4f (avg.squared.dist) | sqrt.score = %.4f m | discretization = "
    "%.4f m",
    best_multi_frame_calibration_multi_frame_score,
    std::sqrt(best_multi_frame_calibration_multi_frame_score), params_.leaf_size_dense_map_);

  // Publish ROS data
  PointcloudType::Ptr initial_source_aligned_map_ptr(new PointcloudType());
  PointcloudType::Ptr calibrated_source_aligned_map_ptr(new PointcloudType());

  for (std::size_t i = 0; i < accepted_calibration_frames.size(); i++) {
    PointcloudType::Ptr initial_tmp_ptr(new PointcloudType());
    PointcloudType::Ptr calibrated_tmp_ptr(new PointcloudType());

    pcl::transformPointCloud(
      *sources[i], *initial_tmp_ptr,
      accepted_calibration_frames[i].target_frame_->pose_ * initial_calibration_transform);
    pcl::transformPointCloud(
      *sources[i], *calibrated_tmp_ptr,
      accepted_calibration_frames[i].target_frame_->pose_ * best_multi_frame_calibration_transform);

    *initial_source_aligned_map_ptr += *initial_tmp_ptr;
    *calibrated_source_aligned_map_ptr += *calibrated_tmp_ptr;
  }

  sensor_msgs::msg::PointCloud2 initial_source_aligned_map_msg, calibrated_source_aligned_map_msg;
  initial_source_aligned_map_ptr->width = initial_source_aligned_map_ptr->points.size();
  initial_source_aligned_map_ptr->height = 1;
  calibrated_source_aligned_map_ptr->width = calibrated_source_aligned_map_ptr->points.size();
  calibrated_source_aligned_map_ptr->height = 1;

  pcl::toROSMsg(*initial_source_aligned_map_ptr, initial_source_aligned_map_msg);
  pcl::toROSMsg(*calibrated_source_aligned_map_ptr, calibrated_source_aligned_map_msg);

  initial_source_aligned_map_msg.header = accepted_calibration_frames[0].source_header_;
  initial_source_aligned_map_msg.header.frame_id = map_frame_;
  calibrated_source_aligned_map_msg.header = accepted_calibration_frames[0].source_header_;
  calibrated_source_aligned_map_msg.header.frame_id = map_frame_;

  initial_source_aligned_map_pub->publish(initial_source_aligned_map_msg);
  calibrated_source_aligned_map_pub->publish(calibrated_source_aligned_map_msg);

  response->success = false;
}

#pragma GCC pop_options

void ExtrinsicMappingBasedCalibrator::loadDatabaseCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Response> response)
{
  std::ifstream ifs(request->path);
  boost::archive::binary_iarchive ia(ifs);

  calibration_frames_map_.clear();
  processed_frames_.clear();
  keyframes_.clear();

  ia >> calibration_frames_map_;
  for (auto it = calibration_frames_map_.begin(); it != calibration_frames_map_.end(); it++) {
    RCLCPP_INFO(
      this->get_logger(), "Loaded %ld calibration frames (%s)...", it->second.size(),
      it->first.c_str());
  }

  ia >> processed_frames_;
  RCLCPP_INFO(this->get_logger(), "Loaded %ld frames...", processed_frames_.size());

  ia >> keyframes_;
  RCLCPP_INFO(this->get_logger(), "Loaded %ld keyframes...", keyframes_.size());

  ia >> mapping_lidar_header_;

  response->success = true;
}

void ExtrinsicMappingBasedCalibrator::saveDatabaseCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Response> response)
{
  std::ofstream ofs(request->path);
  boost::archive::binary_oarchive oa(ofs);

  for (auto it = calibration_frames_map_.begin(); it != calibration_frames_map_.end(); it++) {
    RCLCPP_INFO(
      this->get_logger(), "Saving %ld calibration frames (%s)...", it->second.size(),
      it->first.c_str());
  }
  oa << calibration_frames_map_;

  RCLCPP_INFO(this->get_logger(), "Saving %ld frames...", processed_frames_.size());
  oa << processed_frames_;

  RCLCPP_INFO(this->get_logger(), "Saving %ld keyframes...", keyframes_.size());
  oa << keyframes_;

  oa << mapping_lidar_header_;

  response->success = true;
}
