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

#include <Eigen/Core>
#include <extrinsic_tag_based_base_calibrator/calibration_scene_extractor.hpp>
#include <extrinsic_tag_based_base_calibrator/ceres/tag_reprojection_error.hpp>
#include <extrinsic_tag_based_base_calibrator/extrinsic_tag_based_base_calibrator.hpp>
#include <extrinsic_tag_based_base_calibrator/intrinsics_calibrator.hpp>
#include <extrinsic_tag_based_base_calibrator/math.hpp>
#include <extrinsic_tag_based_base_calibrator/serialization.hpp>
#include <extrinsic_tag_based_base_calibrator/visualization.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <ceres/ceres.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>

#define UNUSED(x) (void)x;

namespace extrinsic_tag_based_base_calibrator
{

ExtrinsicTagBasedBaseCalibrator::ExtrinsicTagBasedBaseCalibrator(
  const rclcpp::NodeOptions & options)
: Node("extrinsic_tag_based_base_calibrator_node", options),
  data_(std::make_shared<CalibrationData>())
{
  waypoint_tag_size_ = this->declare_parameter<double>("waypoint_tag_size", 0.8);
  wheel_tag_size_ = this->declare_parameter<double>("wheel_tag_size", 0.8);
  ground_tag_size_ = this->declare_parameter<double>("ground_tag_size", 0.135);

  calibration_sensor_type_ =
    this->declare_parameter<std::string>("calibration_sensor_type", "camera");

  bool ba_optimize_intrinsics_ = this->declare_parameter<bool>("ba_optimize_intrinsics", false);
  bool ba_share_intrinsics_ = this->declare_parameter<bool>("ba_share_intrinsics", false);
  bool ba_force_shared_ground_plane_ =
    this->declare_parameter<bool>("ba_force_shared_ground_plane", false);

  calibration_problem_.setOptimizeIntrinsics(ba_optimize_intrinsics_);
  calibration_problem_.setShareIntrinsics(ba_share_intrinsics_);
  calibration_problem_.setForceSharedGroundPlane(ba_force_shared_ground_plane_);

  std::vector<int64_t> waypoint_tag_ids =
    this->declare_parameter<std::vector<int64_t>>("waypoint_tag_ids", std::vector<int64_t>{0});

  std::transform(
    waypoint_tag_ids.begin(), waypoint_tag_ids.end(), std::back_inserter(waypoint_tag_ids_),
    [](const auto id) { return static_cast<int>(id); });

  left_wheel_tag_id_ = this->declare_parameter<int>("left_wheel_tag_id", 3);
  right_wheel_tag_id_ = this->declare_parameter<int>("right_wheel_tag_id", 4);
  wheel_tag_ids_ = std::vector<int>{left_wheel_tag_id_, right_wheel_tag_id_};

  // std::vector<int64_t> ground_tag_ids = this->declare_parameter<std::vector<int64_t>>(
  //   "ground_tag_ids", std::vector<int64_t>{/*5, 6,*/ 7, /*8, */9, 10, /*11, 12, */ 13, 14, 15});

  std::vector<int64_t> ground_tag_ids = this->declare_parameter<std::vector<int64_t>>(
    "ground_tag_ids", std::vector<int64_t>{7, 10, 12, 13, 14, 15});

  std::transform(
    ground_tag_ids.begin(), ground_tag_ids.end(), std::back_inserter(ground_tag_ids_),
    [](auto & id) { return static_cast<int>(id); });

  std::vector<int64_t> intrinsic_calibration_tag_ids =
    this->declare_parameter<std::vector<int64_t>>(
      "intrinsic_calibration_tag_ids",
      std::vector<int64_t>{0, 3, 4, 7, 13});  // , 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
  std::transform(
    intrinsic_calibration_tag_ids.begin(), intrinsic_calibration_tag_ids.end(),
    std::back_inserter(intrinsic_calibration_tag_ids_),
    [](auto & id) { return static_cast<int>(id); });

  for (const auto & id : waypoint_tag_ids_) {
    waypoint_tag_ids_set_.insert(id);
    tag_size_map_[id] = waypoint_tag_size_;
  }

  for (const auto & id : wheel_tag_ids_) {
    wheel_tag_ids_set_.insert(id);
    tag_size_map_[id] = wheel_tag_size_;
  }

  for (const auto & id : ground_tag_ids_) {
    ground_tag_ids_set_.insert(id);
    tag_size_map_[id] = ground_tag_size_;
  }

  calibration_problem_.setTagIds(
    waypoint_tag_ids_, ground_tag_ids_, left_wheel_tag_id_, right_wheel_tag_id_);

  apriltag_parameters_.family = this->declare_parameter<std::string>("apriltag_family", "16h5");
  apriltag_parameters_.max_hamming = this->declare_parameter<int>("apriltag_max_hamming", 0);
  apriltag_parameters_.min_margin = this->declare_parameter<double>("apriltag_min_margin", 20.0);
  apriltag_parameters_.max_h_error =
    this->declare_parameter<double>("apriltag_max_h_error", 10000.0);
  apriltag_parameters_.quad_decimate =
    this->declare_parameter<double>("apriltag_quad_decimate", 1.0);
  apriltag_parameters_.quad_sigma = this->declare_parameter<double>("apriltag_quad_sigma", 0.0);
  apriltag_parameters_.nthreads = this->declare_parameter<int>("apriltag_nthreads", 1);
  apriltag_parameters_.debug = this->declare_parameter<bool>("apriltag_debug", false);
  apriltag_parameters_.refine_edges = this->declare_parameter<bool>("apriltag_refine_edges", true);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);

  visualization_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(1),
    std::bind(&ExtrinsicTagBasedBaseCalibrator::visualizationTimerCallback, this));

  // Scene related services
  add_scene_srv_ = this->create_service<std_srvs::srv::Empty>(
    "add_scene", std::bind(
                   &ExtrinsicTagBasedBaseCalibrator::addSceneCallback, this, std::placeholders::_1,
                   std::placeholders::_2));

  add_external_camera_images_srv_ = this->create_service<tier4_calibration_msgs::srv::Files>(
    "add_external_camera_images_to_scene",
    std::bind(
      &ExtrinsicTagBasedBaseCalibrator::addExternalCameraImagesCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  add_lidar_detections_to_scene_srv_ = this->create_service<std_srvs::srv::Empty>(
    "add_calibration_lidar_detections_to_scene",
    std::bind(
      &ExtrinsicTagBasedBaseCalibrator::addLidarDetectionsCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  add_camera_detections_to_scene_srv_ = this->create_service<std_srvs::srv::Empty>(
    "add_calibration_camera_detections_to_scene",
    std::bind(
      &ExtrinsicTagBasedBaseCalibrator::addCameraDetectionsCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  add_calibration_camera_images_srv_ = this->create_service<tier4_calibration_msgs::srv::Files>(
    "add_calibration_camera_images_to_scene",
    std::bind(
      &ExtrinsicTagBasedBaseCalibrator::addCalibrationImagesCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  // Intrinsics realated services
  load_external_camera_intrinsics_srv_ = this->create_service<tier4_calibration_msgs::srv::Files>(
    "load_external_camera_intrinsics",
    std::bind(
      &ExtrinsicTagBasedBaseCalibrator::loadExternalIntrinsicsCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  save_external_camera_intrinsics_srv_ = this->create_service<tier4_calibration_msgs::srv::Files>(
    "save_external_camera_intrinsics",
    std::bind(
      &ExtrinsicTagBasedBaseCalibrator::saveExternalIntrinsicsCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  calibrate_external_camera_intrinsics_srv_ =
    this->create_service<tier4_calibration_msgs::srv::Files>(
      "calibrate_external_camera_intrinsics",
      std::bind(
        &ExtrinsicTagBasedBaseCalibrator::calibrateExternalIntrinsicsCallback, this,
        std::placeholders::_1, std::placeholders::_2));

  load_calibration_camera_intrinsics_srv_ =
    this->create_service<tier4_calibration_msgs::srv::Files>(
      "load_calibration_camera_intrinsics",
      std::bind(
        &ExtrinsicTagBasedBaseCalibrator::loadCalibrationIntrinsicsCallback, this,
        std::placeholders::_1, std::placeholders::_2));
  save_calibration_camera_intrinsics_srv_ =
    this->create_service<tier4_calibration_msgs::srv::Files>(
      "save_calibration_camera_intrinsics",
      std::bind(
        &ExtrinsicTagBasedBaseCalibrator::saveCalibrationIntrinsicsCallback, this,
        std::placeholders::_1, std::placeholders::_2));
  calibrate_calibration_camera_intrinsics_srv_ =
    this->create_service<tier4_calibration_msgs::srv::Files>(
      "calibrate_calibration_camera_intrinsics",
      std::bind(
        &ExtrinsicTagBasedBaseCalibrator::calibrateCalibrationIntrinsicsCallback, this,
        std::placeholders::_1, std::placeholders::_2));

  // Calibration related services
  process_scenes_srv_ = this->create_service<std_srvs::srv::Empty>(
    "process_scenes", std::bind(
                        &ExtrinsicTagBasedBaseCalibrator::preprocessScenesCallback, this,
                        std::placeholders::_1, std::placeholders::_2));
  calibration_srv_ = this->create_service<std_srvs::srv::Empty>(
    "calibrate", std::bind(
                   &ExtrinsicTagBasedBaseCalibrator::calibrationCallback, this,
                   std::placeholders::_1, std::placeholders::_2));

  // Calibration related services
  load_database_srv_ = this->create_service<tier4_calibration_msgs::srv::Files>(
    "load_database", std::bind(
                       &ExtrinsicTagBasedBaseCalibrator::loadDatabaseCallback, this,
                       std::placeholders::_1, std::placeholders::_2));
  save_database_srv_ = this->create_service<tier4_calibration_msgs::srv::Files>(
    "save_database", std::bind(
                       &ExtrinsicTagBasedBaseCalibrator::saveDatabaseCallback, this,
                       std::placeholders::_1, std::placeholders::_2));
}

void ExtrinsicTagBasedBaseCalibrator::visualizationTimerCallback()
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker base_marker;
  base_marker.header.frame_id = "/calibration_sensor";
  base_marker.ns = "raw_detections";
  next_color_index_ = 0;

  std::map<int, cv::Affine3d> sensor_to_waypoint_transform_map;

  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    const CalibrationScene & scene = data_->scenes[scene_index];

    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 1.0;
    color.a = 1.0;

    for (auto & detection : scene.calibration_sensor_detections) {
      sensor_to_waypoint_transform_map[detection.id] = detection.pose;

      addTagMarkers(
        markers, std::to_string(detection.id), detection.size, color, detection.pose, base_marker);
    }

    for (auto & frame : scene.external_camera_frames) {
      // Should choose a color per camera to debug
      std_msgs::msg::ColorRGBA color = getNextColor();

      std::vector<ApriltagDetection> waypoint_detections;
      std::vector<ApriltagDetection> wheel_detections;
      std::vector<ApriltagDetection> ground_detections;

      std::copy_if(
        frame.detections.begin(), frame.detections.end(), std::back_inserter(waypoint_detections),
        [this](const ApriltagDetection & detection) {
          return waypoint_tag_ids_set_.count(detection.id) > 0;
        });

      std::copy_if(
        frame.detections.begin(), frame.detections.end(), std::back_inserter(wheel_detections),
        [this](const ApriltagDetection & detection) {
          return wheel_tag_ids_set_.count(detection.id) > 0;
        });

      std::copy_if(
        frame.detections.begin(), frame.detections.end(), std::back_inserter(ground_detections),
        [this](const ApriltagDetection & detection) {
          return ground_tag_ids_set_.count(detection.id) > 0;
        });

      for (const auto & waypoint_detection : waypoint_detections) {
        cv::Affine3d sensor_to_waypoint_pose =
          sensor_to_waypoint_transform_map[waypoint_detection.id];
        const cv::Affine3d & external_camera_to_waypoint_pose = waypoint_detection.pose;

        cv::Affine3d sensor_to_external_camera_pose =
          sensor_to_waypoint_pose * external_camera_to_waypoint_pose.inv();

        addAxesMarkers(markers, 0.5, sensor_to_external_camera_pose, base_marker);
        addLineMarker(
          markers, color, sensor_to_waypoint_pose, sensor_to_external_camera_pose, base_marker);

        for (const auto & wheel_detection : wheel_detections) {
          const cv::Affine3d & external_camera_to_wheel_pose = wheel_detection.pose;
          cv::Affine3d sensor_to_wheel_pose =
            sensor_to_external_camera_pose * external_camera_to_wheel_pose;

          addTagMarkers(
            markers, std::to_string(wheel_detection.id), wheel_detection.size, color,
            sensor_to_wheel_pose, base_marker);
          addLineMarker(
            markers, color, sensor_to_external_camera_pose, sensor_to_wheel_pose, base_marker);
        }

        for (const auto & ground_detection : ground_detections) {
          const cv::Affine3d & external_camera_to_ground_pose = ground_detection.pose;
          cv::Affine3d sensor_to_ground_transform = sensor_to_waypoint_pose *
                                                    external_camera_to_waypoint_pose.inv() *
                                                    external_camera_to_ground_pose;

          addTagMarkers(
            markers, std::to_string(ground_detection.id), ground_detection.size, color,
            sensor_to_ground_transform, base_marker);
          addLineMarker(
            markers, color, sensor_to_external_camera_pose, sensor_to_ground_transform,
            base_marker);
        }
      }
    }
  }

  // Initial estimations
  base_marker.ns = "initial_estimations";

  std_msgs::msg::ColorRGBA initial_estimations_color;
  initial_estimations_color.r = 1.f;
  initial_estimations_color.g = 0.f;
  initial_estimations_color.b = 0.f;
  initial_estimations_color.a = 1.f;

  for (auto it = data_->initial_tag_poses_map.begin(); it != data_->initial_tag_poses_map.end();
       it++) {
    const UID & tag_uid = it->first;
    auto & pose = it->second;

    addTagMarkers(
      markers, tag_uid.to_string(), tag_size_map_[tag_uid.tag_id], initial_estimations_color, *pose,
      base_marker);
  }

  for (auto it = data_->initial_external_camera_poses.begin();
       it != data_->initial_external_camera_poses.end(); it++) {
    const UID & camera_uid = it->first;
    auto & camera_pose = it->second;

    addTextMarker(
      markers, camera_uid.to_string(), initial_estimations_color, *camera_pose, base_marker);
    addAxesMarkers(markers, 0.5, *camera_pose, base_marker);

    // Iterate over all the detections of said camera
    for (const auto & detection : data_->scenes[camera_uid.scene_id]
                                    .external_camera_frames[camera_uid.frame_id]
                                    .detections) {
      UID detection_uid = waypoint_tag_ids_set_.count(detection.id) > 0
                            ? UID::makeWaypointUID(camera_uid.scene_id, detection.id)
                          : wheel_tag_ids_set_.count(detection.id)
                            ? UID::makeWheelTagUID(detection.id)
                            : UID::makeGroundTagUID(detection.id);

      addLineMarker(
        markers, initial_estimations_color, *camera_pose,
        *data_->initial_tag_poses_map[detection_uid], base_marker);
    }
  }

  // Initial estimations
  base_marker.ns = "optimized_estimations";

  std_msgs::msg::ColorRGBA optimized_estimations_color;
  optimized_estimations_color.r = 0.f;
  optimized_estimations_color.g = 1.f;
  optimized_estimations_color.b = 0.f;
  optimized_estimations_color.a = 1.f;

  for (auto it = data_->optimized_tag_poses_map.begin(); it != data_->optimized_tag_poses_map.end();
       it++) {
    const UID & tag_uid = it->first;
    auto & pose = it->second;

    addTagMarkers(
      markers, tag_uid.to_string(), tag_size_map_[tag_uid.tag_id], optimized_estimations_color,
      *pose, base_marker);
  }

  for (auto it = data_->optimized_external_camera_poses.begin();
       it != data_->optimized_external_camera_poses.end(); it++) {
    const UID & camera_uid = it->first;
    auto & camera_pose = it->second;

    addTextMarker(
      markers, camera_uid.to_string(), optimized_estimations_color, *camera_pose, base_marker);
    addAxesMarkers(markers, 0.5, *camera_pose, base_marker);

    // Iterate over all the detections of said camera
    for (const auto & detection : data_->scenes[camera_uid.scene_id]
                                    .external_camera_frames[camera_uid.frame_id]
                                    .detections) {
      UID detection_uid = waypoint_tag_ids_set_.count(detection.id) > 0
                            ? UID::makeWaypointUID(camera_uid.scene_id, detection.id)
                          : wheel_tag_ids_set_.count(detection.id)
                            ? UID::makeWheelTagUID(detection.id)
                            : UID::makeGroundTagUID(detection.id);

      addLineMarker(
        markers, optimized_estimations_color, *camera_pose,
        *data_->optimized_tag_poses_map[detection_uid], base_marker);
    }
  }

  visualization_msgs::msg::Marker initial_ground_base_marker = base_marker;
  initial_ground_base_marker.ns = "initial_ground_plane";
  visualization_msgs::msg::Marker optimized_ground_base_marker = base_marker;
  optimized_ground_base_marker.ns = "optimized_ground_plane";
  visualization_msgs::msg::Marker initial_base_link_base_marker = base_marker;
  initial_base_link_base_marker.ns = "initial_base_link";
  visualization_msgs::msg::Marker optimized_base_link_base_marker = base_marker;
  optimized_base_link_base_marker.ns = "optimized_base_link";

  cv::Affine3d initial_ground_pose, optimized_ground_pose;
  if (computeGroundPlane(data_->initial_ground_tag_poses, ground_tag_size_, initial_ground_pose)) {
    addGrid(markers, initial_ground_pose, 100, 0.2, initial_ground_base_marker);
    addAxesMarkers(markers, 0.5, initial_ground_pose, initial_ground_base_marker);

    if (data_->initial_left_wheel_tag_pose && data_->initial_right_wheel_tag_pose) {
      cv::Affine3d initial_base_link_pose = computeBaseLink(
        *data_->initial_left_wheel_tag_pose, *data_->initial_right_wheel_tag_pose,
        initial_ground_pose);
      addAxesMarkers(markers, 0.5, initial_base_link_pose, initial_base_link_base_marker);
    }
  }

  if (computeGroundPlane(
        data_->optimized_ground_tag_poses, ground_tag_size_, optimized_ground_pose)) {
    addGrid(markers, optimized_ground_pose, 100, 0.2, optimized_ground_base_marker);
    addAxesMarkers(markers, 1.0, optimized_ground_pose, optimized_ground_base_marker);

    if (data_->optimized_left_wheel_tag_pose && data_->optimized_right_wheel_tag_pose) {
      cv::Affine3d optimized_base_link_pose = computeBaseLink(
        *data_->optimized_left_wheel_tag_pose, *data_->optimized_right_wheel_tag_pose,
        optimized_ground_pose);
      addAxesMarkers(markers, 1.0, optimized_base_link_pose, optimized_base_link_base_marker);
    }
  }

  for (std::size_t marker_index = 0; marker_index < markers.markers.size(); marker_index++) {
    markers.markers[marker_index].id = marker_index;
  }

  markers_pub_->publish(markers);
}

std_msgs::msg::ColorRGBA ExtrinsicTagBasedBaseCalibrator::getNextColor()
{
  static std::vector<unsigned int> colors = {0x1f77b4, 0xff7f0e, 0x2ca02c, 0xd62728, 0x9467bd,
                                             0x8c564b, 0xe377c2, 0x7f7f7f, 0xbcbd22, 0x17becf};

  unsigned int hex = colors[next_color_index_];

  std_msgs::msg::ColorRGBA color;
  color.r = ((hex >> 16) & 255) / 255.0;
  color.g = ((hex >> 8) & 255) / 255.0;
  color.b = (hex & 255) / 255.0;
  color.a = 1.f;

  next_color_index_ = (next_color_index_ + 1) % colors.size();

  return color;
}

bool ExtrinsicTagBasedBaseCalibrator::addSceneCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  UNUSED(request);
  UNUSED(response);

  if (
    current_external_camera_images_.size() == 0 || current_calibration_camera_images_.size() != 1) {
    RCLCPP_ERROR(
      this->get_logger(),
      "The scene must contain at least one external calibration image and exactly one calibration "
      "image");
    return false;
  }

  scenes_external_camera_images_.push_back(current_external_camera_images_);
  scenes_calibration_camera_images_.push_back(current_calibration_camera_images_[0]);

  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::addExternalCameraImagesCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Adding external camera images to the current scene");

  if (request->files.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "We expected at least one image!");
    response->success = false;
    return false;
  }

  current_external_camera_images_ = request->files;

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::addLidarDetectionsCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  RCLCPP_ERROR(this->get_logger(), "Unimplemented!");
  UNUSED(request);
  UNUSED(response);
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::addCameraDetectionsCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  RCLCPP_ERROR(this->get_logger(), "Unimplemented!");
  UNUSED(request);
  UNUSED(response);
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::addCalibrationImagesCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Adding a calibration camera image to the current scene");

  if (request->files.size() != 1) {
    RCLCPP_ERROR(this->get_logger(), "We expected a single image!");
    response->success = false;
    return false;
  }

  current_calibration_camera_images_ = request->files;

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::loadExternalIntrinsicsCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Loading external camera intrinsics");

  if (!external_camera_intrinsics_.loadCalibration(request->files[0])) {
    RCLCPP_ERROR(this->get_logger(), "Could not load intrinsics");
    response->success = false;
    return false;
  }

  calibration_problem_.setExternalCameraIntrinsics(external_camera_intrinsics_);

  RCLCPP_INFO_STREAM(this->get_logger(), "k = " << external_camera_intrinsics_.camera_matrix);
  RCLCPP_INFO_STREAM(this->get_logger(), "d = " << external_camera_intrinsics_.dist_coeffs);

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::saveExternalIntrinsicsCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Saving external camera intrinsics");

  external_camera_intrinsics_.saveCalibration(request->files[0]);

  RCLCPP_INFO(this->get_logger(), "External camera intrinsics saved");

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::calibrateExternalIntrinsicsCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Calibrating external cameras intrinsics");

  IntrinsicsCalibrator external_camera_intrinsics_calibrator(
    apriltag_parameters_, intrinsic_calibration_tag_ids_, true);

  external_camera_intrinsics_calibrator.setCalibrationImageFiles(request->files);
  external_camera_intrinsics_calibrator.calibrate(external_camera_intrinsics_);
  calibration_problem_.setExternalCameraIntrinsics(external_camera_intrinsics_);

  RCLCPP_INFO_STREAM(this->get_logger(), "k = " << external_camera_intrinsics_.camera_matrix);
  RCLCPP_INFO_STREAM(this->get_logger(), "d = " << external_camera_intrinsics_.dist_coeffs);

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::loadCalibrationIntrinsicsCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Loading 'calibration sensor' intrinsics");

  if (!calibration_sensor_intrinsics_.loadCalibration(request->files[0])) {
    RCLCPP_ERROR(this->get_logger(), "Could not load intrinsics");
    response->success = false;
    return false;
  }

  calibration_problem_.setCalibrationSensorIntrinsics(calibration_sensor_intrinsics_);

  RCLCPP_INFO_STREAM(this->get_logger(), "k = " << calibration_sensor_intrinsics_.camera_matrix);
  RCLCPP_INFO_STREAM(this->get_logger(), "d = " << calibration_sensor_intrinsics_.dist_coeffs);

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::saveCalibrationIntrinsicsCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Saving 'calibration sensor' intrinsics");

  calibration_sensor_intrinsics_.saveCalibration(request->files[0]);

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::calibrateCalibrationIntrinsicsCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Calibrating 'calibration sensor' intrinsics");

  IntrinsicsCalibrator calibration_sensor_intrinsics_calibrator(
    apriltag_parameters_, intrinsic_calibration_tag_ids_, true);

  calibration_sensor_intrinsics_calibrator.setCalibrationImageFiles(request->files);
  calibration_sensor_intrinsics_calibrator.calibrate(calibration_sensor_intrinsics_);
  calibration_problem_.setCalibrationSensorIntrinsics(calibration_sensor_intrinsics_);

  RCLCPP_INFO_STREAM(this->get_logger(), "k = " << calibration_sensor_intrinsics_.camera_matrix);
  RCLCPP_INFO_STREAM(this->get_logger(), "d = " << calibration_sensor_intrinsics_.dist_coeffs);

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::preprocessScenesCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  UNUSED(request);
  UNUSED(response);

  RCLCPP_INFO(this->get_logger(), "Processing scenes...");

  CalibrationSceneExtractor calibration_scene_extractor(apriltag_parameters_);
  calibration_scene_extractor.setCalibrationSensorIntrinsics(calibration_sensor_intrinsics_);
  calibration_scene_extractor.setExternalCameraIntrinsics(external_camera_intrinsics_);

  calibration_scene_extractor.setWaypointTagSize(waypoint_tag_size_);
  calibration_scene_extractor.setWheelTagSize(wheel_tag_size_);
  calibration_scene_extractor.setGroundTagSize(ground_tag_size_);

  calibration_scene_extractor.setWaypointTagIds(waypoint_tag_ids_);
  calibration_scene_extractor.setLeftWheelTagId(left_wheel_tag_id_);
  calibration_scene_extractor.setRightWheelTagId(right_wheel_tag_id_);
  calibration_scene_extractor.setGroundTagIds(ground_tag_ids_);

  for (std::size_t i = 0; i < scenes_external_camera_images_.size(); i++) {
    CalibrationScene scene = calibration_scene_extractor.processScene(
      scenes_calibration_camera_images_[i], scenes_external_camera_images_[i]);

    data_->scenes.push_back(scene);
  }

  // Estimate the the initial poses for all the tags
  std::map<UID, std::vector<cv::Affine3d>> poses_vector_map;

  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    const CalibrationScene & scene = data_->scenes[scene_index];

    // Add the waypoints seen from the calibration sensor
    for (auto & detection : scene.calibration_sensor_detections) {
      UID waypoint_uid = UID::makeWaypointUID(scene_index, detection.id);
      poses_vector_map[waypoint_uid].push_back(detection.pose);
    }

    // Add the remaining tags and poses when possib;e
    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      auto & frame = scene.external_camera_frames[frame_id];
      std::vector<ApriltagDetection> waypoint_detections;
      std::vector<ApriltagDetection> wheel_detections;
      std::vector<ApriltagDetection> ground_detections;

      std::copy_if(
        frame.detections.begin(), frame.detections.end(), std::back_inserter(waypoint_detections),
        [this](const ApriltagDetection & detection) {
          return waypoint_tag_ids_set_.count(detection.id) > 0;
        });

      std::copy_if(
        frame.detections.begin(), frame.detections.end(), std::back_inserter(wheel_detections),
        [this](const ApriltagDetection & detection) {
          return wheel_tag_ids_set_.count(detection.id) > 0;
        });

      std::copy_if(
        frame.detections.begin(), frame.detections.end(), std::back_inserter(ground_detections),
        [this](const ApriltagDetection & detection) {
          return ground_tag_ids_set_.count(detection.id) > 0;
        });

      for (const auto & waypoint_detection : waypoint_detections) {
        UID waypoint_uid = UID::makeWaypointUID(scene_index, waypoint_detection.id);
        cv::Affine3d sensor_to_waypoint_pose = poses_vector_map[waypoint_uid].back();

        const cv::Affine3d & external_camera_to_waypoint_pose = waypoint_detection.pose;

        cv::Affine3d sensor_to_external_camera_pose =
          sensor_to_waypoint_pose * external_camera_to_waypoint_pose.inv();

        UID external_camera_uid = UID::makeCameraUID(scene_index, frame_id);

        poses_vector_map[external_camera_uid].push_back(sensor_to_external_camera_pose);

        for (const auto & wheel_detection : wheel_detections) {
          const cv::Affine3d & external_camera_to_wheel_pose = wheel_detection.pose;

          cv::Affine3d sensor_to_wheel_pose =
            sensor_to_external_camera_pose * external_camera_to_wheel_pose;

          UID wheel_tag_uid = UID::makeWheelTagUID(wheel_detection.id);
          poses_vector_map[wheel_tag_uid].push_back(sensor_to_wheel_pose);
        }

        for (const auto & ground_detection : ground_detections) {
          cv::Affine3d external_camera_to_ground_pose = ground_detection.pose;
          cv::Affine3d sensor_to_ground_pose = sensor_to_waypoint_pose *
                                               external_camera_to_waypoint_pose.inv() *
                                               external_camera_to_ground_pose;

          UID ground_tag_uid = UID::makeGroundTagUID(ground_detection.id);
          poses_vector_map[ground_tag_uid].push_back(sensor_to_ground_pose);
        }
      }
    }
  }

  // Some external cameras are not conected to the waypoints, so we make another pass
  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    const CalibrationScene & scene = data_->scenes[scene_index];

    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      // Need to make sure all the cameras are in the map
      UID external_camera_uid = UID::makeCameraUID(scene_index, frame_id);

      auto & frame = scene.external_camera_frames[frame_id];
      std::vector<ApriltagDetection> linked_detections;
      std::vector<ApriltagDetection> unlinked_wheel_detections;
      std::vector<ApriltagDetection> unlinked_ground_detections;

      std::copy_if(
        frame.detections.begin(), frame.detections.end(), std::back_inserter(linked_detections),
        [this, &poses_vector_map](const ApriltagDetection & detection) {
          UID wheel_tag_uid = UID::makeWheelTagUID(detection.id);
          UID ground_tag_uid = UID::makeGroundTagUID(detection.id);
          return poses_vector_map.count(wheel_tag_uid) > 0 ||
                 poses_vector_map.count(ground_tag_uid) > 0;
        });

      std::copy_if(
        frame.detections.begin(), frame.detections.end(),
        std::back_inserter(unlinked_wheel_detections),
        [this, &poses_vector_map](const ApriltagDetection & detection) {
          UID wheel_tag_uid = UID::makeWheelTagUID(detection.id);
          return wheel_tag_ids_set_.count(detection.id) > 0 &&
                 poses_vector_map.count(wheel_tag_uid) == 0;
        });

      std::copy_if(
        frame.detections.begin(), frame.detections.end(),
        std::back_inserter(unlinked_ground_detections),
        [this, &poses_vector_map](const ApriltagDetection & detection) {
          UID ground_tag_uid = UID::makeGroundTagUID(detection.id);
          return ground_tag_ids_set_.count(detection.id) > 0 &&
                 poses_vector_map.count(ground_tag_uid) == 0;
        });

      assert(linked_detections.size() > 0);

      if (poses_vector_map.count(external_camera_uid) == 0) {
        auto & linked_detection = linked_detections.front();

        UID wheel_tag_uid = UID::makeWheelTagUID(linked_detection.id);
        UID ground_tag_uid = UID::makeGroundTagUID(linked_detection.id);
        cv::Affine3d sensor_to_linked_tag = poses_vector_map.count(wheel_tag_uid) > 0
                                              ? poses_vector_map[wheel_tag_uid].front()
                                              : poses_vector_map[ground_tag_uid].front();

        const cv::Affine3d & external_camera_to_linked_tag_affine = linked_detection.pose;
        poses_vector_map[external_camera_uid].push_back(
          sensor_to_linked_tag * external_camera_to_linked_tag_affine.inv());
      }

      for (auto & unlinked_wheel_detection : unlinked_wheel_detections) {
        auto & linked_detection = linked_detections.front();

        UID unlinked_wheel_tag_uid = UID::makeWheelTagUID(unlinked_wheel_detection.id);
        UID linked_wheel_tag_uid = UID::makeWheelTagUID(linked_detection.id);
        UID linked_ground_tag_uid = UID::makeGroundTagUID(linked_detection.id);
        cv::Affine3d sensor_to_linked_tag = poses_vector_map.count(linked_wheel_tag_uid) > 0
                                              ? poses_vector_map[linked_wheel_tag_uid].front()
                                              : poses_vector_map[linked_ground_tag_uid].front();

        const cv::Affine3d & external_camera_to_linked_tag_affine = linked_detection.pose;
        const cv::Affine3d & external_camera_to_unlinked_tag_affine = unlinked_wheel_detection.pose;

        poses_vector_map[unlinked_wheel_tag_uid].push_back(
          sensor_to_linked_tag * external_camera_to_linked_tag_affine.inv() *
          external_camera_to_unlinked_tag_affine);
      }

      for (auto & unlinked_ground_detection : unlinked_ground_detections) {
        auto & linked_detection = linked_detections.front();

        UID unlinked_ground_tag_uid = UID::makeGroundTagUID(unlinked_ground_detection.id);
        UID linked_wheel_tag_uid = UID::makeWheelTagUID(linked_detection.id);
        UID linked_ground_tag_uid = UID::makeGroundTagUID(linked_detection.id);

        cv::Affine3d sensor_to_linked_tag = poses_vector_map.count(linked_wheel_tag_uid) > 0
                                              ? poses_vector_map[linked_wheel_tag_uid].front()
                                              : poses_vector_map[linked_ground_tag_uid].front();

        const cv::Affine3d & external_camera_to_linked_tag_affine = linked_detection.pose;
        const cv::Affine3d & external_camera_to_unlinked_tag_affine =
          unlinked_ground_detection.pose;

        poses_vector_map[unlinked_ground_tag_uid].push_back(
          sensor_to_linked_tag * external_camera_to_linked_tag_affine.inv() *
          external_camera_to_unlinked_tag_affine);
      }
    }
  }

  for (auto it = poses_vector_map.begin(); it != poses_vector_map.end(); it++) {
    std::cout << "UID: " << it->first.to_string() << "\t poses: " << it->second.size() << std::endl;
  }

  std::array<double, CalibrationData::INTRINSICS_DIM> initial_intrinsics;
  initial_intrinsics[0] = external_camera_intrinsics_.undistorted_camera_matrix(0, 2);
  initial_intrinsics[1] = external_camera_intrinsics_.undistorted_camera_matrix(1, 2);
  initial_intrinsics[2] = external_camera_intrinsics_.undistorted_camera_matrix(0, 0);
  initial_intrinsics[3] = external_camera_intrinsics_.undistorted_camera_matrix(1, 1);
  initial_intrinsics[4] = 0.0;
  initial_intrinsics[5] = 0.0;

  for (auto it = poses_vector_map.begin(); it != poses_vector_map.end(); it++) {
    const UID & uid = it->first;
    auto poses = it->second;

    Eigen::Vector3d avg_translation = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector4d> quats;

    for (auto & pose : poses) {
      Eigen::Vector3d translation;
      Eigen::Matrix3d rotation;
      cv::cv2eigen(pose.translation(), translation);
      cv::cv2eigen(pose.rotation(), rotation);
      Eigen::Quaterniond quat(rotation);
      quats.emplace_back(quat.w(), quat.x(), quat.y(), quat.z());

      avg_translation += translation;
    }

    avg_translation /= poses.size();
    Eigen::Vector4d avg_quat = quaternionAverage(quats);

    Eigen::Matrix3d avg_rotation =
      Eigen::Quaterniond(avg_quat(0), avg_quat(1), avg_quat(2), avg_quat(3)).toRotationMatrix();

    cv::Vec3d avg_pose_translation;
    cv::Matx33d avg_pose_rotation;
    cv::eigen2cv(avg_translation, avg_pose_translation);
    cv::eigen2cv(avg_rotation, avg_pose_rotation);

    auto initial_pose = std::make_shared<cv::Affine3d>(avg_pose_rotation, avg_pose_translation);
    (void)initial_pose;

    if (uid.is_tag) {
      data_->initial_tag_poses_map[uid] = initial_pose;

      if (uid.is_waypoint_tag) {
        data_->initial_waypoint_tag_poses.push_back(initial_pose);
      } else if (uid.is_ground_tag) {
        data_->initial_ground_tag_poses.push_back(initial_pose);
      } else if (uid.is_wheel_tag && uid.tag_id == left_wheel_tag_id_) {
        data_->initial_left_wheel_tag_pose = initial_pose;
      } else if (uid.is_wheel_tag && uid.tag_id == right_wheel_tag_id_) {
        data_->initial_right_wheel_tag_pose = initial_pose;
      }
    } else if (uid.is_camera) {
      data_->initial_external_camera_poses[uid] = initial_pose;
      data_->initial_external_camera_intrinsics[uid] =
        std::make_shared<std::array<double, CalibrationData::INTRINSICS_DIM>>(initial_intrinsics);
    }
  }

  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::calibrationCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  UNUSED(request);
  UNUSED(response);

  calibration_problem_.setData(data_);

  calibration_problem_.dataToPlaceholders();
  calibration_problem_.evaluate();
  calibration_problem_.solve();
  calibration_problem_.placeholdersToData();
  calibration_problem_.evaluate();
  calibration_problem_.writeDebugImages();
  RCLCPP_INFO(this->get_logger(), "Finished optimization");

  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::loadDatabaseCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Loading database...");
  std::ifstream ifs(request->files[0]);
  boost::archive::text_iarchive ia(ifs);

  ia >> data_;

  RCLCPP_INFO(this->get_logger(), "Database loaded");

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::saveDatabaseCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Saving database");
  std::ofstream ofs(request->files[0]);
  boost::archive::text_oarchive oa(ofs);

  oa << data_;

  RCLCPP_INFO(this->get_logger(), "Database saved");

  response->success = true;
  return true;
}

}  // namespace extrinsic_tag_based_base_calibrator
