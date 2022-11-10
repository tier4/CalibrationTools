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
#include <extrinsic_tag_based_base_calibrator/ceres/fixed_intrinsics_tag_reprojection_error.hpp>
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

#include <chrono>
#include <fstream>
#include <iostream>
#include <random>

#define UNUSED(x) (void)x;

namespace extrinsic_tag_based_base_calibrator
{

ExtrinsicTagBasedBaseCalibrator::ExtrinsicTagBasedBaseCalibrator(
  const rclcpp::NodeOptions & options)
: Node("extrinsic_tag_based_base_calibrator_node", options)
{
  waypoint_tag_size_ = this->declare_parameter<double>("waypoint_tag_size", 0.8);
  wheel_tag_size_ = this->declare_parameter<double>("wheel_tag_size", 0.8);
  ground_tag_size_ = this->declare_parameter<double>("ground_tag_size", 0.135);

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

  preprocess_srv_ = this->create_service<std_srvs::srv::Empty>(
    "preproces", std::bind(
                   &ExtrinsicTagBasedBaseCalibrator::preprocessCallback, this,
                   std::placeholders::_1, std::placeholders::_2));

  save_srv_ = this->create_service<std_srvs::srv::Empty>(
    "save", std::bind(
              &ExtrinsicTagBasedBaseCalibrator::saveCallback, this, std::placeholders::_1,
              std::placeholders::_2));

  load_srv_ = this->create_service<std_srvs::srv::Empty>(
    "load", std::bind(
              &ExtrinsicTagBasedBaseCalibrator::loadCallback, this, std::placeholders::_1,
              std::placeholders::_2));

  calibration_dummy_srv_ = this->create_service<std_srvs::srv::Empty>(
    "calibrate", std::bind(
                   &ExtrinsicTagBasedBaseCalibrator::calibrateCallback, this, std::placeholders::_1,
                   std::placeholders::_2));
}

void ExtrinsicTagBasedBaseCalibrator::visualizationTimerCallback()
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker base_marker;
  base_marker.header.frame_id = "/calibration_sensor";
  base_marker.ns = "raw_detections";
  next_color_index_ = 0;

  std::map<int, cv::Affine3f> sensor_to_waypoint_transform_map;

  for (std::size_t scene_index = 0; scene_index < data_.scenes.size(); scene_index++) {
    const CalibrationScene & scene = data_.scenes[scene_index];

    std_msgs::msg::ColorRGBA color;
    color.r = 1.f;
    color.g = 1.f;
    color.b = 1.f;
    color.a = 1.f;

    for (auto & detection : scene.calibration_sensor_detections) {
      sensor_to_waypoint_transform_map[detection.id] = detection.pose;

      add_tag_markers(
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
        cv::Affine3f sensor_to_waypoint_pose =
          sensor_to_waypoint_transform_map[waypoint_detection.id];
        const cv::Affine3f & external_camera_to_waypoint_pose = waypoint_detection.pose;

        cv::Affine3f sensor_to_external_camera_pose =
          sensor_to_waypoint_pose * external_camera_to_waypoint_pose.inv();

        add_axes_markers(markers, 0.5f, sensor_to_external_camera_pose, base_marker);
        add_line_marker(
          markers, color, sensor_to_waypoint_pose, sensor_to_external_camera_pose, base_marker);

        for (const auto & wheel_detection : wheel_detections) {
          const cv::Affine3f & external_camera_to_wheel_pose = wheel_detection.pose;
          cv::Affine3d sensor_to_wheel_pose =
            sensor_to_external_camera_pose * external_camera_to_wheel_pose;

          add_tag_markers(
            markers, std::to_string(wheel_detection.id), wheel_detection.size, color,
            sensor_to_wheel_pose, base_marker);
          add_line_marker(
            markers, color, sensor_to_external_camera_pose, sensor_to_wheel_pose, base_marker);
        }

        for (const auto & ground_detection : ground_detections) {
          const cv::Affine3f & external_camera_to_ground_pose = ground_detection.pose;
          cv::Affine3d sensor_to_ground_transform = sensor_to_waypoint_pose *
                                                    external_camera_to_waypoint_pose.inv() *
                                                    external_camera_to_ground_pose;

          add_tag_markers(
            markers, std::to_string(ground_detection.id), ground_detection.size, color,
            sensor_to_ground_transform, base_marker);
          add_line_marker(
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

  for (auto it = data_.initial_tag_poses_map.begin(); it != data_.initial_tag_poses_map.end();
       it++) {
    const UID & tag_uid = it->first;
    auto & pose = it->second;

    add_tag_markers(
      markers, tag_uid.to_string(), tag_size_map_[tag_uid.tag_id], initial_estimations_color, *pose,
      base_marker);
  }

  for (auto it = data_.initial_external_camera_poses.begin();
       it != data_.initial_external_camera_poses.end(); it++) {
    const UID & camera_uid = it->first;
    auto & camera_pose = it->second;

    add_text_marker(
      markers, camera_uid.to_string(), initial_estimations_color, *camera_pose, base_marker);
    add_axes_markers(markers, 0.5f, *camera_pose, base_marker);

    // Iterate over all the detections of said camera
    for (const auto & detection :
         data_.scenes[camera_uid.scene_id].external_camera_frames[camera_uid.frame_id].detections) {
      UID detection_uid = waypoint_tag_ids_set_.count(detection.id) > 0
                            ? UID::makeWaypointUID(camera_uid.scene_id, detection.id)
                          : wheel_tag_ids_set_.count(detection.id)
                            ? UID::makeWheelTagUID(detection.id)
                            : UID::makeGroundTagUID(detection.id);

      add_line_marker(
        markers, initial_estimations_color, *camera_pose,
        *data_.initial_tag_poses_map[detection_uid], base_marker);
    }
  }

  // Initial estimations
  base_marker.ns = "optimized_estimations";

  std_msgs::msg::ColorRGBA optimized_estimations_color;
  optimized_estimations_color.r = 0.f;
  optimized_estimations_color.g = 1.f;
  optimized_estimations_color.b = 0.f;
  optimized_estimations_color.a = 1.f;

  for (auto it = data_.optimized_tag_poses_map.begin(); it != data_.optimized_tag_poses_map.end();
       it++) {
    const UID & tag_uid = it->first;
    auto & pose = it->second;

    add_tag_markers(
      markers, tag_uid.to_string(), tag_size_map_[tag_uid.tag_id], optimized_estimations_color,
      *pose, base_marker);
  }

  for (auto it = data_.optimized_external_camera_poses.begin();
       it != data_.optimized_external_camera_poses.end(); it++) {
    const UID & camera_uid = it->first;
    auto & camera_pose = it->second;

    add_text_marker(
      markers, camera_uid.to_string(), optimized_estimations_color, *camera_pose, base_marker);
    add_axes_markers(markers, 0.5f, *camera_pose, base_marker);

    // Iterate over all the detections of said camera
    for (const auto & detection :
         data_.scenes[camera_uid.scene_id].external_camera_frames[camera_uid.frame_id].detections) {
      UID detection_uid = waypoint_tag_ids_set_.count(detection.id) > 0
                            ? UID::makeWaypointUID(camera_uid.scene_id, detection.id)
                          : wheel_tag_ids_set_.count(detection.id)
                            ? UID::makeWheelTagUID(detection.id)
                            : UID::makeGroundTagUID(detection.id);

      add_line_marker(
        markers, optimized_estimations_color, *camera_pose,
        *data_.optimized_tag_poses_map[detection_uid], base_marker);
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

  cv::Affine3f initial_ground_pose, optimized_ground_pose;
  if (computeGroundPlane(data_.initial_ground_tag_poses, ground_tag_size_, initial_ground_pose)) {
    add_grid(markers, initial_ground_pose, 100, 0.2, initial_ground_base_marker);
    add_axes_markers(markers, 0.5f, initial_ground_pose, initial_ground_base_marker);

    if (data_.initial_left_wheel_tag_pose && data_.initial_right_wheel_tag_pose) {
      cv::Affine3f initial_base_link_pose = computeBaseLink(
        *data_.initial_left_wheel_tag_pose, *data_.initial_right_wheel_tag_pose,
        initial_ground_pose);
      add_axes_markers(markers, 0.5f, initial_base_link_pose, initial_base_link_base_marker);
    }
  }

  if (computeGroundPlane(
        data_.optimized_ground_tag_poses, ground_tag_size_, optimized_ground_pose)) {
    add_grid(markers, optimized_ground_pose, 100, 0.2, optimized_ground_base_marker);
    add_axes_markers(markers, 1.f, optimized_ground_pose, optimized_ground_base_marker);

    if (data_.optimized_left_wheel_tag_pose && data_.optimized_right_wheel_tag_pose) {
      cv::Affine3f optimized_base_link_pose = computeBaseLink(
        *data_.optimized_left_wheel_tag_pose, *data_.optimized_right_wheel_tag_pose,
        optimized_ground_pose);
      add_axes_markers(markers, 1.f, optimized_base_link_pose, optimized_base_link_base_marker);
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

bool ExtrinsicTagBasedBaseCalibrator::preprocessCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request;
  (void)response;

  std::vector<std::string> external_camera_intrinsic_images_names = {
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0007.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0008.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0009.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0010.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0011.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0012.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0013.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0014.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0015.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0016.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0017.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0018.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0019.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0020.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0021.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0022.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0023.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0024.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0025.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0026.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0027.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0028.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0029.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0030.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0031.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0032.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0033.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0034.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0035.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0036.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0037.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0038.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0039.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0040.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0041.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0042.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_intrinsics/DSC_0043.JPG"};

  std::vector<std::string> sensor_camera_intrinsic_images_names = {
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3852.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3847.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3846.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3855.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3861.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3863.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3849.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3851.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3848.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3841.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3867.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3858.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3843.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3853.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3870.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3860.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3864.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3869.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3865.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3845.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3862.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3842.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3854.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3844.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3859.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3856.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3857.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3850.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3868.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_intrinsics/DSC_3866.JPG"};

  std::string scene1_sensor_calibration_image_name =
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_extrinsics_scene1/DSC_3875.JPG";

  std::vector<std::string> scene1_external_camera_image_names = {
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0051.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0047.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0048.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0050.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0044.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0053.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0056.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0058.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0057.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0049.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0055.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0045.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0052.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0046.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene1/DSC_0054.JPG"};

  std::string scene2_sensor_calibration_image_name =
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon7200_extrinsics_scene2/DSC_3877.JPG";

  std::vector<std::string> scene2_external_camera_image_names = {
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0061.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0063.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0071.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0066.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0067.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0065.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0070.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0062.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0064.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0069.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0073.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0068.JPG",
    "/home/kenzolobos/repos/lib-dt-apriltags/test/nikon5100_extrinsics_scene2/DSC_0060.JPG"};

  // Experimental: use all the images to calibrate the intrinsics
  external_camera_intrinsic_images_names.insert(
    external_camera_intrinsic_images_names.end(), scene1_external_camera_image_names.begin(),
    scene1_external_camera_image_names.end());
  // external_camera_intrinsic_images_names.insert( external_camera_intrinsic_images_names.end(),
  // scene2_external_camera_image_names.begin(), scene2_external_camera_image_names.end());

  IntrinsicsCalibrator external_camera_intrinsics_calibrator(
    apriltag_parameters_, intrinsic_calibration_tag_ids_, true);
  IntrinsicsCalibrator calibration_sensor_intrinsics_calibrator(
    apriltag_parameters_, intrinsic_calibration_tag_ids_, true);

  if (!external_camera_intrinsics_.loadCalibration("external_camera_intrinsics.cfg")) {
    RCLCPP_INFO(this->get_logger(), "External camera intrinsic calibration not found");
    external_camera_intrinsics_calibrator.setCalibrationImageFiles(
      external_camera_intrinsic_images_names);
    external_camera_intrinsics_calibrator.calibrate(external_camera_intrinsics_);
    external_camera_intrinsics_.saveCalibration("external_camera_intrinsics.cfg");
  } else {
    RCLCPP_INFO(this->get_logger(), "Loading external camera intrinsic calibration");
  }

  if (!calibration_sensor_intrinsics_.loadCalibration("sensor_camera_intrinsics.cfg")) {
    RCLCPP_INFO(this->get_logger(), "Sensor camera intrinsic calibration not found");
    calibration_sensor_intrinsics_calibrator.setCalibrationImageFiles(
      sensor_camera_intrinsic_images_names);
    calibration_sensor_intrinsics_calibrator.calibrate(calibration_sensor_intrinsics_);
    calibration_sensor_intrinsics_.saveCalibration("sensor_camera_intrinsics.cfg");
  } else {
    RCLCPP_INFO(this->get_logger(), "Loading calibration sensor camera intrinsic calibration");
  }

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

  CalibrationScene scene1 = calibration_scene_extractor.processScene(
    scene1_sensor_calibration_image_name, scene1_external_camera_image_names);

  CalibrationScene scene2 = calibration_scene_extractor.processScene(
    scene2_sensor_calibration_image_name, scene2_external_camera_image_names);

  data_.scenes.push_back(scene1);
  data_.scenes.push_back(scene2);

  // Estimate the the initial poses for all the tags
  std::map<UID, std::vector<cv::Affine3f>> poses_vector_map;

  for (std::size_t scene_index = 0; scene_index < data_.scenes.size(); scene_index++) {
    const CalibrationScene & scene = data_.scenes[scene_index];

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
        cv::Affine3f sensor_to_waypoint_pose = poses_vector_map[waypoint_uid].back();

        const cv::Affine3f & external_camera_to_waypoint_pose = waypoint_detection.pose;

        cv::Affine3f sensor_to_external_camera_pose =
          sensor_to_waypoint_pose * external_camera_to_waypoint_pose.inv();

        UID external_camera_uid = UID::makeCameraUID(scene_index, frame_id);

        poses_vector_map[external_camera_uid].push_back(sensor_to_external_camera_pose);

        for (const auto & wheel_detection : wheel_detections) {
          const cv::Affine3f & external_camera_to_wheel_pose = wheel_detection.pose;

          cv::Affine3d sensor_to_wheel_pose =
            sensor_to_external_camera_pose * external_camera_to_wheel_pose;

          UID wheel_tag_uid = UID::makeWheelTagUID(wheel_detection.id);
          poses_vector_map[wheel_tag_uid].push_back(sensor_to_wheel_pose);
        }

        for (const auto & ground_detection : ground_detections) {
          cv::Affine3f external_camera_to_ground_pose = ground_detection.pose;
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
  for (std::size_t scene_index = 0; scene_index < data_.scenes.size(); scene_index++) {
    const CalibrationScene & scene = data_.scenes[scene_index];

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
        cv::Affine3f sensor_to_linked_tag = poses_vector_map.count(wheel_tag_uid) > 0
                                              ? poses_vector_map[wheel_tag_uid].front()
                                              : poses_vector_map[ground_tag_uid].front();

        const cv::Affine3f & external_camera_to_linked_tag_affine = linked_detection.pose;
        poses_vector_map[external_camera_uid].push_back(
          sensor_to_linked_tag * external_camera_to_linked_tag_affine.inv());
      }

      for (auto & unlinked_wheel_detection : unlinked_wheel_detections) {
        auto & linked_detection = linked_detections.front();

        UID unlinked_wheel_tag_uid = UID::makeWheelTagUID(unlinked_wheel_detection.id);
        UID linked_wheel_tag_uid = UID::makeWheelTagUID(linked_detection.id);
        UID linked_ground_tag_uid = UID::makeGroundTagUID(linked_detection.id);
        cv::Affine3f sensor_to_linked_tag = poses_vector_map.count(linked_wheel_tag_uid) > 0
                                              ? poses_vector_map[linked_wheel_tag_uid].front()
                                              : poses_vector_map[linked_ground_tag_uid].front();

        const cv::Affine3f & external_camera_to_linked_tag_affine = linked_detection.pose;
        const cv::Affine3f & external_camera_to_unlinked_tag_affine = unlinked_wheel_detection.pose;

        poses_vector_map[unlinked_wheel_tag_uid].push_back(
          sensor_to_linked_tag * external_camera_to_linked_tag_affine.inv() *
          external_camera_to_unlinked_tag_affine);
      }

      for (auto & unlinked_ground_detection : unlinked_ground_detections) {
        auto & linked_detection = linked_detections.front();

        UID unlinked_ground_tag_uid = UID::makeGroundTagUID(unlinked_ground_detection.id);
        UID linked_wheel_tag_uid = UID::makeWheelTagUID(linked_detection.id);
        UID linked_ground_tag_uid = UID::makeGroundTagUID(linked_detection.id);

        cv::Affine3f sensor_to_linked_tag = poses_vector_map.count(linked_wheel_tag_uid) > 0
                                              ? poses_vector_map[linked_wheel_tag_uid].front()
                                              : poses_vector_map[linked_ground_tag_uid].front();

        const cv::Affine3f & external_camera_to_linked_tag_affine = linked_detection.pose;
        const cv::Affine3f & external_camera_to_unlinked_tag_affine =
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

  for (auto it = poses_vector_map.begin(); it != poses_vector_map.end(); it++) {
    const UID & uid = it->first;
    auto poses = it->second;

    Eigen::Vector3f avg_translation = Eigen::Vector3f::Zero();
    std::vector<Eigen::Vector4f> quats;

    for (auto & pose : poses) {
      Eigen::Vector3f translation;
      Eigen::Matrix3f rotation;
      cv::cv2eigen(pose.translation(), translation);
      cv::cv2eigen(pose.rotation(), rotation);
      Eigen::Quaternionf quat(rotation);
      quats.emplace_back(quat.w(), quat.x(), quat.y(), quat.z());

      avg_translation += translation;
    }

    avg_translation /= poses.size();
    Eigen::Vector4f avg_quat = quaternionAverage(quats);

    Eigen::Matrix3f avg_rotation =
      Eigen::Quaternionf(avg_quat(0), avg_quat(1), avg_quat(2), avg_quat(3)).toRotationMatrix();

    cv::Vec3f avg_pose_translation;
    cv::Matx33f avg_pose_rotation;
    cv::eigen2cv(avg_translation, avg_pose_translation);
    cv::eigen2cv(avg_rotation, avg_pose_rotation);

    auto initial_pose = std::make_shared<cv::Affine3f>(avg_pose_rotation, avg_pose_translation);
    (void)initial_pose;

    if (uid.is_tag) {
      data_.initial_tag_poses_map[uid] = initial_pose;

      if (uid.is_waypoint_tag) {
        data_.initial_waypoint_tag_poses.push_back(initial_pose);
      } else if (uid.is_ground_tag) {
        data_.initial_ground_tag_poses.push_back(initial_pose);
      } else if (uid.is_wheel_tag && uid.tag_id == left_wheel_tag_id_) {
        data_.initial_left_wheel_tag_pose = initial_pose;
      } else if (uid.is_wheel_tag && uid.tag_id == right_wheel_tag_id_) {
        data_.initial_right_wheel_tag_pose = initial_pose;
      }
    } else if (uid.is_camera) {
      data_.initial_external_camera_poses[uid] = initial_pose;
    }
  }

  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::saveCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request;
  (void)response;

  std::ofstream ofs("base_calibration.data");
  boost::archive::text_oarchive oa(ofs);

  oa << data_;

  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::loadCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request;
  (void)response;

  if (
    !external_camera_intrinsics_.loadCalibration("external_camera_intrinsics.cfg") ||
    !calibration_sensor_intrinsics_.loadCalibration("sensor_camera_intrinsics.cfg")) {
    RCLCPP_ERROR(this->get_logger(), "Could not load intrinsics");
    return false;
  }

  std::ifstream ifs("base_calibration.data");
  boost::archive::text_iarchive ia(ifs);

  ia >> data_;

  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::calibrateCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request;
  (void)response;

  auto affine_to_placeholder =
    [](
      cv::Affine3f pose,
      std::array<double, CalibrationData::POSE_OPTIMIZATION_DIMENSIONALITY> & placeholder,
      bool invert) {
      if (invert) {
        pose = pose.inv();
      }

      Eigen::Vector3f translation;
      Eigen::Matrix3f rotation;
      cv::cv2eigen(pose.translation(), translation);
      cv::cv2eigen(pose.rotation(), rotation);
      Eigen::Quaternionf quat(rotation);

      std::fill(placeholder.begin(), placeholder.end(), 0);
      placeholder[CalibrationData::ROTATION_W_INDEX] = quat.w();
      placeholder[CalibrationData::ROTATION_X_INDEX] = quat.x();
      placeholder[CalibrationData::ROTATION_Y_INDEX] = quat.y();
      placeholder[CalibrationData::ROTATION_Z_INDEX] = quat.z();
      placeholder[CalibrationData::TRANSLATION_X_INDEX] = translation.x();
      placeholder[CalibrationData::TRANSLATION_Y_INDEX] = translation.y();
      placeholder[CalibrationData::TRANSLATION_Z_INDEX] = translation.z();
    };

  auto placeholder_to_affine =
    [](
      std::array<double, CalibrationData::POSE_OPTIMIZATION_DIMENSIONALITY> & placeholder,
      std::shared_ptr<cv::Affine3f> & pose, bool invert) {
      const float scale =
        1.f / std::sqrt(
                placeholder[0] * placeholder[0] + placeholder[1] * placeholder[1] +
                placeholder[2] * placeholder[2] + placeholder[3] * placeholder[3]);

      Eigen::Quaternionf quat = Eigen::Quaterniond(
                                  scale * placeholder[CalibrationData::ROTATION_W_INDEX],
                                  scale * placeholder[CalibrationData::ROTATION_X_INDEX],
                                  scale * placeholder[CalibrationData::ROTATION_Y_INDEX],
                                  scale * placeholder[CalibrationData::ROTATION_Z_INDEX])
                                  .cast<float>();

      Eigen::Vector3f translation = Eigen::Vector3d(
                                      placeholder[CalibrationData::TRANSLATION_X_INDEX],
                                      placeholder[CalibrationData::TRANSLATION_Y_INDEX],
                                      placeholder[CalibrationData::TRANSLATION_Z_INDEX])
                                      .cast<float>();

      Eigen::Matrix3f rotation = quat.toRotationMatrix();

      cv::Matx33f cv_rot;
      cv::Vec3f cv_transl;
      cv::eigen2cv(translation, cv_transl);
      cv::eigen2cv(rotation, cv_rot);

      pose = std::make_shared<cv::Affine3f>(cv_rot, cv_transl);

      if (invert) {
        *pose = pose->inv();
      }
    };

  // Prepare the placeholders
  for (auto it = data_.initial_external_camera_poses.begin();
       it != data_.initial_external_camera_poses.end(); it++) {
    const UID & uid = it->first;
    const auto & pose = it->second;

    affine_to_placeholder(*pose, data_.optimization_placeholders_map[uid], true);
    placeholder_to_affine(
      data_.optimization_placeholders_map[uid], data_.optimized_external_camera_poses[uid], true);
  }

  for (auto it = data_.initial_tag_poses_map.begin(); it != data_.initial_tag_poses_map.end();
       it++) {
    const UID & uid = it->first;
    const auto & pose = it->second;

    affine_to_placeholder(*pose, data_.optimization_placeholders_map[uid], false);
    placeholder_to_affine(
      data_.optimization_placeholders_map[uid], data_.optimized_tag_poses_map[uid], false);
  }

  ceres::Problem problem;
  using CalibrationSensorResidualFunction = FixedIntrinsicsTagReprojectionError;
  using ExternalCameraResidualFunction = TagReprojectionError;

  // Build the optimization problem
  for (std::size_t scene_index = 0; scene_index < data_.scenes.size(); scene_index++) {
    CalibrationScene & scene = data_.scenes[scene_index];

    for (auto detection : scene.calibration_sensor_detections) {
      UID sensor_uid = UID::makeCameraUID(scene_index, -1);
      UID detection_uid = UID::makeWaypointUID(scene_index, detection.id);
      std::array<double, CalibrationData::POSE_OPTIMIZATION_DIMENSIONALITY> identity{
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      std::array<double, 8> residuals;

      auto f = CalibrationSensorResidualFunction(
        sensor_uid, calibration_sensor_intrinsics_, detection, identity,
        CalibrationSensorResidualFunction::FixedCameraPose);

      f(data_.optimization_placeholders_map[detection_uid].data(), residuals.data());
      double sum_res = std::accumulate(residuals.begin(), residuals.end(), 0.0);

      std::cout << sensor_uid.to_string() << " <-> " << detection_uid.to_string()
                << " initial error: " << sum_res << std::endl;

      ceres::CostFunction * res =
        CalibrationSensorResidualFunction::createResidualWithFixedCameraPose(
          sensor_uid, calibration_sensor_intrinsics_, detection, identity);

      problem.AddResidualBlock(
        res,
        nullptr,  // L2
        data_.optimization_placeholders_map[detection_uid].data());
    }

    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      // Need to make sure all the cameras are in the map
      UID external_camera_uid = UID::makeCameraUID(scene_index, frame_id);

      auto & frame = scene.external_camera_frames[frame_id];

      for (const auto & detection : frame.detections) {
        UID detection_uid = waypoint_tag_ids_set_.count(detection.id) > 0
                              ? UID::makeWaypointUID(scene_index, detection.id)
                            : wheel_tag_ids_set_.count(detection.id)
                              ? UID::makeWheelTagUID(detection.id)
                              : UID::makeGroundTagUID(detection.id);

        std::array<double, 8> residuals;
        auto f = ExternalCameraResidualFunction(
          external_camera_uid, external_camera_intrinsics_, detection);

        f(data_.optimization_placeholders_map[external_camera_uid].data(),
          data_.optimization_placeholders_map[detection_uid].data(), residuals.data());
        double sum_res = std::accumulate(residuals.begin(), residuals.end(), 0.0);

        std::cout << external_camera_uid.to_string() << " <-> " << detection_uid.to_string()
                  << " initial error: " << sum_res << std::endl;
        ;

        ceres::CostFunction * res = ExternalCameraResidualFunction::createResidual(
          external_camera_uid, external_camera_intrinsics_, detection);

        problem.AddResidualBlock(
          res,
          nullptr,  // L2
          data_.optimization_placeholders_map[external_camera_uid].data(),
          data_.optimization_placeholders_map[detection_uid].data());
      }
    }
  }

  int numresidualsblocks = problem.NumResidualBlocks();
  int numresiduals = problem.NumResiduals();

  (void)numresidualsblocks;
  (void)numresiduals;

  std::vector<double> residuals;
  ceres::Problem::EvaluateOptions eval_opt;
  eval_opt.num_threads = 1;
  problem.GetResidualBlocks(&eval_opt.residual_blocks);
  problem.Evaluate(eval_opt, NULL, &residuals, NULL, NULL);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n" << std::flush;

  // - Iterate all scenes
  //    - Iterate all waypoints from the calibration sensor
  //      - Keep the waypoint pose (should be unique !). Keep a different
  //      - Iterate for all the external camera samples. Select the ones that have the waypoint
  //        - Iterate for all the tags presen in
  //  - Iterate for

  // Prepare the placeholders
  for (auto it = data_.optimized_external_camera_poses.begin();
       it != data_.optimized_external_camera_poses.end(); it++) {
    const UID & uid = it->first;

    placeholder_to_affine(
      data_.optimization_placeholders_map[uid], data_.optimized_external_camera_poses[uid], true);
  }

  for (auto it = data_.optimized_tag_poses_map.begin(); it != data_.optimized_tag_poses_map.end();
       it++) {
    const UID & uid = it->first;
    auto & pose = data_.optimized_tag_poses_map[uid];

    placeholder_to_affine(data_.optimization_placeholders_map[uid], pose, false);

    if (uid.is_waypoint_tag) {
      data_.optimized_waypoint_tag_poses.push_back(pose);
    } else if (uid.is_ground_tag) {
      data_.optimized_ground_tag_poses.push_back(pose);
    } else if (uid.is_wheel_tag && uid.tag_id == left_wheel_tag_id_) {
      data_.optimized_left_wheel_tag_pose = pose;
    } else if (uid.is_wheel_tag && uid.tag_id == right_wheel_tag_id_) {
      data_.optimized_right_wheel_tag_pose = pose;
    }
  }

  // Build the optimization problem
  for (std::size_t scene_index = 0; scene_index < data_.scenes.size(); scene_index++) {
    CalibrationScene & scene = data_.scenes[scene_index];

    for (auto detection : scene.calibration_sensor_detections) {
      UID sensor_uid = UID::makeCameraUID(scene_index, -1);
      UID detection_uid = UID::makeWaypointUID(scene_index, detection.id);
      std::array<double, CalibrationData::POSE_OPTIMIZATION_DIMENSIONALITY> identity{
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      std::array<double, 8> residuals;

      auto f = CalibrationSensorResidualFunction(
        sensor_uid, calibration_sensor_intrinsics_, detection, identity,
        CalibrationSensorResidualFunction::FixedCameraPose);

      f(data_.optimization_placeholders_map[detection_uid].data(), residuals.data());
      double sum_res = std::accumulate(residuals.begin(), residuals.end(), 0.0);

      std::cout << sensor_uid.to_string() << " <-> " << detection_uid.to_string()
                << " Optimized error: " << sum_res << std::endl;
    }

    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      // Need to make sure all the cameras are in the map
      UID external_camera_uid = UID::makeCameraUID(scene_index, frame_id);

      auto & frame = scene.external_camera_frames[frame_id];

      for (const auto & detection : frame.detections) {
        UID detection_uid = waypoint_tag_ids_set_.count(detection.id) > 0
                              ? UID::makeWaypointUID(scene_index, detection.id)
                            : wheel_tag_ids_set_.count(detection.id)
                              ? UID::makeWheelTagUID(detection.id)
                              : UID::makeGroundTagUID(detection.id);

        if (external_camera_uid.to_string() == "s0_c8" && detection_uid.tag_id == 4) {
          // Here have a reference of both poses
          auto camera_placeholder = data_.optimization_placeholders_map[external_camera_uid];
          auto tag_placeholder = data_.optimization_placeholders_map[detection_uid];
          auto inv_camera_pose = data_.optimized_external_camera_poses[external_camera_uid]->inv();
          auto camera_pose = data_.optimized_external_camera_poses[external_camera_uid];
          auto tag_pose = *data_.optimized_tag_poses_map[detection_uid];
          (void)camera_placeholder;
          (void)inv_camera_pose;
          (void)camera_pose;
          (void)tag_placeholder;
          (void)tag_pose;
          int x = 0;
          (void)x;
        }

        std::array<double, 8> residuals;
        auto f = ExternalCameraResidualFunction(
          external_camera_uid, external_camera_intrinsics_, detection);

        f(data_.optimization_placeholders_map[external_camera_uid].data(),
          data_.optimization_placeholders_map[detection_uid].data(), residuals.data());
        double sum_res = std::accumulate(residuals.begin(), residuals.end(), 0.0);

        std::cout << external_camera_uid.to_string() << " <-> " << detection_uid.to_string()
                  << " Optimized error: " << sum_res << std::endl;
        ;
      }
    }
  }

  // Debug the optimization
  auto draw_detection = [](cv::Mat & img, const ApriltagDetection & detection, cv::Scalar color) {
    std::vector<float> edge_sizes;

    for (std::size_t i = 0; i < detection.corners.size(); ++i) {
      std::size_t j = (i + 1) % detection.corners.size();
      edge_sizes.push_back(cv::norm(detection.corners[i] - detection.corners[j]));
    }

    float tag_size = *std::max_element(edge_sizes.begin(), edge_sizes.end());

    for (std::size_t i = 0; i < detection.corners.size(); ++i) {
      std::size_t j = (i + 1) % detection.corners.size();
      cv::line(
        img, detection.corners[i], detection.corners[j], color,
        static_cast<int>(std::max(tag_size / 512.f, 1.f)), cv::LINE_AA);
    }

    cv::putText(
      img, std::to_string(detection.id), detection.center, cv::FONT_HERSHEY_SIMPLEX,
      std::max(tag_size / 128.f, 1.f), color, static_cast<int>(std::max(tag_size / 128.f, 1.f)));
  };

  for (std::size_t scene_index = 0; scene_index < data_.scenes.size(); scene_index++) {
    CalibrationScene & scene = data_.scenes[scene_index];

    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      // Need to make sure all the cameras are in the map
      UID external_camera_uid = UID::makeCameraUID(scene_index, frame_id);
      std::string file_name = scene.external_camera_frames[frame_id].image_filename;

      cv::Mat distorted_img = cv::imread(file_name, cv::IMREAD_COLOR);
      cv::Mat undistorted_img;
      cv::undistort(
        distorted_img, undistorted_img, external_camera_intrinsics_.camera_matrix,
        external_camera_intrinsics_.dist_coeffs,
        external_camera_intrinsics_.undistorted_camera_matrix);

      for (auto & detection : scene.external_camera_frames[frame_id].detections) {
        UID detection_uid = waypoint_tag_ids_set_.count(detection.id) > 0
                              ? UID::makeWaypointUID(scene_index, detection.id)
                            : wheel_tag_ids_set_.count(detection.id)
                              ? UID::makeWheelTagUID(detection.id)
                              : UID::makeGroundTagUID(detection.id);

        if (external_camera_uid.to_string() == "s0_c8" && detection_uid.tag_id == 4) {
          int x = 0;
          (void)x;
        }

        cv::Affine3f initial_camera_pose =
          *data_.initial_external_camera_poses[external_camera_uid];
        cv::Affine3f initial_tag_pose = *data_.initial_tag_poses_map[detection_uid];

        cv::Affine3f optimized_camera_pose =
          *data_.optimized_external_camera_poses[external_camera_uid];
        cv::Affine3f optimized_tag_pose = *data_.optimized_tag_poses_map[detection_uid];

        ApriltagDetection initial_detection = detection;
        ApriltagDetection optimized_detection = detection;

        auto project_corners = [this, &external_camera_uid](
                                 ApriltagDetection & detection, const cv::Affine3f & camera_pose,
                                 const cv::Affine3f & tag_pose) {
          cv::Vec3f template_corners[4] = {
            {-1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, -1.0, 0.0}, {-1.0, -1.0, 0.0}};

          for (int j = 0; j < 4; ++j) {
            template_corners[j] *= 0.5 * detection.size;
          }

          std::vector<cv::Vec3f> corners_wcs{
            tag_pose * template_corners[0], tag_pose * template_corners[1],
            tag_pose * template_corners[2], tag_pose * template_corners[3]};
          std::vector<cv::Vec3f> corners_ccs{
            camera_pose.inv() * corners_wcs[0], camera_pose.inv() * corners_wcs[1],
            camera_pose.inv() * corners_wcs[2], camera_pose.inv() * corners_wcs[3]};

          float k1 = data_.optimization_placeholders_map[external_camera_uid]
                                                        [CalibrationData::INTRINSICS_K1_INDEX];
          float k2 = data_.optimization_placeholders_map[external_camera_uid]
                                                        [CalibrationData::INTRINSICS_K2_INDEX];
          float df = data_.optimization_placeholders_map[external_camera_uid]
                                                        [CalibrationData::INTRINSICS_F_INDEX];

          float fx = df + external_camera_intrinsics_.undistorted_camera_matrix(0, 0);
          float fy = df + external_camera_intrinsics_.undistorted_camera_matrix(1, 1);
          float cx = external_camera_intrinsics_.undistorted_camera_matrix(0, 2);
          float cy = external_camera_intrinsics_.undistorted_camera_matrix(1, 2);

          auto camera_ccs_to_image = [&fx, &fy, &cx, &cy, &k1, &k2](cv::Vec3f & p) {
            const float xp = p(0) / p(2);
            const float yp = p(1) / p(2);
            const float r2 = xp * xp + yp * yp;
            const float d = 1.0 + r2 * (k1 + k2 * r2);
            return cv::Point2f(cx + fx * d * xp, cy + fy * d * yp);
          };

          detection.corners = std::vector<cv::Point2f>{
            camera_ccs_to_image(corners_ccs[0]), camera_ccs_to_image(corners_ccs[1]),
            camera_ccs_to_image(corners_ccs[2]), camera_ccs_to_image(corners_ccs[3])};
        };

        project_corners(initial_detection, initial_camera_pose, initial_tag_pose);
        project_corners(optimized_detection, optimized_camera_pose, optimized_tag_pose);

        draw_detection(undistorted_img, detection, cv::Scalar(255, 0, 255));
        draw_detection(undistorted_img, initial_detection, cv::Scalar(0, 0, 255));
        draw_detection(undistorted_img, optimized_detection, cv::Scalar(0, 255, 0));
      }

      std::string output_name = external_camera_uid.to_string() + "_debug.jpg";
      cv::imwrite(output_name, undistorted_img);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Finished optimization");
  return true;
}

}  // namespace extrinsic_tag_based_base_calibrator
