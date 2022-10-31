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

#include <extrinsic_tag_based_base_calibrator/calibration_scene_extractor.hpp>
#include <extrinsic_tag_based_base_calibrator/extrinsic_tag_based_base_calibrator.hpp>
#include <extrinsic_tag_based_base_calibrator/intrinsics_calibrator.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <extrinsic_tag_based_base_calibrator/visualization.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <iostream>

#define UNUSED(x) (void)x;

using std::chrono_literals;

ExtrinsicTagBasedBaseCalibrator::ExtrinsicTagBasedBaseCalibrator(
  const rclcpp::NodeOptions & options)
: Node("extrinsic_tag_based_base_calibrator_node", options)
{
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

  ApriltagParameters apriltag_parameters;

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

  std::vector<int64_t> ground_tag_ids = this->declare_parameter<std::vector<int64_t>>(
    "ground_tag_ids", std::vector<int64_t>{5, 6, 7, 8, 9, 10, /*11, */ 12, 13, 14, 15});
  std::transform(
    ground_tag_ids.begin(), ground_tag_ids.end(), std::back_inserter(ground_tag_ids_),
    [](auto & id) { return static_cast<int>(id); });

  std::vector<int64_t> intrinsic_calibration_tag_ids =
    this->declare_parameter<std::vector<int64_t>>(
      "intrinsic_calibration_tag_ids", std::vector<int64_t>{0});
  std::transform(
    intrinsic_calibration_tag_ids.begin(), intrinsic_calibration_tag_ids.end(),
    std::back_inserter(intrinsic_calibration_tag_ids_),
    [](auto & id) { return static_cast<int>(id); });

  for (const auto & id : waypoint_tag_ids_) {
    waypoint_tag_ids_set_.insert(id);
  }

  for (const auto & id : wheel_tag_ids_) {
    wheel_tag_ids_set_.insert(id);
  }

  for (const auto & id : ground_tag_ids_) {
    ground_tag_ids_set_.insert(id);
  }

  apriltag_parameters.family = this->declare_parameter<std::string>("apriltag_family", "16h5");
  apriltag_parameters.max_hamming = this->declare_parameter<int>("apriltag_max_hamming", 0);
  apriltag_parameters.min_margin = this->declare_parameter<double>("apriltag_min_margin", 20.0);
  apriltag_parameters.max_h_error =
    this->declare_parameter<double>("apriltag_max_h_error", 10000.0);
  apriltag_parameters.quad_decimate =
    this->declare_parameter<double>("apriltag_quad_decimate", 1.0);
  apriltag_parameters.quad_sigma = this->declare_parameter<double>("apriltag_quad_sigma", 0.0);
  apriltag_parameters.nthreads = this->declare_parameter<int>("apriltag_nthreads", 1);
  apriltag_parameters.debug = this->declare_parameter<bool>("apriltag_debug", false);
  apriltag_parameters.refine_edges = this->declare_parameter<bool>("apriltag_refine_edges", true);

  IntrinsicsCalibrator external_camera_intrinsics_calibrator(
    apriltag_parameters, intrinsic_calibration_tag_ids_, true);
  IntrinsicsCalibrator calibration_sensor_intrinsics_calibrator(
    apriltag_parameters, intrinsic_calibration_tag_ids_, true);

  IntrinsicParameters external_camera_intrinsics;
  IntrinsicParameters calibration_sensor_intrinsics;

  if (!external_camera_intrinsics.loadCalibration("external_camera_intrinsics.cfg")) {
    RCLCPP_INFO(this->get_logger(), "External camera intrinsic calibration not found");
    external_camera_intrinsics_calibrator.setCalibrationImageFiles(
      external_camera_intrinsic_images_names);
    external_camera_intrinsics_calibrator.calibrate(external_camera_intrinsics);
    external_camera_intrinsics.saveCalibration("external_camera_intrinsics.cfg");
  } else {
    RCLCPP_INFO(this->get_logger(), "Loading external camera intrinsic calibration");
  }

  if (!calibration_sensor_intrinsics.loadCalibration("sensor_camera_intrinsics.cfg")) {
    RCLCPP_INFO(this->get_logger(), "Sensor camera intrinsic calibration not found");
    calibration_sensor_intrinsics_calibrator.setCalibrationImageFiles(
      sensor_camera_intrinsic_images_names);
    calibration_sensor_intrinsics_calibrator.calibrate(calibration_sensor_intrinsics);
    calibration_sensor_intrinsics.saveCalibration("sensor_camera_intrinsics.cfg");
  } else {
    RCLCPP_INFO(this->get_logger(), "Loading external camera intrinsic calibration");
  }

  CalibrationSceneExtractor calibration_scene_extrator(apriltag_parameters);
  calibration_scene_extrator.setCalibrationSensorIntrinsics(calibration_sensor_intrinsics);
  calibration_scene_extrator.setExternalCameraIntrinsics(external_camera_intrinsics);

  calibration_scene_extrator.setWaypointTagSize(waypoint_tag_size_);
  calibration_scene_extrator.setWheelTagSize(wheel_tag_size_);
  calibration_scene_extrator.setGroundTagSize(ground_tag_size_);

  calibration_scene_extrator.setWaypointTagIds(waypoint_tag_ids_);
  calibration_scene_extrator.setLeftWheelTagId(left_wheel_tag_id_);
  calibration_scene_extrator.setRightWheelTagId(right_wheel_tag_id_);
  calibration_scene_extrator.setGroundTagIds(ground_tag_ids_);

  CalibrationScene scene = calibration_scene_extrator.processScene(
    scene1_sensor_calibration_image_name, scene1_external_camera_image_names);

  scenes_.push_back(scene);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);

  visualization_timer_ = rclcpp::create_timer(
    this, get_clock(), 1s,
    std::bind(&ExtrinsicTagBasedBaseCalibrator::visualizationTimerCallback, this));
}

void ExtrinsicTagBasedBaseCalibrator::visualizationTimerCallback()
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker base_marker;
  base_marker.header.frame_id = "/calibration_sensor";
  base_marker.ns = "raw_detections";
  next_color_index_ = 0;

  std::unordered_map<int, cv::Affine3f> sensor_to_waypoint_transform_map;

  for (std::size_t scene_index = 0; scene_index < scenes_.size(); scene_index++) {
    const CalibrationScene & scene = scenes_[scene_index];

    std_msgs::msg::ColorRGBA color;
    color.r = 1.f;
    color.g = 1.f;
    color.b = 1.f;
    color.a = 1.f;

    for (auto & detection : scene.calibration_sensor_detections) {
      cv::Vec3f translation(
        detection.pose_translation(0), detection.pose_translation(1),
        detection.pose_translation(2));
      cv::Affine3f affine(detection.pose_rotation, translation);
      sensor_to_waypoint_transform_map[detection.id] = affine;

      visualization_msgs::msg::MarkerArray new_markers =
        create_tag_markers(detection.id, detection.size, color, affine, base_marker);
      markers.markers.insert(
        markers.markers.end(), new_markers.markers.begin(), new_markers.markers.end());
    }

    for (auto & detections : scene.external_camera_detections) {
      // Should choose a color per camera to debug
      std_msgs::msg::ColorRGBA color = getNextColor();

      std::vector<ApriltagDetection> waypoint_detections;
      std::vector<ApriltagDetection> wheel_detections;
      std::vector<ApriltagDetection> ground_detections;

      std::copy_if(
        detections.begin(), detections.end(), std::back_inserter(waypoint_detections),
        [this](const ApriltagDetection & detection) {
          return waypoint_tag_ids_set_.count(detection.id) > 0;
        });

      std::copy_if(
        detections.begin(), detections.end(), std::back_inserter(wheel_detections),
        [this](const ApriltagDetection & detection) {
          return wheel_tag_ids_set_.count(detection.id) > 0;
        });

      std::copy_if(
        detections.begin(), detections.end(), std::back_inserter(ground_detections),
        [this](const ApriltagDetection & detection) {
          return ground_tag_ids_set_.count(detection.id) > 0;
        });

      for (const auto & waypoint_detection : waypoint_detections) {
        cv::Affine3f sensor_to_waypoint_transform =
          sensor_to_waypoint_transform_map[waypoint_detection.id];

        cv::Vec3f external_camera_to_waypoint_translation(
          waypoint_detection.pose_translation(0), waypoint_detection.pose_translation(1),
          waypoint_detection.pose_translation(2));
        cv::Affine3f external_camera_to_waypoint_transform(
          waypoint_detection.pose_rotation, external_camera_to_waypoint_translation);

        cv::Affine3f sensor_to_external_camera_transform =
          sensor_to_waypoint_transform * external_camera_to_waypoint_transform.inv();

        visualization_msgs::msg::MarkerArray axes_markers =
          create_axes_markers(0.5f, sensor_to_external_camera_transform, base_marker);
        markers.markers.insert(
          markers.markers.end(), axes_markers.markers.begin(), axes_markers.markers.end());

        visualization_msgs::msg::MarkerArray line_markers = create_line(
          color, sensor_to_waypoint_transform, sensor_to_external_camera_transform, base_marker);
        markers.markers.insert(
          markers.markers.end(), line_markers.markers.begin(), line_markers.markers.end());

        for (const auto & wheel_detection : wheel_detections) {
          cv::Vec3f external_camera_to_wheel_translation(
            wheel_detection.pose_translation(0), wheel_detection.pose_translation(1),
            wheel_detection.pose_translation(2));
          cv::Affine3f external_camera_to_wheel_transform(
            wheel_detection.pose_rotation, external_camera_to_wheel_translation);

          cv::Affine3d sensor_to_wheel_transform =
            sensor_to_external_camera_transform * external_camera_to_wheel_transform;

          visualization_msgs::msg::MarkerArray tag_markers = create_tag_markers(
            wheel_detection.id, wheel_detection.size, color, sensor_to_wheel_transform,
            base_marker);
          markers.markers.insert(
            markers.markers.end(), tag_markers.markers.begin(), tag_markers.markers.end());

          visualization_msgs::msg::MarkerArray line_markers = create_line(
            color, sensor_to_external_camera_transform, sensor_to_wheel_transform, base_marker);
          markers.markers.insert(
            markers.markers.end(), line_markers.markers.begin(), line_markers.markers.end());
        }

        for (const auto & ground_detection : ground_detections) {
          cv::Vec3f external_camera_to_ground_translation(
            ground_detection.pose_translation(0), ground_detection.pose_translation(1),
            ground_detection.pose_translation(2));
          cv::Affine3f external_camera_to_ground_transform(
            ground_detection.pose_rotation, external_camera_to_ground_translation);

          cv::Affine3d sensor_to_ground_transform = sensor_to_waypoint_transform *
                                                    external_camera_to_waypoint_transform.inv() *
                                                    external_camera_to_ground_transform;

          visualization_msgs::msg::MarkerArray tag_markers = create_tag_markers(
            ground_detection.id, ground_detection.size, color, sensor_to_ground_transform,
            base_marker);
          markers.markers.insert(
            markers.markers.end(), tag_markers.markers.begin(), tag_markers.markers.end());

          visualization_msgs::msg::MarkerArray line_markers = create_line(
            color, sensor_to_external_camera_transform, sensor_to_ground_transform, base_marker);
          markers.markers.insert(
            markers.markers.end(), line_markers.markers.begin(), line_markers.markers.end());
        }
      }
    }
  }

  for (std::size_t marker_index = 0; marker_index < markers.markers.size(); marker_index++) {
    markers.markers[marker_index].id = marker_index;
  }

  markers_pub_->publish(markers);

  // Markers from origin to waypoint and waypoint marker
}

std_msgs::msg::ColorRGBA ExtrinsicTagBasedBaseCalibrator::getNextColor()
{
  if (next_color_index_ < precomputed_colors_.size()) {
    return precomputed_colors_[next_color_index_++];
  }

  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<double>(rand_r()) / RAND_MAX;
  color.g = static_cast<double>(rand_r()) / RAND_MAX;
  color.b = static_cast<double>(rand_r()) / RAND_MAX;
  color.a = 1.f;

  precomputed_colors_.push_back(color);
  next_color_index_++;

  return color;
}
