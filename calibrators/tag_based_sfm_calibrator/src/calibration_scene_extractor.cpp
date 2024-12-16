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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tag_based_sfm_calibrator/calibration_scene_extractor.hpp>
#include <tag_based_sfm_calibrator/visualization.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <iostream>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

namespace tag_based_sfm_calibrator
{

void CalibrationSceneExtractor::setCalibrationSensorIntrinsics(IntrinsicParameters & intrinsics)
{
  calibration_sensor_intrinsics_ = intrinsics;
  calibration_sensor_detector_.setIntrinsics(
    intrinsics.undistorted_camera_matrix(0, 0), intrinsics.undistorted_camera_matrix(1, 1),
    intrinsics.undistorted_camera_matrix(0, 2), intrinsics.undistorted_camera_matrix(1, 2));
}

void CalibrationSceneExtractor::setExternalCameraIntrinsics(IntrinsicParameters & intrinsics)
{
  external_camera_intrinsics_ = intrinsics;
  external_camera_detector_.setIntrinsics(
    intrinsics.undistorted_camera_matrix(0, 0), intrinsics.undistorted_camera_matrix(1, 1),
    intrinsics.undistorted_camera_matrix(0, 2), intrinsics.undistorted_camera_matrix(1, 2));
}

CalibrationScene CalibrationSceneExtractor::processScene(
  const std::unordered_map<std::string, sensor_msgs::msg::CompressedImage::SharedPtr> &
    camera_images_map,
  const std::unordered_map<std::string, LidartagDetections> & lidar_detections_map,
  const std::unordered_map<std::string, GroupedApriltagGridDetections> & camera_detections_map,
  const std::vector<std::string> & calibration_lidar_frames,
  const std::vector<std::string> & calibration_camera_frames,
  const std::vector<std::string> & external_camera_image_names)
{
  CalibrationScene scene;

  for (std::size_t calibration_lidar_id = 0; calibration_lidar_id < calibration_lidar_frames.size();
       calibration_lidar_id++) {
    SingleCalibrationLidarDetections lidar_detections;
    const std::string calibration_frame = calibration_lidar_frames[calibration_lidar_id];
    lidar_detections.calibration_frame = calibration_frame;
    lidar_detections.calibration_lidar_id = calibration_lidar_id;
    lidar_detections.detections = lidar_detections_map.at(calibration_frame);

    scene.calibration_lidars_detections.push_back(lidar_detections);
  }

  for (std::size_t calibration_camera_id = 0;
       calibration_camera_id < calibration_camera_frames.size(); calibration_camera_id++) {
    SingleCalibrationCameraDetections camera_detections;
    const std::string calibration_frame = calibration_camera_frames[calibration_camera_id];
    camera_detections.calibration_frame = calibration_frame;
    camera_detections.calibration_camera_id = calibration_camera_id;
    camera_detections.grouped_detections = camera_detections_map.at(calibration_frame);
    camera_detections.calibration_image = camera_images_map.at(calibration_frame);

    scene.calibration_cameras_detections.push_back(camera_detections);
  }

  processExternalCameraImages(scene, external_camera_image_names);

  return scene;
}

void CalibrationSceneExtractor::processExternalCameraImages(
  CalibrationScene & scene, const std::vector<std::string> & external_camera_image_names)
{
  for (const auto & image_name : external_camera_image_names) {
    GroupedApriltagGridDetections detections =
      detect(external_camera_detector_, external_camera_intrinsics_, image_name);

    int num_detections = std::transform_reduce(
      detections.begin(), detections.end(), 0, std::plus{},
      [](auto it) { return it.second.size(); });

    if (num_detections == 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("scene_extractor"), "Image name: %s contained only 0 detections.",
        image_name.c_str());
      continue;
    } else if (num_detections < 2) {
      RCLCPP_INFO(
        rclcpp::get_logger("scene_extractor"),
        "Image name: %s contained only 1 detection. We need at least two to contribute to the "
        "graph",
        image_name.c_str());
      continue;
    }

    ExternalCameraFrame frame;
    frame.image_filename = image_name;
    frame.detections = detections;

    RCLCPP_INFO(
      rclcpp::get_logger("scene_extractor"), "Processed: %s Detections: %d", image_name.c_str(),
      num_detections);

    scene.external_camera_frames.emplace_back(frame);

    if (debug_) {
      cv::Mat distorted_img =
        cv::imread(image_name, cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION);
      cv::Mat undistorted_img;
      cv::undistort(
        distorted_img, undistorted_img, external_camera_intrinsics_.camera_matrix,
        external_camera_intrinsics_.dist_coeffs,
        external_camera_intrinsics_.undistorted_camera_matrix);

      for (const auto & detection_group : frame.detections) {
        for (const auto & detection_grid : detection_group.second) {
          for (const auto & detection : detection_grid.sub_detections) {
            drawDetection(undistorted_img, detection, cv::Scalar(0, 255, 0));
            drawAxes(undistorted_img, detection, external_camera_intrinsics_);
          }
        }
      }

      std::size_t name_start_pos = image_name.find_last_of("/\\");
      std::size_t name_end_pos = image_name.find_last_of(".\\");
      std::string output_name =
        image_name.substr(name_start_pos + 1, name_end_pos - name_start_pos - 1) +
        "_undistorted.jpg";
      cv::imwrite(output_name, undistorted_img);
    }
  }
}

GroupedApriltagGridDetections CalibrationSceneExtractor::detect(
  const ApriltagDetector & detector, const IntrinsicParameters & intrinsics,
  const std::string & img_name)
{
  cv::Mat distorted_img =
    cv::imread(img_name, cv::IMREAD_GRAYSCALE | cv::IMREAD_IGNORE_ORIENTATION);
  cv::Mat undistorted_img;

  if (distorted_img.rows != intrinsics.size.height || distorted_img.cols != intrinsics.size.width) {
    RCLCPP_ERROR(
      rclcpp::get_logger("scene_extractor"),
      "Input image has different dimensions that the intrinsics!");
    RCLCPP_ERROR(rclcpp::get_logger("scene_extractor"), "Image name: %s", img_name.c_str());
  }

  assert(distorted_img.rows == intrinsics.size.height);
  assert(distorted_img.cols == intrinsics.size.width);

  cv::undistort(
    distorted_img, undistorted_img, intrinsics.camera_matrix, intrinsics.dist_coeffs,
    intrinsics.undistorted_camera_matrix);

  return detector.detect(undistorted_img);
}

}  // namespace tag_based_sfm_calibrator
