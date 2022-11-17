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
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <unordered_set>

namespace extrinsic_tag_based_base_calibrator
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

void CalibrationSceneExtractor::setWaypointTagSize(double size) { waypoint_tag_size_ = size; }

void CalibrationSceneExtractor::setWheelTagSize(double size) { wheel_tag_size_ = size; }

void CalibrationSceneExtractor::setGroundTagSize(double size) { ground_tag_size_ = size; }

void CalibrationSceneExtractor::setWaypointTagIds(const std::vector<int> & ids)
{
  waypoint_tag_ids_ = ids;
}

void CalibrationSceneExtractor::setLeftWheelTagId(int id) { left_wheel_tag_id_ = id; }

void CalibrationSceneExtractor::setRightWheelTagId(int id) { right_wheel_tag_id_ = id; }

void CalibrationSceneExtractor::setGroundTagIds(const std::vector<int> & ids)
{
  ground_tag_ids_ = ids;
}

CalibrationScene CalibrationSceneExtractor::processScene(
  std::string calibration_sensor_image_name, std::vector<std::string> external_camera_image_names)
{
  std::unordered_set<int> waypoint_ids_set;
  std::unordered_map<int, double> calibration_sensor_tag_sizes;
  std::unordered_map<int, double> external_camera_tag_sizes;

  for (auto id : waypoint_tag_ids_) {
    waypoint_ids_set.insert(id);
    calibration_sensor_tag_sizes[id] = waypoint_tag_size_;
    external_camera_tag_sizes[id] = waypoint_tag_size_;
  }

  external_camera_tag_sizes[left_wheel_tag_id_] = wheel_tag_size_;
  external_camera_tag_sizes[right_wheel_tag_id_] = wheel_tag_size_;

  for (auto id : ground_tag_ids_) {
    external_camera_tag_sizes[id] = ground_tag_size_;
  }

  calibration_sensor_detector_.setTagSizes(calibration_sensor_tag_sizes);
  external_camera_detector_.setTagSizes(external_camera_tag_sizes);

  auto detection_fn = [](
                        const ApriltagDetector & detector, const IntrinsicParameters & intrinsics,
                        const std::string & img_name) -> std::vector<ApriltagDetection> {
    cv::Mat distorted_img = cv::imread(img_name, cv::IMREAD_GRAYSCALE);
    cv::Mat undistorted_img;

    cv::undistort(
      distorted_img, undistorted_img, intrinsics.camera_matrix, intrinsics.dist_coeffs,
      intrinsics.undistorted_camera_matrix);

    return detector.detect(undistorted_img);
  };

  CalibrationScene scene;

  auto calibration_sensor_detections = detection_fn(
    calibration_sensor_detector_, calibration_sensor_intrinsics_, calibration_sensor_image_name);

  std::copy_if(
    calibration_sensor_detections.begin(), calibration_sensor_detections.end(),
    std::back_inserter(scene.calibration_sensor_detections),
    [&waypoint_ids_set](const ApriltagDetection & detection) {
      return waypoint_ids_set.count(detection.id) > 0;
    });

  for (const auto & image_name : external_camera_image_names) {
    std::vector<ApriltagDetection> detections =
      detection_fn(external_camera_detector_, external_camera_intrinsics_, image_name);

    int num_waypoints = std::count_if(
      detections.begin(), detections.end(),
      [&waypoint_ids_set](const ApriltagDetection & detection) {
        return waypoint_ids_set.count(detection.id) > 0;
      });

    if (num_waypoints == 0) {
      continue;
    }

    ExternalCameraFrame frame;
    frame.image_filename = image_name;

    std::copy_if(
      detections.begin(), detections.end(), std::back_inserter(frame.detections),
      [external_camera_tag_sizes](const ApriltagDetection & detection) {
        return external_camera_tag_sizes.count(detection.id) > 0;
      });

    std::cout << "Porcesed: " << image_name << " Detections: " << frame.detections.size()
              << std::endl;

    scene.external_camera_frames.emplace_back(frame);

    if (debug_) {
      auto draw_detection =
        [](cv::Mat & img, const ApriltagDetection & detection, cv::Scalar color) {
          std::vector<double> edge_sizes;

          for (std::size_t i = 0; i < detection.corners.size(); ++i) {
            std::size_t j = (i + 1) % detection.corners.size();
            edge_sizes.push_back(cv::norm(detection.corners[i] - detection.corners[j]));
          }

          double tag_size = *std::max_element(edge_sizes.begin(), edge_sizes.end());

          for (std::size_t i = 0; i < detection.corners.size(); ++i) {
            std::size_t j = (i + 1) % detection.corners.size();
            cv::line(
              img, detection.corners[i], detection.corners[j], color,
              static_cast<int>(std::max(tag_size / 256.0, 1.0)), cv::LINE_AA);
          }

          cv::putText(
            img, std::to_string(detection.id), detection.center, cv::FONT_HERSHEY_SIMPLEX,
            std::max(tag_size / 128.0, 1.0), color,
            static_cast<int>(std::max(tag_size / 128.0, 1.0)));
        };

      cv::Mat distorted_img = cv::imread(image_name, cv::IMREAD_COLOR);
      cv::Mat undistorted_img;
      cv::undistort(
        distorted_img, undistorted_img, external_camera_intrinsics_.camera_matrix,
        external_camera_intrinsics_.dist_coeffs,
        external_camera_intrinsics_.undistorted_camera_matrix);

      for (const auto & detection : frame.detections) {
        draw_detection(undistorted_img, detection, cv::Scalar(0, 255, 0));
      }

      std::size_t name_start_pos = image_name.find_last_of("/\\");
      std::size_t name_end_pos = image_name.find_last_of(".\\");
      std::string output_name =
        image_name.substr(name_start_pos + 1, name_end_pos - name_start_pos - 1) +
        "undistorted.jpg";
      cv::imwrite(output_name, undistorted_img);
    }
  }

  return scene;
}

}  // namespace extrinsic_tag_based_base_calibrator
