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
#include <rclcpp/rclcpp.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

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

void CalibrationSceneExtractor::setLidartagToApriltagScale(double scale)
{
  lidartag_to_apriltag_scale_ = scale;
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
  const lidartag_msgs::msg::LidarTagDetectionArray & msg,
  const std::vector<std::string> & external_camera_image_names)
{
  setUp();

  CalibrationScene scene;

  // Lidartag and apriltag poses have different directions (orientation) so we need to choose one
  // and adapt the other We choose to use the coordinate system defined by apriltag and convert the
  // lidartag poses to match the apriltag definitions E.j corner^{1}_{apriltag} = (-hsize, hsize, 0)
  // <=> corner^{1}_{lidartag} = (0, -hsize, -hsize) corner^{2}_{apriltag} = (hsize, hsize, 0)   <=>
  // corner^{2}_{lidartag} = (0, hsize, -hsize) corner^{3}_{apriltag} = (hsize, -hsize, 0)  <=>
  // corner^{3}_{lidartag} = (0, hsize, hsize) corner^{4}_{apriltag} = (-hsize, -hsize, 0) <=>
  // corner^{4}_{lidartag} = (0, -hsize, hsize) corner_{lidartag} = R * corner_{apriltag} R = [[0,
  // 0, -1],
  //      [1,  0, 0],
  //      [0, -1, 0]]
  // Rot_{apriltag} = R^{T} * Rot_{lidartag}

  std::vector<LidartagDetection> tmp_detections;
  std::transform(
    msg.detections.begin(), msg.detections.end(), std::back_inserter(tmp_detections),
    [&](const lidartag_msgs::msg::LidarTagDetection & msg) {
      LidartagDetection detection;
      detection.id = msg.id;
      detection.size = msg.size * lidartag_to_apriltag_scale_;

      Eigen::Isometry3d pose_eigen;
      tf2::fromMsg(msg.pose, pose_eigen);

      Eigen::Vector3d translation_eigen = pose_eigen.translation();
      Eigen::Matrix3d rotation_eigen = pose_eigen.rotation();

      Eigen::Matrix3d apriltag_to_lidartag_rot;
      apriltag_to_lidartag_rot << 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0;

      rotation_eigen = rotation_eigen * apriltag_to_lidartag_rot;

      cv::Vec3d translation_cv;
      cv::Matx33d rotation_cv;
      cv::eigen2cv(translation_eigen, translation_cv);
      cv::eigen2cv(rotation_eigen, rotation_cv);

      detection.pose = cv::Affine3d(rotation_cv, translation_cv);

      return detection;
    });

  std::copy_if(
    tmp_detections.begin(), tmp_detections.end(),
    std::back_inserter(scene.calibration_lidar_detections),
    [&](const LidartagDetection & detection) { return waypoint_ids_set_.count(detection.id) > 0; });

  processExternalCamaeraImages(scene, external_camera_image_names);

  return scene;
}

CalibrationScene CalibrationSceneExtractor::processScene(
  const std::string & calibration_sensor_image_name,
  const std::vector<std::string> & external_camera_image_names)
{
  setUp();

  CalibrationScene scene;

  auto calibration_sensor_detections = detect(
    calibration_sensor_detector_, calibration_sensor_intrinsics_, calibration_sensor_image_name);

  std::copy_if(
    calibration_sensor_detections.begin(), calibration_sensor_detections.end(),
    std::back_inserter(scene.calibration_camera_detections),
    [&](const ApriltagDetection & detection) { return waypoint_ids_set_.count(detection.id) > 0; });

  processExternalCamaeraImages(scene, external_camera_image_names);

  return scene;
}

void CalibrationSceneExtractor::processExternalCamaeraImages(
  CalibrationScene & scene, const std::vector<std::string> & external_camera_image_names)
{
  for (const auto & image_name : external_camera_image_names) {
    std::vector<ApriltagDetection> detections =
      detect(external_camera_detector_, external_camera_intrinsics_, image_name);

    if (detections.size() < 2) {
      continue;
    }

    ExternalCameraFrame frame;
    frame.image_filename = image_name;

    std::copy_if(
      detections.begin(), detections.end(), std::back_inserter(frame.detections),
      [&](const ApriltagDetection & detection) {
        return external_camera_tag_sizes_.count(detection.id) > 0;
      });

    RCLCPP_INFO(
      rclcpp::get_logger("scene_extractor"), "Processed: %s Detections: %lu", image_name.c_str(),
      frame.detections.size());

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

            cv::putText(
              img, std::to_string(i), detection.corners[i], cv::FONT_HERSHEY_SIMPLEX,
              std::max(tag_size / 256.0, 1.0), color,
              static_cast<int>(std::max(tag_size / 128.0, 1.0)));
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
}

std::vector<ApriltagDetection> CalibrationSceneExtractor::detect(
  const ApriltagDetector & detector, const IntrinsicParameters & intrinsics,
  const std::string & img_name)
{
  cv::Mat distorted_img = cv::imread(img_name, cv::IMREAD_GRAYSCALE);
  cv::Mat undistorted_img;

  cv::undistort(
    distorted_img, undistorted_img, intrinsics.camera_matrix, intrinsics.dist_coeffs,
    intrinsics.undistorted_camera_matrix);

  return detector.detect(undistorted_img);
}

void CalibrationSceneExtractor::setUp()
{
  for (auto id : waypoint_tag_ids_) {
    waypoint_ids_set_.insert(id);
    calibration_sensor_tag_sizes_[id] = waypoint_tag_size_;
    external_camera_tag_sizes_[id] = waypoint_tag_size_;
  }

  external_camera_tag_sizes_[left_wheel_tag_id_] = wheel_tag_size_;
  external_camera_tag_sizes_[right_wheel_tag_id_] = wheel_tag_size_;

  for (auto id : ground_tag_ids_) {
    external_camera_tag_sizes_[id] = ground_tag_size_;
  }

  calibration_sensor_detector_.setTagSizes(calibration_sensor_tag_sizes_);
  external_camera_detector_.setTagSizes(external_camera_tag_sizes_);
}

}  // namespace extrinsic_tag_based_base_calibrator
