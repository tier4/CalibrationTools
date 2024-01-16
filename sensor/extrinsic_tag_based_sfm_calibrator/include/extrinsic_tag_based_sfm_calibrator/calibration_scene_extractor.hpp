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

#ifndef EXTRINSIC_TAG_BASED_SFM_CALIBRATOR__CALIBRATION_SCENE_EXTRACTOR_HPP_
#define EXTRINSIC_TAG_BASED_SFM_CALIBRATOR__CALIBRATION_SCENE_EXTRACTOR_HPP_

#include <extrinsic_tag_based_sfm_calibrator/apriltag_detector.hpp>
#include <extrinsic_tag_based_sfm_calibrator/scene_types.hpp>
#include <extrinsic_tag_based_sfm_calibrator/types.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <lidartag_msgs/msg/lidar_tag_detection_array.hpp>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace extrinsic_tag_based_sfm_calibrator
{

class CalibrationSceneExtractor
{
public:
  explicit CalibrationSceneExtractor(
    const ApriltagDetectorParameters & detector_parameters,
    const std::vector<TagParameters> & tag_parameters_vector, bool debug = true)
  : calibration_sensor_detector_(detector_parameters, tag_parameters_vector),
    external_camera_detector_(detector_parameters, tag_parameters_vector),
    debug_(debug)
  {
  }

  /*!
   * Sets the intrinsics of the calibration camera prior to process a scene to include the 3d poses
   * of the tags
   * @param[in] parameters The intrinsics of the calibration camera
   */
  void setCalibrationSensorIntrinsics(IntrinsicParameters & parameters);

  /*!
   * Sets the intrinsics of the external camera prior to process a scene to include the 3d poses of
   * the tags
   * @param[in] parameters The intrinsics of the external camera
   */
  void setExternalCameraIntrinsics(IntrinsicParameters & parameters);

  /*!
   * Process a scene, rectifying the images in the scene and obtaining the detections and 3d poses
   * @param[in] detections The lidartag detections
   * @param[in] external_camera_image_names The vector containing the images of the scene
   * corresponding to the external camera
   */
  CalibrationScene processScene(
    const std::unordered_map<std::string, sensor_msgs::msg::CompressedImage::SharedPtr> &
      camera_images_map,
    const std::unordered_map<std::string, LidartagDetections> & lidar_detections_map,
    const std::unordered_map<std::string, GroupedApriltagGridDetections> & camera_detections_map,
    const std::vector<std::string> & calibration_lidar_frames,
    const std::vector<std::string> & calibration_camera_frames,
    const std::vector<std::string> & external_camera_image_names);

protected:
  /*!
   * Process the external camera images
   * @param[in] external_camera_image_names The vector containing the images of the scene
   * corresponding to the external camera
   */
  void processExternalCameraImages(
    CalibrationScene & scene, const std::vector<std::string> & external_camera_image_names);

  GroupedApriltagGridDetections detect(
    const ApriltagDetector & detector, const IntrinsicParameters & intrinsics,
    const std::string & img_name);

  void setUp();

  ApriltagDetector calibration_sensor_detector_;
  ApriltagDetector external_camera_detector_;

  IntrinsicParameters calibration_sensor_intrinsics_;
  IntrinsicParameters external_camera_intrinsics_;
  bool debug_;
};

}  // namespace extrinsic_tag_based_sfm_calibrator

#endif  // EXTRINSIC_TAG_BASED_SFM_CALIBRATOR__CALIBRATION_SCENE_EXTRACTOR_HPP_
