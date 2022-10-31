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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CALIBRATION_SCENE_EXTRACTOR_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CALIBRATION_SCENE_EXTRACTOR_HPP_

#include <extrinsic_tag_based_base_calibrator/apriltag_detector.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>

#include <string>
#include <vector>

class CalibrationSceneExtractor
{
public:
  explicit CalibrationSceneExtractor(
    const ApriltagParameters & apriltag_parameters, bool debug = true)
  : calibration_sensor_detector_(apriltag_parameters),
    external_camera_detector_(apriltag_parameters),
    debug_(debug)
  {
  }

  void setCalibrationSensorIntrinsics(IntrinsicParameters & parameters);
  void setExternalCameraIntrinsics(IntrinsicParameters & parameters);

  void setWaypointTagSize(float size);
  void setWheelTagSize(float size);
  void setGroundTagSize(float size);

  void setWaypointTagIds(const std::vector<int> & ids);
  void setLeftWheelTagId(int ids);
  void setRightWheelTagId(int ids);
  void setGroundTagIds(const std::vector<int> & ids);

  CalibrationScene processScene(
    std::string calibration_sensor_image_name,
    std::vector<std::string> external_camera_image_names);

protected:
  ApriltagDetector calibration_sensor_detector_;
  ApriltagDetector external_camera_detector_;

  float waypoint_tag_size_;
  float wheel_tag_size_;
  float ground_tag_size_;

  std::vector<int> waypoint_tag_ids_;
  int left_wheel_tag_id_;
  int right_wheel_tag_id_;
  std::vector<int> ground_tag_ids_;

  IntrinsicParameters calibration_sensor_intrinsics_;
  IntrinsicParameters external_camera_intrinsics_;
  bool debug_;
};

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CALIBRATION_SCENE_EXTRACTOR_HPP_
