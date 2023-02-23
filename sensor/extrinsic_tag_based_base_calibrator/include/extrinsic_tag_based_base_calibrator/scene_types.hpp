// Copyright 2023 Tier IV, Inc.
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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__SCENE_TYPES_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__SCENE_TYPES_HPP_

#include <extrinsic_tag_based_base_calibrator/apriltag_detection.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>

#include <string>
#include <vector>

namespace extrinsic_tag_based_base_calibrator
{

struct ExternalCameraFrame
{
  std::string image_filename;
  GroupedApriltagGridDetections detections;
};

struct SingleCalibrationLidarDetections
{
  std::string calibration_frame;
  int calibration_lidar_id;
  LidartagDetections detections;
};

struct SingleCalibrationCameraDetections
{
  std::string calibration_frame;
  int calibration_camera_id;
  GroupedApriltagGridDetections grouped_detections;
};

struct CalibrationScene
{
  std::vector<SingleCalibrationLidarDetections>
    calibration_lidars_detections;  // multiple lidars x multiple detections (detections can only be
                                    // waypoints)
  std::vector<SingleCalibrationCameraDetections>
    calibration_cameras_detections;  // multiple cameras x multiple detection groups x detections
                                     // per group (groups: ground, wheel, waypoint)
  std::vector<ExternalCameraFrame> external_camera_frames;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__SCENE_TYPES_HPP_
