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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__TYPES_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__TYPES_HPP_

#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace extrinsic_tag_based_base_calibrator
{

struct ApriltagDetectorParameters
{
  // std::vector<std::string> families; moved out TODOKL remove
  int max_hamming;
  double min_margin;
  double max_homography_error;
  double quad_decimate;
  double quad_sigma;
  int nthreads;
  bool debug;
  bool refine_edges;
};

enum class TagType {
  Unknown,
  IntrinsicCalibrationTag,
  WaypointTag,
  WheelTag,
  GroundTag,
};

enum class SensorType {
  Unknown,
  CalibrationCamera,
  CalibrationLidar,
  ExternalCamera,
};

struct TagParameters
{
  TagType tag_type;
  std::string family;
  int rows;
  int cols;
  double size;
  double spacing;
  std::unordered_set<int> ids;
};

class IntrinsicParameters
{
public:
  bool loadCalibration(const std::string file_name)
  {
    cv::FileStorage fs(file_name, cv::FileStorage::READ);

    if (!fs.isOpened()) {
      return false;
    }

    fs["size"] >> size;
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    fs["undistorted_camera_matrix"] >> undistorted_camera_matrix;

    return true;
  }

  void saveCalibration(const std::string file_name)
  {
    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
    fs << "size" << size;
    fs << "camera_matrix" << camera_matrix;
    fs << "dist_coeffs" << dist_coeffs;
    fs << "undistorted_camera_matrix" << undistorted_camera_matrix;
  }

  bool isValid() const
  {
    return !size.empty() && camera_matrix(0, 0) > 0.0 && camera_matrix(1, 1) > 0.0 &&
           camera_matrix(0, 2) > 0.0 && camera_matrix(1, 2) > 0.0 &&
           undistorted_camera_matrix(0, 0) > 0.0 && undistorted_camera_matrix(1, 1) > 0.0 &&
           undistorted_camera_matrix(0, 2) > 0.0 && undistorted_camera_matrix(1, 2) > 0.0;
  }

  cv::Size size;
  cv::Mat_<double> camera_matrix;
  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> undistorted_camera_matrix;
};

struct UID
{
  enum UIDType { InvalidUID, SensorUID, TagUID };

  UID()
  : type(InvalidUID),
    sensor_type(SensorType::Unknown),
    tag_type(TagType::Unknown),
    scene_id(-1),
    calibration_sensor_id(-1),
    frame_id(-1),
    tag_id(-1)
  {
  }

  std::string toString() const
  {
    if (sensor_type == SensorType::CalibrationCamera) {
      return "c" + std::to_string(frame_id);
    } else if (sensor_type == SensorType::CalibrationLidar) {
      return "l" + std::to_string(frame_id);
    } else if (sensor_type == SensorType::ExternalCamera) {
      return "s" + std::to_string(scene_id) + "_e" + std::to_string(frame_id);
    } else if (tag_type == TagType::WaypointTag) {
      return "s" + std::to_string(scene_id) + "_w" + std::to_string(tag_id);
    } else if (tag_type == TagType::GroundTag) {
      return "g" + std::to_string(tag_id);
    } else if (tag_type == TagType::WheelTag) {
      return "t" + std::to_string(tag_id);
    } else {
      throw std::invalid_argument("Invalid UID");
    }
  }

  bool isValid() const
  {
    if (sensor_type == SensorType::Unknown || tag_type == TagType::Unknown) {
      return false;
    } else if (
      (sensor_type == SensorType::CalibrationCamera ||
       sensor_type == SensorType::CalibrationLidar) &&
      ((frame_id >= 0 || scene_id >= 0 || calibration_sensor_id < 0))) {
      return false;
    } else if (
      (sensor_type == SensorType::ExternalCamera) &&
      (frame_id < 0 || scene_id < 0 || calibration_sensor_id >= 0)) {
      return false;
    } else if (
      (tag_type == TagType::GroundTag || tag_type == TagType::WheelTag) &&
      (frame_id >= 0 || scene_id >= 0 || calibration_sensor_id >= 0)) {
      return false;
    } else if (
      (tag_type == TagType::WaypointTag) &&
      (frame_id >= 0 || scene_id < 0 || calibration_sensor_id >= 0)) {
      return false;
    } else {
      return true;
    }
  }

  bool operator<(const UID & other) const
  {
    return (this->toString().compare(other.toString())) > 0;
  }

  bool operator==(const UID & other) const { return this->toString() == other.toString(); }

  static UID makeSensorUID(SensorType sensor_type, int scene_id, int frame_id)
  {
    assert(sensor_type == SensorType::ExternalCamera);

    UID uid;
    uid.type = SensorUID;
    uid.sensor_type = sensor_type;
    uid.scene_id = scene_id;
    uid.frame_id = frame_id;

    return uid;
  }

  static UID makeSensorUID(SensorType sensor_type, int frame_id)
  {
    assert(sensor_type != SensorType::ExternalCamera);

    UID uid;
    uid.type = SensorUID;
    uid.sensor_type = sensor_type;
    uid.frame_id = frame_id;

    return uid;
  }

  static UID makeTagUID(TagType tag_type, int scene_id, int tag_id)
  {
    assert(tag_type == TagType::WaypointTag);

    UID uid;
    uid.type = TagUID;
    uid.tag_type = tag_type;
    uid.scene_id = scene_id;
    uid.tag_id = tag_id;

    return uid;
  }

  static UID makeTagUID(TagType tag_type, int tag_id)
  {
    assert(tag_type != TagType::WaypointTag);

    UID uid;
    uid.type = TagUID;
    uid.tag_type = tag_type;
    uid.tag_id = tag_id;

    return uid;
  }

  UIDType type;
  SensorType sensor_type;
  TagType tag_type;

  int scene_id;               // Scenes are needed when the waypoints required to be moved
  int calibration_sensor_id;  // Multiple sensors can be calibrated simultaneously
  int frame_id;               // Each scene can have multiple external camera samples
  int tag_id;  // Tag ids are unique for each tag type. They can be repeated between different tag
               // types
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__TYPES_HPP_
