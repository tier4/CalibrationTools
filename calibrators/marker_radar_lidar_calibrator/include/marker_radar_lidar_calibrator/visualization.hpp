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

#pragma once

#include <Eigen/Dense>
#include <marker_radar_lidar_calibrator/types.hpp>
#include <marker_radar_lidar_calibrator/utils.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <unordered_map>
#include <vector>

namespace marker_radar_lidar_calibrator
{

struct VisualizationParameters
{
  std_msgs::msg::Header lidar_header;
  std_msgs::msg::Header radar_header;
  TransformationType transformation_type;
  double reflector_radius;
  int marker_size_per_track;
  Eigen::Isometry3d initial_radar_to_lidar_eigen;
};

struct DetectionMarkers
{
  visualization_msgs::msg::MarkerArray lidar_detections_marker_array;
  visualization_msgs::msg::MarkerArray radar_detections_marker_array;
  visualization_msgs::msg::MarkerArray matches_marker_array;
};

class Visualization
{
public:
  Visualization() = default;
  ~Visualization() = default;

  void setParameters(VisualizationParameters params);
  DetectionMarkers visualizeDetectionMarkers(
    const std::vector<Eigen::Vector3d> & lidar_detections,
    const std::vector<Eigen::Vector3d> & radar_detections,
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> & matched_detections);
  visualization_msgs::msg::MarkerArray visualizeTrackMarkers(
    const std::vector<Track> & converged_tracks,
    const Eigen::Isometry3d & calibrated_radar_to_lidar_eigen);

  visualization_msgs::msg::MarkerArray deleteTrackMarkers(const size_t converged_tracks_size);
  visualization_msgs::msg::Marker drawCalibrationStatusText(
    const size_t converged_tracks_size, TransformationType type, CalibrationErrorMetrics metrics);

private:
  VisualizationParameters params_;
  static constexpr double m_to_cm = 100.0;
};

}  // namespace marker_radar_lidar_calibrator
