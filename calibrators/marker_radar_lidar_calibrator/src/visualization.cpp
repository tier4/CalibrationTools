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

#include <marker_radar_lidar_calibrator/types.hpp>
#include <marker_radar_lidar_calibrator/visualization.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace marker_radar_lidar_calibrator
{

void Visualization::setParameters(VisualizationParameters params) { params_ = params; }

DetectionMarkers Visualization::visualizeDetectionMarkers(
  const std::vector<Eigen::Vector3d> & lidar_detections,
  const std::vector<Eigen::Vector3d> & radar_detections,
  const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> & matched_detections)
{
  DetectionMarkers detection_markers;
  for (std::size_t detection_index = 0; detection_index < lidar_detections.size();
       detection_index++) {
    const auto & detection_center = lidar_detections[detection_index];
    visualization_msgs::msg::Marker marker;
    marker.header = params_.lidar_header;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.id = detection_index;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = detection_center.x();
    marker.pose.position.y = detection_center.y();
    marker.pose.position.z = detection_center.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 0.6;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    detection_markers.lidar_detections_marker_array.markers.push_back(marker);
  }

  // Radar makers
  for (std::size_t detection_index = 0; detection_index < radar_detections.size();
       detection_index++) {
    const auto & detection_center = radar_detections[detection_index];
    visualization_msgs::msg::Marker marker;
    marker.header = params_.radar_header;
    marker.id = detection_index;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.ns = "center";
    marker.pose.position.x = detection_center.x();
    marker.pose.position.y = detection_center.y();
    marker.pose.position.z = detection_center.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = params_.reflector_radius;
    marker.scale.y = params_.reflector_radius;
    marker.scale.z = params_.reflector_radius;
    marker.color.a = 0.6;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    detection_markers.radar_detections_marker_array.markers.push_back(marker);

    // For 2D radar detection to represent that it has no z values.
    if (
      params_.transformation_type == TransformationType::svd_2d ||
      params_.transformation_type == TransformationType::yaw_only_rotation_2d) {
      geometry_msgs::msg::Point p1, p2;
      p1.z -= 0.5;
      p2.z += 0.5;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.ns = "line";
      marker.scale.x = 0.2 * params_.reflector_radius;
      marker.scale.y = 0.2 * params_.reflector_radius;
      marker.scale.z = 0.2 * params_.reflector_radius;
      marker.points.push_back(p1);
      marker.points.push_back(p2);
      detection_markers.radar_detections_marker_array.markers.push_back(marker);
    }
  }

  for (std::size_t match_index = 0; match_index < matched_detections.size(); match_index++) {
    const auto & [lidar_detection, radar_detection] = matched_detections[match_index];
    const auto lidar_detection_transformed = params_.initial_radar_to_lidar_eigen * lidar_detection;

    visualization_msgs::msg::Marker marker;
    marker.header = params_.radar_header;
    marker.id = match_index;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.ns = "match";
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 0.6;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.points.push_back(eigenToPointMsg(lidar_detection_transformed));
    marker.points.push_back(eigenToPointMsg(radar_detection));
    detection_markers.matches_marker_array.markers.push_back(marker);
  }

  return detection_markers;
}

visualization_msgs::msg::MarkerArray Visualization::visualizeTrackMarkers(
  const std::vector<Track> & converged_tracks,
  const Eigen::Isometry3d & calibrated_radar_to_lidar_eigen)
{
  auto add_track_markers = [&](
                             const Eigen::Vector3d & lidar_estimation,
                             const Eigen::Vector3d & radar_estimation_transformed, Track track,
                             const std::string ns, const std_msgs::msg::ColorRGBA & color,
                             std::vector<visualization_msgs::msg::Marker> & markers) {
    visualization_msgs::msg::Marker marker;

    marker.header = params_.lidar_header;
    marker.id = markers.size();
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = ns;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2 * params_.reflector_radius;
    marker.scale.y = 0.2 * params_.reflector_radius;
    marker.scale.z = 0.2 * params_.reflector_radius;
    marker.color = color;
    marker.points.push_back(eigenToPointMsg(radar_estimation_transformed));
    marker.points.push_back(eigenToPointMsg(lidar_estimation));
    markers.push_back(marker);

    marker.id = markers.size();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position = eigenToPointMsg(radar_estimation_transformed);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = params_.reflector_radius;
    marker.scale.y = params_.reflector_radius;
    marker.scale.z = params_.reflector_radius;
    marker.points.clear();
    markers.push_back(marker);

    // For 2D radar detection to represent that it has no z values.
    if (
      params_.transformation_type == TransformationType::svd_2d ||
      params_.transformation_type == TransformationType::yaw_only_rotation_2d) {
      marker.id = markers.size();
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.scale.x = 0.2 * params_.reflector_radius;
      marker.scale.y = 0.2 * params_.reflector_radius;
      marker.scale.z = 0.2 * params_.reflector_radius;
      marker.points.push_back(eigenToPointMsg(Eigen::Vector3d(0.0, 0.0, -0.5)));
      marker.points.push_back(eigenToPointMsg(Eigen::Vector3d(0.0, 0.0, 0.5)));
      markers.push_back(marker);
    }

    marker.id = markers.size();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position = eigenToPointMsg(lidar_estimation);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = params_.reflector_radius;
    marker.scale.y = params_.reflector_radius;
    marker.scale.z = params_.reflector_radius;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.points.clear();
    markers.push_back(marker);

    if (ns == "calibrated") {
      marker.id = markers.size();
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.pose.position = eigenToPointMsg(lidar_estimation + Eigen::Vector3d(0, 0, 1));
      marker.scale.z = 0.3;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker.text = "\n ID=" + std::to_string(track.id) +
                    "\n dist_err=" + toStringWithPrecision(track.distance_error * m_to_cm, 2) +
                    "\n yaw_err=" + toStringWithPrecision(track.yaw_error, 2);
      markers.push_back(marker);
    }
  };

  // Visualization
  visualization_msgs::msg::MarkerArray tracking_marker_array;
  std_msgs::msg::ColorRGBA initial_color;
  initial_color.r = 1.0;
  initial_color.a = 1.0;

  std_msgs::msg::ColorRGBA calibrated_color;
  calibrated_color.g = 1.0;
  calibrated_color.a = 1.0;

  for (const auto & track : converged_tracks) {
    const auto initial_radar_estimation_transformed =
      params_.initial_radar_to_lidar_eigen.inverse() * track.radar_estimation;
    const auto calibrated_radar_estimation_transformed =
      calibrated_radar_to_lidar_eigen.inverse() * track.radar_estimation;

    add_track_markers(
      track.lidar_estimation, initial_radar_estimation_transformed, track, "initial", initial_color,
      tracking_marker_array.markers);
    add_track_markers(
      track.lidar_estimation, calibrated_radar_estimation_transformed, track, "calibrated",
      calibrated_color, tracking_marker_array.markers);
  }

  return tracking_marker_array;
}

visualization_msgs::msg::MarkerArray Visualization::deleteTrackMarkers(
  const size_t converged_tracks_size)
{
  visualization_msgs::msg::MarkerArray tracking_marker_array;
  visualization_msgs::msg::Marker marker;

  for (size_t i = 0; i < converged_tracks_size * params_.marker_size_per_track; i++) {
    marker.id = i;
    marker.ns = "initial";
    marker.action = visualization_msgs::msg::Marker::DELETE;
    tracking_marker_array.markers.push_back(marker);

    marker.ns = "calibrated";
    marker.action = visualization_msgs::msg::Marker::DELETE;
    tracking_marker_array.markers.push_back(marker);
  }

  return tracking_marker_array;
}

visualization_msgs::msg::Marker Visualization::drawCalibrationStatusText(
  const size_t converged_tracks_size, TransformationType type, CalibrationErrorMetrics metrics)
{
  visualization_msgs::msg::Marker text_marker;

  text_marker.id = 0;
  text_marker.header = params_.lidar_header;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.ns = "calibration_status";
  text_marker.scale.z = 0.3;

  text_marker.text = toString(type) + "\npairs=" + std::to_string(converged_tracks_size);
  if (converged_tracks_size) {
    // Display average errors
    text_marker.text +=
      "\naverage_distance_error[cm]=" +
      toStringWithPrecision(metrics.calibrated_distance_errors.back() * m_to_cm, 2) +
      "\naverage_yaw_error[deg]=" + toStringWithPrecision(metrics.calibrated_yaw_errors.back(), 2);

    // Display cross-validation errors
    if (converged_tracks_size > 3) {
      text_marker.text +=
        "\ncrossval_distance_error[cm]=" +
        toStringWithPrecision(metrics.avg_crossval_calibrated_distance_errors.back() * m_to_cm, 2) +
        "\ncrossval_yaw_error[deg]=" +
        toStringWithPrecision(metrics.avg_crossval_calibrated_yaw_errors.back(), 2);
    }
  }
  text_marker.pose.position.x = 1.0;
  text_marker.pose.position.y = 1.0;
  text_marker.pose.position.z = 1.0;
  text_marker.pose.orientation.x = 0.0;
  text_marker.pose.orientation.y = 0.0;
  text_marker.pose.orientation.z = 0.0;
  text_marker.pose.orientation.w = 1.0;

  return text_marker;
}

}  // namespace marker_radar_lidar_calibrator
