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
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>

#include <string>
#include <utility>
#include <vector>

namespace marker_radar_lidar_calibrator
{

std::string toString(TransformationType type);
std::string toStringWithPrecision(const float value, const int n);
geometry_msgs::msg::Point eigenToPointMsg(const Eigen::Vector3d & p_eigen);

void updateTrackIds(std::vector<Track> & converged_tracks);

std::pair<double, double> computeCalibrationError(
  std::vector<Track>::iterator & begin, std::vector<Track>::iterator & end,
  const TransformationType transformation_type, const Eigen::Isometry3d & radar_to_lidar_isometry,
  const bool record_error_in_track);
double getDistanceError(
  TransformationType transformation_type, Eigen::Vector3d v1, Eigen::Vector3d v2);
double getYawError(Eigen::Vector3d v1, Eigen::Vector3d v2);

size_t combination_count(const size_t n, const size_t k);
void generateAllCombinations(
  const std::size_t n, const std::size_t k, std::vector<std::vector<std::size_t>> & combinations);
void selectCombinations(
  const std::size_t n, const std::size_t k, const std::size_t max_number_of_combination_samples,
  std::vector<std::vector<std::size_t>> & combinations);

// Load database
void parseHeader(
  std::ifstream & file, const std::string & header_name, std_msgs::msg::Header & header);
void parseConvergedTracks(std::ifstream & file, std::vector<Track> & converged_tracks);

}  // namespace marker_radar_lidar_calibrator
