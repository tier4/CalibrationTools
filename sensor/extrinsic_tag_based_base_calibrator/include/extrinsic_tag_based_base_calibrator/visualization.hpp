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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__VISUALIZATION_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__VISUALIZATION_HPP_

#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <string>

namespace extrinsic_tag_based_base_calibrator
{

void addTextMarker(
  visualization_msgs::msg::MarkerArray & markers, const std::string & text,
  const std_msgs::msg::ColorRGBA & color, const cv::Affine3f & pose,
  const visualization_msgs::msg::Marker & base_marker);

void addTagMarkers(
  visualization_msgs::msg::MarkerArray & markers, const std::string & tag_name, float size,
  const std_msgs::msg::ColorRGBA & color, const cv::Affine3f & pose,
  const visualization_msgs::msg::Marker & base_marker);

void addLineMarker(
  visualization_msgs::msg::MarkerArray & markers, const std_msgs::msg::ColorRGBA & color,
  const cv::Vec3f & v1, const cv::Vec3f & v2, const visualization_msgs::msg::Marker & base_marker);

void addLineMarker(
  visualization_msgs::msg::MarkerArray & markers, const std_msgs::msg::ColorRGBA & color,
  const cv::Affine3f & affine1, const cv::Affine3f & affine2,
  const visualization_msgs::msg::Marker & base_marker);

void addAxesMarkers(
  visualization_msgs::msg::MarkerArray & markers, float size, const cv::Affine3f & affine,
  const visualization_msgs::msg::Marker & base_marker);

void addGrid(
  visualization_msgs::msg::MarkerArray & markers, const cv::Affine3f & affine, int cells,
  float resolution, const visualization_msgs::msg::Marker & base_marker);

void drawDetection(cv::Mat & img, const ApriltagDetection & detection, cv::Scalar color);

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__VISUALIZATION_HPP_
