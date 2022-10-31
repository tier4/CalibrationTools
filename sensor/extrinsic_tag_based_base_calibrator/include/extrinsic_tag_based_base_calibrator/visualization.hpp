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

#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>

visualization_msgs::msg::MarkerArray create_tag_markers(
  int tag_id, float size, const std_msgs::msg::ColorRGBA & color, const cv::Affine3f & affine,
  const visualization_msgs::msg::Marker & base_marker)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;

  marker = base_marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.color = color;
  marker.scale.x = 0.01;
  marker.points.clear();

  std::array<cv::Matx31f, 4> template_points{
    cv::Matx31f{-0.5f * size, -0.5f * size, 0.f}, cv::Matx31f{0.5f * size, -0.5f * size, 0.f},
    cv::Matx31f{0.5f * size, 0.5f * size, 0.f}, cv::Matx31f{-0.5f * size, 0.5f * size, 0.f}};

  for (std::size_t index = 0; index < 5; ++index) {
    int normalized_index = index % 4;

    cv::Matx31f p = affine.rotation() * template_points[normalized_index] + affine.translation();
    geometry_msgs::msg::Point p_msg;
    p_msg.x = p(0);
    p_msg.y = p(1);
    p_msg.z = p(2);

    marker.points.push_back(p_msg);
  }

  markers.markers.push_back(marker);
  marker.points.clear();

  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.text = std::to_string(tag_id);
  marker.scale.z = 0.3;
  marker.pose.orientation.w = 1.f;
  marker.pose.position.x = affine.translation()(0);
  marker.pose.position.y = affine.translation()(1);
  marker.pose.position.z = affine.translation()(2);
  markers.markers.push_back(marker);

  return markers;
}

visualization_msgs::msg::MarkerArray create_line(
  const std_msgs::msg::ColorRGBA & color, const cv::Affine3f & affine1,
  const cv::Affine3f & affine2, const visualization_msgs::msg::Marker & base_marker)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;

  marker = base_marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.color = color;
  marker.scale.x = 0.01;

  auto affine_to_msg = [](const cv::Affine3f & affine, geometry_msgs::msg::Point & p) {
    p.x = affine.translation()(0);
    p.y = affine.translation()(1);
    p.z = affine.translation()(2);
  };

  geometry_msgs::msg::Point p1;
  geometry_msgs::msg::Point p2;

  affine_to_msg(affine1, p1);
  affine_to_msg(affine2, p2);

  marker.points.push_back(p1);
  marker.points.push_back(p2);

  markers.markers.push_back(marker);
  return markers;
}

visualization_msgs::msg::MarkerArray create_axes_markers(
  float size, const cv::Affine3f & affine, const visualization_msgs::msg::Marker & base_marker)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;

  marker = base_marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = 0.01;
  marker.points.clear();

  std::array<cv::Matx31f, 3> axes_points{
    cv::Matx31f{size, 0.f, 0.f}, cv::Matx31f{0.f, size, 0.f}, cv::Matx31f{0.f, 0.f, size}};

  std::array<std_msgs::msg::ColorRGBA, 3> colors;
  colors[0].r = 1.f;
  colors[1].g = 1.f;
  colors[2].b = 1.f;
  colors[0].a = 1.f;
  colors[1].a = 1.f;
  colors[2].a = 1.f;

  geometry_msgs::msg::Point p_center;
  p_center.x = affine.translation()(0);
  p_center.y = affine.translation()(1);
  p_center.z = affine.translation()(2);

  for (std::size_t index = 0; index < 3; ++index) {
    cv::Matx31f p = affine.rotation() * axes_points[index] + affine.translation();
    geometry_msgs::msg::Point p_msg;
    p_msg.x = p(0);
    p_msg.y = p(1);
    p_msg.z = p(2);

    marker.color = colors[index];
    marker.points.clear();
    marker.points.push_back(p_center);
    marker.points.push_back(p_msg);
    markers.markers.push_back(marker);
  }

  return markers;
}

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__VISUALIZATION_HPP_
