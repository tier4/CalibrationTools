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
#include <string>

namespace extrinsic_tag_based_base_calibrator
{

void add_text_marker(
  visualization_msgs::msg::MarkerArray & markers, const std::string & text,
  const std_msgs::msg::ColorRGBA & color, const cv::Affine3f & pose,
  const visualization_msgs::msg::Marker & base_marker)
{
  visualization_msgs::msg::Marker marker;

  marker = base_marker;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.text = text;
  marker.color = color;
  marker.scale.z = 0.3;

  marker.pose.orientation.w = 1.f;
  marker.pose.position.x = pose.translation()(0);
  marker.pose.position.y = pose.translation()(1);
  marker.pose.position.z = pose.translation()(2);
  markers.markers.push_back(marker);
}

void add_tag_markers(
  visualization_msgs::msg::MarkerArray & markers, const std::string & tag_name, float size,
  const std_msgs::msg::ColorRGBA & color, const cv::Affine3f & pose,
  const visualization_msgs::msg::Marker & base_marker)
{
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

    cv::Matx31f p = pose.rotation() * template_points[normalized_index] + pose.translation();
    geometry_msgs::msg::Point p_msg;
    p_msg.x = p(0);
    p_msg.y = p(1);
    p_msg.z = p(2);

    marker.points.push_back(p_msg);
  }

  markers.markers.push_back(marker);
  marker.points.clear();

  add_text_marker(markers, tag_name, color, pose, base_marker);
}

void add_line_marker(
  visualization_msgs::msg::MarkerArray & markers, const std_msgs::msg::ColorRGBA & color,
  const cv::Vec3f & v1, const cv::Vec3f & v2, const visualization_msgs::msg::Marker & base_marker)
{
  visualization_msgs::msg::Marker marker;

  marker = base_marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.color = color;
  marker.scale.x = 0.005;

  auto vec_to_msg = [](const cv::Vec3f & v, geometry_msgs::msg::Point & p) {
    p.x = v(0);
    p.y = v(1);
    p.z = v(2);
  };

  geometry_msgs::msg::Point p1;
  geometry_msgs::msg::Point p2;

  vec_to_msg(v1, p1);
  vec_to_msg(v2, p2);

  marker.points.push_back(p1);
  marker.points.push_back(p2);

  markers.markers.push_back(marker);
}

void add_line_marker(
  visualization_msgs::msg::MarkerArray & markers, const std_msgs::msg::ColorRGBA & color,
  const cv::Affine3f & affine1, const cv::Affine3f & affine2,
  const visualization_msgs::msg::Marker & base_marker)
{
  add_line_marker(markers, color, affine1.translation(), affine2.translation(), base_marker);
}

void add_axes_markers(
  visualization_msgs::msg::MarkerArray & markers, float size, const cv::Affine3f & affine,
  const visualization_msgs::msg::Marker & base_marker)
{
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
}

void add_grid(
  visualization_msgs::msg::MarkerArray & markers, const cv::Affine3f & affine, int cells,
  float resolution, const visualization_msgs::msg::Marker & base_marker)
{
  int half_cells = cells / 2;

  std_msgs::msg::ColorRGBA color;
  color.a = 0.5;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;

  for (int i = -half_cells; i <= half_cells; i++) {
    cv::Vec3f v1 = affine * cv::Vec3f(i * resolution, -half_cells * resolution, 0.f);
    cv::Vec3f v2 = affine * cv::Vec3f(i * resolution, half_cells * resolution, 0.f);
    cv::Vec3f v3 = affine * cv::Vec3f(-half_cells * resolution, i * resolution, 0.f);
    cv::Vec3f v4 = affine * cv::Vec3f(half_cells * resolution, i * resolution, 0.f);
    add_line_marker(markers, color, v1, v2, base_marker);
    add_line_marker(markers, color, v3, v4, base_marker);
  }
}

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__VISUALIZATION_HPP_
