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

#include <extrinsic_tag_based_sfm_calibrator/math.hpp>
#include <extrinsic_tag_based_sfm_calibrator/visualization.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

namespace extrinsic_tag_based_sfm_calibrator
{

void addTextMarker(
  visualization_msgs::msg::MarkerArray & markers, const std::string & text,
  const std_msgs::msg::ColorRGBA & color, const cv::Affine3d & pose,
  const visualization_msgs::msg::Marker & base_marker)
{
  visualization_msgs::msg::Marker marker;

  marker = base_marker;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.text = text;
  marker.color = color;
  marker.scale.z = 0.3;

  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = pose.translation()(0);
  marker.pose.position.y = pose.translation()(1);
  marker.pose.position.z = pose.translation()(2);
  markers.markers.push_back(marker);
}

void addTagMarkers(
  visualization_msgs::msg::MarkerArray & markers, const std::string & tag_name,
  const TagParameters & params, const std_msgs::msg::ColorRGBA & color, const cv::Affine3d & pose,
  const visualization_msgs::msg::Marker & base_marker)
{
  double factor = params.size * (1.0 + params.spacing);
  double size = params.size;

  std::array<cv::Matx31d, 4> template_points{
    cv::Matx31d{-0.5 * size, -0.5 * size, 0.0}, cv::Matx31d{0.5 * size, -0.5 * size, 0.0},
    cv::Matx31d{0.5 * size, 0.5 * size, 0.0}, cv::Matx31d{-0.5 * size, 0.5 * size, 0.0}};

  for (int row = 0; row < params.rows; row++) {
    for (int col = 0; col < params.cols; col++) {
      double corner_offset_x = (col - 0.5 * (params.cols - 1)) * factor;
      double corner_offset_y = (row - 0.5 * (params.rows - 1)) * factor;
      cv::Matx31d offset(corner_offset_x, corner_offset_y, 0.0);
      visualization_msgs::msg::Marker marker;

      marker = base_marker;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.color = color;
      marker.scale.x = 0.001;
      marker.points.clear();

      for (std::size_t index = 0; index < 5; ++index) {
        int normalized_index = index % 4;

        cv::Matx31d p =
          pose.rotation() * (template_points[normalized_index] + offset) + pose.translation();
        geometry_msgs::msg::Point p_msg;
        p_msg.x = p(0);
        p_msg.y = p(1);
        p_msg.z = p(2);

        marker.points.push_back(p_msg);
      }

      markers.markers.push_back(marker);
      marker.points.clear();
    }
  }

  addAxesMarkers(markers, 0.5 * size, pose, base_marker);
  addTextMarker(markers, tag_name, color, pose, base_marker);
}

void addLineMarker(
  visualization_msgs::msg::MarkerArray & markers, const std_msgs::msg::ColorRGBA & color,
  const cv::Vec3d & v1, const cv::Vec3d & v2, const visualization_msgs::msg::Marker & base_marker)
{
  visualization_msgs::msg::Marker marker;

  marker = base_marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.color = color;
  marker.scale.x = 0.001;

  auto vec_to_msg = [](const cv::Vec3d & v, geometry_msgs::msg::Point & p) {
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

void addLineMarker(
  visualization_msgs::msg::MarkerArray & markers, const std_msgs::msg::ColorRGBA & color,
  const cv::Affine3d & affine1, const cv::Affine3d & affine2,
  const visualization_msgs::msg::Marker & base_marker)
{
  addLineMarker(markers, color, affine1.translation(), affine2.translation(), base_marker);
}

void addAxesMarkers(
  visualization_msgs::msg::MarkerArray & markers, double size, const cv::Affine3d & affine,
  const visualization_msgs::msg::Marker & base_marker)
{
  visualization_msgs::msg::Marker marker;

  marker = base_marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = 0.01;
  marker.points.clear();

  std::array<cv::Matx31d, 3> axes_points{
    cv::Matx31d{size, 0.0, 0.0}, cv::Matx31d{0.0, size, 0.0}, cv::Matx31d{0.0, 0.0, size}};

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
    cv::Matx31d p = affine.rotation() * axes_points[index] + affine.translation();
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

void addGrid(
  visualization_msgs::msg::MarkerArray & markers, const cv::Affine3d & affine, int cells,
  double resolution, const visualization_msgs::msg::Marker & base_marker)
{
  int half_cells = cells / 2;

  std_msgs::msg::ColorRGBA color;
  color.a = 0.5;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;

  for (int i = -half_cells; i <= half_cells; i++) {
    cv::Vec3d v1 = affine * cv::Vec3d(i * resolution, -half_cells * resolution, 0.0);
    cv::Vec3d v2 = affine * cv::Vec3d(i * resolution, half_cells * resolution, 0.0);
    cv::Vec3d v3 = affine * cv::Vec3d(-half_cells * resolution, i * resolution, 0.0);
    cv::Vec3d v4 = affine * cv::Vec3d(half_cells * resolution, i * resolution, 0.0);
    addLineMarker(markers, color, v1, v2, base_marker);
    addLineMarker(markers, color, v3, v4, base_marker);
  }
}

void drawDetection(cv::Mat & img, const ApriltagDetection & detection, cv::Scalar color)
{
  std::vector<double> edge_sizes;
  cv::Point2d estimated_center(0.0, 0.0);

  for (std::size_t i = 0; i < detection.image_corners.size(); ++i) {
    std::size_t j = (i + 1) % detection.image_corners.size();
    edge_sizes.push_back(cv::norm(detection.image_corners[i] - detection.image_corners[j]));
    estimated_center += detection.image_corners[i];
  }

  double tag_size = *std::max_element(edge_sizes.begin(), edge_sizes.end());
  estimated_center /= 4;

  for (std::size_t i = 0; i < detection.image_corners.size(); ++i) {
    std::size_t j = (i + 1) % detection.image_corners.size();
    cv::line(
      img, detection.image_corners[i], detection.image_corners[j], color,
      static_cast<int>(std::max(tag_size / 512.0, 1.0)), cv::LINE_AA);

    cv::putText(
      img, std::to_string(i), detection.image_corners[i], cv::FONT_HERSHEY_SIMPLEX,
      std::max(tag_size / 256.0, 1.0), color, static_cast<int>(std::max(tag_size / 128.0, 1.0)));
  }

  cv::putText(
    img, std::to_string(detection.id), estimated_center, cv::FONT_HERSHEY_SIMPLEX,
    std::max(tag_size / 128.0, 1.0), color, static_cast<int>(std::max(tag_size / 128.0, 1.0)));
}

void drawAxes(
  cv::Mat & img, const ApriltagDetection & detection, const cv::Affine3d & camera_to_tag_pose,
  const std::array<double, 6> & intrinsics, double thickness_factor)
{
  cv::Vec3d px3d = camera_to_tag_pose * cv::Vec3d(0.5 * detection.size, 0.0, 0.0);
  cv::Vec3d py3d = camera_to_tag_pose * cv::Vec3d(0.0, 0.5 * detection.size, 0.0);
  cv::Vec3d pz3d = camera_to_tag_pose * cv::Vec3d(0.0, 0.0, 0.5 * detection.size);
  cv::Vec3d center3d = camera_to_tag_pose.translation();
  cv::Point2d px2d = projectPoint(px3d, intrinsics);
  cv::Point2d py2d = projectPoint(py3d, intrinsics);
  cv::Point2d pz2d = projectPoint(pz3d, intrinsics);
  cv::Point2d center2d = projectPoint(center3d, intrinsics);

  std::vector<double> edge_sizes;

  for (std::size_t i = 0; i < detection.image_corners.size(); ++i) {
    std::size_t j = (i + 1) % detection.image_corners.size();
    edge_sizes.push_back(cv::norm(detection.image_corners[i] - detection.image_corners[j]));
  }

  double tag_size = *std::max_element(edge_sizes.begin(), edge_sizes.end());

  cv::line(
    img, center2d, px2d, cv::Scalar(0, 0, 255),
    static_cast<int>(std::max(thickness_factor * tag_size / 512.0, 1.0)), cv::LINE_AA);

  cv::line(
    img, center2d, py2d, cv::Scalar(0, 255, 0),
    static_cast<int>(std::max(thickness_factor * tag_size / 512.0, 1.0)), cv::LINE_AA);

  cv::line(
    img, center2d, pz2d, cv::Scalar(255, 0, 0),
    static_cast<int>(std::max(thickness_factor * tag_size / 512.0, 1.0)), cv::LINE_AA);
}

void drawAxes(
  cv::Mat & undistorted_image, const ApriltagDetection & detection,
  const IntrinsicParameters & intrinsics)
{
  double fx = intrinsics.undistorted_camera_matrix(0, 0);
  double fy = intrinsics.undistorted_camera_matrix(1, 1);
  double cx = intrinsics.undistorted_camera_matrix(0, 2);
  double cy = intrinsics.undistorted_camera_matrix(1, 2);

  cv::Vec3d px3d = detection.pose * cv::Vec3d(0.5 * detection.size, 0.0, 0.0);
  cv::Vec3d py3d = detection.pose * cv::Vec3d(0.0, 0.5 * detection.size, 0.0);
  cv::Vec3d pz3d = detection.pose * cv::Vec3d(0.0, 0.0, 0.5 * detection.size);
  cv::Vec3d center3d = detection.pose.translation();
  cv::Point2d px2d = projectPoint(px3d, fx, fy, cx, cy, 0.0, 0.0);
  cv::Point2d py2d = projectPoint(py3d, fx, fy, cx, cy, 0.0, 0.0);
  cv::Point2d pz2d = projectPoint(pz3d, fx, fy, cx, cy, 0.0, 0.0);
  cv::Point2d center2d = projectPoint(center3d, fx, fy, cx, cy, 0.0, 0.0);

  std::cout << "Drawing axes for id=" << detection.id << std::endl;
  std::cout << "\t pose translation =" << detection.pose.translation() << std::endl;
  std::cout << "\t pose translation =" << detection.pose.rotation() << std::endl;
  std::cout << "\t delta pz3d =" << pz3d - center3d << std::endl;
  std::cout << "\t delta pz2d =" << pz2d - center2d << std::endl;

  std::vector<double> edge_sizes;

  for (std::size_t i = 0; i < detection.image_corners.size(); ++i) {
    std::size_t j = (i + 1) % detection.image_corners.size();
    edge_sizes.push_back(cv::norm(detection.image_corners[i] - detection.image_corners[j]));
  }

  double tag_size = *std::max_element(edge_sizes.begin(), edge_sizes.end());

  cv::line(
    undistorted_image, center2d, px2d, cv::Scalar(0, 0, 255),
    static_cast<int>(std::max(tag_size / 512.0, 1.0)), cv::LINE_AA);

  cv::line(
    undistorted_image, center2d, py2d, cv::Scalar(0, 255, 0),
    static_cast<int>(std::max(tag_size / 512.0, 1.0)), cv::LINE_AA);

  cv::line(
    undistorted_image, center2d, pz2d, cv::Scalar(255, 0, 0),
    static_cast<int>(std::max(tag_size / 512.0, 1.0)), cv::LINE_AA);
}

}  // namespace extrinsic_tag_based_sfm_calibrator
