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

/*!
 * Adds a marker containing text
 * @param[out] markers the output markers vector
 * @param[in] text the text to be displayed in the marker
 * @param[in] color the color of the text
 * @param[in] pose the pose of the text
 * @param[in] base_marker the marker to use as a template
 */
void addTextMarker(
  visualization_msgs::msg::MarkerArray & markers, const std::string & text,
  const std_msgs::msg::ColorRGBA & color, const cv::Affine3d & pose,
  const visualization_msgs::msg::Marker & base_marker);

/*!
 * Adds markers representing a tag
 * @param[out] markers the output markers vector
 * @param[in] tag_name the text to be displayed in the marker
 * @param[in] size the size of the tag
 * @param[in] color the color of the text
 * @param[in] pose the pose of the tag
 * @param[in] base_marker the marker to use as a template
 */
void addTagMarkers(
  visualization_msgs::msg::MarkerArray & markers, const std::string & tag_name, double size,
  const std_msgs::msg::ColorRGBA & color, const cv::Affine3d & pose,
  const visualization_msgs::msg::Marker & base_marker);

/*!
 * Adds marker containing a line
 * @param[out] markers the output markers vector
 * @param[in] color the color of the text
 * @param[in] v1 the starting point of the line
 * @param[in] v2 the ending point of the line
 * @param[in] base_marker the marker to use as a template
 */
void addLineMarker(
  visualization_msgs::msg::MarkerArray & markers, const std_msgs::msg::ColorRGBA & color,
  const cv::Vec3d & v1, const cv::Vec3d & v2, const visualization_msgs::msg::Marker & base_marker);

/*!
 * Adds marker containing a line
 * @param[out] markers the output markers vector
 * @param[in] color the color of the text
 * @param[in] affine1 the starting point of the line
 * @param[in] affine2 the ending point of the line
 * @param[in] base_marker the marker to use as a template
 */
void addLineMarker(
  visualization_msgs::msg::MarkerArray & markers, const std_msgs::msg::ColorRGBA & color,
  const cv::Affine3d & affine1, const cv::Affine3d & affine2,
  const visualization_msgs::msg::Marker & base_marker);

/*!
 * Adds markers containing axes
 * @param[out] markers the output markers vector
 * @param[in] size the length of each axis
 * @param[in] affine the pose of the axes
 * @param[in] base_marker the marker to use as a template
 */
void addAxesMarkers(
  visualization_msgs::msg::MarkerArray & markers, double size, const cv::Affine3d & affine,
  const visualization_msgs::msg::Marker & base_marker);

/*!
 * Adds markers containing axes
 * @param[out] markers the output markers vector
 * @param[in] affine the pose of the grid center
 * @param[in] cells the number of cells to display
 * @param[in] resolution the distance between cells
 * @param[in] base_marker the marker to use as a template
 */
void addGrid(
  visualization_msgs::msg::MarkerArray & markers, const cv::Affine3d & affine, int cells,
  double resolution, const visualization_msgs::msg::Marker & base_marker);

/*!
 * Draws the detections in an image
 * @param[inout] img the image to draw on
 * @param[in] detection the detection to draw
 * @param[in] color the color to colour in
 */
void drawDetection(cv::Mat & img, const ApriltagDetection & detection, cv::Scalar color);

/*!
 * Draws the detections in an image
 * @param[inout] img the image to draw on
 * @param[in] center the center of the axes
 * @param[in] px the point in the x direction
 * @param[in] py the point in the y direction
 * @param[in] pz the point in the z direction
 */
void drawAxes(
  cv::Mat & img, const ApriltagDetection & detection, const cv::Point2d & center,
  const cv::Point2d & px, const cv::Point2d & py, const cv::Point2d & pz);

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__VISUALIZATION_HPP_
