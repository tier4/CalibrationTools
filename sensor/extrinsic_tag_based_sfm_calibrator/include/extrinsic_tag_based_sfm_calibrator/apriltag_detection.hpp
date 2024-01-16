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

#ifndef EXTRINSIC_TAG_BASED_SFM_CALIBRATOR__APRILTAG_DETECTION_HPP_
#define EXTRINSIC_TAG_BASED_SFM_CALIBRATOR__APRILTAG_DETECTION_HPP_

#include <extrinsic_tag_based_sfm_calibrator/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <lidartag_msgs/msg/lidar_tag_detection.hpp>

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace extrinsic_tag_based_sfm_calibrator
{

struct LidartagDetection
{
  static LidartagDetection fromLidartagDetectionMsg(
    const lidartag_msgs::msg::LidarTagDetection & msg, double scale_factor);
  void computeObjectCorners();
  void computeTemplateCorners(double new_size);
  void computeTemplateCorners();

  std::string family = "";
  int id = -1;
  std::vector<cv::Point3d> object_corners;
  std::vector<cv::Point3d> template_corners;
  cv::Affine3d pose;
  double size = 0.0;
};

struct ApriltagDetection : public LidartagDetection
{
  static ApriltagDetection fromApriltagDetectionMsg(
    const apriltag_msgs::msg::AprilTagDetection & msg, const IntrinsicParameters & intrinsics,
    double size);
  double computePose(const IntrinsicParameters & intrinsics);
  double computeReprojectionError(const IntrinsicParameters & intrinsics) const;
  double computeReprojectionError(double cx, double cy, double fx, double fy) const;
  double detectionDiagonalRatio() const;

  std::vector<cv::Point2d> image_corners;

  cv::Point2d center;
};

struct ApriltagGridDetection : public ApriltagDetection
{
  ApriltagGridDetection() {}

  explicit ApriltagGridDetection(const ApriltagDetection & other)
  {
    family = other.family;
    id = other.id;
    object_corners = other.object_corners;
    template_corners = other.template_corners;
    pose = other.pose;
    size = other.size;
    image_corners = other.image_corners;
    center = other.center;
    rows = 1;
    cols = 1;
    sub_detections.push_back(other);
  }

  static std::map<TagType, std::vector<ApriltagGridDetection>> fromGroupedApriltagDetections(
    const std::map<TagType, std::vector<ApriltagDetection>> & grouped_detection,
    std::unordered_map<TagType, TagParameters> & parameters_map);

  void computeObjectCorners();
  void computeTemplateCorners(const TagParameters & tag_parameters);
  double recomputeFromSubDetections(const TagParameters & tag_parameters);
  double detectionDiagonalRatio() const;

  std::vector<ApriltagDetection> sub_detections;
  int rows;
  int cols;
};

using LidartagDetections = std::vector<LidartagDetection>;
using ApriltagDetections = std::vector<ApriltagDetection>;
using ApriltagGridDetections = std::vector<ApriltagGridDetection>;
using GroupedApriltagDetections = std::map<TagType, ApriltagDetections>;
using GroupedApriltagGridDetections = std::map<TagType, ApriltagGridDetections>;

}  // namespace extrinsic_tag_based_sfm_calibrator

#endif  // EXTRINSIC_TAG_BASED_SFM_CALIBRATOR__APRILTAG_DETECTION_HPP_
