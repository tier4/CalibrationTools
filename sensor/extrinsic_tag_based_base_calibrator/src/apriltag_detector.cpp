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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <extrinsic_tag_based_base_calibrator/apriltag_detector.hpp>
#include <extrinsic_tag_based_base_calibrator/math.hpp>
#include <opencv2/core/eigen.hpp>
#include <rclcpp/rclcpp.hpp>

#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h11.h>

#include <iostream>

namespace extrinsic_tag_based_base_calibrator
{

std::unordered_map<std::string, ApriltagDetector::create_family_fn_type>
  ApriltagDetector::tag_create_fn_map = {
    {"tag16h5", tag16h5_create},
    {"tag25h9", tag25h9_create},
    {"tag36h11", tag36h11_create},
};

std::unordered_map<std::string, ApriltagDetector::destroy_family_fn_type>
  ApriltagDetector::tag_destroy_fn_map = {
    {"tag16h5", tag16h5_destroy},
    {"tag25h9", tag25h9_destroy},
    {"tag36h11", tag36h11_destroy},
};

ApriltagDetector::ApriltagDetector(
  const ApriltagDetectorParameters & detector_parameters,
  const std::vector<TagParameters> & tag_parameters)
: detector_parameters_(detector_parameters),
  apriltag_detector_(apriltag_detector_create()),
  fx_(-1),
  fy_(-1),
  cx_(-1),
  cy_(-1)
{
  for (const auto & tag_parameters : tag_parameters) {
    const std::string & family_name = tag_parameters.family;
    assert(tag_create_fn_map.count(family_name) == 1);

    if (apriltag_family_map.count(family_name) != 0 || tag_create_fn_map.count(family_name) != 1) {
      continue;
    }

    apriltag_family_t * family_ptr = tag_create_fn_map[family_name]();
    apriltag_family_map[family_name] = family_ptr;

    apriltag_detector_add_family(apriltag_detector_, family_ptr);
  }

  apriltag_detector_->quad_decimate = detector_parameters_.quad_decimate;
  apriltag_detector_->quad_sigma = detector_parameters_.quad_sigma;
  apriltag_detector_->nthreads = detector_parameters_.nthreads;
  apriltag_detector_->debug = detector_parameters_.debug;
  apriltag_detector_->refine_edges = detector_parameters_.refine_edges;

  std::unordered_map<std::string, std::unordered_set<int>> tag_uniqueness_map;

  for (const auto & tag_parameters : tag_parameters) {
    tag_parameters_map_[tag_parameters.tag_type] = tag_parameters;
    tag_id_to_offset_map_[tag_parameters.tag_type];
    tag_uniqueness_map[tag_parameters.family];
    const std::string tag_family_name = tag_parameters.family;

    tag_sizes_map_[tag_parameters.tag_type] = tag_parameters.size;

    for (const auto & id : tag_parameters.ids) {
      for (int offset = 0; offset < tag_parameters.rows * tag_parameters.cols; offset++) {
        tag_id_to_offset_map_[tag_parameters.tag_type][id + offset] = offset;
        tag_family_and_id_to_type_map_[tag_family_name + "_" + std::to_string(id + offset)] =
          tag_parameters.tag_type;

        if (tag_uniqueness_map[tag_parameters.family].count(id + offset) != 0) {
          RCLCPP_FATAL(
            rclcpp::get_logger("apriltag_detector"),
            "The tag family %s has the tag %d more than once", tag_parameters.family.c_str(),
            id + offset);
        } else {
          tag_uniqueness_map[tag_parameters.family].emplace(id + offset);
        }
      }
    }
  }
}

ApriltagDetector::~ApriltagDetector()
{
  apriltag_detector_destroy(apriltag_detector_);

  for (auto it = apriltag_family_map.begin(); it != apriltag_family_map.end(); it++) {
    tag_destroy_fn_map.at(it->first)(it->second);
  }

  apriltag_family_map.clear();
}

void ApriltagDetector::setIntrinsics(double fx, double fy, double cx, double cy)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
}

GroupedApriltagGridDetections ApriltagDetector::detect(const cv::Mat & cv_img) const
{
  GroupedApriltagDetections individual_detections_map;
  GroupedApriltagGridDetections grid_detections_map;

  // Detect individual tags and filter out invalid ones
  image_u8_t apriltag_img = {cv_img.cols, cv_img.rows, cv_img.cols, cv_img.data};
  zarray_t * detections = apriltag_detector_detect(apriltag_detector_, &apriltag_img);

  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t * det;
    zarray_get(detections, i, &det);

    if (
      det->hamming > detector_parameters_.max_hamming ||
      det->decision_margin < detector_parameters_.min_margin) {
      continue;
    }

    ApriltagDetection result;
    result.id = det->id;
    result.center = cv::Point2d(det->c[0], det->c[1]);

    for (int i = 0; i < 4; ++i) {
      result.image_corners.emplace_back(det->p[i][0], det->p[i][1]);
    }

    // Extra filter since the dectector finds false positives with a high margin for out-of-plane
    // rotations
    double max_homography_error = 0.0;

    cv::Mat_<double> H(3, 3, det->H->data);
    cv::Mat H_inv = H.inv();

    for (const auto & image_corner : result.image_corners) {
      cv::Mat_<double> p_corner(3, 1);
      p_corner(0, 0) = image_corner.x;
      p_corner(1, 0) = image_corner.y;
      p_corner(2, 0) = 1.0;

      cv::Mat p_corner2 = H_inv * p_corner;

      // According to the equation (x2, y2, 1) = H *(x1, y1, 1) the third component should be 1.0
      double h_error = std::abs(p_corner2.at<double>(2, 0) - 1.0);
      max_homography_error = std::max(max_homography_error, h_error);
    }

    if (max_homography_error > detector_parameters_.max_homography_error) {
      RCLCPP_WARN(
        rclcpp::get_logger("apriltag_detector"),
        "Detection rejected due to its homography error. This may be due to its having a high "
        "out-of-plane rotation but we prefer to void them");
      continue;
    }

    std::string tag_family(det->family->name);
    std::string tag_family_and_id = tag_family + "_" + std::to_string(result.id);

    if (tag_family_and_id_to_type_map_.count(tag_family_and_id) == 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("apriltag_detector"),
        "Detected apriltag: %s \t but discarded since it is not part of the detection tags",
        tag_family_and_id.c_str());
      continue;
    }

    TagType tag_type = tag_family_and_id_to_type_map_.at(tag_family_and_id);
    double tag_size = tag_sizes_map_.at(tag_type);
    result.size = tag_size;
    result.computeTemplateCorners();

    if (fx_ > 0.0 && fy_ > 0.0 && cx_ > 0.0 && cy_ > 0.0) {
      apriltag_detection_info_t detection_info;
      detection_info.det = det;
      detection_info.fx = fx_;
      detection_info.fy = fy_;
      detection_info.cx = cx_;
      detection_info.cy = cy_;
      detection_info.tagsize = tag_size;

      apriltag_pose_t pose;
      estimate_tag_pose(&detection_info, &pose);

      cv::Matx33d rotation(pose.R->data);
      cv::Vec3d translation(pose.t->data);

      if (std::abs(cv::determinant(rotation) - 1.0) > 1e-5) {
        RCLCPP_WARN(
          rclcpp::get_logger("apriltag_detector"),
          "Detected apriltag: %s but dicarded due to its rotation not having unit determinant\t "
          "det=%.2f",
          tag_family_and_id.c_str(), std::abs(cv::determinant(rotation)));
        continue;
      }

      result.pose = cv::Affine3d(rotation, translation);
      result.size = tag_size;

      RCLCPP_WARN(
        rclcpp::get_logger("apriltag_detector"), "det=%.2f | det=%.2f", cv::determinant(rotation),
        cv::determinant(result.pose.rotation()));

      matd_destroy(pose.R);
      matd_destroy(pose.t);
    }

    cv::Point3d v_front = result.pose.rotation() * cv::Point3d(0.0, 0.0, 1.0);
    cv::Point3d v_to_tag = cv::Point3d(
      result.pose.translation()(0), result.pose.translation()(1), result.pose.translation()(2));
    v_to_tag = v_to_tag / cv::norm(v_to_tag);
    double rotation_angle = (180.0 / CV_PI) * std::acos(v_front.dot(v_to_tag));

    result.computeObjectCorners();
    double reprojection_error = result.computeReprojectionError(cx_, cy_, fx_, fy_);

    if (
      fx_ > 0.0 && fy_ > 0.0 && cx_ > 0.0 && cy_ > 0.0 &&
      reprojection_error > detector_parameters_.max_reprojection_error) {
      RCLCPP_WARN(
        rclcpp::get_logger("apriltag_detector"),
        "Detected apriltag: %s but dicarded due to its reprojection error\t margin: %.2f\t "
        "hom.error=%.2f\t repr.error=%.2f out_angle=%.2f deg",
        tag_family_and_id.c_str(), det->decision_margin, max_homography_error, reprojection_error,
        rotation_angle);
      continue;
    }

    if (
      fx_ > 0.0 && fy_ > 0.0 && cx_ > 0.0 && cy_ > 0.0 &&
      rotation_angle > detector_parameters_.max_out_of_plane_angle) {
      RCLCPP_WARN(
        rclcpp::get_logger("apriltag_detector"),
        "Detected apriltag: %s but dicarded due to its out-of-plane angle\t margin: %.2f\t "
        "hom.error=%.2f\t repr.error=%.2f out_angle=%.2f deg",
        tag_family_and_id.c_str(), det->decision_margin, max_homography_error, reprojection_error,
        rotation_angle);
      continue;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("apriltag_detector"),
      "Detected apriltag: %s \t margin: %.2f\t hom.error=%.2f\t repr.error=%.2f out_angle=%.2f deg",
      tag_family_and_id.c_str(), det->decision_margin, max_homography_error, reprojection_error,
      rotation_angle);

    individual_detections_map[tag_type].emplace_back(result);
  }

  apriltag_detections_destroy(detections);

  // Group tags into grid
  for (const auto & it : individual_detections_map) {
    const TagType & tag_type = it.first;
    const std::vector<ApriltagDetection> & detections = it.second;

    const TagParameters & tag_parameters = tag_parameters_map_.at(tag_type);
    const int & rows = tag_parameters.rows;
    const int & cols = tag_parameters.cols;
    const int & size = tag_parameters.size;

    std::unordered_map<int, std::vector<ApriltagDetection>> grouped_detections;

    // Group the detections into their respective grids
    for (const ApriltagDetection & detection : detections) {
      int base_id = detection.id - tag_id_to_offset_map_.at(tag_type).at(detection.id);
      grouped_detections[base_id].push_back(detection);
    }

    // Create the grid detections after checking their status and consistency
    for (const auto & grouped_detections_it : grouped_detections) {
      if (grouped_detections_it.second.size() != static_cast<std::size_t>(rows * cols)) {
        RCLCPP_INFO(
          rclcpp::get_logger("apriltag_detector"), "Discarding: id%d since it has size %lu",
          grouped_detections_it.first, grouped_detections_it.second.size());
        continue;
      }

      ApriltagGridDetection grid_detection;
      grid_detection.rows = rows;
      grid_detection.cols = cols;
      grid_detection.size = size;
      grid_detection.id = grouped_detections_it.first;
      grid_detection.family = tag_parameters.family;

      grid_detection.sub_detections.insert(
        grid_detection.sub_detections.end(), grouped_detections_it.second.begin(),
        grouped_detections_it.second.end());

      // Recompute the corners, center, and pose
      double max_distance = grid_detection.recomputeFromSubDetections(tag_parameters);
      if (max_distance > tag_parameters.size * (1.0 + tag_parameters.spacing)) {
        RCLCPP_WARN(
          rclcpp::get_logger("apriltag_detector"),
          "There was a misdetection filtered through pose consistency");
        continue;
      }

      grid_detections_map[tag_type].push_back(grid_detection);
    }
  }
  return grid_detections_map;
}

}  // namespace extrinsic_tag_based_base_calibrator
