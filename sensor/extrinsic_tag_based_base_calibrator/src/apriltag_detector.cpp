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

#include <extrinsic_tag_based_base_calibrator/apriltag_detector.hpp>
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
    {"16h5", tag16h5_create},
    {"25h9", tag25h9_create},
    {"36h11", tag36h11_create},
};

std::unordered_map<std::string, ApriltagDetector::destroy_family_fn_type>
  ApriltagDetector::tag_destroy_fn_map = {
    {"16h5", tag16h5_destroy},
    {"25h9", tag25h9_destroy},
    {"36h11", tag36h11_destroy},
};

ApriltagDetector::ApriltagDetector(const ApriltagParameters & parameters)
: parameters_(parameters),
  apriltag_detector_(apriltag_detector_create()),
  fx_(-1),
  fy_(-1),
  cx_(-1),
  cy_(-1)
{
  assert(tag_create_fn_map.count(parameters_.family) == 1);
  apriltag_family_ = tag_create_fn_map[parameters_.family]();
  apriltag_detector_add_family(apriltag_detector_, apriltag_family_);

  apriltag_detector_->quad_decimate = parameters_.quad_decimate;
  apriltag_detector_->quad_sigma = parameters_.quad_sigma;
  apriltag_detector_->nthreads = parameters_.nthreads;
  apriltag_detector_->debug = parameters_.debug;
  apriltag_detector_->refine_edges = parameters_.refine_edges;
}

ApriltagDetector::~ApriltagDetector()
{
  apriltag_detector_destroy(apriltag_detector_);
  tag_destroy_fn_map.at(parameters_.family)(apriltag_family_);
}

void ApriltagDetector::setTagSizes(const std::unordered_map<int, double> & tag_sizes_map)
{
  tag_sizes_map_ = tag_sizes_map;
}

void ApriltagDetector::setIntrinsics(double fx, double fy, double cx, double cy)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
}

std::vector<ApriltagDetection> ApriltagDetector::detect(const cv::Mat & cv_img) const
{
  std::vector<ApriltagDetection> results;
  image_u8_t apriltag_img = {cv_img.cols, cv_img.rows, cv_img.cols, cv_img.data};

  zarray_t * detections = apriltag_detector_detect(apriltag_detector_, &apriltag_img);

  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t * det;
    zarray_get(detections, i, &det);

    if (det->hamming > parameters_.max_hamming || det->decision_margin < parameters_.min_margin) {
      continue;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("intrinsics_calibrator"), "Detected apriltag: %d \t margin: %.2f", det->id,
      det->decision_margin);

    ApriltagDetection result;
    result.id = det->id;
    result.center = cv::Point2d(det->c[0], det->c[1]);

    for (int i = 0; i < 4; ++i) {
      result.corners.emplace_back(det->p[i][0], det->p[i][1]);
    }

    if (tag_sizes_map_.count(det->id) > 0 && fx_ > 0.0 && fy_ > 0.0 && cx_ > 0.0 && cy_ > 0.0) {
      apriltag_detection_info_t detection_info;
      detection_info.det = det;
      detection_info.fx = fx_;
      detection_info.fy = fy_;
      detection_info.cx = cx_;
      detection_info.cy = cy_;
      detection_info.tagsize = tag_sizes_map_.at(det->id);

      apriltag_pose_t pose;
      estimate_tag_pose(&detection_info, &pose);

      cv::Matx33d rotation(pose.R->data);
      cv::Vec3d translation(pose.t->data);

      result.pose = cv::Affine3d(rotation, translation);
      result.size = tag_sizes_map_.at(det->id);

      matd_destroy(pose.R);
      matd_destroy(pose.t);
    }

    results.emplace_back(result);
  }

  apriltag_detections_destroy(detections);

  return results;
}

}  // namespace extrinsic_tag_based_base_calibrator
