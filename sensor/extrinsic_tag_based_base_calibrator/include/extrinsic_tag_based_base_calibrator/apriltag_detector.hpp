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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__APRILTAG_DETECTOR_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__APRILTAG_DETECTOR_HPP_

#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <opencv2/core.hpp>

#include <apriltag/apriltag.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace extrinsic_tag_based_base_calibrator
{

class ApriltagDetector
{
public:
  typedef apriltag_family_t * (*create_family_fn_type)();
  typedef void (*destroy_family_fn_type)(apriltag_family_t *);

  explicit ApriltagDetector(const ApriltagParameters & parameters);
  ~ApriltagDetector();

  /*!
   * Sets the id-dependent tag sizes to estimate the 3d poses
   * @param[in] tag_sizes_map A map containing the size (side-to-side) of the tags
   */
  void setTagSizes(const std::unordered_map<int, double> & tag_sizes_map);

  /*!
   * Sets the intrinsics of the camera for the detector, which are used to estimate the 3d pose of
   * the detections
   * @param[in] fx Focal distance
   * @param[in] fy Focal distance
   * @param[in] cx Optical center
   * @param[in] cy Optical center
   */
  void setIntrinsics(double fx, double fy, double cx, double cy);

  /*!
   * Detetect all the apriltags in an image filtering by hamming distance and detection margin
   * If the tag size and intrinsics are known, it also estiamtes the 3d pose of the tag
   * @param[in] img The image to obtain detections from
   * @return a vector of ApriltagDetection found in the img
   */
  std::vector<ApriltagDetection> detect(const cv::Mat & img) const;

protected:
  ApriltagParameters parameters_;
  apriltag_family_t * apriltag_family_;
  apriltag_detector_t * apriltag_detector_;

  std::unordered_map<int, double> tag_sizes_map_;
  double fx_, fy_, cx_, cy_;

  static std::unordered_map<std::string, create_family_fn_type> tag_create_fn_map;
  static std::unordered_map<std::string, destroy_family_fn_type> tag_destroy_fn_map;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__APRILTAG_DETECTOR_HPP_
