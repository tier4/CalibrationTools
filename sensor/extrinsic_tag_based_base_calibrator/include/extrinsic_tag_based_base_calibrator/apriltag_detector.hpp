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

class ApriltagDetector
{
public:
  typedef apriltag_family_t * (*create_family_fn_type)();
  typedef void (*destroy_family_fn_type)(apriltag_family_t *);

  explicit ApriltagDetector(const ApriltagParameters & parameters);
  ~ApriltagDetector();

  void setTagSizes(const std::unordered_map<int, float> & tag_sizes_map);
  void setIntrinsics(float fx, float fy, float cx, float cy);

  std::vector<ApriltagDetection> detect(const cv::Mat & img) const;

protected:
  ApriltagParameters parameters_;
  apriltag_family_t * apriltag_family_;
  apriltag_detector_t * apriltag_detector_;

  std::unordered_map<int, float> tag_sizes_map_;
  float fx_, fy_, cx_, cy_;

  static std::unordered_map<std::string, create_family_fn_type> tag_create_fn_map;
  static std::unordered_map<std::string, destroy_family_fn_type> tag_destroy_fn_map;
};

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__APRILTAG_DETECTOR_HPP_
