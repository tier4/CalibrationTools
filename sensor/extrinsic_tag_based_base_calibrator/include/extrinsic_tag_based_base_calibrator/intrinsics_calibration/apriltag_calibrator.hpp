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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__INTRINSICS_CALIBRATION__APRILTAG_CALIBRATOR_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__INTRINSICS_CALIBRATION__APRILTAG_CALIBRATOR_HPP_

#include <extrinsic_tag_based_base_calibrator/apriltag_detection.hpp>
#include <extrinsic_tag_based_base_calibrator/apriltag_detector.hpp>
#include <extrinsic_tag_based_base_calibrator/intrinsics_calibration/intrinsics_calibrator.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace extrinsic_tag_based_base_calibrator
{

class ApriltagBasedCalibrator : public IntrinsicsCalibrator
{
public:
  ApriltagBasedCalibrator(
    const ApriltagDetectorParameters & detector_parameters, const TagParameters & tag_parameters,
    bool use_tangent_distortion, int num_radial_distortion_coeffs, bool debug = false)
  : IntrinsicsCalibrator(use_tangent_distortion, num_radial_distortion_coeffs, debug),
    detector_(detector_parameters, {tag_parameters})  //,
  {
  }

protected:
  void extractCalibrationPoints() override;
  void writeDebugImages(const IntrinsicParameters & intrinsics) override;

  ApriltagDetector detector_;

  std::unordered_map<std::string, std::vector<int>> filtered_image_file_name_to_calibration_id_map_;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__INTRINSICS_CALIBRATION__APRILTAG_CALIBRATOR_HPP_
