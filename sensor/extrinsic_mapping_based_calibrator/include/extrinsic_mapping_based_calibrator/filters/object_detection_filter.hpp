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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR__FILTERS__OBJECT_DETECTION_FILTER_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR__FILTERS__OBJECT_DETECTION_FILTER_HPP_

#include <extrinsic_mapping_based_calibrator/filters/filter.hpp>
#include <extrinsic_mapping_based_calibrator/types.hpp>

#include <tf2_ros/buffer.h>

#include <vector>

class ObjectDetectionFilter : public Filter
{
public:
  ObjectDetectionFilter(
    const CalibrationParameters::Ptr & parameters,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
  : Filter(parameters), tf_buffer_(tf_buffer)
  {
    name_ = "ObjectDetectionFilter";
  }
  ObjectDetectionFilter(
    const std::string & name, const CalibrationParameters::Ptr & parameters,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
  : Filter(parameters), tf_buffer_(tf_buffer)
  {
    setName(name);
  }
  virtual ~ObjectDetectionFilter() {}

  virtual std::vector<CalibrationFrame> filter(
    const std::vector<CalibrationFrame> & calibration_frames,
    MappingData::Ptr & mapping_data) override;
  virtual void setName(const std::string & name) override;

protected:
  void filter(
    const CalibrationFrame & calibration_frame, MappingData::Ptr & mapping_data,
    const Eigen::Affine3f & source_lidar_to_mapping_lidar_transform,
    const Eigen::Affine3f & mapping_to_detection_frame_transform);
  void filter(
    const CalibrationFrame & calibration_frame, MappingData::Ptr & mapping_data,
    const ObjectsBB & objects, const Eigen::Affine3f & source_lidar_to_mapping_lidar_transform,
    const Eigen::Affine3f & mapping_to_detection_frame_transform, const Eigen::Vector4f & min_p,
    const Eigen::Vector4f & max_p);
  void filter(
    const CalibrationFrame & calibration_frame, const ObjectBB & object,
    const Eigen::Affine3f & source_to_detections_transform, const Eigen::Vector4f & min_p,
    const Eigen::Vector4f & max_p);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR__FILTERS__OBJECT_DETECTION_FILTER_HPP_
