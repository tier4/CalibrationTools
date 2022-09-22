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

#include <extrinsic_mapping_based_calibrator/filters/object_detection_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#define UNUSED(x) (void)x;

void ObjectDetectionFilter::setName(const std::string & name)
{
  name_ = name + " (ObjectDetectionFilter)";
}

std::vector<CalibrationFrame> ObjectDetectionFilter::filter(
  const std::vector<CalibrationFrame> & calibration_frames, MappingData::Ptr & mapping_data)
{
  // Get the initial source -> target tf
  // Get the tf between the detections' frame and the mapping/target one
  // Make sure all the detections are in the same frame

  for (auto & calibration_frame : calibration_frames) {
    filter(calibration_frame, mapping_data);
  }

  return calibration_frames;
}

void ObjectDetectionFilter::filter(
  const CalibrationFrame & calibration_frame, MappingData::Ptr & mapping_data)
{
  // Here compute the min/max of the source pointcloud in its own frame

  for (auto & objects : mapping_data->detected_objects_) {
    filter(calibration_frame, mapping_data, objects);
  }
}

void ObjectDetectionFilter::filter(
  const CalibrationFrame & calibration_frame, MappingData::Ptr & mapping_data,
  const ObjectsBB & objects)
{
  // Interpolate the pose of the detections
  for (auto & object : objects.objects_) {
    filter(calibration_frame, mapping_data, object);
  }
}

void ObjectDetectionFilter::filter(
  const CalibrationFrame & calibration_frame, MappingData::Ptr & mapping_data,
  const ObjectBB & object)
{
  // Compute the vertices of the bbox into the source frame
  // If not of them are into the area of the source pointcloud abort

  // Otherwise project the source pointcloud int othe detection's frame and apply negative crop box
  // filter Win
}
