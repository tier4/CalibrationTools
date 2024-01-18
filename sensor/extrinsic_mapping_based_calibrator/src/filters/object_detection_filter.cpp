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

#include <extrinsic_mapping_based_calibrator/filters/object_detection_filter.hpp>
#include <extrinsic_mapping_based_calibrator/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

#include <array>
#include <limits>

#define UNUSED(x) (void)x;

void ObjectDetectionFilter::setName(const std::string & name)
{
  name_ = name + " (ObjectDetectionFilter)";
}

std::vector<CalibrationFrame> ObjectDetectionFilter::filter(
  const std::vector<CalibrationFrame> & calibration_frames, MappingData::Ptr & mapping_data)
{
  if (mapping_data->detected_objects_.size() == 0 || calibration_frames.size() == 0) {
    return calibration_frames;
  }

  Eigen::Affine3f mapping_lidar_to_detection_frame_transform;
  Eigen::Affine3f source_lidar_to_mapping_lidar_transform;

  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    geometry_msgs::msg::Transform mapping_lidar_to_detection_frame_msg_ =
      tf_buffer_
        ->lookupTransform(
          mapping_data->mapping_lidar_frame_,
          mapping_data->detected_objects_.front().header_.frame_id, t, timeout)
        .transform;

    geometry_msgs::msg::Transform source_lidar_to_mapping_lidar_msg_ =
      tf_buffer_
        ->lookupTransform(
          calibration_frames.front().source_header_.frame_id, mapping_data->mapping_lidar_frame_, t,
          timeout)
        .transform;

    mapping_lidar_to_detection_frame_transform =
      tf2::transformToEigen(mapping_lidar_to_detection_frame_msg_).cast<float>();
    source_lidar_to_mapping_lidar_transform =
      tf2::transformToEigen(source_lidar_to_mapping_lidar_msg_).cast<float>();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      rclcpp::get_logger(name_), "Could not get initial tfs. Aborting filter. %s", ex.what());
    return calibration_frames;
  }

  for (const auto & det : mapping_data->detected_objects_) {
    UNUSED(det);
    assert(mapping_data->detected_objects_.front().header_.frame_id == det.header_.frame_id);
  }

  // Detections should be sorted via timestamps but we sort them anyways
  std::sort(
    mapping_data->detected_objects_.begin(), mapping_data->detected_objects_.end(),
    [](auto & lhs, auto & rhs) {
      return rclcpp::Time(lhs.header_.stamp) < rclcpp::Time(rhs.header_.stamp);
    });

  for (auto & calibration_frame : calibration_frames) {
    filter(
      calibration_frame, mapping_data, source_lidar_to_mapping_lidar_transform,
      mapping_lidar_to_detection_frame_transform);
  }

  return calibration_frames;
}

void ObjectDetectionFilter::filter(
  const CalibrationFrame & calibration_frame, MappingData::Ptr & mapping_data,
  const Eigen::Affine3f & source_lidar_to_mapping_lidar_transform,
  const Eigen::Affine3f & mapping_lidar_to_detection_frame_transform)
{
  // Compute the min/max from the source pointcloud to use as axis-aligned-bounding-boxes on the
  // source pointcloud frame
  Eigen::Array4f min_p, max_p;
  min_p.setConstant(std::numeric_limits<float>::max());
  max_p.setConstant(-std::numeric_limits<float>::max());

  for (const auto & point : *calibration_frame.source_pointcloud_) {
    pcl::Array4fMapConst pt = point.getArray4fMap();
    min_p = min_p.min(pt);
    max_p = max_p.max(pt);
  }

  Eigen::Vector4f min_vector = min_p;
  Eigen::Vector4f max_vector = max_p;
  min_vector.w() = 1.f;
  max_vector.w() = 1.f;

  // Obtain all detections close in time to the source pointcloud
  auto left_predicate = [&](const ObjectsBB & objects) {
    return rclcpp::Time(objects.header_.stamp) +
             rclcpp::Duration::from_seconds(parameters_->detection_max_time_tolerance_) >
           rclcpp::Time(calibration_frame.source_header_.stamp);
  };

  auto right_predicate = [&](const ObjectsBB & objects) {
    return rclcpp::Time(objects.header_.stamp) >
           rclcpp::Time(calibration_frame.source_header_.stamp) +
             rclcpp::Duration::from_seconds(parameters_->detection_max_time_tolerance_);
  };

  std::vector<ObjectsBB>::iterator left_it, right_it;

  left_it = std::find_if(
    mapping_data->detected_objects_.begin(), mapping_data->detected_objects_.end(), left_predicate);
  right_it = std::find_if(left_it, mapping_data->detected_objects_.end(), right_predicate);

  for (auto & it = left_it; it != right_it; ++it) {
    filter(
      calibration_frame, mapping_data, *it, source_lidar_to_mapping_lidar_transform,
      mapping_lidar_to_detection_frame_transform, min_p, max_p);
  }
}

void ObjectDetectionFilter::filter(
  const CalibrationFrame & calibration_frame, MappingData::Ptr & mapping_data,
  const ObjectsBB & objects, const Eigen::Affine3f & source_lidar_to_mapping_lidar_transform,
  const Eigen::Affine3f & mapping_lidar_to_detection_frame_transform, const Eigen::Vector4f & min_p,
  const Eigen::Vector4f & max_p)
{
  // Compute the mapping lidar's pose in the map frame at the detection's timestamp
  auto predicate = [&](const Frame::Ptr & frame) {
    return rclcpp::Time(objects.header_.stamp) < rclcpp::Time(frame->header_.stamp);
  };

  const auto it = std::find_if(
    mapping_data->processed_frames_.begin(), mapping_data->processed_frames_.end(), predicate);

  if (it == mapping_data->processed_frames_.begin()) {
    return;
  }

  const Frame::Ptr left_frame = *(it - 1);
  const Frame::Ptr right_frame = *it;
  assert(rclcpp::Time(right_frame->header_.stamp) > rclcpp::Time(objects.header_.stamp));
  Eigen::Matrix4f detections_mcs = poseInterpolation(
    (rclcpp::Time(objects.header_.stamp) - rclcpp::Time(left_frame->header_.stamp)).seconds(), 0.f,
    (rclcpp::Time(right_frame->header_.stamp) - rclcpp::Time(left_frame->header_.stamp)).seconds(),
    left_frame->pose_, right_frame->pose_);

  // Compute the source -> detections transform
  Eigen::Affine3f source_to_detections_transform =
    source_lidar_to_mapping_lidar_transform *
    Eigen::Affine3f(calibration_frame.local_map_pose_.inverse()) * Eigen::Affine3f(detections_mcs) *
    mapping_lidar_to_detection_frame_transform;

  for (auto & object : objects.objects_) {
    filter(calibration_frame, object, source_to_detections_transform, min_p, max_p);
  }
}

void ObjectDetectionFilter::filter(
  const CalibrationFrame & calibration_frame, const ObjectBB & object,
  const Eigen::Affine3f & source_to_detections_transform, const Eigen::Vector4f & min_p,
  const Eigen::Vector4f & max_p)
{
  Eigen::Vector3f bbox_size =
    object.size_ + Eigen::Vector3f(
                     parameters_->detection_size_tolerance_, parameters_->detection_size_tolerance_,
                     parameters_->detection_size_tolerance_);
  std::array<Eigen::Vector3f, 8> vertices{
    {{-0.5f * bbox_size.x(), -0.5f * bbox_size.y(), -0.5f * bbox_size.z()},
     {-0.5f * bbox_size.x(), -0.5f * bbox_size.y(), 0.5f * bbox_size.z()},
     {-0.5f * bbox_size.x(), 0.5f * bbox_size.y(), -0.5f * bbox_size.z()},
     {-0.5f * bbox_size.x(), 0.5f * bbox_size.y(), 0.5f * bbox_size.z()},
     {0.5f * bbox_size.x(), -0.5f * bbox_size.y(), -0.5f * bbox_size.z()},
     {0.5f * bbox_size.x(), -0.5f * bbox_size.y(), 0.5f * bbox_size.z()},
     {0.5f * bbox_size.x(), 0.5f * bbox_size.y(), -0.5f * bbox_size.z()},
     {0.5f * bbox_size.x(), 0.5f * bbox_size.y(), 0.5f * bbox_size.z()}}};

  Eigen::Affine3f source_to_object_transform =
    source_to_detections_transform * Eigen::Affine3f(object.pose_);

  auto no_intersection_predicate = [&source_to_object_transform, &min_p,
                                    &max_p](const Eigen::Vector3f & vertex) {
    Eigen::Vector3f v = source_to_object_transform * vertex;
    return (v.x() < min_p.x() || v.x() > max_p.x()) && (v.y() < min_p.y() || v.y() > max_p.y()) &&
           (v.z() < min_p.z() || v.z() > max_p.z());
  };

  if (std::all_of(vertices.cbegin(), vertices.cend(), no_intersection_predicate)) {
    return;
  }

  pcl::CropBox<PointType> boxFilter;
  boxFilter.setTransform(source_to_object_transform.inverse());
  boxFilter.setNegative(true);
  boxFilter.setMin(
    Eigen::Vector4f(-0.5f * bbox_size.x(), -0.5f * bbox_size.y(), -0.5f * bbox_size.z(), 1.f));
  boxFilter.setMax(
    Eigen::Vector4f(0.5f * bbox_size.x(), 0.5f * bbox_size.y(), 0.5f * bbox_size.z(), 1.f));
  boxFilter.setInputCloud(calibration_frame.source_pointcloud_);
  boxFilter.filter(*calibration_frame.source_pointcloud_);
}
