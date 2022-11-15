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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__SERIALIZATION_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__SERIALIZATION_HPP_

#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <opencv2/core.hpp>

#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>

#include <memory>
#include <vector>

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)

namespace boost
{
namespace serialization
{

template <class Archive>
void save(Archive & ar, const cv::Mat & m, const unsigned int version)
{
  (void)version;
  size_t elem_size = m.elemSize();
  size_t elem_type = m.type();

  ar & m.cols;
  ar & m.rows;
  ar & elem_size;
  ar & elem_type;

  const size_t data_size = m.cols * m.rows * elem_size;
  ar & make_array(m.ptr(), data_size);
}

template <class Archive>
void load(Archive & ar, cv::Mat & m, const unsigned int version)
{
  (void)version;
  int cols, rows;
  size_t elem_size, elem_type;

  ar & cols;
  ar & rows;
  ar & elem_size;
  ar & elem_type;

  m.create(rows, cols, elem_type);

  size_t data_size = m.cols * m.rows * elem_size;
  ar & make_array(m.ptr(), data_size);
}

template <class Archive, typename _Tp>
inline void serialize(Archive & ar, cv::Point_<_Tp> & p, const unsigned int version)
{
  (void)version;

  ar & p.x;
  ar & p.y;
}

template <class Archive, typename _Tp, int _m, int _n>
inline void serialize(Archive & ar, cv::Matx<_Tp, _m, _n> & m, const unsigned int version)
{
  (void)version;
  ar & m.val;
}

template <class Archive, typename _Tp>
void serialize(Archive & ar, cv::Affine3<_Tp> & pose, const unsigned int version)
{
  (void)version;
  ar & pose.matrix;
}

template <class Archive>
void serialize(
  Archive & ar, extrinsic_tag_based_base_calibrator::ApriltagDetection & detection,
  const unsigned int version)
{
  (void)version;
  ar & detection.id;
  ar & detection.corners;
  ar & detection.center;
  ar & detection.pose;
  ar & detection.size;
}

template <class Archive>
void serialize(
  Archive & ar, extrinsic_tag_based_base_calibrator::ExternalCameraFrame & frame,
  const unsigned int version)
{
  (void)version;
  ar & frame.image_filename;
  ar & frame.detections;
}

template <class Archive>
void serialize(
  Archive & ar, extrinsic_tag_based_base_calibrator::CalibrationScene & scene,
  const unsigned int version)
{
  (void)version;
  ar & scene.calibration_sensor_detections;
  ar & scene.external_camera_frames;
}

template <class Archive>
void serialize(
  Archive & ar, extrinsic_tag_based_base_calibrator::UID & uid, const unsigned int version)
{
  (void)version;
  ar & uid.is_camera;
  ar & uid.is_tag;
  ar & uid.is_waypoint_tag;
  ar & uid.is_ground_tag;
  ar & uid.is_wheel_tag;

  ar & uid.scene_id;
  ar & uid.frame_id;
  ar & uid.tag_id;
}

template <class Archive>
void serialize(
  Archive & ar, extrinsic_tag_based_base_calibrator::CalibrationData & data,
  const unsigned int version)
{
  (void)version;
  ar & data.scenes;
  ar & data.detected_tag_ids_set;
  ar & data.initial_external_camera_poses;
  ar & data.initial_external_camera_intrinsics;
  ar & data.initial_tag_poses_map;
  ar & data.initial_waypoint_tag_poses;
  ar & data.initial_ground_tag_poses;
  ar & data.initial_left_wheel_tag_pose;
  ar & data.initial_right_wheel_tag_pose;

  ar & data.pose_opt_map;
  ar & data.intrinsics_opt_map;
  ar & data.shrd_ground_tag_pose_opt;
  ar & data.indep_ground_tag_pose_opt_map;

  ar & data.optimized_external_camera_poses;
  ar & data.optimized_external_camera_intrinsics;
  ar & data.optimized_tag_poses_map;
  ar & data.optimized_waypoint_tag_poses;
  ar & data.optimized_ground_tag_poses;
  ar & data.optimized_left_wheel_tag_pose;
  ar & data.optimized_right_wheel_tag_pose;
}

}  // namespace serialization
}  // namespace boost

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__SERIALIZATION_HPP_
