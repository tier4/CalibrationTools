// Copyright 2024 TIER IV, Inc.
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

#ifndef TAG_BASED_SFM_CALIBRATOR__SERIALIZATION_HPP_
#define TAG_BASED_SFM_CALIBRATOR__SERIALIZATION_HPP_

#include <opencv2/core.hpp>
#include <tag_based_sfm_calibrator/calibration_types.hpp>
#include <tag_based_sfm_calibrator/scene_types.hpp>
#include <tag_based_sfm_calibrator/types.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>

#include <memory>
#include <vector>

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)
BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat_<double>)

namespace boost
{
namespace serialization
{

template <class Archive>
void save(Archive & ar, const cv::Mat & m, [[maybe_unused]] const unsigned int version)
{
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
void load(Archive & ar, cv::Mat & m, [[maybe_unused]] const unsigned int version)
{
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
void save(Archive & ar, const cv::Mat_<_Tp> & m, [[maybe_unused]] const unsigned int version)
{
  size_t elem_size = m.elemSize();

  ar & m.cols;
  ar & m.rows;
  ar & elem_size;

  const size_t data_size = m.cols * m.rows * elem_size;
  ar & make_array(m.ptr(), data_size);
}

template <class Archive, typename _Tp>
void load(Archive & ar, cv::Mat_<_Tp> & m, [[maybe_unused]] const unsigned int version)
{
  int cols, rows;
  size_t elem_size;

  ar & cols;
  ar & rows;
  ar & elem_size;

  m.create(rows, cols);

  size_t data_size = m.cols * m.rows * elem_size;
  ar & make_array(m.ptr(), data_size);
}

template <class Archive, typename _Tp>
inline void serialize(
  Archive & ar, cv::Size_<_Tp> & size, [[maybe_unused]] const unsigned int version)
{
  ar & size.height;
  ar & size.width;
}

template <class Archive, typename _Tp>
inline void serialize(
  Archive & ar, cv::Point_<_Tp> & p, [[maybe_unused]] const unsigned int version)
{
  ar & p.x;
  ar & p.y;
}

template <class Archive, typename _Tp>
inline void serialize(
  Archive & ar, cv::Point3_<_Tp> & p, [[maybe_unused]] const unsigned int version)
{
  ar & p.x;
  ar & p.y;
  ar & p.z;
}

template <class Archive, typename _Tp, int _m, int _n>
inline void serialize(
  Archive & ar, cv::Matx<_Tp, _m, _n> & m, [[maybe_unused]] const unsigned int version)
{
  ar & m.val;
}

template <class Archive, typename _Tp>
void serialize(Archive & ar, cv::Affine3<_Tp> & pose, [[maybe_unused]] const unsigned int version)
{
  ar & pose.matrix;
}

template <class Archive>
void serialize(
  Archive & ar, tag_based_sfm_calibrator::ApriltagDetection & detection,
  [[maybe_unused]] const unsigned int version)
{
  ar & detection.family;
  ar & detection.id;
  ar & detection.image_corners;
  ar & detection.object_corners;
  ar & detection.template_corners;
  ar & detection.center;
  ar & detection.pose;
  ar & detection.size;
}

template <class Archive>
void serialize(
  Archive & ar, tag_based_sfm_calibrator::ApriltagGridDetection & detection,
  [[maybe_unused]] const unsigned int version)
{
  ar & detection.cols;
  ar & detection.rows;
  ar & detection.sub_detections;
  ar & detection.family;
  ar & detection.id;
  ar & detection.image_corners;
  ar & detection.object_corners;
  ar & detection.template_corners;
  ar & detection.center;
  ar & detection.pose;
  ar & detection.size;
}

template <class Archive>
void serialize(
  Archive & ar, tag_based_sfm_calibrator::LidartagDetection & detection,
  [[maybe_unused]] const unsigned int version)
{
  ar & detection.id;
  ar & detection.object_corners;
  ar & detection.template_corners;
  ar & detection.pose;
  ar & detection.size;
}

template <class Archive>
void serialize(
  Archive & ar, tag_based_sfm_calibrator::ExternalCameraFrame & frame,
  [[maybe_unused]] const unsigned int version)
{
  ar & frame.image_filename;
  ar & frame.detections;
}

template <class Archive>
void serialize(
  Archive & ar, tag_based_sfm_calibrator::SingleCalibrationLidarDetections & lidar_detections,
  [[maybe_unused]] const unsigned int version)
{
  ar & lidar_detections.calibration_frame;
  ar & lidar_detections.calibration_lidar_id;
  ar & lidar_detections.detections;
}

template <class Archive>
void serialize(
  Archive & ar, tag_based_sfm_calibrator::SingleCalibrationCameraDetections & camera_detections,
  [[maybe_unused]] const unsigned int version)
{
  ar & camera_detections.calibration_frame;
  ar & camera_detections.calibration_camera_id;
  ar & camera_detections.grouped_detections;
  ar & camera_detections.calibration_image;
}

template <class Archive>
void serialize(
  Archive & ar, tag_based_sfm_calibrator::CalibrationScene & scene,
  [[maybe_unused]] const unsigned int version)
{
  ar & scene.calibration_cameras_detections;
  ar & scene.calibration_lidars_detections;
  ar & scene.external_camera_frames;
}

template <class Archive>
void serialize(
  Archive & ar, tag_based_sfm_calibrator::UID & uid, [[maybe_unused]] const unsigned int version)
{
  ar & uid.type;
  ar & uid.sensor_type;
  ar & uid.tag_type;

  ar & uid.scene_id;
  ar & uid.calibration_sensor_id;
  ar & uid.frame_id;
  ar & uid.tag_id;
}

template <class Archive>
void serialize(
  Archive & ar, tag_based_sfm_calibrator::IntrinsicParameters & intrinsics,
  [[maybe_unused]] const unsigned int version)
{
  ar & intrinsics.size;
  ar & intrinsics.camera_matrix;
  ar & intrinsics.dist_coeffs;
  ar & intrinsics.undistorted_camera_matrix;
}

template <class Archive>
void serialize(
  Archive & ar, tag_based_sfm_calibrator::CalibrationData & data,
  [[maybe_unused]] const unsigned int version)
{
  ar & data.scenes;
  ar & data.main_calibration_sensor_uid;
  ar & data.uid_connections_map;
  ar & data.detections_relative_poses_map;
  ar & data.detection_diagonal_ratio_map;
  ar & data.calibration_camera_intrinsics_map_;

  ar & data.initial_sensor_poses_map;
  ar & data.initial_camera_intrinsics_map;
  ar & data.initial_tag_poses_map;
  ar & data.initial_ground_tag_poses_map;
  ar & data.initial_left_wheel_tag_pose;
  ar & data.initial_right_wheel_tag_pose;

  ar & data.optimized_sensor_poses_map;
  ar & data.optimized_camera_intrinsics_map;
  ar & data.optimized_tag_poses_map;
  ar & data.optimized_ground_tag_poses_map;
  ar & data.optimized_left_wheel_tag_pose;
  ar & data.optimized_right_wheel_tag_pose;
}

template <class Archive>
void serialize(
  Archive & ar, sensor_msgs::msg::CompressedImage & msg,
  [[maybe_unused]] const unsigned int version)
{
  ar & msg.format;
  ar & msg.data;
}

}  // namespace serialization
}  // namespace boost

#endif  // TAG_BASED_SFM_CALIBRATOR__SERIALIZATION_HPP_
