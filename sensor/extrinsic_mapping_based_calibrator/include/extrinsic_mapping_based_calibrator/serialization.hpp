// Copyright 2024 Tier IV, Inc.
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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR__SERIALIZATION_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR__SERIALIZATION_HPP_

#include <Eigen/Core>
#include <extrinsic_mapping_based_calibrator/types.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <std_msgs/msg/header.hpp>

#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <vector>

namespace boost
{
namespace serialization
{
template <class Archive>
void serialize(Archive & ar, pcl::PCLHeader & g, const unsigned int version)
{
  (void)version;
  ar & g.seq;
  ar & g.stamp;
  ar & g.frame_id;
}

template <class Archive>
void serialize(Archive & ar, pcl::PointXYZ & g, const unsigned int version)
{
  (void)version;
  ar & g.x;
  ar & g.y;
  ar & g.z;
}

template <class Archive>
void serialize(Archive & ar, pcl::PointCloud<pcl::PointXYZ> & g, const unsigned int version)
{
  (void)version;
  ar & g.header;
  ar & g.points;
  ar & g.height;
  ar & g.width;
  ar & g.is_dense;
}

template <
  class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void serialize(
  Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t,
  const unsigned int file_version)
{
  (void)file_version;
  int rows = t.rows();
  int cols = t.cols();

  ar & rows;
  ar & cols;

  if (rows * cols != t.size()) {
    t.resize(rows, cols);
  }

  for (int i = 0; i < t.size(); i++) {
    ar & t.data()[i];
  }
}

template <class Archive>
void serialize(Archive & ar, ObjectBB & object, const unsigned int version)
{
  (void)version;
  ar & object.pose_;
  ar & object.size_;
}

template <class Archive>
void serialize(Archive & ar, ObjectsBB & objects, const unsigned int version)
{
  (void)version;
  ar & objects.header_;
  ar & objects.objects_;
}

template <class Archive>
void serialize(Archive & ar, Frame & frame, const unsigned int version)
{
  (void)version;
  ar & frame.distance_;
  ar & frame.delta_translation_;
  ar & frame.rough_speed_;
  ar & frame.aux_delta_translation_;
  ar & frame.header_;
  ar & frame.pointcloud_raw_;
  ar & frame.pointcloud_subsampled_;
  ar & frame.frame_id_;
  ar & frame.keyframe_id_;
  ar & frame.processed_;
  ar & frame.is_key_frame_;
  ar & frame.stopped_;
  ar & frame.lost_;
  ar & frame.frames_since_stop_;
  ar & frame.pose_;
  ar & frame.dt_;
}

template <class Archive>
void serialize(Archive & ar, CalibrationFrame & frame, const unsigned int version)
{
  (void)version;
  ar & frame.source_camera_info_;
  ar & frame.source_image_;
  ar & frame.source_pointcloud_;
  ar & frame.source_header_;

  ar & frame.target_frame_;
  ar & frame.local_map_pose_;

  ar & frame.interpolated_distance_;
  ar & frame.interpolated_angle_;
  ar & frame.interpolated_time_;
  ar & frame.estimated_speed_;
  ar & frame.estimated_accel_;
  ar & frame.stopped_;
}

template <class Archive>
void serialize(Archive & ar, std_msgs::msg::Header & header, const unsigned int version)
{
  (void)version;
  ar & header.frame_id;
  ar & header.stamp;
}

template <class Archive>
void serialize(Archive & ar, builtin_interfaces::msg::Time & stamp, const unsigned int version)
{
  (void)version;
  ar & stamp.nanosec;
  ar & stamp.sec;
}

template <class Archive>
void serialize(Archive & ar, sensor_msgs::msg::RegionOfInterest & roi, const unsigned int version)
{
  (void)version;
  ar & roi.do_rectify;
  ar & roi.height;
  ar & roi.width;
  ar & roi.x_offset;
  ar & roi.y_offset;
}

template <class Archive>
void serialize(Archive & ar, sensor_msgs::msg::CameraInfo & camera_info, const unsigned int version)
{
  (void)version;
  ar & camera_info.binning_x;
  ar & camera_info.binning_y;
  ar & camera_info.d;
  ar & camera_info.distortion_model;
  ar & camera_info.header;
  ar & camera_info.height;
  ar & camera_info.k;
  ar & camera_info.p;
  ar & camera_info.roi;
  ar & camera_info.width;
}

template <class Archive>
void serialize(Archive & ar, sensor_msgs::msg::CompressedImage & image, const unsigned int version)
{
  (void)version;
  ar & image.data;
  ar & image.format;
  ar & image.header;
}

}  // namespace serialization
}  // namespace boost

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR__SERIALIZATION_HPP_
