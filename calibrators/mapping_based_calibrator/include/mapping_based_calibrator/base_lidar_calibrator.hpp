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

#ifndef MAPPING_BASED_CALIBRATOR__BASE_LIDAR_CALIBRATOR_HPP_
#define MAPPING_BASED_CALIBRATOR__BASE_LIDAR_CALIBRATOR_HPP_

#include <mapping_based_calibrator/filters/filter.hpp>
#include <mapping_based_calibrator/sensor_calibrator.hpp>
#include <mapping_based_calibrator/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_calibration_msgs/srv/frame.hpp>

#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>
#include <tuple>
#include <vector>

class BaseLidarCalibrator : public SensorCalibrator
{
public:
  using Ptr = std::shared_ptr<BaseLidarCalibrator>;
  using PointPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
  using PointSubscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>;
  using FrameService = rclcpp::Service<tier4_calibration_msgs::srv::Frame>;

  BaseLidarCalibrator(
    CalibrationParameters::Ptr & parameters, MappingData::Ptr & mapping_data,
    std::shared_ptr<tf2_ros::Buffer> & tf_buffer, tf2_ros::StaticTransformBroadcaster & broadcaster,
    PointPublisher::SharedPtr & augmented_pointcloud_pub,
    PointPublisher::SharedPtr & ground_pointcloud_pub);

  /*!
   * Calibrate the sensor
   * @returns a tuple containing the calibration success status, the transform, and a score
   */
  std::tuple<bool, Eigen::Matrix4d, float> calibrate() override;

  void configureCalibrators() override {}

protected:
  /*!
   * Publish the calibration results
   */
  void publishResults(
    const Eigen::Vector4d & ground_model, const Eigen::Matrix4f & pose,
    const pcl::PointCloud<PointType>::Ptr & ground_plane_inliers,
    const pcl::PointCloud<PointType>::Ptr & augmented_pointcloud_ptr);

  tf2_ros::StaticTransformBroadcaster & tf_broadcaster_;
  PointPublisher::SharedPtr augmented_pointcloud_pub_;
  PointPublisher::SharedPtr ground_pointcloud_pub_;
};

#endif  // MAPPING_BASED_CALIBRATOR__BASE_LIDAR_CALIBRATOR_HPP_
