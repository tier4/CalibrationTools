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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR__BASE_LIDAR_CALIBRATOR_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR__BASE_LIDAR_CALIBRATOR_HPP_

#include <extrinsic_mapping_based_calibrator/filters/filter.hpp>
#include <extrinsic_mapping_based_calibrator/sensor_calibrator.hpp>
#include <extrinsic_mapping_based_calibrator/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_calibration_msgs/srv/frame.hpp>

#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>
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

  void calibrationCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response);

  /*!
   * Calibrate the lidar
   */
  bool calibrate(Eigen::Matrix4f & best_transform, float & best_score) override;

  virtual void singleSensorCalibrationCallback(
    __attribute__((unused)) const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request>
      request,
    __attribute__((unused)) const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response>
      response)
  {
  }

  virtual void multipleSensorCalibrationCallback(
    __attribute__((unused)) const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request>
      request,
    __attribute__((unused)) const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response>
      response)
  {
  }

  virtual void configureCalibrators() {}

protected:
  /*!
   * Extract the ground plane of a pointcloud
   * @param[in] pointcloud Source pointcloud
   * @param[in] initial_normal Target pointcloud
   * @param[in] model Target pointcloud
   * @param[in] inliers_pointcloud Target pointcloud
   */
  bool extractGroundPlane(
    pcl::PointCloud<PointType>::Ptr & pointcloud, const Eigen::Vector3f & initial_normal,
    Eigen::Vector4d & model, pcl::PointCloud<PointType>::Ptr & inliers_pointcloud);

  /*!
   * Publish the calibration results
   */
  void publishResults(
    const Eigen::Vector4d & ground_model,
    const pcl::PointCloud<PointType>::Ptr & ground_plane_inliers,
    const pcl::PointCloud<PointType>::Ptr & augmented_pointcloud_ptr);

  Eigen::Isometry3d modelPlaneToPose(const Eigen::Vector4d & model) const;

  tf2_ros::StaticTransformBroadcaster & tf_broadcaster_;
  PointPublisher::SharedPtr augmented_pointcloud_pub_;
  PointPublisher::SharedPtr ground_pointcloud_pub_;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR__BASE_LIDAR_CALIBRATOR_HPP_
