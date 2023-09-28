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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR__SENSOR_CALIBRATOR_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR__SENSOR_CALIBRATOR_HPP_

#include <Eigen/Core>
#include <extrinsic_mapping_based_calibrator/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_calibration_msgs/srv/frame.hpp>

#include <tf2_ros/buffer.h>

#include <memory>
#include <string>

class SensorCalibrator
{
public:
  using Ptr = std::shared_ptr<SensorCalibrator>;
  using PointPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
  using PointSubscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>;
  using FrameService = rclcpp::Service<tier4_calibration_msgs::srv::Frame>;

  SensorCalibrator(
    const std::string calibration_lidar_frame, const std::string calibration_name,
    CalibrationParameters::Ptr & parameters, MappingData::Ptr & mapping_data,
    std::shared_ptr<tf2_ros::Buffer> & tf_buffer);

  virtual void singleSensorCalibrationCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response) = 0;
  virtual void multipleSensorCalibrationCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response) = 0;

  /*!
   * Calibrate the sensor
   */
  virtual bool calibrate(Eigen::Matrix4f & best_transform, float & best_score) = 0;

  /*!
   * Configure the calibrator parameters
   */
  virtual void configureCalibrators() = 0;

protected:
  /*!
   * Compute a map with a user-defined resolution around a certain frame centered in `pose`
   * @param[in] pose The pose in map coordinates the pointcloud should be centered in
   * @param[in] frame Frame to use as a center for constructing the pointcloud
   * @param[in] resolution Max resolution of the resulting pointcloud
   * @param[in] max_range Max range of the resulting pointcloud
   * @retval Source to distance pointcloud distance
   */
  PointcloudType::Ptr getDensePointcloudFromMap(
    const Eigen::Matrix4f & pose, const Frame::Ptr & frame, double resolution, double min_range,
    double max_range);

  std::string calibrator_sensor_frame_;
  std::string calibrator_name_;

  CalibrationParameters::Ptr parameters_;
  MappingData::Ptr data_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR__SENSOR_CALIBRATOR_HPP_
