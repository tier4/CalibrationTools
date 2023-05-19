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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR__CAMERA_CALIBRATOR_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR__CAMERA_CALIBRATOR_HPP_

#include <extrinsic_mapping_based_calibrator/filters/filter.hpp>
#include <extrinsic_mapping_based_calibrator/sensor_calibrator.hpp>
#include <extrinsic_mapping_based_calibrator/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_calibration_msgs/srv/frame.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>

class CameraCalibrator : public SensorCalibrator
{
public:
  using Ptr = std::shared_ptr<CameraCalibrator>;
  using MarkersPublisher = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>;
  using PointPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
  using PointSubscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>;
  using FrameService = rclcpp::Service<tier4_calibration_msgs::srv::Frame>;

  CameraCalibrator(
    const std::string & calibration_camera_optical_link_frame,
    CalibrationParameters::Ptr & parameters, MappingData::Ptr & mapping_data,
    std::shared_ptr<tf2_ros::Buffer> & tf_buffer, PointPublisher::SharedPtr & target_map_pub,
    MarkersPublisher::SharedPtr & target_markers_pub);

  virtual void singleSensorCalibrationCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response) override;
  virtual void multipleSensorCalibrationCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response) override;

  /*!
   * Calibrate the lidar
   */
  virtual bool calibrate(Eigen::Matrix4f & best_transform, float & best_score) override;

  /*!
   * Configure the calibrator parameters
   */
  virtual void configureCalibrators() override;

protected:
  /*!
   * Prepare the augmented calibration data
   * @param[in] calibration_frames The calibrated frames
   * @param[in] initial_distance The initial distance between frames
   * @param[out] targets The calibrated frames
   * @param[out] targets_thin The calibrated frames
   */
  void prepareCalibrationData(
    const std::vector<CalibrationFrame> & calibration_frames,
    const Eigen::Matrix4f & initial_calibration_transform, const float initial_distance,
    std::vector<pcl::PointCloud<PointType>::Ptr> & targets);

  /*!
   * Publish the pointclouds used for calibration, their plane, and line features
   * @param[in] calibration_frames The calibrated frames
   * @param[in] targets The pointclouds used for calibration
   * @param[in] map_frame The map frame
   */
  void publishResults(
    const std::vector<CalibrationFrame> & calibration_frames,
    const std::vector<pcl::PointCloud<PointType>::Ptr> & targets, const std::string & map_frame,
    const Eigen::Matrix4f & initial_calibration_transform);

  PointPublisher::SharedPtr target_map_pub_;
  MarkersPublisher::SharedPtr target_markers_pub_;

  // Filters
  Filter::Ptr filter_;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR__CAMERA_CALIBRATOR_HPP_
