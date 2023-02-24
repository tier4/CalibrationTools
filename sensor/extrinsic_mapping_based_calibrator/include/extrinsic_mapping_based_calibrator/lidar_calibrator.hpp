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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR__LIDAR_CALIBRATOR_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR__LIDAR_CALIBRATOR_HPP_

#include <extrinsic_mapping_based_calibrator/filters/filter.hpp>
#include <extrinsic_mapping_based_calibrator/sensor_calibrator.hpp>
#include <extrinsic_mapping_based_calibrator/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_calibration_pcl_extensions/joint_icp_extended.hpp>

#include <tier4_calibration_msgs/srv/frame.hpp>

#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pclomp/ndt_omp.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <vector>

class LidarCalibrator : public SensorCalibrator
{
public:
  using Ptr = std::shared_ptr<LidarCalibrator>;
  using PointPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
  using PointSubscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>;
  using FrameService = rclcpp::Service<tier4_calibration_msgs::srv::Frame>;

  LidarCalibrator(
    const std::string & calibration_lidar_frame, CalibrationParameters::Ptr & parameters,
    MappingData::Ptr & mapping_data, std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    PointPublisher::SharedPtr & initial_source_aligned_map_pub,
    PointPublisher::SharedPtr & calibrated_source_aligned_map_pub,
    PointPublisher::SharedPtr & target_map_pub);

  void singleSensorCalibrationCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response) override;
  void multipleSensorCalibrationCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response) override;

  /*!
   * Calibrate the lidar
   */
  bool calibrate(Eigen::Matrix4f & best_transform, float & best_score) override;

  /*!
   * Configure the calibrator parameters
   */
  void configureCalibrators() override;

protected:
  /*!
   * Prepare calibrators for a specific pair of pointclouds
   * @param[in] source_pointcloud_ptr Source pointcloud
   * @param[in] target_pointcloud_ptr Target pointcloud
   */
  void setUpCalibrators(
    PointcloudType::Ptr & source_pointcloud_ptr, PointcloudType::Ptr & target_pointcloud_ptr);

  // General methods

  /*!
   * Prepare the augmented calibration data
   * @param[in] calibration_frames The calibrated frames
   * @param[in] initial_distance The initial distance between frames
   * @param[out] sources The calibrated frames
   * @param[out] targets The calibrated frames
   * @param[out] targets_thin The calibrated frames
   */
  void prepareCalibrationData(
    const std::vector<CalibrationFrame> & calibration_frames, const float initial_distance,
    const Eigen::Matrix4f & initial_calibration_transform,
    std::vector<pcl::PointCloud<PointType>::Ptr> & sources,
    std::vector<pcl::PointCloud<PointType>::Ptr> & targets,
    std::vector<pcl::PointCloud<PointType>::Ptr> & targets_thin);

  /*!
   * Publish the calibration results
   */
  void publishResults(
    const std::vector<CalibrationFrame> & calibration_frames,
    const std::vector<pcl::PointCloud<PointType>::Ptr> & sources,
    const std::vector<pcl::PointCloud<PointType>::Ptr> & targets,
    const Eigen::Matrix4f & initial_transform, const Eigen::Matrix4f & calibrated_transform,
    const std::string & map_frame);

  PointPublisher::SharedPtr initial_source_aligned_map_pub_;
  PointPublisher::SharedPtr calibrated_source_aligned_map_pub_;
  PointPublisher::SharedPtr target_map_pub_;

  // Filters
  Filter::Ptr filter_;

  // Calibration
  std::vector<pcl::Registration<PointType, PointType>::Ptr> calibration_registrators_;
  std::vector<pcl::JointIterativeClosestPointExtended<PointType, PointType>::Ptr>
    calibration_batch_registrators_;
  pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr calibration_ndt_;
  pcl::GeneralizedIterativeClosestPoint<PointType, PointType>::Ptr calibration_gicp_;
  pcl::IterativeClosestPoint<PointType, PointType>::Ptr calibration_icp_coarse_;
  pcl::IterativeClosestPoint<PointType, PointType>::Ptr calibration_icp_fine_;
  pcl::IterativeClosestPoint<PointType, PointType>::Ptr calibration_icp_ultrafine_;
  pcl::registration::CorrespondenceEstimation<PointType, PointType>::Ptr correspondence_estimator_;

  pcl::JointIterativeClosestPointExtended<PointType, PointType>::Ptr calibration_batch_icp_coarse_;
  pcl::JointIterativeClosestPointExtended<PointType, PointType>::Ptr calibration_batch_icp_fine_;
  pcl::JointIterativeClosestPointExtended<PointType, PointType>::Ptr
    calibration_batch_icp_ultrafine_;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR__LIDAR_CALIBRATOR_HPP_
