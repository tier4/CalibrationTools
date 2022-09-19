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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR_LIDAR_CALIBRATOR_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR_LIDAR_CALIBRATOR_HPP_

#include <extrinsic_mapping_based_calibrator/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_pcl_extensions/joint_icp_extended.hpp>

#include <tier4_calibration_msgs/srv/frame.hpp>

#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pclomp/ndt_omp.h>
#include <tf2_ros/buffer.h>

class LidarCalibrator
{
public:
  using Ptr = std::shared_ptr<LidarCalibrator>;
  using PointPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
  using PointSubscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>;
  using FrameService = rclcpp::Service<tier4_calibration_msgs::srv::Frame>;

  LidarCalibrator(
    const std::string & calibration_lidar_frame, LidarCalibrationParameters::Ptr & parameters,
    MappingData::Ptr & mapping_data, std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    PointPublisher::SharedPtr & initial_source_aligned_map_pub,
    PointPublisher::SharedPtr & calibrated_source_aligned_map_pub,
    PointPublisher::SharedPtr & target_map_pub);

  void singleLidarCalibrationCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response);
  void multipleLidarCalibrationCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response);

  /*!
   * Configure the calibrator parameters
   */
  void configureCalibrators();

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
   * Compute a map with a user-defined resolution around a certain frame centered in `pose`
   * @param[in] pose The pose in map coordinates the pointcloud should be centered in
   * @param[in] frame Frame to use as a center for constructing the pointcloud
   * @param[in] resolution Max resolution of the resulting pointcloud
   * @param[in] max_range Max range of the resulting pointcloud
   * @retval Source to distance pointcloud distance
   */
  PointcloudType::Ptr getDensePointcloudFromMap(
    const Eigen::Matrix4f & pose, Frame::Ptr & frame, double resolution, double max_range);

  /*!
   * Filter calibration frames that are close to frames where the robot is assumes to be lost
   * @param[in] calibration_frames The raw calibrated frames
   */
  std::vector<CalibrationFrame> filterCalibrationFramesByLostState(
    const std::vector<CalibrationFrame> & calibration_frames);

  /*!
   * Filter calibration frames to avoid high speed, acceleration, interpolation, etc
   * @param[in] calibration_frames The raw calibrated frames
   */
  std::vector<CalibrationFrame> filterCalibrationFramesByDynamics(
    const std::vector<CalibrationFrame> & calibration_frames);

  /*!
   * Select best K calibration frames based on information and spatial correlation
   * @param[in] calibration_frames The raw calibrated frames
   */
  std::vector<CalibrationFrame> selectBestKCalibrationFrames(
    const std::vector<CalibrationFrame> & calibration_frames, int num_frames);

  std::string calibration_lidar_frame_;
  std::string calibrator_name_;
  LidarCalibrationParameters::Ptr parameters_;
  MappingData::Ptr data_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  PointPublisher::SharedPtr initial_source_aligned_map_pub_;
  PointPublisher::SharedPtr calibrated_source_aligned_map_pub_;
  PointPublisher::SharedPtr target_map_pub_;

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

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR_LIDAR_CALIBRATOR_HPP_
