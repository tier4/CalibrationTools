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

#ifndef EXTRINSIC_LIDAR_TO_LIDAR_2D_CALIBRATOR__EXTRINSIC_LIDAR_TO_LIDAR_2D_CALIBRATOR_HPP_
#define EXTRINSIC_LIDAR_TO_LIDAR_2D_CALIBRATOR__EXTRINSIC_LIDAR_TO_LIDAR_2D_CALIBRATOR_HPP_

#include <Eigen/Dense>
#include <kalman_filter/kalman_filter.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tier4_calibration_msgs/srv/new_extrinsic_calibrator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

namespace extrinsic_lidar_to_lidar_2d_calibrator
{

using PointType = pcl::PointXYZ;

/**
 * A 2D lidar-lidar calibrator.
 * This tools assumes the existance of two lidars that have been independently calibrated to the
 * base frame (at least to the plane of the base plane), and uses that information to reduce the 3D
 * lidar-lidar calibration problem into a 2D one (x, y, and rotation). Since ladars have different
 * resolutions and FOV, this metod selects a range of z values (measured from the base frame),
 * flattens them (ignores the z value) and perform classic ICP to find the 2D transform. Optionally,
 * this method also implemented basic filtering to improve the results.
 *
 * Since ICP works finding the closest target point for every source point, we formulate this
 * problem as finding the tf from target to source (target should usually be the lidar with the
 * highest resolution or FOV)
 */

class LidarToLidar2DCalibrator : public rclcpp::Node
{
public:
  explicit LidarToLidar2DCalibrator(const rclcpp::NodeOptions & options);

protected:
  /*!
   * External interface to start the calibration process and retrieve the result.
   * The call gets blocked until the calibration finishes
   *
   * @param request An empty service request
   * @param response A vector of calibration results
   */
  void requestReceivedCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::NewExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::NewExtrinsicCalibrator::Response> response);

  /*!
   * Source pointcloud callback
   *
   * @param msg The source pointcloud in msg format
   */
  void sourcePointCloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr pc);

  /*!
   * Target pointcloud callback
   *
   * @param msg The target pointcloud in msg format
   */
  void targetPointCloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr pc);

  /*!
   * The main calibration function.
   * Performs calibration, publishes debug pointclouds and optionally broadcasts the results and
   * performs filtering
   */
  void calibrationTimerCallback();

  /*!
   * Check if the required TFs are available and computes them
   */
  bool checkInitialTransforms();

  /*!
   * Computes the alignment error between two pointclouds.
   * For every point in the source pointcloud their closes point in the target pointcloud is
   * computed and the distance between the points is used to determine the alignment error
   *
   * @param source_pointcloud The source pointcloud
   * @param target_pointcloud The target pointcloud
   * @return the alignment error
   */
  double getAlignmentError(
    pcl::PointCloud<PointType>::Ptr source_pointcloud,
    pcl::PointCloud<PointType>::Ptr target_pointcloud);

  /*!
   * The core optimization method
   * Using multiple registration algorithms, looks for the transform that best solves ths
   * source->target problem
   *
   * @param source_pointcloud_ptr The source pointcloud
   * @param target_pointcloud_ptr The target pointcloud
   * @param target_pointcloud_ptr A vector of pointcloud registrators to solve the calibration
   * problem
   * @return A tuple containin the best aligned pointcloud, trasform,  and score
   */
  std::tuple<pcl::PointCloud<PointType>::Ptr, Eigen::Matrix4f, float> findBestTransform(
    pcl::PointCloud<PointType>::Ptr & source_pointcloud_ptr,
    pcl::PointCloud<PointType>::Ptr & target_pointcloud_ptr,
    std::vector<pcl::Registration<PointType, PointType>::Ptr> & registratators);

  // Parameters
  std::string base_frame_;
  bool broadcast_calibration_tf_;
  bool filter_estimations_;
  double max_calibration_range_;
  double max_corr_distance_;
  int max_iterations_;

  double initial_angle_cov_;
  double initial_xy_cov_;
  double angle_measurement_cov_;
  double angle_process_cov_;
  double xy_measurement_cov_;
  double xy_process_cov_;
  double angle_convergence_threshold_;
  double xy_convergence_threshold_;

  double min_z_;
  double max_z_;

  // ROS Interface
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  rclcpp::TimerBase::SharedPtr calibration_timer_;

  rclcpp::CallbackGroup::SharedPtr srv_callback_group_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_source_initial_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_source_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr flat_source_initial_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr flat_source_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr flat_target_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr source_pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr target_pointcloud_sub_;

  rclcpp::Service<tier4_calibration_msgs::srv::NewExtrinsicCalibrator>::SharedPtr service_server_;

  // Threading
  std::mutex mutex_;

  // ROS Data
  sensor_msgs::msg::PointCloud2::UniquePtr source_pointcloud_msg_;
  sensor_msgs::msg::PointCloud2::UniquePtr target_pointcloud_msg_;
  std_msgs::msg::Header source_pointcloud_header_;
  std_msgs::msg::Header target_pointcloud_header_;
  std::string source_pointcloud_frame_;
  std::string target_pointcloud_frame_;

  // Initial tfs comparable with the one with our method
  Eigen::Affine3d initial_base_to_source_eigen_;
  Eigen::Affine3d initial_base_to_target_eigen_;

  std::vector<pcl::Registration<PointType, PointType>::Ptr> calibration_registrators_;
  pcl::GeneralizedIterativeClosestPoint<PointType, PointType>::Ptr calibration_gicp_;
  pcl::IterativeClosestPoint<PointType, PointType>::Ptr calibration_icp_, calibration_icp_fine_,
    calibration_icp_fine_2_, calibration_icp_fine_3_;

  // Calibration output
  geometry_msgs::msg::TransformStamped output_calibration_msg_;

  bool got_initial_transform_;
  bool received_request_;
  bool broadcast_tf_;
  bool calibration_done_;

  // Filtering
  KalmanFilter kalman_filter_;
  bool first_observation_;
};

}  // namespace extrinsic_lidar_to_lidar_2d_calibrator

#endif  // EXTRINSIC_LIDAR_TO_LIDAR_2D_CALIBRATOR__EXTRINSIC_LIDAR_TO_LIDAR_2D_CALIBRATOR_HPP_
