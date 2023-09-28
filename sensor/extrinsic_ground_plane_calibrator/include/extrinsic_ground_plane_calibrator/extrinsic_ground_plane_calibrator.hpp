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

#ifndef EXTRINSIC_GROUND_PLANE_CALIBRATOR__EXTRINSIC_GROUND_PLANE_CALIBRATOR_HPP_
#define EXTRINSIC_GROUND_PLANE_CALIBRATOR__EXTRINSIC_GROUND_PLANE_CALIBRATOR_HPP_

#define PCL_NO_PRECOMPILE
#include <Eigen/Dense>
#include <extrinsic_ground_plane_calibrator/utils.hpp>
#include <kalman_filter/kalman_filter.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tier4_calibration_msgs/srv/extrinsic_calibrator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using PointType = pcl::PointXYZ;

class ExtrinsicGroundPlaneCalibrator : public rclcpp::Node
{
public:
  explicit ExtrinsicGroundPlaneCalibrator(const rclcpp::NodeOptions & options);

protected:
  void requestReceivedCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);

  /*!
   * ROS pointcloud callback
   * @param[in] pc the input pointcloud
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);

  /*!
   * Checks that all the needed tfs are available
   * @retval wether or not all the needed tfs are available
   */
  bool checkInitialTransforms();

  /*!
   * Extracts the ground plane from a pointcloud
   * @param[in] pointcloud the input pointcloud
   * @param[in] model the estimated ground plane model
   * @param[in] inliers the inliers of the current estimated model
   * @retval wether or not th calibration plane was found
   */
  bool extractGroundPlane(
    pcl::PointCloud<PointType>::Ptr & pointcloud, Eigen::Vector4d & model,
    pcl::PointCloud<PointType>::Ptr & inliers);

  /*!
   * Computes the fitting error of an estimated model and the initial one
   * @param[in] estimated_model the estimated model
   * @param[in] inliers the inliers of the current estimated model
   */
  void evaluateModels(
    const Eigen::Vector4d & estimated_model, pcl::PointCloud<PointType>::Ptr inliers) const;

  /*!
   * Filter individual calibration plane estimations and accumulate the inliers for a final
   * regression
   * @param[in] estimated_model the estimated model
   * @param[in] inliers the inliers of the current estimated model
   */
  void filterCalibration(
    const Eigen::Vector4d & estimated_model, pcl::PointCloud<PointType>::Ptr inliers);

  /*!
   * Creates a visualization of a calibration plane
   * @param[in] ground_plane_model the calibration plane
   */
  void visualizeCalibration(const Eigen::Vector4d & ground_plane_model);

  /*!
   * Creates a visualization of a calibration plane
   * @param[in] name the name of the calibration plane
   * @param[in] model the model of the calibration plane
   * @param[out] makers the output markers of the calibration plane
   */
  void visualizePlaneModel(
    const std::string & name, Eigen::Vector4d model, visualization_msgs::msg::MarkerArray & makers);

  /*!
   * Creates a visualization of a calibration plane
   * @param[in] name the name of the calibration plane
   * @param[in] lidar_base_pose the pose of the calibration plane
   * @param[out] makers the output markers of the calibration plane
   */
  void visualizePlaneModel(
    const std::string & name, const Eigen::Isometry3d & lidar_base_pose,
    visualization_msgs::msg::MarkerArray & makers);

  /*!
   * Publishes tfs related to the calibration process
   * @param[in] ground_plane_model the current calibration plane
   */
  void publishTf(const Eigen::Vector4d & ground_plane_model);

  /*!
   * Computes a plane model given a pose.
   * The normal of the plane is given by the z-axis of the rotation of the pose
   * @param[in] pointcloud Point cloud to crop
   * @param[in] max_range Range to crop the pointcloud to
   * @retval the plane model
   */
  Eigen::Vector4d poseToPlaneModel(const Eigen::Isometry3d & pose) const;

  /*!
   * Compute a pose from a plane model a*x + b*y +c*z +d = 0
   * The pose lies has its origin on the z-projection of the plane
   * @param[in] model Point cloud to crop
   * @retval the plane pose
   */
  Eigen::Isometry3d modelPlaneToPose(const Eigen::Vector4d & model) const;

  /*!
   * Refine a lidar-base pose given an estimated ground plane
   * Projects the initial base lidar pose into the ground plane.
   * @param[in] base_lidar_pose Initial base lidar pose
   * @param[in] ground_plane_model ground plane model
   * @retval the refined base lidar pose
   */
  Eigen::Isometry3d refineBaseLidarPose(
    const Eigen::Isometry3d & base_lidar_pose, const Eigen::Vector4d & model) const;

  /*!
   * Removes the point that are consistent with an input plane from the pointcloud
   * @param[in] input_pointcloud the pointcloud to filter
   * @param[in] outlier_model the model that represents the outliers
   * @param[in] outlier_tolerance the tolerance with which a point is still considered an outlier
   * @retval the refined base lidar pose
   */
  pcl::PointCloud<PointType>::Ptr removeOutliers(
    pcl::PointCloud<PointType>::Ptr input_pointcloud, const Eigen::Vector4d & outlier_plane_model,
    double outlier_tolerance) const;

  // Parameters
  // We perform base-lidar pose estimation but the output are frames in between
  // base -> parent -> child -> lidar
  std::string base_frame_;
  std::string parent_frame_;
  std::string child_frame_;

  double marker_size_;
  bool use_crop_box_filter_;
  double crop_box_min_x_;
  double crop_box_min_y_;
  double crop_box_min_z_;
  double crop_box_max_x_;
  double crop_box_max_y_;
  double crop_box_max_z_;
  bool remove_outliers_;
  double remove_outlier_tolerance_;
  bool use_pca_rough_normal_;
  double max_inlier_distance_;
  int min_plane_points_;
  int min_plane_points_percentage_;
  double max_cos_distance_;
  int max_iterations_;
  bool verbose_;
  bool broadcast_calibration_tf_;
  bool filter_estimations_;
  double initial_angle_cov_;
  double initial_translation_cov_;
  double angle_measurement_cov_;
  double angle_process_cov_;
  double translation_measurement_cov_;
  double translation_process_cov_;
  double angle_convergence_threshold_;
  double translation_convergence_threshold_;

  // ROS Interface
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr srv_callback_group_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inliers_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr service_server_;

  // Threading, sync, and result
  std::mutex mutex_;

  // ROS Data
  std_msgs::msg::Header header_;
  std::string lidar_frame_;

  // Initial tfs comparable with the one with our method
  geometry_msgs::msg::Transform initial_base_to_lidar_msg_;
  tf2::Transform initial_base_to_lidar_tf2_;
  Eigen::Isometry3d initial_base_to_lidar_eigen_;

  // Other tfs to calculate the complete chain. There are constant for our purposes
  geometry_msgs::msg::Transform base_to_parent_msg_;
  tf2::Transform base_to_parent_tf2_;
  Eigen::Isometry3d base_to_parent_eigen_;

  geometry_msgs::msg::Transform child_to_lidar_msg_;
  tf2::Transform child_to_lidar_tf2_;
  Eigen::Isometry3d child_to_lidar_eigen_;

  geometry_msgs::msg::Pose output_parent_to_child_msg_;
  Eigen::Isometry3d output_parent_to_child_eigen_;

  bool got_initial_transform_;
  bool broadcast_tf_;
  bool calibration_done_;

  // Filtering
  KalmanFilter kalman_filter_;
  bool first_observation_;
  RingBuffer<pcl::PointCloud<PointType>::Ptr> inlier_observations_;
  std::vector<Eigen::Vector4d> outlier_models_;
};

#endif  // EXTRINSIC_GROUND_PLANE_CALIBRATOR__EXTRINSIC_GROUND_PLANE_CALIBRATOR_HPP_
