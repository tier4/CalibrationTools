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

#ifndef EXTRINSIC_GROUND_PLANE_CALIBRATOR__EXTRINSIC_GROUND_PLANE_CALIBRATOR_HPP_
#define EXTRINSIC_GROUND_PLANE_CALIBRATOR__EXTRINSIC_GROUND_PLANE_CALIBRATOR_HPP_

#include <Eigen/Dense>
#include <extrinsic_ground_plane_calibrator/utils.hpp>
#include <kalman_filter/kalman_filter.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_ground_plane_utils/ground_plane_utils.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tier4_calibration_msgs/srv/new_extrinsic_calibrator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
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

namespace extrinsic_ground_plane_calibrator
{

using PointType = pcl::PointXYZ;
using tier4_ground_plane_utils::GroundPlaneExtractorParameters;

/**
 * A base-lidar calibrator.
 *
 * This calibrator assumes that the area around the vehicle consists on a flat surface and consist
 * essentially on a plane estimation algorithm,
 *
 * Once the plane has been estimated, the create an arbitrary coordinate system in the estimated
 * ground (gcs = ground coordinate system). Then, we proceed to estimate the initial base link pose
 * in gcs, and proceed to project it into the new "ground", that is dropping the z component. Once
 * that has been done, we can recompute the "calibrated" base lidar transform.
 *
 * Note: Although the result of this algorithm is a full 3D pose, not all the parameters were really
 * calibrated. With only the ground, we can only calibrate roll, pitch, and z. That is despite the
 * fact that the initial and output transforms may have different x, y, and yaw values. This is due
 * to the fact once we obtain the ground plane, we only know for certain the normal and distance of
 * the plane with respect to the lidar. All other values and transformation are derived from the
 * initial base lidar calibration
 */

class ExtrinsicGroundPlaneCalibrator : public rclcpp::Node
{
public:
  explicit ExtrinsicGroundPlaneCalibrator(const rclcpp::NodeOptions & options);

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
   * ROS pointcloud callback
   * @param[in] pc the input pointcloud
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);

  /*!
   * Checks that all the needed tfs are available
   * @return wether or not all the needed tfs are available
   */
  bool checkInitialTransforms();

  /*!
   * Filter individual calibration plane estimations and accumulate the inliers for a final
   * regression
   * @param[in] estimated_model the estimated model
   * @param[in] inliers the inliers of the current estimated model
   */
  void filterGroundModelEstimation(
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

  // Parameters
  std::string base_frame_;
  std::string lidar_frame_;

  GroundPlaneExtractorParameters ground_plane_extractor_parameters_;
  double marker_size_;
  bool verbose_;
  bool overwrite_xy_yaw_;
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

  rclcpp::Service<tier4_calibration_msgs::srv::NewExtrinsicCalibrator>::SharedPtr service_server_;

  // Threading, sync, and result
  std::mutex mutex_;

  // ROS Data
  std_msgs::msg::Header header_;

  // Initial tfs comparable with the one with our method
  geometry_msgs::msg::TransformStamped initial_base_to_lidar_transform_msg_;
  Eigen::Isometry3d initial_base_to_lidar_transform_;

  Eigen::Isometry3d calibrated_base_to_lidar_transform_;

  bool got_initial_transform_{false};
  bool received_request_{false};
  bool calibration_done_{false};

  // Filtering
  KalmanFilter kalman_filter_;
  bool first_observation_{true};
  RingBuffer<pcl::PointCloud<PointType>::Ptr> inlier_observations_;
  std::vector<Eigen::Vector4d> outlier_models_;
};

}  // namespace extrinsic_ground_plane_calibrator

#endif  // EXTRINSIC_GROUND_PLANE_CALIBRATOR__EXTRINSIC_GROUND_PLANE_CALIBRATOR_HPP_
