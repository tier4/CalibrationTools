// Copyright 2021 Tier IV, Inc.
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

#ifndef EXTRINSIC_TAG_BASED_CALIBRATOR_EXTRINSIC_TAG_BASED_CALIBRATOR_HPP_
#define EXTRINSIC_TAG_BASED_CALIBRATOR_EXTRINSIC_TAG_BASED_CALIBRATOR_HPP_

#include <extrinsic_tag_based_calibrator/calibration_estimator.hpp>
#include <extrinsic_tag_based_calibrator/tag_calibrator_visualizer.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lidartag_msgs/msg/lidar_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_calibration_msgs/msg/calibration_points.hpp>
#include <tier4_calibration_msgs/srv/extrinsic_calibrator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

class ExtrinsicTagBasedCalibrator : public rclcpp::Node
{
public:
  ExtrinsicTagBasedCalibrator(const rclcpp::NodeOptions & options);

protected:
  void cameraImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci);

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);

  void lidarTagDetectionsCallback(
    const lidartag_msgs::msg::LidarTagDetectionArray::SharedPtr detections_msg);

  void aprilTagDetectionsCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr detections_msg);

  void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point_msg);

  void requestReceivedCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);

  void tfTimerCallback();
  void manualCalibrationTimerCallback();
  void semiautomaticCalibrationTimerCallback();
  void automaticCalibrationTimerCallback();

  void publishCalibrationPoints(
    const std::vector<cv::Point3d> & object_points, const std::vector<cv::Point2d> & image_points);

  bool calibrate(
    const std::vector<cv::Point3d> & object_points, const std::vector<cv::Point2d> & image_points,
    const rclcpp::Time & timestamp);

  // Parameters
  std::string parent_frame_;
  std::string child_frame_;
  std::string base_frame_;
  std::string calibration_mode_;
  float calib_rate_;

  // Filter parameters
  double min_tag_size_;
  double max_tag_distance_;
  double max_allowed_homography_error_;
  int min_pnp_points_;
  CalibrationEstimator estimator_;

  // ROS Interface
  tf2_ros::StaticTransformBroadcaster tf_broascaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr srv_callback_group_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    filtered_projections_markers_pub_;

  rclcpp::Publisher<tier4_calibration_msgs::msg::CalibrationPoints>::SharedPtr
    calibration_points_pub_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  rclcpp::Subscription<lidartag_msgs::msg::LidarTagDetectionArray>::SharedPtr
    lidartag_detections_array_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
    apriltag_detections_array_sub_;

  rclcpp::TimerBase::SharedPtr calib_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr service_server_;

  // Threading, sync, and result
  std::mutex mutex_;

  // Rviz visualizations
  std::unique_ptr<TagCalibratorVisualizer> visualizer_;

  // ROS Data
  rclcpp::Time latest_timestamp_;
  std_msgs::msg::Header header_;
  sensor_msgs::msg::CameraInfo camera_info_;
  image_geometry::PinholeCameraModel pinhole_camera_model_;

  lidartag_msgs::msg::LidarTagDetectionArray::SharedPtr lidartag_detections_array_;
  apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr apriltag_detections_array_;
  std::string lidar_frame_;
  std::string optical_frame_;

  tf2::Transform parent_to_lidar_tf2_;
  tf2::Transform optical_axis_to_camera_tf2_;
  tf2::Transform initial_optical_axis_to_lidar_tf2_;
  tf2::Transform base_to_lidar_tf2_;
  bool got_initial_transform;

  double initial_reproj_error_;
  double current_reproj_error_;
  double filtered_reproj_error_;
};

#endif  // EXTRINSIC_TAG_BASED_CALIBRATOR_EXTRINSIC_TAG_BASED_CALIBRATOR_HPP_
