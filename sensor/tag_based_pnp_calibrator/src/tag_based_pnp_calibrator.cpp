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

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <rclcpp/time.hpp>
#include <tag_based_pnp_calibrator/tag_based_pnp_calibrator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tier4_calibration_msgs/msg/calibration_result.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2/utils.h>

ExtrinsicTagBasedPNPCalibrator::ExtrinsicTagBasedPNPCalibrator(const rclcpp::NodeOptions & options)
: Node("tag_based_pnp_calibrator_node", options),
  tf_broadcaster_(this),
  request_received_(false),
  got_initial_transform(false)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  calib_rate_ = this->declare_parameter<double>("calib_rate");
  base_frame_ = this->declare_parameter<std::string>("base_frame");
  min_tag_size_ = this->declare_parameter<double>("min_tag_size");
  max_tag_distance_ = this->declare_parameter<double>("max_tag_distance");
  max_allowed_homography_error_ = this->declare_parameter<double>("max_allowed_homography_error");
  use_receive_time_ = this->declare_parameter<bool>("use_receive_time");
  use_rectified_image_ = this->declare_parameter<bool>("use_rectified_image");

  double calibration_crossvalidation_training_ratio =
    this->declare_parameter<double>("calibration_crossvalidation_training_ratio");
  int calibration_convergence_min_pairs =
    this->declare_parameter<int>("calibration_convergence_min_pairs");
  double calibration_convergence_min_area_percentage =
    this->declare_parameter<double>("calibration_convergence_min_area_percentage");
  min_pnp_points_ = this->declare_parameter<int>("min_pnp_points");
  double min_convergence_time = this->declare_parameter<double>("min_convergence_time");
  double max_no_observation_time = this->declare_parameter<double>("max_no_observation_time");
  double new_hypothesis_distance = this->declare_parameter<double>("new_hypothesis_distance");
  std::vector<int64_t> tag_ids = this->declare_parameter<std::vector<int64_t>>("tag_ids");
  std::vector<double> tag_sizes = this->declare_parameter<std::vector<double>>("tag_sizes");

  double lidartag_max_convergence_transl =
    this->declare_parameter<double>("lidartag_max_convergence_transl");
  double lidartag_max_convergence_transl_dot =
    this->declare_parameter<double>("lidartag_max_convergence_transl_dot");
  double lidartag_max_convergence_rot =
    this->declare_parameter<double>("lidartag_max_convergence_rot");
  double lidartag_max_convergence_rot_dot =
    this->declare_parameter<double>("lidartag_max_convergence_rot_dot");
  double lidartag_new_hypothesis_transl =
    this->declare_parameter<double>("lidartag_new_hypothesis_transl");
  double lidartag_new_hypothesis_rot =
    this->declare_parameter<double>("lidartag_new_hypothesis_rot");
  double lidartag_measurement_noise_transl =
    this->declare_parameter<double>("lidartag_measurement_noise_transl");
  double lidartag_measurement_noise_rot =
    this->declare_parameter<double>("lidartag_measurement_noise_rot");
  double lidartag_process_noise_transl =
    this->declare_parameter<double>("lidartag_process_noise_transl");
  double lidartag_process_noise_transl_dot =
    this->declare_parameter<double>("lidartag_process_noise_transl_dot");
  double lidartag_process_noise_rot = this->declare_parameter<double>("lidartag_process_noise_rot");
  double lidartag_process_noise_rot_dot =
    this->declare_parameter<double>("lidartag_process_noise_rot_dot");

  double apriltag_max_convergence_transl =
    this->declare_parameter<double>("apriltag_max_convergence_transl");
  double apriltag_new_hypothesis_transl =
    this->declare_parameter<double>("apriltag_new_hypothesis_transl");
  double apriltag_measurement_noise_transl =
    this->declare_parameter<double>("apriltag_measurement_noise_transl");
  double apriltag_process_noise_transl =
    this->declare_parameter<double>("apriltag_process_noise_transl");

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::QoS(1).best_effort(),
    std::bind(&ExtrinsicTagBasedPNPCalibrator::cameraInfoCallback, this, std::placeholders::_1));

  lidartag_detections_array_sub_ =
    this->create_subscription<lidartag_msgs::msg::LidarTagDetectionArray>(
      "lidartag/detections_array", 1,
      std::bind(
        &ExtrinsicTagBasedPNPCalibrator::lidarTagDetectionsCallback, this, std::placeholders::_1));

  apriltag_detections_array_sub_ =
    this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "apriltag/detection_array", 1,
      std::bind(
        &ExtrinsicTagBasedPNPCalibrator::aprilTagDetectionsCallback, this, std::placeholders::_1));

  filtered_projections_markers_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("filtered_projections", 10);

  calibration_points_pub_ = this->create_publisher<tier4_calibration_msgs::msg::CalibrationPoints>(
    "calibration_points", 10);

  estimator_.setCrossvalidationTrainingRatio(calibration_crossvalidation_training_ratio);
  estimator_.setCalibrationConvergenceCriteria(
    calibration_convergence_min_pairs, calibration_convergence_min_area_percentage);
  estimator_.setMinPnpPairs(min_pnp_points_);
  estimator_.setMinConvergenceTime(min_convergence_time);
  estimator_.setMaxNoObservationTime(max_no_observation_time);
  estimator_.setNewHypothesisDistance(new_hypothesis_distance);
  estimator_.setTagSizes(tag_ids, tag_sizes);

  estimator_.setLidartagMaxConvergenceThreshold(
    lidartag_max_convergence_transl, lidartag_max_convergence_transl_dot,
    lidartag_max_convergence_rot, lidartag_max_convergence_rot_dot);
  estimator_.setLidartagNewHypothesisThreshold(
    lidartag_new_hypothesis_transl, lidartag_new_hypothesis_rot);
  estimator_.setLidartagMeasurementNoise(
    lidartag_measurement_noise_transl, lidartag_measurement_noise_rot);
  estimator_.setLidartagProcessNoise(
    lidartag_process_noise_transl, lidartag_process_noise_transl_dot, lidartag_process_noise_rot,
    lidartag_process_noise_rot_dot);

  estimator_.setApriltagMaxConvergenceThreshold(apriltag_max_convergence_transl);
  estimator_.setApriltagNewHypothesisThreshold(apriltag_new_hypothesis_transl);
  estimator_.setApriltagMeasurementNoise(apriltag_measurement_noise_transl);
  estimator_.setApriltagProcessNoise(apriltag_process_noise_transl);

  tf_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<double>(1.0 / calib_rate_),
    std::bind(&ExtrinsicTagBasedPNPCalibrator::tfTimerCallback, this));

  calib_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<double>(1.0 / calib_rate_),
    std::bind(&ExtrinsicTagBasedPNPCalibrator::automaticCalibrationTimerCallback, this));

  srv_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // initialize service server
  service_server_ = this->create_service<tier4_calibration_msgs::srv::NewExtrinsicCalibrator>(
    "extrinsic_calibration",
    std::bind(
      &ExtrinsicTagBasedPNPCalibrator::requestReceivedCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, srv_callback_group_);

  visualizer_ = std::make_unique<TagCalibratorVisualizer>(filtered_projections_markers_pub_);

  visualizer_->setTagSizes(tag_ids, tag_sizes);
  visualizer_->setBaseFrame(base_frame_);

  visualizer_->setMinConvergenceTime(min_convergence_time);
  visualizer_->setMaxNoObservationTime(max_no_observation_time);
  visualizer_->setLidartagMaxConvergenceThreshold(
    lidartag_max_convergence_transl, lidartag_max_convergence_transl_dot,
    lidartag_max_convergence_rot, lidartag_max_convergence_rot_dot);
  visualizer_->setApriltagMaxConvergenceThreshold(apriltag_max_convergence_transl);
}

void ExtrinsicTagBasedPNPCalibrator::lidarTagDetectionsCallback(
  const lidartag_msgs::msg::LidarTagDetectionArray::SharedPtr detections_msg_ptr)
{
  lidartag_detections_array_ = detections_msg_ptr;

  if (use_receive_time_) {
    lidartag_detections_array_->header.stamp = this->now();
  }

  latest_timestamp_ = rclcpp::Time(lidartag_detections_array_->header.stamp);
  lidar_frame_ = lidartag_detections_array_->header.frame_id;

  estimator_.update(*lidartag_detections_array_);

  visualizer_->setLidarFrame(lidar_frame_);
}

void ExtrinsicTagBasedPNPCalibrator::aprilTagDetectionsCallback(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr detections_msg_ptr)
{
  // Filter apriltag detections that are too far away from the sensor
  double max_distance_px = min_tag_size_ * pinhole_camera_model_.fx() / max_tag_distance_;

  auto filtered_detections = std::make_shared<apriltag_msgs::msg::AprilTagDetectionArray>();
  filtered_detections->header = detections_msg_ptr->header;

  for (auto & detection : detections_msg_ptr->detections) {
    const int & corners_size = detection.corners.size();
    double max_side_distance = 0.0;
    double max_homography_error = 0.0;

    cv::Mat H(3, 3, CV_64F);
    std::memcpy(H.data, detection.homography.data(), 9 * sizeof(double));
    cv::Mat H_inv = H.inv();

    for (int i = 0; i < corners_size; i++) {
      const auto & p1 = detection.corners[i];
      const auto & p2 = detection.corners[(i + 1) % corners_size];

      cv::Mat_<double> p_corner(3, 1);
      p_corner(0, 0) = detection.corners[i].x;
      p_corner(1, 0) = detection.corners[i].y;
      p_corner(2, 0) = 1.0;

      cv::Mat p_corner2 = H_inv * p_corner;

      double h_error = std::abs(p_corner2.at<double>(2, 0) - 1.0);
      max_homography_error = std::max(max_homography_error, h_error);

      double side_distance = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
      max_side_distance = std::max(max_side_distance, side_distance);
    }

    // We discard detections that are theoretically detected too far away
    if (max_side_distance < max_distance_px) {
      RCLCPP_DEBUG_STREAM(
        get_logger(), "Discarding apriltag: size " << max_side_distance
                                                   << " px. Expecting at least " << max_distance_px
                                                   << " px");
      continue;
    }

    // We also discard detections with an unreliable homography
    if (max_homography_error > max_allowed_homography_error_) {
      RCLCPP_DEBUG_STREAM(
        get_logger(), "Discarding apriltag: homography error " << max_homography_error);
      continue;
    }

    filtered_detections->detections.push_back(detection);
  }

  apriltag_detections_array_ = filtered_detections;

  if (use_receive_time_) {
    apriltag_detections_array_->header.stamp = this->now();
  }

  latest_timestamp_ = rclcpp::Time(apriltag_detections_array_->header.stamp);
  estimator_.update(*apriltag_detections_array_);
}

void ExtrinsicTagBasedPNPCalibrator::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
{
  optical_frame_ = camera_info_msg->header.frame_id;
  camera_info_ = *camera_info_msg;

  if (use_receive_time_) {
    camera_info_.header.stamp = this->now();
  }

  if (use_rectified_image_) {
    camera_info_.k[0] = camera_info_.p[0];
    camera_info_.k[2] = camera_info_.p[2];
    camera_info_.k[4] = camera_info_.p[5];
    camera_info_.k[5] = camera_info_.p[6];
    std::fill(camera_info_.d.begin(), camera_info_.d.end(), 0.0);
  }

  header_ = camera_info_.header;
  latest_timestamp_ = rclcpp::Time(header_.stamp);

  visualizer_->setCameraFrame(optical_frame_);
  pinhole_camera_model_.fromCameraInfo(camera_info_);
  visualizer_->setCameraModel(camera_info_);
  estimator_.setCameraModel(camera_info_);
}

void ExtrinsicTagBasedPNPCalibrator::requestReceivedCallback(
  [[maybe_unused]] const std::shared_ptr<
    tier4_calibration_msgs::srv::NewExtrinsicCalibrator::Request>
    request,
  const std::shared_ptr<tier4_calibration_msgs::srv::NewExtrinsicCalibrator::Response> response)
{
  using std::chrono_literals::operator""s;
  RCLCPP_INFO(this->get_logger(), "Received calibration request");

  {
    std::unique_lock<std::mutex> lock(mutex_);
    request_received_ = true;
  }

  // Wait for subscription topic
  while (rclcpp::ok()) {
    rclcpp::sleep_for(1s);
    std::unique_lock<std::mutex> lock(mutex_);

    if (got_initial_transform && estimator_.converged() && estimator_.calibrate()) {
      break;
    }
  }

  tf2::Transform optical_axis_to_lidar_tf2 = estimator_.getFilteredPoseAsTF();

  geometry_msgs::msg::Transform transform_msg;
  transform_msg = tf2::toMsg(optical_axis_to_lidar_tf2);

  tier4_calibration_msgs::msg::CalibrationResult result;
  result.success = true;
  result.score = estimator_.getCrossValidationReprojectionError();
  result.message.data =
    "Calibrated using " + std::to_string(estimator_.getCurrentCalibrationPairsNumber()) + " pairs";
  result.transform_stamped.transform = tf2::toMsg(optical_axis_to_lidar_tf2);
  result.transform_stamped.header.frame_id = optical_frame_;
  result.transform_stamped.child_frame_id = lidar_frame_;

  response->results.push_back(result);
}

void ExtrinsicTagBasedPNPCalibrator::tfTimerCallback()
{
  if (!got_initial_transform && lidar_frame_ != "" && optical_frame_ != "") {
    try {
      geometry_msgs::msg::TransformStamped initial_optical_axis_to_lidar_transform_msg;
      geometry_msgs::msg::TransformStamped base_to_lidar_transform_msg;

      initial_optical_axis_to_lidar_transform_msg = tf_buffer_->lookupTransform(
        optical_frame_, lidar_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

      base_to_lidar_transform_msg = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

      fromMsg(
        initial_optical_axis_to_lidar_transform_msg.transform, initial_optical_axis_to_lidar_tf2_);
      fromMsg(base_to_lidar_transform_msg.transform, base_to_lidar_tf2_);

      // Set the fixed base-lidar tf to the visualizers
      cv::Matx33d base_lidar_rot_matrix;
      cv::Matx31d base_lidar_trans_vector;

      Eigen::Isometry3d base_lidar_transform_eigen =
        tf2::transformToEigen(tf2::toMsg(base_to_lidar_tf2_));
      Eigen::Matrix3d base_lidar_rotation_eigen = base_lidar_transform_eigen.rotation();
      Eigen::Vector3d base_lidar_translation_eigen = base_lidar_transform_eigen.translation();
      cv::eigen2cv(base_lidar_rotation_eigen, base_lidar_rot_matrix);
      cv::eigen2cv(base_lidar_translation_eigen, base_lidar_trans_vector);

      visualizer_->setBaseLidarTransform(base_lidar_trans_vector, base_lidar_rot_matrix);

      got_initial_transform = true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "could not get initial tf. %s", ex.what());
      return;
    }
  }

  std::unique_lock<std::mutex> lock(mutex_);

  if (!estimator_.valid() || !got_initial_transform) {
    return;
  }

  tf2::Transform optical_axis_to_lidar_tf2 = estimator_.getFilteredPoseAsTF();

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = header_.stamp;
  transform_stamped.header.frame_id = optical_frame_;
  transform_stamped.child_frame_id = lidar_frame_;
  transform_stamped.transform = tf2::toMsg(optical_axis_to_lidar_tf2);
  tf_broadcaster_.sendTransform(transform_stamped);
}

void ExtrinsicTagBasedPNPCalibrator::automaticCalibrationTimerCallback()
{
  std::unique_lock<std::mutex> lock(mutex_);

  if (!request_received_ || !apriltag_detections_array_ || !lidartag_detections_array_) {
    return;
  }

  estimator_.update(latest_timestamp_);

  if (estimator_.calibrate()) {
    // Visualization
    cv::Matx33d initial_rot_matrix;
    cv::Matx31d initial_trans_vector;

    auto [current_trans_vector, current_rot_matrix] = estimator_.getCurrentPose();
    auto [filtered_trans_vector, filtered_rot_matrix] = estimator_.getFilteredPose();

    Eigen::Isometry3d initial_transform_eigen =
      tf2::transformToEigen(tf2::toMsg(initial_optical_axis_to_lidar_tf2_));
    Eigen::Matrix3d initial_rotation_eigen = initial_transform_eigen.rotation();
    Eigen::Vector3d initial_translation_eigen = initial_transform_eigen.translation();
    cv::eigen2cv(initial_rotation_eigen, initial_rot_matrix);
    cv::eigen2cv(initial_translation_eigen, initial_trans_vector);

    // Calculate the reprojection errors
    cv::Matx31d initial_rvec, current_rvec, filtered_rvec;

    cv::Rodrigues(initial_rot_matrix, initial_rvec);
    cv::Rodrigues(current_rot_matrix, current_rvec);
    cv::Rodrigues(filtered_rot_matrix, filtered_rvec);

    visualizer_->setCameraLidarTransform(filtered_trans_vector, filtered_rot_matrix);

    std::vector<cv::Point2d> current_projected_points, initial_projected_points,
      filtered_projected_points;

    auto camera_intrinsics = pinhole_camera_model_.intrinsicMatrix();
    auto distortion_coeffs = pinhole_camera_model_.distortionCoeffs();

    // Obtain the calibration points
    auto [object_points, image_points] = estimator_.getCalibrationPoints(false);

    if (object_points.size() == 0 || image_points.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Could not get the calibration points");
      return;
    }

    cv::projectPoints(
      object_points, initial_rvec, initial_trans_vector, camera_intrinsics, distortion_coeffs,
      initial_projected_points);

    cv::projectPoints(
      object_points, current_rvec, current_trans_vector, camera_intrinsics, distortion_coeffs,
      current_projected_points);

    cv::projectPoints(
      object_points, filtered_rvec, filtered_trans_vector, camera_intrinsics, distortion_coeffs,
      filtered_projected_points);

    auto reprojection_error = [](auto & points1, auto & points2) -> double {
      double error = 0.0;

      for (std::size_t i = 0; i < points1.size(); i++) {
        error += cv::norm(points1[i] - points2[i]);
      }

      return error / points1.size();
    };

    double initial_reprojection_error = reprojection_error(image_points, initial_projected_points);
    double current_reprojection_error = reprojection_error(image_points, current_projected_points);
    double filtered_reprojection_error =
      reprojection_error(image_points, filtered_projected_points);

    RCLCPP_INFO(
      this->get_logger(),
      "Partial calibration results (%d/%d pairs):", estimator_.getCurrentCalibrationPairsNumber(),
      estimator_.getConvergencePairNumber());
    RCLCPP_INFO(
      this->get_logger(), "\tInitial reprojection error=%.2f", initial_reprojection_error);
    RCLCPP_INFO(
      this->get_logger(), "\tCurrent reprojection error=%.2f", current_reprojection_error);
    RCLCPP_INFO(
      this->get_logger(), "\tFiltered reprojection error=%.2f", filtered_reprojection_error);

    // Publish calibration points
    publishCalibrationPoints(object_points, image_points);
  }

  visualizer_->drawCalibrationStatus(estimator_, latest_timestamp_);

  // Consume detections
  apriltag_detections_array_.reset();
  lidartag_detections_array_.reset();
}

void ExtrinsicTagBasedPNPCalibrator::publishCalibrationPoints(
  const std::vector<cv::Point3d> & object_points, const std::vector<cv::Point2d> & image_points)
{
  tier4_calibration_msgs::msg::CalibrationPoints msg;
  geometry_msgs::msg::Point object_point;
  geometry_msgs::msg::Point image_point;

  assert(object_points.size() == image_points.size());

  for (std::size_t i = 0; i < object_points.size(); ++i) {
    object_point.x = object_points[i].x;
    object_point.y = object_points[i].y;
    object_point.z = object_points[i].z;

    image_point.x = image_points[i].x;
    image_point.y = image_points[i].y;

    msg.object_points.push_back(object_point);
    msg.image_points.push_back(image_point);
  }

  calibration_points_pub_->publish(msg);
}
