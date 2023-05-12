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

#include <cv/sqpnp.hpp>
#include <extrinsic_tag_based_calibrator/extrinsic_tag_based_calibrator.hpp>
#include <extrinsic_tag_based_calibrator/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/time.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

ExtrinsicTagBasedCalibrator::ExtrinsicTagBasedCalibrator(const rclcpp::NodeOptions & options)
: Node("extrinsic_tag_based_calibrator_node", options),
  tf_broascaster_(this),
  got_initial_transform(false)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  calib_rate_ = this->declare_parameter<double>("calib_rate");
  parent_frame_ = this->declare_parameter<std::string>("parent_frame");
  child_frame_ = this->declare_parameter<std::string>("child_frame");
  base_frame_ = this->declare_parameter<std::string>("base_frame");
  min_tag_size_ = this->declare_parameter<double>("min_tag_size");
  max_tag_distance_ = this->declare_parameter<double>("max_tag_distance");
  max_allowed_homography_error_ = this->declare_parameter<double>("max_allowed_homography_error");

  std::string dynamics_model = this->declare_parameter<std::string>("dynamics_model");
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
    std::bind(&ExtrinsicTagBasedCalibrator::cameraInfoCallback, this, std::placeholders::_1));

  lidartag_detections_array_sub_ =
    this->create_subscription<lidartag_msgs::msg::LidarTagDetectionArray>(
      "lidartag/detections_array", 1,
      std::bind(
        &ExtrinsicTagBasedCalibrator::lidarTagDetectionsCallback, this, std::placeholders::_1));

  apriltag_detections_array_sub_ =
    this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "apriltag/detection_array", 1,
      std::bind(
        &ExtrinsicTagBasedCalibrator::aprilTagDetectionsCallback, this, std::placeholders::_1));

  filtered_projections_markers_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("filtered_projections", 10);

  calibration_points_pub_ = this->create_publisher<tier4_calibration_msgs::msg::CalibrationPoints>(
    "calibration_points", 10);

  estimator_.setDynamicsModel(
    dynamics_model == "constant_velocity" ? DynamicsModel::ConstantVelocity
                                          : DynamicsModel::Static);

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

  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / calib_rate_));

  auto tf_timer_callback = std::bind(&ExtrinsicTagBasedCalibrator::tfTimerCallback, this);

  tf_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(tf_timer_callback)>>(
    this->get_clock(), period_ns, std::move(tf_timer_callback),
    this->get_node_base_interface()->get_context());

  auto calib_timer_callback =
    std::bind(&ExtrinsicTagBasedCalibrator::automaticCalibrationTimerCallback, this);

  calib_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(calib_timer_callback)>>(
    this->get_clock(), period_ns, std::move(calib_timer_callback),
    this->get_node_base_interface()->get_context());

  this->get_node_timers_interface()->add_timer(tf_timer_, nullptr);
  this->get_node_timers_interface()->add_timer(calib_timer_, nullptr);

  srv_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // initialize service server
  service_server_ = this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
    "extrinsic_calibration",
    std::bind(
      &ExtrinsicTagBasedCalibrator::requestReceivedCallback, this, std::placeholders::_1,
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

void ExtrinsicTagBasedCalibrator::lidarTagDetectionsCallback(
  const lidartag_msgs::msg::LidarTagDetectionArray::SharedPtr detections_msg)
{
  latest_timestamp_ = rclcpp::Time(detections_msg->header.stamp);
  lidar_frame_ = detections_msg->header.frame_id;
  lidartag_detections_array_ = detections_msg;

  estimator_.update(*detections_msg);

  visualizer_->setLidarFrame(lidar_frame_);
}

void ExtrinsicTagBasedCalibrator::aprilTagDetectionsCallback(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr detections_msg)
{
  latest_timestamp_ = rclcpp::Time(detections_msg->header.stamp);

  // Filter apriltag detections that are too far away from the sensor
  double max_distance_px = min_tag_size_ * pinhole_camera_model_.fx() / max_tag_distance_;

  auto filtered_detections = std::make_shared<apriltag_msgs::msg::AprilTagDetectionArray>();
  filtered_detections->header = detections_msg->header;

  for (auto & detection : detections_msg->detections) {
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

      // According to the equation (x2, y2, 1) = H *(x1, y1, 1) the third component should be 1.0
      double h_error = std::abs(p_corner2.at<double>(2, 0) - 1.0);
      max_homography_error = std::max(max_homography_error, h_error);

      double side_distance = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
      max_side_distance = std::max(max_side_distance, side_distance);
    }

    // We discard detections that are theoretically detected too far away
    if (max_side_distance < max_distance_px) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Discarding apriltag: size " << max_side_distance
                                                   << " px. Expecting at least " << max_distance_px
                                                   << " px");
      continue;
    }

    // We also discard detections with an unreliable homography
    if (max_homography_error > max_allowed_homography_error_) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Discarding apriltag: homography error " << max_homography_error);
      continue;
    }

    filtered_detections->detections.push_back(detection);
  }

  apriltag_detections_array_ = filtered_detections;

  estimator_.update(*apriltag_detections_array_);
}

/*void ExtrinsicTagBasedCalibrator::cameraImageCallback(const
sensor_msgs::msg::Image::ConstSharedPtr & msg_img, const
sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci)
{
  header_ = msg_ci->header;
  optical_frame_ = msg_ci->header.frame_id;
  camera_info_ = *msg_ci;

  pinhole_camera_model_.fromCameraInfo(camera_info_);
  visualizer_->setCameraModel(camera_info_);

  if (manual_calibrator_ui)
  {
    cv::Mat cv_img_ = cv_bridge::toCvShare(msg_img)->image.clone();
    cv::cvtColor(cv_img_, cv_img_, CV_BayerRG2RGB);

    manual_calibrator_ui->setImage(cv_img_);
  }
}*/

void ExtrinsicTagBasedCalibrator::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
{
  latest_timestamp_ = rclcpp::Time(camera_info_msg->header.stamp);
  header_ = camera_info_msg->header;
  optical_frame_ = camera_info_msg->header.frame_id;
  camera_info_ = *camera_info_msg;

  visualizer_->setCameraFrame(optical_frame_);

  pinhole_camera_model_.fromCameraInfo(camera_info_);
  visualizer_->setCameraModel(camera_info_);
  estimator_.setCameraModel(camera_info_);
}

void ExtrinsicTagBasedCalibrator::requestReceivedCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  CV_UNUSED(request);

  // Wait for subscription topic
  while (rclcpp::ok()) {
    rclcpp::sleep_for(1s);
    std::unique_lock<std::mutex> lock(mutex_);

    if (estimator_.converged() && got_initial_transform && estimator_.calibrate()) {
      break;
    }

    // RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Waiting for the lidar-camera calibration to end");
  }

  tf2::Transform optical_axis_to_lidar_tf2 = estimator_.getFilteredPose();
  tf2::Transform parent_to_child_tf2 =
    parent_to_lidar_tf2_ * optical_axis_to_lidar_tf2.inverse() * optical_axis_to_camera_tf2_;

  geometry_msgs::msg::Transform transform_msg;
  transform_msg = tf2::toMsg(parent_to_child_tf2);

  response->success = true;
  response->result_pose.position.x = transform_msg.translation.x;
  response->result_pose.position.y = transform_msg.translation.y;
  response->result_pose.position.z = transform_msg.translation.z;
  response->result_pose.orientation = transform_msg.rotation;

  response->score = estimator_.getCrossValidationReprojError();
}

void ExtrinsicTagBasedCalibrator::tfTimerCallback()
{
  if (/*!got_initial_transform && */ lidar_frame_ != "" && optical_frame_ != "") {
    try {
      geometry_msgs::msg::TransformStamped parent_to_lidar_transform_msg;
      geometry_msgs::msg::TransformStamped optical_axis_to_camera_transform_msg;
      geometry_msgs::msg::TransformStamped initial_optical_axis_to_lidar_transform_msg;
      geometry_msgs::msg::TransformStamped base_to_lidar_transform_msg;

      parent_to_lidar_transform_msg = tf_buffer_->lookupTransform(
        parent_frame_, lidar_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

      optical_axis_to_camera_transform_msg = tf_buffer_->lookupTransform(
        optical_frame_, child_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

      initial_optical_axis_to_lidar_transform_msg = tf_buffer_->lookupTransform(
        optical_frame_, lidar_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

      base_to_lidar_transform_msg = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

      fromMsg(parent_to_lidar_transform_msg.transform, parent_to_lidar_tf2_);
      fromMsg(optical_axis_to_camera_transform_msg.transform, optical_axis_to_camera_tf2_);
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

  tf2::Transform optical_axis_to_lidar_tf2 = estimator_.getFilteredPose();
  tf2::Transform parent_to_child_tf2 =
    parent_to_lidar_tf2_ * optical_axis_to_lidar_tf2.inverse() * optical_axis_to_camera_tf2_;

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = header_.stamp;
  transform_stamped.header.frame_id = parent_frame_;
  transform_stamped.child_frame_id = child_frame_;
  transform_stamped.transform = tf2::toMsg(parent_to_child_tf2);
  tf_broascaster_.sendTransform(transform_stamped);
}

void ExtrinsicTagBasedCalibrator::automaticCalibrationTimerCallback()
{
  std::unique_lock<std::mutex> lock(mutex_);

  if (!apriltag_detections_array_ || !lidartag_detections_array_) {
    return;
  }

  estimator_.update(latest_timestamp_);

  if (estimator_.calibrate()) {
    // Visualization
    cv::Matx33d initial_rot_matrix, current_rot_matrix, filtered_rot_matrix;
    cv::Matx31d initial_trans_vector, current_trans_vector, filtered_trans_vector;

    estimator_.getCurrentPose(current_trans_vector, current_rot_matrix);
    estimator_.getFilteredPose(filtered_trans_vector, filtered_rot_matrix);

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
    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> image_points;

    estimator_.getCalibrationPoints(object_points, image_points, false);

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

    auto reprojection_error = [](auto & points1, auto & points2) {
      double error = 0.0;

      for (unsigned long i = 0; i < points1.size(); i++) {
        error += cv::norm(points1[i] - points2[i]);
      }

      return error / points1.size();
    };

    initial_reproj_error_ = reprojection_error(image_points, initial_projected_points);
    current_reproj_error_ = reprojection_error(image_points, current_projected_points);
    filtered_reproj_error_ = reprojection_error(image_points, filtered_projected_points);

    // Publish calibration points
    publishCalibrationPoints(object_points, image_points);
  }

  visualizer_->drawCalibrationStatus(estimator_, latest_timestamp_);

  // Consume detections
  apriltag_detections_array_.reset();
  lidartag_detections_array_.reset();
}

void ExtrinsicTagBasedCalibrator::publishCalibrationPoints(
  const std::vector<cv::Point3d> & object_points, const std::vector<cv::Point2d> & image_points)
{
  tier4_calibration_msgs::msg::CalibrationPoints msg;
  geometry_msgs::msg::Point object_point;
  geometry_msgs::msg::Point image_point;

  assert(object_points.size() == image_points.size());

  for (unsigned long i = 0; i < object_points.size(); ++i) {
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
