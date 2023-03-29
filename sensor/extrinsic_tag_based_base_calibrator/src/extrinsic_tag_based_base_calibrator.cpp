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

#include <Eigen/Core>
#include <extrinsic_tag_based_base_calibrator/calibration_scene_extractor.hpp>
#include <extrinsic_tag_based_base_calibrator/extrinsic_tag_based_base_calibrator.hpp>
#include <extrinsic_tag_based_base_calibrator/intrinsics_calibration/apriltag_calibrator.hpp>
#include <extrinsic_tag_based_base_calibrator/intrinsics_calibration/chessboard_calibrator.hpp>
#include <extrinsic_tag_based_base_calibrator/intrinsics_calibration/intrinsics_calibrator.hpp>
#include <extrinsic_tag_based_base_calibrator/math.hpp>
#include <extrinsic_tag_based_base_calibrator/serialization.hpp>
#include <extrinsic_tag_based_base_calibrator/visualization.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <ceres/ceres.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>

namespace extrinsic_tag_based_base_calibrator
{

ExtrinsicTagBasedBaseCalibrator::ExtrinsicTagBasedBaseCalibrator(
  const rclcpp::NodeOptions & options)
: Node("extrinsic_tag_based_base_calibrator_node", options),
  tf_broadcaster_(this),
  calibration_done_(false),
  data_(std::make_shared<CalibrationData>())
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

  std::vector<std::string> lidar_sensor_kit_frames =
    this->declare_parameter<std::vector<std::string>>("lidar_sensor_kit_frames");
  std::vector<std::string> calibration_lidar_parent_frames =
    this->declare_parameter<std::vector<std::string>>("calibration_lidar_parent_frames");

  std::vector<std::string> camera_sensor_kit_frames =
    this->declare_parameter<std::vector<std::string>>("camera_sensor_kit_frames");
  std::vector<std::string> calibration_camera_parent_frames =
    this->declare_parameter<std::vector<std::string>>("calibration_camera_parent_frames");

  std::vector<std::string> lidar_calibration_service_names =
    this->declare_parameter<std::vector<std::string>>("lidar_calibration_service_names");
  std::vector<std::string> camera_calibration_service_names =
    this->declare_parameter<std::vector<std::string>>("camera_calibration_service_names");

  main_calibration_sensor_frame_ =
    this->declare_parameter<std::string>("main_calibration_sensor_frame");

  calibration_lidar_frames_vector_ =
    this->declare_parameter<std::vector<std::string>>("calibration_lidar_frames");
  calibration_camera_frames_vector_ =
    this->declare_parameter<std::vector<std::string>>("calibration_camera_frames");

  auto remove_empty_strings = [](const std::vector<std::string> & v1) {
    std::vector<std::string> v2;
    std::copy_if(v1.begin(), v1.end(), std::back_inserter(v2), [](const std::string & s) {
      return s.size() > 0;
    });

    return v2;
  };

  lidar_sensor_kit_frames = remove_empty_strings(lidar_sensor_kit_frames);
  calibration_lidar_parent_frames = remove_empty_strings(calibration_lidar_parent_frames);
  camera_sensor_kit_frames = remove_empty_strings(camera_sensor_kit_frames);
  calibration_camera_parent_frames = remove_empty_strings(calibration_camera_parent_frames);
  calibration_lidar_frames_vector_ = remove_empty_strings(calibration_lidar_frames_vector_);
  calibration_camera_frames_vector_ = remove_empty_strings(calibration_camera_frames_vector_);

  calibration_sensor_frames_vector_.insert(
    calibration_sensor_frames_vector_.end(), calibration_lidar_frames_vector_.begin(),
    calibration_lidar_frames_vector_.end());
  calibration_sensor_frames_vector_.insert(
    calibration_sensor_frames_vector_.end(), calibration_camera_frames_vector_.begin(),
    calibration_camera_frames_vector_.end());

  std::vector<std::string> calibration_lidar_detections_topics =
    this->declare_parameter<std::vector<std::string>>("calibration_lidar_detections_topics");
  std::vector<std::string> calibration_image_detections_topics =
    this->declare_parameter<std::vector<std::string>>("calibration_image_detections_topics");
  std::vector<std::string> calibration_camera_info_topics =
    this->declare_parameter<std::vector<std::string>>("calibration_camera_info_topics");

  calibration_lidar_detections_topics = remove_empty_strings(calibration_lidar_detections_topics);
  calibration_image_detections_topics = remove_empty_strings(calibration_image_detections_topics);
  calibration_camera_info_topics = remove_empty_strings(calibration_camera_info_topics);

  assert(lidar_sensor_kit_frames.size() == calibration_lidar_parent_frames.size());
  assert(lidar_sensor_kit_frames.size() == calibration_lidar_frames_vector_.size());
  assert(lidar_sensor_kit_frames.size() == calibration_lidar_detections_topics.size());

  assert(camera_sensor_kit_frames.size() == calibration_camera_parent_frames.size());
  assert(camera_sensor_kit_frames.size() == calibration_camera_frames_vector_.size());
  assert(camera_sensor_kit_frames.size() == calibration_image_detections_topics.size());
  assert(camera_sensor_kit_frames.size() == calibration_camera_info_topics.size());

  for (std::size_t lidar_index = 0; lidar_index < calibration_lidar_frames_vector_.size();
       lidar_index++) {
    const std::string lidar_name = calibration_lidar_frames_vector_[lidar_index];

    sensor_kit_frame_map_[lidar_name] = lidar_sensor_kit_frames[lidar_index];
    calibration_sensor_parent_frame_map_[lidar_name] = calibration_lidar_parent_frames[lidar_index];
    calibration_lidar_detections_topic_map_[lidar_name] =
      calibration_lidar_detections_topics[lidar_index];
    calibration_service_names_map_[lidar_name] = lidar_calibration_service_names[lidar_index];
  }

  for (std::size_t camera_index = 0; camera_index < calibration_camera_frames_vector_.size();
       camera_index++) {
    const std::string camera_name = calibration_camera_frames_vector_[camera_index];

    sensor_kit_frame_map_[camera_name] = camera_sensor_kit_frames[camera_index];
    calibration_sensor_parent_frame_map_[camera_name] =
      calibration_camera_parent_frames[camera_index];

    calibration_image_detections_topic_map_[camera_name] =
      calibration_image_detections_topics[camera_index];
    calibration_camera_info_topic_map_[camera_name] = calibration_camera_info_topics[camera_index];
    calibration_service_names_map_[camera_name] = lidar_calibration_service_names[camera_index];
  }

  assert(
    std::find(
      calibration_sensor_frames_vector_.begin(), calibration_sensor_frames_vector_.end(),
      main_calibration_sensor_frame_) != calibration_sensor_frames_vector_.end());

  lidartag_to_apriltag_scale_ = this->declare_parameter<double>("lidartag_to_apriltag_scale", 0.75);

  waypoint_tag_parameters_.tag_type = TagType::WaypointTag;
  waypoint_tag_parameters_.family =
    this->declare_parameter<std::string>("waypoint_tag_family", "16h5");
  waypoint_tag_parameters_.rows = this->declare_parameter<int>("waypoint_tag_rows", 1);
  waypoint_tag_parameters_.cols = this->declare_parameter<int>("waypoint_tag_cols", 1);
  waypoint_tag_parameters_.size = this->declare_parameter<double>("waypoint_tag_size", 0.6);
  waypoint_tag_parameters_.spacing = this->declare_parameter<double>("waypoint_tag_spacing", 0.2);
  std::vector<int64_t> waypoint_tag_ids =
    this->declare_parameter<std::vector<int64_t>>("waypoint_tag_ids");
  std::for_each(waypoint_tag_ids.begin(), waypoint_tag_ids.end(), [&](const auto & id) {
    waypoint_tag_parameters_.ids.insert(static_cast<int>(id));
  });

  ground_tag_parameters_.tag_type = TagType::GroundTag;
  ground_tag_parameters_.family =
    this->declare_parameter<std::string>("ground_tag_family", "36h11");
  ground_tag_parameters_.rows = this->declare_parameter<int>("ground_tag_rows", 1);
  ground_tag_parameters_.cols = this->declare_parameter<int>("ground_tag_cols", 1);
  ground_tag_parameters_.size = this->declare_parameter<double>("ground_tag_size", 0.6);
  ground_tag_parameters_.spacing = this->declare_parameter<double>("ground_tag_spacing", 0.2);

  std::vector<int64_t> ground_tag_ids =
    this->declare_parameter<std::vector<int64_t>>("ground_tag_ids");
  std::for_each(ground_tag_ids.begin(), ground_tag_ids.end(), [&](const auto & id) {
    ground_tag_parameters_.ids.insert(static_cast<int>(id));
  });

  wheel_tag_parameters_.tag_type = TagType::WheelTag;
  wheel_tag_parameters_.family = this->declare_parameter<std::string>("wheel_tag_family", "16h5");
  wheel_tag_parameters_.rows = this->declare_parameter<int>("wheel_tag_rows", 2);
  wheel_tag_parameters_.cols = this->declare_parameter<int>("wheel_tag_cols", 2);
  wheel_tag_parameters_.size = this->declare_parameter<double>("wheel_tag_size", 0.6);
  wheel_tag_parameters_.spacing = this->declare_parameter<double>("wheel_tag_spacing", 0.2);

  left_wheel_tag_id = this->declare_parameter<int>("left_wheel_tag_id", 3);
  right_wheel_tag_id = this->declare_parameter<int>("right_wheel_tag_id", 4);
  wheel_tag_parameters_.ids.insert(left_wheel_tag_id);
  wheel_tag_parameters_.ids.insert(right_wheel_tag_id);

  tag_parameters_map_[TagType::WaypointTag] = waypoint_tag_parameters_;
  tag_parameters_map_[TagType::GroundTag] = ground_tag_parameters_;
  tag_parameters_map_[TagType::WheelTag] = wheel_tag_parameters_;
  tag_parameters_vector_ = {
    waypoint_tag_parameters_, ground_tag_parameters_, wheel_tag_parameters_};

  // Optimization options
  ba_optimize_intrinsics_ = this->declare_parameter<bool>("ba_optimize_intrinsics", false);
  ba_share_intrinsics_ = this->declare_parameter<bool>("ba_share_intrinsics", false);
  ba_force_shared_ground_plane_ =
    this->declare_parameter<bool>("ba_force_shared_ground_plane", false);
  virtual_lidar_f_ = this->declare_parameter<double>("virtual_lidar_f", 10000.0);

  // Initial intrinsic calibration parameters
  initial_intrinsic_calibration_board_type_ =
    this->declare_parameter<std::string>("initial_intrinsic_calibration_board_type", "apriltag");
  initial_intrinsic_calibration_tangent_distortion_ =
    this->declare_parameter<bool>("initial_intrinsic_calibration_tangent_distortion", true);
  initial_intrinsic_calibration_radial_distortion_coeffs_ =
    this->declare_parameter<int>("initial_intrinsic_calibration_radial_distortion_coeffs", 2);
  initial_intrinsic_calibration_debug_ =
    this->declare_parameter<bool>("initial_intrinsic_calibration_debug", true);

  initial_intrinsic_calibration_tag_parameters_.tag_type = TagType::IntrinsicCalibrationTag;
  initial_intrinsic_calibration_tag_parameters_.family =
    this->declare_parameter<std::string>("initial_intrinsic_calibration_tag_family", "16h5");
  initial_intrinsic_calibration_tag_parameters_.rows =
    this->declare_parameter<int>("initial_intrinsic_calibration_tag_rows", 1);
  initial_intrinsic_calibration_tag_parameters_.cols =
    this->declare_parameter<int>("initial_intrinsic_calibration_tag_cols", 1);
  initial_intrinsic_calibration_tag_parameters_.size =
    this->declare_parameter<double>("initial_intrinsic_calibration_tag_size", 0.8);
  initial_intrinsic_calibration_tag_parameters_.spacing =
    this->declare_parameter<double>("initial_intrinsic_calibration_tag_spacing", 0.2);

  std::vector<int64_t> intrinsic_calibration_tag_ids =
    this->declare_parameter<std::vector<int64_t>>(
      "intrinsic_calibration_tag_ids", std::vector<int64_t>{0});

  std::for_each(
    intrinsic_calibration_tag_ids.cbegin(), intrinsic_calibration_tag_ids.cend(),
    [&](auto & id) { initial_intrinsic_calibration_tag_parameters_.ids.insert(id); });

  initial_intrinsic_calibration_board_cols_ =
    this->declare_parameter<int>("initial_intrinsic_calibration_board_cols", 8);
  initial_intrinsic_calibration_board_rows_ =
    this->declare_parameter<int>("initial_intrinsic_calibration_board_rows", 6);

  apriltag_detector_parameters_.max_hamming =
    this->declare_parameter<int>("apriltag_max_hamming", 0);
  apriltag_detector_parameters_.min_margin =
    this->declare_parameter<double>("apriltag_min_margin", 20.0);
  apriltag_detector_parameters_.max_out_of_plane_angle =
    this->declare_parameter<double>("apriltag_max_out_of_plane_angle", 90.0);
  apriltag_detector_parameters_.max_reproj_error =
    this->declare_parameter<double>("apriltag_max_reproj_error", 10.0);
  apriltag_detector_parameters_.max_homography_error =
    this->declare_parameter<double>("apriltag_max_homography_error", 0.5);
  apriltag_detector_parameters_.quad_decimate =
    this->declare_parameter<double>("apriltag_quad_decimate", 1.0);
  apriltag_detector_parameters_.quad_sigma =
    this->declare_parameter<double>("apriltag_quad_sigma", 0.0);
  apriltag_detector_parameters_.nthreads = this->declare_parameter<int>("apriltag_nthreads", 1);
  apriltag_detector_parameters_.debug = this->declare_parameter<bool>("apriltag_debug", false);
  apriltag_detector_parameters_.refine_edges =
    this->declare_parameter<bool>("apriltag_refine_edges", true);

  for (const std::string & lidar_frame : calibration_lidar_frames_vector_) {
    lidartag_detections_sub_map_[lidar_frame] =
      this->create_subscription<lidartag_msgs::msg::LidarTagDetectionArray>(
        calibration_lidar_detections_topic_map_[lidar_frame],
        rclcpp::SystemDefaultsQoS().keep_all(),
        [&](const lidartag_msgs::msg::LidarTagDetectionArray::SharedPtr msg) {
          this->lidartagDetectionsCallback(msg, lidar_frame);
        });
  }

  for (const std::string & camera_frame : calibration_camera_frames_vector_) {
    apriltag_detections_sub_map_[camera_frame] =
      this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        calibration_image_detections_topic_map_[camera_frame], rclcpp::SystemDefaultsQoS(),
        [&](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
          this->apriltagDetectionsCallback(msg, camera_frame);
        });

    camera_info_sub_map_[camera_frame] = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      calibration_camera_info_topic_map_[camera_frame], rclcpp::SystemDefaultsQoS(),
      [&](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        this->cameraInfoCallback(msg, camera_frame);
      });
  }

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);

  visualization_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(1),
    std::bind(&ExtrinsicTagBasedBaseCalibrator::visualizationTimerCallback, this));

  // Calibration API services
  // The service servers runs in a dedicated threads since they are blocking
  for (const auto & calibration_sensor_frame : calibration_sensor_frames_vector_) {
    const std::string & sensor_kit_frame = sensor_kit_frame_map_[calibration_sensor_frame];
    const std::string & calibration_sensor_parent_frame =
      calibration_sensor_parent_frame_map_[calibration_sensor_frame];

    srv_callback_groups_map_[calibration_sensor_frame] =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    calibration_api_srv_map_[calibration_sensor_frame] =
      this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
        calibration_service_names_map_[calibration_sensor_frame] + "/extrinsic_calibration",
        [&](
          const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
          const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response>
            response) {
          calibrationRequestCallback(
            request, response, sensor_kit_frame, calibration_sensor_parent_frame,
            calibration_sensor_frame);
        },
        rmw_qos_profile_services_default, srv_callback_groups_map_[calibration_sensor_frame]);
  }

  // Scene related services
  add_external_camera_images_srv_ =
    this->create_service<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId>(
      "add_external_camera_images_to_scene",
      std::bind(
        &ExtrinsicTagBasedBaseCalibrator::addExternalCameraImagesCallback, this,
        std::placeholders::_1, std::placeholders::_2));

  add_calibration_sensor_detections_to_scene_srv_ =
    this->create_service<tier4_calibration_msgs::srv::Empty>(
      "add_calibration_sensor_detections_to_new_scene",
      std::bind(
        &ExtrinsicTagBasedBaseCalibrator::addCalibrationSensorDetectionsCallback, this,
        std::placeholders::_1, std::placeholders::_2));

  add_calibration_camera_images_srv_ =
    this->create_service<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId>(
      "add_calibration_camera_images_to_new_scene",
      std::bind(
        &ExtrinsicTagBasedBaseCalibrator::addCalibrationImagesCallback, this, std::placeholders::_1,
        std::placeholders::_2));

  // Intrinsics realated services
  load_external_camera_intrinsics_srv_ = this->create_service<tier4_calibration_msgs::srv::Files>(
    "load_external_camera_intrinsics",
    std::bind(
      &ExtrinsicTagBasedBaseCalibrator::loadExternalIntrinsicsCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  save_external_camera_intrinsics_srv_ = this->create_service<tier4_calibration_msgs::srv::Files>(
    "save_external_camera_intrinsics",
    std::bind(
      &ExtrinsicTagBasedBaseCalibrator::saveExternalIntrinsicsCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  calibrate_external_camera_intrinsics_srv_ =
    this->create_service<tier4_calibration_msgs::srv::Files>(
      "calibrate_external_camera_intrinsics",
      std::bind(
        &ExtrinsicTagBasedBaseCalibrator::calibrateExternalIntrinsicsCallback, this,
        std::placeholders::_1, std::placeholders::_2));

  load_calibration_camera_intrinsics_srv_ =
    this->create_service<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId>(
      "load_calibration_camera_intrinsics",
      std::bind(
        &ExtrinsicTagBasedBaseCalibrator::loadCalibrationIntrinsicsCallback, this,
        std::placeholders::_1, std::placeholders::_2));
  save_calibration_camera_intrinsics_srv_ =
    this->create_service<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId>(
      "save_calibration_camera_intrinsics",
      std::bind(
        &ExtrinsicTagBasedBaseCalibrator::saveCalibrationIntrinsicsCallback, this,
        std::placeholders::_1, std::placeholders::_2));
  calibrate_calibration_camera_intrinsics_srv_ =
    this->create_service<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId>(
      "calibrate_calibration_camera_intrinsics",
      std::bind(
        &ExtrinsicTagBasedBaseCalibrator::calibrateCalibrationIntrinsicsCallback, this,
        std::placeholders::_1, std::placeholders::_2));

  // Calibration related services
  process_scenes_srv_ = this->create_service<tier4_calibration_msgs::srv::Empty>(
    "process_scenes", std::bind(
                        &ExtrinsicTagBasedBaseCalibrator::preprocessScenesCallback, this,
                        std::placeholders::_1, std::placeholders::_2));
  calibration_srv_ = this->create_service<tier4_calibration_msgs::srv::Empty>(
    "calibrate", std::bind(
                   &ExtrinsicTagBasedBaseCalibrator::calibrationCallback, this,
                   std::placeholders::_1, std::placeholders::_2));

  // Calibration related services
  load_database_srv_ = this->create_service<tier4_calibration_msgs::srv::Files>(
    "load_database", std::bind(
                       &ExtrinsicTagBasedBaseCalibrator::loadDatabaseCallback, this,
                       std::placeholders::_1, std::placeholders::_2));
  save_database_srv_ = this->create_service<tier4_calibration_msgs::srv::Files>(
    "save_database", std::bind(
                       &ExtrinsicTagBasedBaseCalibrator::saveDatabaseCallback, this,
                       std::placeholders::_1, std::placeholders::_2));

  for (const auto & tag_parameters_it : tag_parameters_map_) {
    const TagType & tag_type = tag_parameters_it.first;
    const TagParameters & tag_parameters = tag_parameters_it.second;
    const std::string tag_family_name = tag_parameters.family;

    for (const auto & id : tag_parameters.ids) {
      for (int offset = 0; offset < tag_parameters.rows * tag_parameters.cols; offset++) {
        tag_family_and_id_to_type_map_[tag_family_name + std::to_string(id + offset)] = tag_type;
      }
    }
  }

  // Initialize the detections map with null detections
  for (const auto & lidar_frame : calibration_lidar_frames_vector_) {
    latest_lidartag_detections_map_[lidar_frame] = LidartagDetections();
  }

  for (const auto & camera_frame : calibration_camera_frames_vector_) {
    latest_apriltag_detections_map_[camera_frame] = GroupedApriltagGridDetections();
  }
}

void ExtrinsicTagBasedBaseCalibrator::calibrationRequestCallback(
  __attribute__((unused))
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request>
    request,
  __attribute__((unused))
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response>
    response,
  const std::string & sensor_kit_frame, const std::string & calibration_sensor_parent_frame,
  const std::string & calibration_sensor_frame)
{
  using std::chrono_literals::operator""s;

  // Loop until the calibration finishes
  while (rclcpp::ok()) {
    rclcpp::sleep_for(1s);
    std::unique_lock<std::mutex> lock(mutex_);

    if (calibration_done_) {
      break;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 30000, "Waiting for the calibration to end");
  }

  cv::Matx44d base_link_to_lidar_transform_cv =
    calibrated_main_sensor_to_base_link_pose_.inv().matrix;
  Eigen::Matrix4d base_link_to_lidar_transform;
  cv::cv2eigen(base_link_to_lidar_transform_cv, base_link_to_lidar_transform);
  Eigen::Affine3d base_link_to_lidar_pose(base_link_to_lidar_transform);

  geometry_msgs::msg::Transform sensor_kit_to_lidar_base_msg;
  Eigen::Affine3d sensor_kit_to_lidar_base_pose;

  geometry_msgs::msg::Transform lidar_base_to_lidar_msg;
  Eigen::Affine3d lidar_base_to_lidar_pose;

  geometry_msgs::msg::Transform initial_base_link_to_lidar_msg;
  Eigen::Affine3d initial_base_link_to_lidar_pose;

  // We calibrate the lidar base link, not the lidar, so we need to compute that pose
  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    sensor_kit_to_lidar_base_msg =
      tf_buffer_->lookupTransform(sensor_kit_frame, calibration_sensor_parent_frame, t, timeout)
        .transform;

    sensor_kit_to_lidar_base_pose = tf2::transformToEigen(sensor_kit_to_lidar_base_msg);

    lidar_base_to_lidar_msg =
      tf_buffer_
        ->lookupTransform(calibration_sensor_parent_frame, calibration_sensor_frame, t, timeout)
        .transform;

    lidar_base_to_lidar_pose = tf2::transformToEigen(lidar_base_to_lidar_msg);

    initial_base_link_to_lidar_msg =
      tf_buffer_->lookupTransform(base_frame_, calibration_sensor_frame, t, timeout).transform;

    initial_base_link_to_lidar_pose = tf2::transformToEigen(initial_base_link_to_lidar_msg);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not get the necessary tfs for calibration");
    response->success = false;
  }

  Eigen::Affine3d base_link_to_sensor_kit_pose = base_link_to_lidar_pose *
                                                 lidar_base_to_lidar_pose.inverse() *
                                                 sensor_kit_to_lidar_base_pose.inverse();

  Eigen::Affine3d initial_base_link_to_sensor_kit_pose = initial_base_link_to_lidar_pose *
                                                         lidar_base_to_lidar_pose.inverse() *
                                                         sensor_kit_to_lidar_base_pose.inverse();

  geometry_msgs::msg::Pose base_link_to_sensor_kit_msg = tf2::toMsg(base_link_to_sensor_kit_pose);
  geometry_msgs::msg::Pose initial_base_link_to_sensor_kit_msg =
    tf2::toMsg(initial_base_link_to_sensor_kit_pose);
  (void)base_link_to_sensor_kit_msg;
  (void)initial_base_link_to_sensor_kit_msg;

  response->success = true;
  response->result_pose = base_link_to_sensor_kit_msg;

  // Display the initial and calibrated values
  const auto & base_to_sensor_kit_rpy =
    tier4_autoware_utils::getRPY(base_link_to_sensor_kit_msg.orientation);
  const auto & initial_base_to_sensor_kit_rpy =
    tier4_autoware_utils::getRPY(initial_base_link_to_sensor_kit_msg.orientation);
  RCLCPP_INFO(this->get_logger(), "base_link: initial and calibrated statistics statistics");
  RCLCPP_INFO(
    this->get_logger(), "\tinitial: x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
    initial_base_link_to_sensor_kit_msg.position.x, initial_base_link_to_sensor_kit_msg.position.y,
    initial_base_link_to_sensor_kit_msg.position.z, initial_base_to_sensor_kit_rpy.x,
    initial_base_to_sensor_kit_rpy.y, initial_base_to_sensor_kit_rpy.z);
  RCLCPP_INFO(
    this->get_logger(), "\tcalibrated: x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
    base_link_to_sensor_kit_msg.position.x, base_link_to_sensor_kit_msg.position.y,
    base_link_to_sensor_kit_msg.position.z, base_to_sensor_kit_rpy.x, base_to_sensor_kit_rpy.y,
    base_to_sensor_kit_rpy.z);

  // Display the correction in calibration
  Eigen::Affine3d initial_base_link_to_calibrated_base_link_pose =
    initial_base_link_to_lidar_pose * base_link_to_lidar_pose.inverse();
  Eigen::Matrix3d initial_base_link_to_calibrated_base_link_rot =
    initial_base_link_to_calibrated_base_link_pose.rotation();
  Eigen::Vector3d initial_base_link_to_calibrated_base_link_translation =
    initial_base_link_to_calibrated_base_link_pose.translation();

  Eigen::Vector3d initial_normal(0.0, 0.0, 1.0);
  Eigen::Vector3d optimized_norm =
    initial_base_link_to_calibrated_base_link_rot * Eigen::Vector3d(0.0, 0.0, 1.0);
  const double normal_angle_diff = std::acos(initial_normal.dot(optimized_norm));
  const double yaw_angle_diff = std::atan2(
    initial_base_link_to_calibrated_base_link_rot(1, 0),
    initial_base_link_to_calibrated_base_link_rot(0, 0));

  RCLCPP_INFO(this->get_logger(), "base_link: initial to calibrated statistics");
  RCLCPP_INFO(
    this->get_logger(), "\t normal angle difference: %.3f degrees",
    180.0 * normal_angle_diff / M_PI);
  RCLCPP_INFO(
    this->get_logger(), "\t yaw angle difference: %.3f degrees", 180.0 * yaw_angle_diff / M_PI);
  RCLCPP_INFO(
    this->get_logger(), "\t x: %.3f m", initial_base_link_to_calibrated_base_link_translation.x());
  RCLCPP_INFO(
    this->get_logger(), "\t y: %.3f m", initial_base_link_to_calibrated_base_link_translation.y());
  RCLCPP_INFO(
    this->get_logger(), "\t z: %.3f m", initial_base_link_to_calibrated_base_link_translation.z());
}

void ExtrinsicTagBasedBaseCalibrator::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg, const std::string & camera_frame)
{
  if (msg->header.frame_id != camera_frame) {
    RCLCPP_ERROR(
      get_logger(), "The camera info frame should be %s, but is %s!", camera_frame.c_str(),
      msg->header.frame_id.c_str());
    return;
  }

  IntrinsicParameters & intrinsics = calibration_camera_intrinsics_map_[camera_frame];

  if (intrinsics.isValid()) {
    return;
  }

  // We assume the detections come from the undistorted image
  intrinsics.size.height = msg->height;
  intrinsics.size.width = msg->width;
  intrinsics.dist_coeffs.resize(5);
  std::fill(intrinsics.dist_coeffs.begin(), intrinsics.dist_coeffs.end(), 0.0);

  const auto & p = msg->p;

  intrinsics.camera_matrix =
    cv::Mat_<double>({3, 3}, {p[0], p[1], p[2], p[4], p[5], p[6], p[8], p[9], p[10]});
  intrinsics.undistorted_camera_matrix = intrinsics.camera_matrix;
}

void ExtrinsicTagBasedBaseCalibrator::apriltagDetectionsCallback(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr detections_msg,
  const std::string & camera_frame)
{
  if (detections_msg->header.frame_id != camera_frame) {
    RCLCPP_ERROR(
      get_logger(), "The apriltag detections' frame should be %s, but is %s!", camera_frame.c_str(),
      detections_msg->header.frame_id.c_str());
    return;
  }

  const IntrinsicParameters intrinsics = calibration_camera_intrinsics_map_[camera_frame];

  if (!intrinsics.isValid()) {
    RCLCPP_WARN(
      get_logger(), "Received detections before camera intrinsics for camera %s!",
      camera_frame.c_str());
    return;
  }

  GroupedApriltagDetections grouped_apriltag_detections;

  for (const auto & detection_msg : detections_msg->detections) {
    const std::string family_and_id = detection_msg.family + std::to_string(detection_msg.id);
    assert(tag_family_and_id_to_type_map_.count(family_and_id) == 1);

    const TagType tag_type = tag_family_and_id_to_type_map_[family_and_id];
    const TagParameters tag_parameters = tag_parameters_map_[tag_type];
    double tag_size = tag_parameters.size;

    ApriltagDetection detection =
      ApriltagDetection::fromApriltagDetectionMsg(detection_msg, intrinsics, tag_size);
    grouped_apriltag_detections[tag_type].push_back(detection);
  }

  latest_apriltag_detections_map_[camera_frame] =
    ApriltagGridDetection::fromGroupedApriltagDetections(
      grouped_apriltag_detections, tag_parameters_map_);
}

void ExtrinsicTagBasedBaseCalibrator::lidartagDetectionsCallback(
  const lidartag_msgs::msg::LidarTagDetectionArray::SharedPtr detections_msg,
  const std::string & lidar_frame)
{
  if (detections_msg->header.frame_id != lidar_frame) {
    RCLCPP_ERROR(
      get_logger(), "Lidartag detections' frame should be %s, but is %s!",
      detections_msg->header.frame_id.c_str(), lidar_frame.c_str());
    return;
  }

  LidartagDetections detections;

  for (const auto & detection_msg : detections_msg->detections) {
    detections.push_back(
      LidartagDetection::fromLidartagDetectionMsg(detection_msg, lidartag_to_apriltag_scale_));
  }

  latest_lidartag_detections_map_[lidar_frame] = detections;
}

void ExtrinsicTagBasedBaseCalibrator::visualizationTimerCallback()
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker base_marker;
  base_marker.ns = "raw_detections";
  next_color_index_ = 0;

  // Raw detection markers (in their own frame)

  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    const CalibrationScene & scene = data_->scenes[scene_index];

    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 1.0;
    color.a = 1.0;

    for (const auto & scene_camera_detections_it : scene.calibration_cameras_detections) {
      const std::string & camera_frame = scene_camera_detections_it.calibration_frame;
      const GroupedApriltagGridDetections & grouped_grid_detections =
        scene_camera_detections_it.grouped_detections;

      base_marker.header.frame_id = camera_frame;

      for (const auto & group_grid_detections : grouped_grid_detections) {
        const auto & tag_type = group_grid_detections.first;
        const auto & tag_parameters = tag_parameters_map_.at(tag_type);
        const std::vector<ApriltagGridDetection> & grid_detections = group_grid_detections.second;
        for (const auto & grid_detection : grid_detections) {
          for (const auto & detection : grid_detection.sub_detections) {
            addTagMarkers(
              markers, std::to_string(detection.id), tag_parameters, color, detection.pose,
              base_marker);
          }
        }
      }
    }

    for (const auto & scene_lidar_detections_it : scene.calibration_lidars_detections) {
      const std::string & lidar_frame = scene_lidar_detections_it.calibration_frame;
      const LidartagDetections & detections = scene_lidar_detections_it.detections;
      base_marker.header.frame_id = lidar_frame;
      const auto & tag_parameters = tag_parameters_map_.at(TagType::WaypointTag);

      for (const auto & detection : detections) {
        addTagMarkers(
          markers, std::to_string(detection.id), tag_parameters, color, detection.pose,
          base_marker);
      }
    }
  }

  // Initial estimations markers (in the main sensor frame)
  base_marker.ns = "initial_estimations";
  base_marker.header.frame_id = main_calibration_sensor_frame_;

  visualization_msgs::msg::Marker initial_connections_base_marker = base_marker;
  initial_connections_base_marker.ns = "initial_connections";

  std_msgs::msg::ColorRGBA initial_estimations_color;
  initial_estimations_color.r = 1.f;
  initial_estimations_color.g = 0.f;
  initial_estimations_color.b = 0.f;
  initial_estimations_color.a = 1.f;

  std_msgs::msg::ColorRGBA initial_connections_color;
  initial_connections_color.r = 1.f;
  initial_connections_color.g = 0.f;
  initial_connections_color.b = 0.f;
  initial_connections_color.a = 0.5f;

  for (auto it = data_->initial_tag_poses_map.begin(); it != data_->initial_tag_poses_map.end();
       it++) {
    const UID & tag_uid = it->first;
    auto & pose = it->second;
    const TagType & tag_type = tag_uid.tag_type;
    const auto & tag_parameters = tag_parameters_map_.at(tag_type);

    addTagMarkers(
      markers, tag_uid.toString(), tag_parameters, initial_estimations_color, *pose, base_marker);
  }

  for (auto initial_sensor_poses_it : data_->initial_sensor_poses_map) {
    const UID & sensor_uid = initial_sensor_poses_it.first;
    auto & sensor_pose = *initial_sensor_poses_it.second;

    addTextMarker(
      markers, sensor_uid.toString(), initial_estimations_color, sensor_pose, base_marker);
    addAxesMarkers(markers, 0.5, sensor_pose, base_marker);

    // Iterate over all the detections of said camera
    for (const UID & tag_uid : data_->uid_connections_map[sensor_uid]) {
      addLineMarker(
        markers, initial_connections_color, sensor_pose, *data_->initial_tag_poses_map[tag_uid],
        initial_connections_base_marker);
    }
  }

  // Optimized estimations
  base_marker.ns = "optimized_estimations";
  visualization_msgs::msg::Marker optimized_connections_base_marker = base_marker;
  optimized_connections_base_marker.ns = "optimized_connections";
  optimized_connections_base_marker.header.frame_id = main_calibration_sensor_frame_;

  std_msgs::msg::ColorRGBA optimized_estimations_color;
  optimized_estimations_color.r = 0.f;
  optimized_estimations_color.g = 1.f;
  optimized_estimations_color.b = 0.f;
  optimized_estimations_color.a = 1.f;

  std_msgs::msg::ColorRGBA optimized_connections_color;
  optimized_connections_color.r = 0.f;
  optimized_connections_color.g = 1.f;
  optimized_connections_color.b = 0.f;
  optimized_connections_color.a = 0.5f;

  for (auto it = data_->optimized_tag_poses_map.begin(); it != data_->optimized_tag_poses_map.end();
       it++) {
    const UID & tag_uid = it->first;
    auto & pose = it->second;
    const TagType & tag_type = tag_uid.tag_type;

    addTagMarkers(
      markers, tag_uid.toString(), tag_parameters_map_.at(tag_type), optimized_estimations_color,
      *pose, base_marker);
  }

  for (auto optimized_sensor_poses_it : data_->optimized_sensor_poses_map) {
    const UID & sensor_uid = optimized_sensor_poses_it.first;
    auto & sensor_pose = *optimized_sensor_poses_it.second;

    addTextMarker(
      markers, sensor_uid.toString(), optimized_estimations_color, sensor_pose, base_marker);
    addAxesMarkers(markers, 0.5, sensor_pose, base_marker);

    // Iterate over all the detections of said camera
    for (const UID & tag_uid : data_->uid_connections_map[sensor_uid]) {
      addLineMarker(
        markers, optimized_connections_color, sensor_pose, *data_->optimized_tag_poses_map[tag_uid],
        optimized_connections_base_marker);
    }
  }

  visualization_msgs::msg::Marker initial_ground_base_marker = base_marker;
  initial_ground_base_marker.ns = "initial_ground_plane";
  visualization_msgs::msg::Marker optimized_ground_base_marker = base_marker;
  optimized_ground_base_marker.ns = "optimized_ground_plane";
  visualization_msgs::msg::Marker initial_base_link_base_marker = base_marker;
  initial_base_link_base_marker.ns = "initial_base_link";
  visualization_msgs::msg::Marker optimized_base_link_base_marker = base_marker;
  optimized_base_link_base_marker.ns = "optimized_base_link";

  cv::Affine3d initial_ground_pose, optimized_ground_pose;
  if (computeGroundPlane(
        data_->initial_ground_tag_poses_map, ground_tag_parameters_.size, initial_ground_pose)) {
    addGrid(markers, initial_ground_pose, 100, 0.2, initial_ground_base_marker);
    addAxesMarkers(markers, 0.5, initial_ground_pose, initial_ground_base_marker);

    if (data_->initial_left_wheel_tag_pose && data_->initial_right_wheel_tag_pose) {
      cv::Affine3d initial_base_link_pose = computeBaseLink(
        *data_->initial_left_wheel_tag_pose, *data_->initial_right_wheel_tag_pose,
        initial_ground_pose);
      addAxesMarkers(markers, 0.5, initial_base_link_pose, initial_base_link_base_marker);
    }
  }

  if (computeGroundPlane(
        data_->optimized_ground_tag_poses_map, ground_tag_parameters_.size,
        optimized_ground_pose)) {
    addGrid(markers, optimized_ground_pose, 100, 0.2, optimized_ground_base_marker);
    addAxesMarkers(markers, 1.0, optimized_ground_pose, optimized_ground_base_marker);

    if (data_->optimized_left_wheel_tag_pose && data_->optimized_right_wheel_tag_pose) {
      cv::Affine3d optimized_base_link_pose = computeBaseLink(
        *data_->optimized_left_wheel_tag_pose, *data_->optimized_right_wheel_tag_pose,
        optimized_ground_pose);
      addAxesMarkers(markers, 1.0, optimized_base_link_pose, optimized_base_link_base_marker);
    }
  }

  // Publish tf for visualization if available
  if (!calibration_done_) {
    for (std::size_t marker_index = 0; marker_index < markers.markers.size(); marker_index++) {
      markers.markers[marker_index].id = marker_index;
    }

    markers_pub_->publish(markers);
    return;
  }

  Eigen::Matrix4d main_sensor_to_base_link_transform;
  cv::cv2eigen(
    calibrated_main_sensor_to_base_link_pose_.matrix, main_sensor_to_base_link_transform);

  geometry_msgs::msg::TransformStamped tf_msg =
    tf2::eigenToTransform(Eigen::Affine3d(main_sensor_to_base_link_transform));
  tf_msg.header.frame_id = main_calibration_sensor_frame_;
  tf_msg.child_frame_id = "estimated_base_link";
  tf_broadcaster_.sendTransform(tf_msg);

  // Publish the tf to all external cameras
  for (const auto & [uid, sensor_pose_cv] : data_->optimized_sensor_poses_map) {
    Eigen::Matrix4d sensor_pose_eigen;
    cv::cv2eigen(sensor_pose_cv->matrix, sensor_pose_eigen);

    geometry_msgs::msg::TransformStamped tf_msg =
      tf2::eigenToTransform(Eigen::Affine3d(sensor_pose_eigen));
    tf_msg.header.frame_id = main_calibration_sensor_frame_;
    tf_msg.child_frame_id = uid.toString();
    tf_broadcaster_.sendTransform(tf_msg);
  }

  visualization_msgs::msg::Marker detections_base_marker = base_marker;

  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    const CalibrationScene & scene = data_->scenes[scene_index];

    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 1.0;
    color.a = 1.0;

    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      const auto & grouped_grid_detections = scene.external_camera_frames[frame_id].detections;
      UID external_camera_uid =
        UID::makeSensorUID(SensorType::ExternalCamera, scene_index, frame_id);
      detections_base_marker.ns = "raw_detections_" + external_camera_uid.toString();

      detections_base_marker.header.frame_id = external_camera_uid.toString();

      for (const auto & group_grid_detections : grouped_grid_detections) {
        const auto & tag_type = group_grid_detections.first;
        const auto & tag_parameters = tag_parameters_map_.at(tag_type);
        const std::vector<ApriltagGridDetection> & grid_detections = group_grid_detections.second;
        for (const auto & grid_detection : grid_detections) {
          for (const auto & detection : grid_detection.sub_detections) {
            addTagMarkers(
              markers, std::to_string(detection.id), tag_parameters, color, detection.pose,
              detections_base_marker);
          }
        }
      }
    }
  }

  for (std::size_t marker_index = 0; marker_index < markers.markers.size(); marker_index++) {
    markers.markers[marker_index].id = marker_index;
  }

  markers_pub_->publish(markers);
}

std_msgs::msg::ColorRGBA ExtrinsicTagBasedBaseCalibrator::getNextColor()
{
  static std::vector<unsigned int> colors = {0x1f77b4, 0xff7f0e, 0x2ca02c, 0xd62728, 0x9467bd,
                                             0x8c564b, 0xe377c2, 0x7f7f7f, 0xbcbd22, 0x17becf};

  unsigned int hex = colors[next_color_index_];

  std_msgs::msg::ColorRGBA color;
  color.r = ((hex >> 16) & 255) / 255.0;
  color.g = ((hex >> 8) & 255) / 255.0;
  color.b = (hex & 255) / 255.0;
  color.a = 1.f;

  next_color_index_ = (next_color_index_ + 1) % colors.size();

  return color;
}

bool ExtrinsicTagBasedBaseCalibrator::addExternalCameraImagesCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Response> response)
{
  int camera_scenes = std::transform_reduce(
    scenes_calibration_apriltag_detections_.begin(), scenes_calibration_apriltag_detections_.end(),
    0, [](const auto & lhs, const auto & rhs) { return std::max<std::size_t>(lhs, rhs); },
    [](auto it) { return it.second.size(); });

  int lidar_scenes = std::transform_reduce(
    scenes_calibration_lidartag_detections_.begin(), scenes_calibration_lidartag_detections_.end(),
    0, [](const auto & lhs, const auto & rhs) { return std::max<std::size_t>(lhs, rhs); },
    [](auto it) { return it.second.size(); });

  std::size_t num_scenes = std::max(camera_scenes, lidar_scenes);

  RCLCPP_INFO(
    this->get_logger(), "Attempting to add external camera images to scene id=%ld",
    request->scene_id);

  if (request->files.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "We expected at least one image!");
    response->success = false;
    return true;
  }

  if (static_cast<std::size_t>(request->scene_id) >= num_scenes) {
    RCLCPP_ERROR(this->get_logger(), "Attempting to add external images to an inexistent scene");
    response->success = false;
    return true;
  }

  scenes_external_camera_images_.resize(num_scenes);
  scenes_external_camera_images_[request->scene_id] = request->files;

  RCLCPP_INFO(
    this->get_logger(), "Addded %lu external images to scene id=%ld (scenes=%lu)",
    request->files.size(), request->scene_id, num_scenes);

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::addCalibrationSensorDetectionsCallback(
  __attribute__((unused)) const std::shared_ptr<tier4_calibration_msgs::srv::Empty::Request>
    request,
  __attribute__((unused)) std::shared_ptr<tier4_calibration_msgs::srv::Empty::Response> response)
{
  response->success = false;

  for (auto & latest_apriltag_detections_it : latest_apriltag_detections_map_) {
    GroupedApriltagGridDetections & latest_detections = latest_apriltag_detections_it.second;

    if (latest_detections.size() > 0) {
      response->success = true;
      break;
    }
  }

  for (auto & latest_lidartag_detections_it : latest_lidartag_detections_map_) {
    LidartagDetections & latest_detections = latest_lidartag_detections_it.second;

    if (latest_detections.size() > 0) {
      response->success = true;
      break;
    }
  }

  if (!response->success) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not create a new scene since there were no detections in any sensor");
    return true;
  }

  for (auto & latest_apriltag_detections_it : latest_apriltag_detections_map_) {
    const std::string camera_frame = latest_apriltag_detections_it.first;
    GroupedApriltagGridDetections & latest_detections = latest_apriltag_detections_it.second;
    scenes_calibration_apriltag_detections_[camera_frame].push_back(latest_detections);

    if (latest_detections.size() > 0) {
      RCLCPP_INFO(
        this->get_logger(), "Added %lu detections to camera=%s (%lu scenes)",
        latest_detections.size(), camera_frame.c_str(),
        scenes_calibration_apriltag_detections_[camera_frame].size());
      latest_detections.clear();
    } else {
      RCLCPP_WARN(this->get_logger(), "Camera frame %s had no detections", camera_frame.c_str());
    }
  }

  for (auto & latest_lidartag_detections_it : latest_lidartag_detections_map_) {
    const std::string lidar_frame = latest_lidartag_detections_it.first;
    LidartagDetections & latest_detections = latest_lidartag_detections_it.second;
    scenes_calibration_lidartag_detections_[lidar_frame].push_back(latest_detections);

    if (latest_detections.size() > 0) {
      RCLCPP_INFO(
        this->get_logger(), "Added %lu detections to lidar=%s (%lu scenes)",
        latest_detections.size(), lidar_frame.c_str(),
        scenes_calibration_lidartag_detections_[lidar_frame].size());
      latest_detections.clear();
    } else {
      RCLCPP_WARN(this->get_logger(), "Lidar frame %s had no detections", lidar_frame.c_str());
    }
  }

  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::addCalibrationImagesCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Response> response)
{
  const std::string & frame_id = request->frame_id;
  const int & scene_id = request->scene_id;

  if (
    std::find(
      calibration_camera_frames_vector_.begin(), calibration_camera_frames_vector_.end(),
      frame_id) == calibration_camera_frames_vector_.end()) {
    RCLCPP_ERROR(
      this->get_logger(), "Images belong to a frame that we do not attempt to calibrate");
    response->success = false;
    return true;
  }

  int max_scene_id = scene_id;
  for (const auto & camera_frame : calibration_camera_frames_vector_) {
    max_scene_id = std::max(
      max_scene_id, static_cast<int>(scenes_calibration_camera_images_[camera_frame].size()));
  }

  for (const auto & camera_frame : calibration_camera_frames_vector_) {
    scenes_calibration_camera_images_[camera_frame].resize(max_scene_id);
  }

  scenes_calibration_camera_images_[frame_id][scene_id] = request->files[0];

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::loadExternalIntrinsicsCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Loading external camera intrinsics");

  if (!external_camera_intrinsics_.loadCalibration(request->files[0])) {
    RCLCPP_ERROR(this->get_logger(), "Could not load intrinsics");
    response->success = false;
    return true;
  }

  calibration_problem_.setExternalCameraIntrinsics(external_camera_intrinsics_);

  RCLCPP_INFO_STREAM(this->get_logger(), "k = " << external_camera_intrinsics_.camera_matrix);
  RCLCPP_INFO_STREAM(this->get_logger(), "d = " << external_camera_intrinsics_.dist_coeffs);
  RCLCPP_INFO_STREAM(
    this->get_logger(), "new_k = " << external_camera_intrinsics_.undistorted_camera_matrix);

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::saveExternalIntrinsicsCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Saving external camera intrinsics");

  external_camera_intrinsics_.saveCalibration(request->files[0]);

  RCLCPP_INFO(this->get_logger(), "External camera intrinsics saved");

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::calibrateExternalIntrinsicsCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Calibrating external cameras intrinsics");

  IntrinsicsCalibrator::Ptr external_camera_intrinsics_calibrator;
  if (initial_intrinsic_calibration_board_type_ == "apriltag") {
    external_camera_intrinsics_calibrator = IntrinsicsCalibrator::Ptr(new ApriltagBasedCalibrator(
      apriltag_detector_parameters_, initial_intrinsic_calibration_tag_parameters_,
      initial_intrinsic_calibration_tangent_distortion_,
      initial_intrinsic_calibration_radial_distortion_coeffs_, true));
  } else if (initial_intrinsic_calibration_board_type_ == "chessboard") {
    external_camera_intrinsics_calibrator = IntrinsicsCalibrator::Ptr(new ChessboardBasedCalibrator(
      initial_intrinsic_calibration_board_rows_, initial_intrinsic_calibration_board_cols_,
      initial_intrinsic_calibration_tangent_distortion_,
      initial_intrinsic_calibration_radial_distortion_coeffs_, true));
  }

  external_camera_intrinsics_calibrator->setCalibrationImageFiles(request->files);
  external_camera_intrinsics_calibrator->calibrate(external_camera_intrinsics_);
  calibration_problem_.setExternalCameraIntrinsics(external_camera_intrinsics_);

  RCLCPP_INFO_STREAM(this->get_logger(), "k = " << external_camera_intrinsics_.camera_matrix);
  RCLCPP_INFO_STREAM(this->get_logger(), "d = " << external_camera_intrinsics_.dist_coeffs);
  RCLCPP_INFO_STREAM(
    this->get_logger(), "new_k = " << external_camera_intrinsics_.undistorted_camera_matrix);

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::loadCalibrationIntrinsicsCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Loading 'calibration sensor' intrinsics");

  if (!calibration_camera_intrinsics_map_[request->frame_id].loadCalibration(request->files[0])) {
    RCLCPP_ERROR(this->get_logger(), "Could not load intrinsics");
    response->success = false;
    return true;
  }

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "k = " << calibration_camera_intrinsics_map_[request->frame_id].camera_matrix);
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "d = " << calibration_camera_intrinsics_map_[request->frame_id].dist_coeffs);
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "new_k = " << calibration_camera_intrinsics_map_[request->frame_id].undistorted_camera_matrix);

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::saveCalibrationIntrinsicsCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Saving 'calibration sensor' intrinsics");

  calibration_camera_intrinsics_map_[request->frame_id].saveCalibration(request->files[0]);

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::calibrateCalibrationIntrinsicsCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Calibrating 'calibration sensor' intrinsics");

  IntrinsicsCalibrator::Ptr calibration_sensor_intrinsics_calibrator;

  if (initial_intrinsic_calibration_board_type_ == "apriltag") {
    calibration_sensor_intrinsics_calibrator =
      IntrinsicsCalibrator::Ptr(new ApriltagBasedCalibrator(
        apriltag_detector_parameters_, initial_intrinsic_calibration_tag_parameters_,
        initial_intrinsic_calibration_tangent_distortion_,
        initial_intrinsic_calibration_radial_distortion_coeffs_, true));
  } else if (initial_intrinsic_calibration_board_type_ == "chessboard") {
    calibration_sensor_intrinsics_calibrator =
      IntrinsicsCalibrator::Ptr(new ChessboardBasedCalibrator(
        initial_intrinsic_calibration_board_rows_, initial_intrinsic_calibration_board_cols_,
        initial_intrinsic_calibration_tangent_distortion_,
        initial_intrinsic_calibration_radial_distortion_coeffs_, true));
  }

  calibration_sensor_intrinsics_calibrator->setCalibrationImageFiles(request->files);
  calibration_sensor_intrinsics_calibrator->calibrate(
    calibration_camera_intrinsics_map_[request->frame_id]);

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "k = " << calibration_camera_intrinsics_map_[request->frame_id].camera_matrix);
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "d = " << calibration_camera_intrinsics_map_[request->frame_id].dist_coeffs);
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "new_k = " << calibration_camera_intrinsics_map_[request->frame_id].undistorted_camera_matrix);

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::preprocessScenesCallback(
  __attribute__((unused)) const std::shared_ptr<tier4_calibration_msgs::srv::Empty::Request>
    request,
  __attribute__((unused)) std::shared_ptr<tier4_calibration_msgs::srv::Empty::Response> response)
{
  std::size_t num_external_camera_scenes = scenes_external_camera_images_.size();

  auto check_scenes_fn = [](const auto & data, auto expected_size) -> bool {
    for (const auto & it : data) {
      if (it.second.size() != 0 && it.second.size() != expected_size) {
        return false;
      }
    }

    return true;
  };

  if (!check_scenes_fn(scenes_calibration_apriltag_detections_, num_external_camera_scenes)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "The number of apriltag detections scenes differs from the external camera scenes");
    response->success = false;
    return true;
  } else if (!check_scenes_fn(
               scenes_calibration_lidartag_detections_, num_external_camera_scenes)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "The number of lidartag detections scenes differs from the external camera scenes");
    response->success = false;
    return true;
  } else if (!check_scenes_fn(scenes_calibration_camera_images_, num_external_camera_scenes)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "The number of lidartag detections scenes differs from the external camera scenes");
    response->success = false;
    return true;
  }

  for (const auto & external_camera_images : scenes_external_camera_images_) {
    if (external_camera_images.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "External camera scenes can not be empty");
      response->success = false;
      return true;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Processing %lu scenes...", num_external_camera_scenes);

  CalibrationSceneExtractor calibration_scene_extractor(
    apriltag_detector_parameters_, tag_parameters_vector_);
  calibration_scene_extractor.setExternalCameraIntrinsics(external_camera_intrinsics_);

  for (std::size_t scene_index = 0; scene_index < num_external_camera_scenes; scene_index++) {
    std::unordered_map<std::string, GroupedApriltagGridDetections>
      scene_calibration_apriltag_detections;
    std::unordered_map<std::string, LidartagDetections> scene_calibration_lidartag_detections;

    RCLCPP_INFO(this->get_logger(), "Scene %lu:", scene_index);

    for (const auto & it : scenes_calibration_apriltag_detections_) {
      scene_calibration_apriltag_detections[it.first] = it.second[scene_index];

      RCLCPP_INFO(
        this->get_logger(), "\t%s %lu apriltag detections", it.first.c_str(),
        it.second[scene_index].size());
    }

    for (const auto & it : scenes_calibration_lidartag_detections_) {
      scene_calibration_lidartag_detections[it.first] = it.second[scene_index];

      RCLCPP_INFO(
        this->get_logger(), "\t%s %lu lidartag detections", it.first.c_str(),
        it.second[scene_index].size());
    }

    RCLCPP_INFO(
      this->get_logger(), "\texternal camera: %lu apriltag detections",
      scenes_external_camera_images_[scene_index].size());

    CalibrationScene scene = calibration_scene_extractor.processScene(
      scene_calibration_lidartag_detections, scene_calibration_apriltag_detections,
      calibration_lidar_frames_vector_, calibration_camera_frames_vector_,
      main_calibration_sensor_frame_, scenes_external_camera_images_[scene_index]);

    data_->scenes.push_back(scene);
  }

  // Compute connections in the graph
  for (int scene_index = 0; scene_index < static_cast<int>(data_->scenes.size()); scene_index++) {
    const CalibrationScene & scene = data_->scenes[scene_index];

    for (const auto & camera_it : scene.calibration_cameras_detections) {
      UID camera_uid =
        UID::makeSensorUID(SensorType::CalibrationCamera, camera_it.calibration_camera_id);

      if (camera_it.calibration_frame == main_calibration_sensor_frame_) {
        assert(
          !data_->main_calibration_sensor_uid.isValid() ||
          data_->main_calibration_sensor_uid == camera_uid);
        data_->main_calibration_sensor_uid = camera_uid;
      }

      for (const auto & group_detections_it : camera_it.grouped_detections) {
        const TagType tag_type = group_detections_it.first;
        const ApriltagGridDetections & apriltag_grid_detections = group_detections_it.second;

        for (const auto & grid_detection : apriltag_grid_detections) {
          UID tag_uid = UID::makeTagUID(tag_type, scene_index, grid_detection.id);

          data_->uid_connections_map[camera_uid].push_back(tag_uid);
          data_->uid_connections_map[tag_uid].push_back(camera_uid);

          data_->detections_relative_poses_map[std::make_pair(camera_uid, tag_uid)] =
            grid_detection.pose;
          data_->detections_relative_poses_map[std::make_pair(tag_uid, camera_uid)] =
            grid_detection.pose.inv();
        }
      }
    }

    for (const auto & lidar_it : scene.calibration_lidars_detections) {
      UID lidar_uid =
        UID::makeSensorUID(SensorType::CalibrationLidar, lidar_it.calibration_lidar_id);

      if (lidar_it.calibration_frame == main_calibration_sensor_frame_) {
        assert(
          !data_->main_calibration_sensor_uid.isValid() ||
          data_->main_calibration_sensor_uid == lidar_uid);
        data_->main_calibration_sensor_uid = lidar_uid;
      }

      for (const auto & lidartag_detection : lidar_it.detections) {
        const TagType tag_type = TagType::WaypointTag;
        UID tag_uid = UID::makeTagUID(tag_type, scene_index, lidartag_detection.id);

        data_->uid_connections_map[lidar_uid].push_back(tag_uid);
        data_->uid_connections_map[tag_uid].push_back(lidar_uid);

        data_->detections_relative_poses_map[std::make_pair(lidar_uid, tag_uid)] =
          lidartag_detection.pose;
        data_->detections_relative_poses_map[std::make_pair(tag_uid, lidar_uid)] =
          lidartag_detection.pose.inv();
      }
    }

    for (int external_camera_id = 0;
         external_camera_id < static_cast<int>(scene.external_camera_frames.size());
         external_camera_id++) {
      UID external_camera_uid =
        UID::makeSensorUID(SensorType::ExternalCamera, scene_index, external_camera_id);

      for (const auto & group_detections_it :
           scene.external_camera_frames[external_camera_id].detections) {
        const TagType tag_type = group_detections_it.first;
        const ApriltagGridDetections & apriltag_grid_detections = group_detections_it.second;

        for (const auto & grid_detection : apriltag_grid_detections) {
          UID tag_uid = UID::makeTagUID(tag_type, scene_index, grid_detection.id);

          data_->uid_connections_map[external_camera_uid].push_back(tag_uid);
          data_->uid_connections_map[tag_uid].push_back(external_camera_uid);

          data_->detections_relative_poses_map[std::make_pair(external_camera_uid, tag_uid)] =
            grid_detection.pose;
          data_->detections_relative_poses_map[std::make_pair(tag_uid, external_camera_uid)] =
            grid_detection.pose.inv();
        }
      }
    }
  }

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::calibrationCallback(
  __attribute__((unused)) const std::shared_ptr<tier4_calibration_msgs::srv::Empty::Request>
    request,
  __attribute__((unused)) std::shared_ptr<tier4_calibration_msgs::srv::Empty::Response> response)
{
  // if (is_lidar_calibration_) {
  //   calibration_problem_.setFixedWaypointPoses(true);
  // } TODOKL delete since lidars always do participate

  UID main_sensor_uid;

  for (const auto & detections : data_->scenes[0].calibration_cameras_detections) {
    if (detections.calibration_frame == main_calibration_sensor_frame_) {
      main_sensor_uid =
        UID::makeSensorUID(SensorType::CalibrationCamera, detections.calibration_camera_id);
    }
  }

  for (const auto & detections : data_->scenes[0].calibration_lidars_detections) {
    if (detections.calibration_frame == main_calibration_sensor_frame_) {
      main_sensor_uid =
        UID::makeSensorUID(SensorType::CalibrationLidar, detections.calibration_lidar_id);
    }
  }

  assert(main_sensor_uid.isValid());
  UID left_wheel_uid = UID::makeTagUID(TagType::WheelTag, -1, left_wheel_tag_id);
  UID right_wheel_uid = UID::makeTagUID(TagType::WheelTag, -1, right_wheel_tag_id);

  // Estimate the initial poses
  estimateInitialPoses(*data_, main_sensor_uid, left_wheel_uid, right_wheel_uid);

  // Set the initial intrinsics for the external camera
  std::array<double, CalibrationData::INTRINSICS_DIM> initial_intrinsics;
  initial_intrinsics[0] = external_camera_intrinsics_.undistorted_camera_matrix(0, 2);
  initial_intrinsics[1] = external_camera_intrinsics_.undistorted_camera_matrix(1, 2);
  initial_intrinsics[2] = external_camera_intrinsics_.undistorted_camera_matrix(0, 0);
  initial_intrinsics[3] = external_camera_intrinsics_.undistorted_camera_matrix(1, 1);
  initial_intrinsics[4] = 0.0;
  initial_intrinsics[5] = 0.0;

  for (const auto & it : data_->initial_sensor_poses_map) {
    if (it.first.sensor_type != SensorType::ExternalCamera) {
      continue;
    }

    const UID & camera_uid = it.first;
    data_->initial_camera_intrinsics_map[camera_uid] =
      std::make_shared<std::array<double, CalibrationData::INTRINSICS_DIM>>(initial_intrinsics);
  }

  for (const auto & it : data_->scenes[0].calibration_cameras_detections) {
    const UID & calibration_camera_uid =
      UID::makeSensorUID(SensorType::CalibrationCamera, it.calibration_camera_id);
    std::string calibration_camera_frame = it.calibration_frame;

    data_->calibration_camera_intrinsics_map_[calibration_camera_uid] =
      calibration_camera_intrinsics_map_[calibration_camera_frame];
  }

  calibration_problem_.setOptimizeIntrinsics(ba_optimize_intrinsics_);
  calibration_problem_.setShareIntrinsics(ba_share_intrinsics_);
  calibration_problem_.setForceSharedGroundPlane(ba_force_shared_ground_plane_);
  calibration_problem_.setCalibrationLidarIntrinsics(virtual_lidar_f_);
  calibration_problem_.setWheelTagUIDs(
    UID::makeTagUID(TagType::WheelTag, -1, left_wheel_tag_id),
    UID::makeTagUID(TagType::WheelTag, -1, right_wheel_tag_id));
  calibration_problem_.setData(data_);

  calibration_problem_.dataToPlaceholders();
  calibration_problem_.evaluate();
  calibration_problem_.solve();
  calibration_problem_.placeholdersToData();
  calibration_problem_.evaluate();
  calibration_problem_.writeDebugImages();
  RCLCPP_INFO(this->get_logger(), "Finished optimization");

  // Derive the base link pose
  cv::Affine3d ground_pose;

  if (
    !computeGroundPlane(
      data_->optimized_ground_tag_poses_map, ground_tag_parameters_.size, ground_pose) ||
    !data_->optimized_left_wheel_tag_pose || !data_->optimized_right_wheel_tag_pose) {
    RCLCPP_ERROR(this->get_logger(), "Could not compute the base link");
    response->success = false;
    return false;
  }

  calibrated_main_sensor_to_base_link_pose_ = computeBaseLink(
    *data_->optimized_left_wheel_tag_pose, *data_->optimized_right_wheel_tag_pose, ground_pose);

  calibration_done_ = true;
  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::loadDatabaseCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Loading database...");
  std::ifstream ifs(request->files[0]);
  boost::archive::text_iarchive ia(ifs);

  ia >> data_;

  // Overwrite the tag sizes in case the parameters change
  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    CalibrationScene & scene = data_->scenes[scene_index];

    for (auto & scene_camera_detections_it : scene.calibration_cameras_detections) {
      GroupedApriltagGridDetections & grouped_grid_detections =
        scene_camera_detections_it.grouped_detections;

      for (auto & group_grid_detections : grouped_grid_detections) {
        std::vector<ApriltagGridDetection> & grid_detections = group_grid_detections.second;
        const auto tag_parameters = tag_parameters_map_[group_grid_detections.first];

        for (auto & grid_detection : grid_detections) {
          grid_detection.computeTemplateCorners(tag_parameters);
          grid_detection.computeObjectCorners();
        }
      }
    }

    for (auto & scene_lidar_detections_it : scene.calibration_lidars_detections) {
      LidartagDetections & detections = scene_lidar_detections_it.detections;

      for (auto & detection : detections) {
        detection.size = waypoint_tag_parameters_.size;
        detection.computeTemplateCorners();
        detection.computeObjectCorners();
      }
    }

    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      auto & grouped_grid_detections = scene.external_camera_frames[frame_id].detections;
      for (auto & group_grid_detections : grouped_grid_detections) {
        std::vector<ApriltagGridDetection> & grid_detections = group_grid_detections.second;
        const auto tag_parameters = tag_parameters_map_[group_grid_detections.first];

        for (auto & grid_detection : grid_detections) {
          grid_detection.computeTemplateCorners(tag_parameters);
          grid_detection.computeObjectCorners();
        }
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Database loaded");

  response->success = true;
  return true;
}

bool ExtrinsicTagBasedBaseCalibrator::saveDatabaseCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
  std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Saving database");
  std::ofstream ofs(request->files[0]);
  boost::archive::text_oarchive oa(ofs);

  oa << data_;

  RCLCPP_INFO(this->get_logger(), "Database saved");

  response->success = true;
  return true;
}

}  // namespace extrinsic_tag_based_base_calibrator
