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

#include <extrinsic_mapping_based_calibrator/extrinsic_mapping_based_calibrator.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <tier4_pcl_extensions/voxel_grid_triplets.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <tf2/utils.h>
//#include <tf2_eigen/tf2_eigen.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <iostream>
#include <thread>
#include <chrono>

#define UNUSED(x) (void)x;

using namespace std::chrono_literals;

// Move this aux to another file
#pragma GCC push_options
#pragma GCC optimize("O0")
Eigen::Matrix4f poseInterpolation(double t, double t1, double t2, Eigen::Matrix4f const& m1, Eigen::Matrix4f const& m2) {

  assert(t >= t1 && t <= t2);

  float alpha = 0.0;
  if (t2 != t1)
    alpha = (t - t1) / (t2 - t1);

  Eigen::Affine3f aff1(m1);
  Eigen::Affine3f aff2(m2);

  Eigen::Quaternionf rot1(aff1.linear());
  Eigen::Quaternionf rot2(aff2.linear());

  Eigen::Vector3f trans1 = aff1.translation();
  Eigen::Vector3f trans2 = aff2.translation();

  Eigen::Affine3f result;
  result.translation() = (1.0 - alpha) * trans1 + alpha * trans2;
  result.linear() = rot1.slerp(alpha, rot2).toRotationMatrix();

  return result.matrix();
}
#pragma GCC pop_options

ExtrinsicMappingBasedCalibrator::ExtrinsicMappingBasedCalibrator(const rclcpp::NodeOptions & options)
: Node("extrinsic_mapping_based_calibrator_node", options),
  tf_broascaster_(this),
  selected_keyframe_(-1),
  n_processed_frames_(0)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  sensor_kit_frame_ = this->declare_parameter<std::string>("parent_frame");
  lidar_base_frame_ = this->declare_parameter<std::string>("child_frame");
  map_frame_ = this->declare_parameter<std::string>("map_frame");

  max_frames_ = this->declare_parameter<int>("max_frames", 500);
  local_map_num_keyframes_ = this->declare_parameter<int>("local_map_num_keyframes", 15);
  calibration_num_keyframes_ = this->declare_parameter<int>("calibration_num_keyframes_", 10);
  max_pointcloud_range_ = this->declare_parameter<double>("max_pointcloud_range", 80.0);

  ndt_resolution_ = this->declare_parameter<double>("ndt_resolution", 5.0);
  ndt_step_size_ = this->declare_parameter<double>("ndt_step_size", 0.1);
  ndt_max_iterations_ = this->declare_parameter<int>("ndt_max_iterations", 35);
  ndt_num_threads_ = this->declare_parameter<int>("ndt_num_threads", 8);
  leaf_size_input_ = this->declare_parameter<double>("leaf_size_iput", 0.1);
  leaf_size_local_map_ = this->declare_parameter<double>("leaf_size_local_map", 0.1);
  leaf_size_dense_map_ = this->declare_parameter<double>("leaf_size_dense_map", 0.05);
  new_keyframe_min_distance_ = this->declare_parameter<double>("new_keyframe_min_distance", 1.0);
  new_frame_min_distance_ = this->declare_parameter<double>("new_frame_min_distance", 0.05);
  frame_stopped_distance_ = this->declare_parameter<double>("frame_stopped_distance", 0.02); // distance between frames to consider the car to be stopped. uses hysteresis
  frame_nonstopped_distance_ = this->declare_parameter<double>("frame_nonstopped_distance", 0.05); // distance between frames to consider the car to be moving. uses hysteresis
  frames_since_stop_force_frame_ =this->declare_parameter<int>("frames_since_stoped_force_frame", 5);
  calibration_skip_keyframes_ = this->declare_parameter<int>("calibration_skip_keyframes", 5); // Skip the first frames for calibration

  calibration_max_interpolated_time_ = this->declare_parameter<double>("calibration_max_interpolated_time", 0.03);
  calibration_max_interpolated_distance_ = this->declare_parameter<double>("calibration_max_interpolated_distance", 0.05);
  calibration_max_interpolated_angle_ = this->declare_parameter<double>("calibration_max_interpolated_angle", 1.0);
  calibration_max_interpolated_speed_ = this->declare_parameter<double>("calibration_max_interpolated_speed", 3.0);
  calibration_max_interpolated_accel_ = this->declare_parameter<double>("calibration_max_interpolated_accel", 0.2);

  map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_map", 10);
  keyframe_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("keyframe_map", 10);
  keyframe_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("keyframe", 10);
  frame_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("frame_path", 10);
  keyframe_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("keyframe_path", 10);
  keyframe_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("keyframe_markers", 10);

  calibration_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "calibration_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ExtrinsicMappingBasedCalibrator::calibrationPointCloudCallback, this, std::placeholders::_1));

  mapping_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "mapping_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ExtrinsicMappingBasedCalibrator::mappingPointCloudCallback, this, std::placeholders::_1));

  // The service server runs in a dedicated thread
  srv_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  //service_server_ = this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
  //  "extrinsic_calibration",
  //  std::bind(
  //    &ExtrinsicMappingBasedCalibrator::requestReceivedCallback, this, std::placeholders::_1,
  //    std::placeholders::_2),
  //  rmw_qos_profile_services_default, srv_callback_group_);

  keyframe_map_server_ = this->create_service<tier4_calibration_msgs::srv::KeyFrameMap>(
    "keyframe_map",
    std::bind(
      &ExtrinsicMappingBasedCalibrator::keyFrameCallback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);

  timer_ = rclcpp::create_timer(this, get_clock(), 10s, std::bind(&ExtrinsicMappingBasedCalibrator::timerCallback, this));

  published_map_pointcloud_ptr_.reset(new PointcloudType());

  ndt.setResolution(ndt_resolution_); // 1.0
  ndt.setStepSize(ndt_step_size_); // 0.1
  ndt.setMaximumIterations(ndt_max_iterations_); // 35
  ndt.setTransformationEpsilon(0.01);
  ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);

  if (ndt_num_threads_ > 0) {
    ndt.setNumThreads(ndt_num_threads_);
  }

  last_unmatched_keyframe_ = calibration_skip_keyframes_;

  std::thread thread = std::thread(&ExtrinsicMappingBasedCalibrator::mappingThreadWorker, this);
  thread.detach();

  // services
  // timers local map
  // path
}

#pragma GCC push_options
#pragma GCC optimize("O0")
void ExtrinsicMappingBasedCalibrator::calibrationPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  calibration_pointclouds_queue_.push(msg);

  std::unique_lock<std::mutex> lock(mutex_);

  while (last_unmatched_keyframe_ < int(keyframes_and_stopped_.size()) - 1) {

    std::shared_ptr<Frame> keyframe = keyframes_and_stopped_[last_unmatched_keyframe_];
    auto keyframe_stamp = rclcpp::Time(keyframe->header_.stamp);

    // rclcpp::Time(t2.header.stamp).seconds()
    if (calibration_pointclouds_queue_.size() < 3 || rclcpp::Time(calibration_pointclouds_queue_.back()->header.stamp) < keyframe_stamp) {
      return;
    }

    // We need there to be a frame after the keyframe since we may wanto to interpolate in that direction
    if (keyframe->frame_id_ + 1 >= int(processed_frames_.size())) {
      return;
    }

    // Iterate for all frames between the last keyframe and this one looking for a stopped frame, which we want to use as a calibration frame
    auto pc_msg_left = calibration_pointclouds_queue_.front();
    calibration_pointclouds_queue_.pop();
    auto pc_msg_right = calibration_pointclouds_queue_.front();

    while(rclcpp::Time(pc_msg_right->header.stamp) < keyframe_stamp) {
      pc_msg_left = pc_msg_right;
      calibration_pointclouds_queue_.pop();
      pc_msg_right = calibration_pointclouds_queue_.front();
    }

    double dt_left = (keyframe_stamp - rclcpp::Time(pc_msg_left->header.stamp)).seconds();
    double dt_right = (rclcpp::Time(pc_msg_right->header.stamp) - keyframe_stamp).seconds();

    // Drop all the pointcloud messages until we
    sensor_msgs::msg::PointCloud2::SharedPtr pc_msg;
    std::shared_ptr<Frame> frame_left, frame_right, frame_aux;
    double interpolated_distance;
    double interpolated_time;
    double interpolated_speed;
    double interpolated_accel;

    // Compute first and second order derivates with finite differences
    if (dt_left < dt_right) {
      pc_msg = pc_msg_left;

      frame_left = processed_frames_[keyframe->frame_id_ - 1];
      frame_right = keyframe;
      frame_aux = processed_frames_[keyframe->frame_id_ + 1];

      double dt1 = (rclcpp::Time(frame_right->header_.stamp) - rclcpp::Time(frame_left->header_.stamp)).seconds();
      double dt2 = (rclcpp::Time(frame_aux->header_.stamp) - rclcpp::Time(frame_right->header_.stamp)).seconds();
      interpolated_speed = std::abs((frame_aux->distance_ - frame_left->distance_) / (dt1 + dt2));
      interpolated_accel = std::abs(((frame_aux->distance_ - frame_right->distance_) / dt2 - (frame_right->distance_ - frame_left->distance_) / dt1) / (dt1 + dt2));

      interpolated_time = dt_left;
    }
    else {
      pc_msg = pc_msg_right;

      frame_aux = processed_frames_[keyframe->frame_id_ - 1];
      frame_left = keyframe;
      frame_right = processed_frames_[keyframe->frame_id_ + 1];

      double dt1 = (rclcpp::Time(frame_left->header_.stamp) - rclcpp::Time(frame_aux->header_.stamp)).seconds();
      double dt2 = (rclcpp::Time(frame_right->header_.stamp) - rclcpp::Time(frame_left->header_.stamp)).seconds();
      interpolated_speed = std::abs((frame_right->distance_ - frame_aux->distance_) / (dt1 + dt2));
      interpolated_accel = std::abs(((frame_right->distance_ - frame_left->distance_) / dt2 - (frame_left->distance_ - frame_aux->distance_) / dt1) / (dt1 + dt2));

      interpolated_time = dt_right;
    }

    interpolated_distance = (frame_right->distance_ - frame_left->distance_) * interpolated_time / (rclcpp::Time(frame_right->header_.stamp) - rclcpp::Time(frame_left->header_.stamp)).seconds();

    // Interpolate the pose
    Eigen::Matrix4f interpolated_pose = poseInterpolation(
      (rclcpp::Time(pc_msg->header.stamp) - rclcpp::Time(frame_left->header_.stamp)).seconds(),
      0.f,
      (rclcpp::Time(frame_right->header_.stamp) - rclcpp::Time(frame_left->header_.stamp)).seconds(),
      frame_left->pose_,
      frame_right->pose_
    );

    // Compute interpolation angle
    // trR = 1 + 2cos(theta)
    Eigen::Matrix4f aux_pose = keyframe->pose_.inverse() * interpolated_pose;
    Eigen::Affine3f aux_affine(aux_pose);
    double interpolated_angle = 180.0 * std::abs(std::acos(std::min(std::max(0.5*aux_affine.linear().trace() - 0.5, -1.0), 1.0)))/ 3.1416;

    // Decide on whether or not to keep the result as a calibraton frame
    // pc, pose, statistics

    //calibration_max_interpolated_time_

    RCLCPP_INFO(get_logger(), "Attempting to add keyframe id=%d to the calibration list", keyframe->keyframe_id_);
    RCLCPP_INFO(get_logger(), "\t - stopped: %s", keyframe->stopped_ ? " true" : "false");
    RCLCPP_INFO(get_logger(), "\t - interpolated time: %.4f s (%s)", interpolated_time, interpolated_time < calibration_max_interpolated_time_ ? "accepted" : "rejected");
    RCLCPP_INFO(get_logger(), "\t - interpolated distance: %.4f m (%s)", interpolated_distance, interpolated_distance < calibration_max_interpolated_distance_ ? "accepted" : "rejected");
    RCLCPP_INFO(get_logger(), "\t - interpolated angle: %.4f deg (%s)", interpolated_angle, interpolated_angle < calibration_max_interpolated_angle_ ? "accepted" : "rejected");
    RCLCPP_INFO(get_logger(), "\t - interpolated speed: %.4f m/s (%s)", interpolated_speed, interpolated_speed < calibration_max_interpolated_speed_ ? "accepted" : "rejected");
    RCLCPP_INFO(get_logger(), "\t - interpolated accel: %.4f m/s2 (%s)", interpolated_accel, interpolated_accel < calibration_max_interpolated_accel_ ? "accepted" : "rejected");

    if (interpolated_time < calibration_max_interpolated_time_ &&
      interpolated_distance < calibration_max_interpolated_distance_ &&
      interpolated_angle < calibration_max_interpolated_angle_ &&
      interpolated_speed < calibration_max_interpolated_speed_ &&
      interpolated_accel < calibration_max_interpolated_accel_
      )
    {
      PointcloudType::Ptr pc_ptr(new PointcloudType());
      pcl::fromROSMsg(*pc_msg, *pc_ptr);

      CalibrationFrame calibration_frame;

      calibration_frame.source_pointcloud_ = pc_ptr;
      calibration_frame.source_header_ = pc_msg->header;
      calibration_frame.local_map_pose_ = interpolated_pose;

      calibration_frame.interpolated_distance_ = interpolated_distance;
      calibration_frame.interpolated_angle_ = interpolated_angle;
      calibration_frame.interpolated_time_ = interpolated_time;
      calibration_frame.estimated_speed_ = interpolated_speed;
      calibration_frame.estimated_accel_ = interpolated_accel;

      calibration_frames_.emplace_back(calibration_frame);
    }

    last_unmatched_keyframe_ += 1;
  }



  // Fill a queue with the pointclouds, max_size = 2
  // Sadly, we can not rely on this since the mapping takes time!!!

  // Assume we have matched all keyframes to some calibration pointcloud so far
  // We have an unmatched keyframe
  // We wait until we have one calibration pointcloud newer than the keyframe
  // At this point we have one keyframe and two candidate calibration pointclouds
  // Choose the closest calibration to the pointcloud to the keyframe
  // Somehow we need to have a point in the trajectory or in the frames. maybe use 3 frames, and ask two of them newer that the keyframe or two if the last is stopped
  // Interpolate
  // Compute the conditions to accept the combination as a calibration frame. interpolation distance, interpolation time, speed, and min_keyframe id (do not use the first keyframes for calibration since we do not have much info so far)

}
#pragma GCC pop_options

void ExtrinsicMappingBasedCalibrator::mappingPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!lidar_header_) {
    lidar_header_ = std::make_shared<std_msgs::msg::Header>(msg->header);
  }

  PointcloudType::Ptr pc_ptr(new PointcloudType());
  pcl::fromROSMsg(*msg, *pc_ptr);

  //RCLCPP_INFO(get_logger(), "ROS: getting mutex");
  std::unique_lock<std::mutex> lock(mutex_);
  //RCLCPP_INFO(get_logger(), "ROS: got mutex");

  auto frame = std::make_shared<Frame>();
  frame->header_ = msg->header;
  frame->processed_ = false;
  frame->pointcloud_raw_ = pc_ptr;
  frame->is_key_frame_ = false;
  frame->stopped_ = false;
  frame->frames_since_stop_ = 0;
  frame->distance_ = 0.f;
  frame->delta_distance_; 0.f;

  if(rclcpp::Time(msg->header.stamp) < rclcpp::Time(lidar_header_->stamp) || int(processed_frames_.size()) >= max_frames_) {
    return;
  }

  unprocessed_frames_.push(frame);
  RCLCPP_INFO(get_logger(), "ROS: New pointcloud. Unprocessed=%lu Frames=%lu Keyframes=%lu", unprocessed_frames_.size(), processed_frames_.size(), keyframes_.size());
}

void ExtrinsicMappingBasedCalibrator::mappingThreadWorker()
{
  bool first_iteration = true;

  while (rclcpp::ok()){

    std::shared_ptr<Frame> frame, prev_frame;
    Eigen::Matrix4f prev_pose = Eigen::Matrix4f::Identity();
    float prev_distance = 0.f;

    // /RCLCPP_INFO(get_logger(), "Thread. Entering locked");

    // Locked section
    {
      std::unique_lock<std::mutex> lock(mutex_);

      if (unprocessed_frames_.size() == 0) {
        lock.unlock();
        rclcpp::sleep_for(10ms);
        continue;
      }

      frame = unprocessed_frames_.back();
      unprocessed_frames_.pop();

      if (processed_frames_.size() > 0) {
        prev_frame = processed_frames_.back();
        prev_pose = prev_frame->pose_;
        prev_distance = prev_frame->distance_;
      }
    }

    //RCLCPP_INFO(get_logger(), "Thread. Exited locked");

    // If there are no keyframes make the first and recalculate the local map
    if (first_iteration) {
      RCLCPP_INFO(get_logger(), "Thread. Init local map");
      initLocalMap(frame);
    }

    // Subsample the frame
    pcl::VoxelGrid<PointType> voxel_grid;
    frame->pointcloud_subsampled_ = PointcloudType::Ptr(new PointcloudType());
    PointcloudType::Ptr aligned_cloud_ptr(new PointcloudType());
    PointcloudType::Ptr cropped_cloud_ptr = cropPointCloud(frame->pointcloud_raw_, max_pointcloud_range_);

    //RCLCPP_INFO(get_logger(), "Subsampling input...");
    voxel_grid.setLeafSize(leaf_size_input_, leaf_size_input_, leaf_size_input_);
    voxel_grid.setInputCloud(cropped_cloud_ptr);
    voxel_grid.filter(*frame->pointcloud_subsampled_);
    //RCLCPP_INFO(get_logger(), "Subsampled input!");

    //RCLCPP_INFO(get_logger(), "Thread. After voxel");

    // Register the frame to the map
    ndt.setInputTarget(local_map_ptr_);
    ndt.setInputSource(frame->pointcloud_subsampled_);


    if (first_iteration) {
      frame->pose_ = Eigen::Matrix4f::Identity();
    }
    else {
      ndt.align(*aligned_cloud_ptr, prev_pose);
      frame->pose_ = ndt.getFinalTransformation();
    }

    //RCLCPP_INFO(get_logger(), "Thread. After ndt");
    std::unique_lock<std::mutex> lock(mutex_);

    // We record the whole trajectory
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = frame->header_;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = tf2::toMsg(Eigen::Affine3d(frame->pose_.cast<double>()));
    trajectory_.push_back(pose_msg);

    // Fill the frame information
    Eigen::Vector3f translation = Eigen::Affine3f(frame->pose_.inverse() * prev_pose).translation();
    float delta_distance = translation.norm();
    frame->distance_ = prev_distance + delta_distance;
    frame->frame_id_ = processed_frames_.size();
    frame->processed_ = true;

    if (!first_iteration && delta_distance < new_frame_min_distance_) {

      if (std::abs(delta_distance - prev_frame->delta_distance_) < frame_stopped_distance_) {
        // When the vehicle is stopped, we may either skip the frame, record it as a normal frame, or even save it for calibration

        RCLCPP_WARN(get_logger(), "Old prev_frame->frames_since_stop_: %d", prev_frame->frames_since_stop_);
        prev_frame->frames_since_stop_ += 1;
        prev_frame->delta_distance_ = delta_distance;
        RCLCPP_WARN(get_logger(), "New prev_frame->frames_since_stop_: %d", prev_frame->frames_since_stop_);

        frame->frames_since_stop_ = prev_frame->frames_since_stop_;
        frame->stopped_ = true;

        if(prev_frame->frames_since_stop_ == frames_since_stop_force_frame_) {
          RCLCPP_WARN(get_logger(), "Added a keyframe_and_stopped frame");
            keyframes_and_stopped_.push_back(frame);

        }
        else if (prev_frame->stopped_ && std::abs(prev_frame->frames_since_stop_ - frames_since_stop_force_frame_) > 1) {
          RCLCPP_WARN(get_logger(), "Dropped stopped frame (%d). delta_distance=%.4f Unprocessed=%lu Frames=%lu Keyframes=%lu", prev_frame->frames_since_stop_, delta_distance, unprocessed_frames_.size(), processed_frames_.size(), keyframes_.size());
          continue;
        }

        RCLCPP_WARN(get_logger(), "Adding a stopped frame (%d)", frame->frames_since_stop_);
      }
      else {
        // If the vehicle moved to little we drop the frame
        RCLCPP_INFO(get_logger(), "Dropped frame. Unprocessed=%lu Frames=%lu Keyframes=%lu", unprocessed_frames_.size(), processed_frames_.size(), keyframes_.size());
      continue;
      }
    }

    //RCLCPP_INFO(get_logger(), "Thread. New frame");
    //std::unique_lock<std::mutex> lock(mutex_);
    processed_frames_.push_back(frame);
    checkKeyframe(frame);

    RCLCPP_INFO(get_logger(), "New frame. Distance=%.2f Unprocessed=%lu Frames=%lu Keyframes=%lu", frame->distance_, unprocessed_frames_.size(), processed_frames_.size(), keyframes_.size());
    // Publish tf
    first_iteration = false;
  }
}

void ExtrinsicMappingBasedCalibrator::initLocalMap(std::shared_ptr<Frame> frame)
{
  local_map_ptr_.reset(new PointcloudType());
  PointcloudType::Ptr cropped_cloud_ptr = cropPointCloud(frame->pointcloud_raw_, max_pointcloud_range_);
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setLeafSize(leaf_size_local_map_, leaf_size_local_map_, leaf_size_local_map_);
  voxel_grid.setInputCloud(cropped_cloud_ptr);
  voxel_grid.filter(*local_map_ptr_);
}

void ExtrinsicMappingBasedCalibrator::checkKeyframe(std::shared_ptr<Frame> frame)
{
  if (keyframes_.size() == 0 || frame->distance_ >= keyframes_.back()->distance_ + new_keyframe_min_distance_) {
    keyframes_.push_back(frame);
    keyframes_and_stopped_.push_back(frame);
    frame->is_key_frame_ = true;
    frame->keyframe_id_ = keyframes_.size();
    recalculateLocalMap();
  }
}

void ExtrinsicMappingBasedCalibrator::recalculateLocalMap()
{
  local_map_ptr_->clear();


  PointcloudType::Ptr tmp_mcs_ptr(new PointcloudType());

  for (int i = 0; i < local_map_num_keyframes_ && i < int(keyframes_.size()); i++) {
    const auto  & keyframe = keyframes_[keyframes_.size() - i - 1];

    PointcloudType::Ptr keyframe_mcs_ptr(new PointcloudType());
    pcl::transformPointCloud(*keyframe->pointcloud_subsampled_, *keyframe_mcs_ptr, keyframe->pose_);
    *tmp_mcs_ptr += *keyframe_mcs_ptr;
  }

  float minx = std::numeric_limits<float>::max();
  float maxx = -std::numeric_limits<float>::max();
  float miny = minx;
  float maxy = maxy;
  float minz = minx;
  float maxz = maxz;

  //for( auto& p : tmp_mcs_ptr->points){
  //  minx = std::min(minx, p.x);
  //  miny = std::min(miny, p.y);
  //  minz = std::min(minz, p.z);
  //  maxx = std::max(maxx, p.x);
  //  maxy = std::max(maxy, p.y);
  //  maxz = std::max(maxz, p.z);
  //}

  //unsigned long int maxindex = (maxx - minx) * (maxy - miny) * (maxz - minz) / (leaf_size_local_map_*leaf_size_local_map_*leaf_size_local_map_);

  //RCLCPP_INFO(get_logger(), "Subsampling local map...");
  //RCLCPP_INFO(get_logger(), "leaf size=%f", leaf_size_local_map_);
  //RCLCPP_INFO(get_logger(), "dx=%f", maxx - minx);
  //RCLCPP_INFO(get_logger(), "dy=%f", maxy - miny);
  //RCLCPP_INFO(get_logger(), "dz=%f", maxz - minz);
  //RCLCPP_INFO(get_logger(), "maxindex=%lu", maxindex);
  //RCLCPP_INFO(get_logger(), "maxindex2=%lu", std::numeric_limits<std::int32_t>::max());

  //auto start1 = std::chrono::high_resolution_clock::now();
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setLeafSize(leaf_size_local_map_, leaf_size_local_map_, leaf_size_local_map_);
  voxel_grid.setInputCloud(tmp_mcs_ptr);
  voxel_grid.filter(*local_map_ptr_);

  //auto stop1 = std::chrono::high_resolution_clock::now();


  //local_map_ptr_->clear();

  //auto start2 = std::chrono::high_resolution_clock::now();
  //pcl::VoxelGridTriplets<PointType> voxel_grid2;
  //voxel_grid2.setLeafSize(leaf_size_local_map_, leaf_size_local_map_, leaf_size_local_map_);
  //voxel_grid2.setInputCloud(tmp_mcs_ptr);
  //voxel_grid2.filter(*local_map_ptr_);
  //auto stop2 = std::chrono::high_resolution_clock::now();

  //auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start1);
  //auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(stop2 - start2);
  //RCLCPP_WARN(get_logger(), "=== Voxel statistics: ===");
  //RCLCPP_WARN(get_logger(), "Input size = %lu", tmp_mcs_ptr->size());
  //RCLCPP_WARN(get_logger(), "Original implementation time = %d", duration1.count());
  //RCLCPP_WARN(get_logger(), "Triplet implementation time = %d", duration2.count());


  //RCLCPP_INFO(get_logger(), "Subsampled local map...");
}

void ExtrinsicMappingBasedCalibrator::timerCallback()
{
  static int published_frames = 0;
  static int published_keyframes = 0;

  std::unique_lock<std::mutex> lock(mutex_);

  if (int(keyframes_.size()) == published_keyframes) {
    return;
  }

  // Process the new keyframes into the map
  PointcloudType::Ptr tmp_mcs_ptr(new PointcloudType());
  PointcloudType::Ptr subsampled_mcs_ptr(new PointcloudType());
  *tmp_mcs_ptr += *published_map_pointcloud_ptr_;


  //for (auto it = keyframes_.begin() + published_keyframes; it != keyframes_.end(); ++it) {
  //  std::shared_ptr<Frame> keyframe = *it;
  //  PointcloudType::Ptr keyframe_mcs_ptr(new PointcloudType());
  //  pcl::transformPointCloud(*keyframe->pointcloud_subsampled_, *keyframe_mcs_ptr, keyframe->pose_);
  //  *tmp_mcs_ptr += *keyframe_mcs_ptr;
  //}

  for (auto it = processed_frames_.begin() + published_keyframes; it != processed_frames_.end(); ++it) {
    std::shared_ptr<Frame> frame = *it;
    PointcloudType::Ptr frame_mcs_ptr(new PointcloudType());
    pcl::transformPointCloud(*frame->pointcloud_subsampled_, *frame_mcs_ptr, frame->pose_);
    *tmp_mcs_ptr += *frame_mcs_ptr;
  }

  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setLeafSize(leaf_size_input_, leaf_size_input_, leaf_size_input_);
  voxel_grid.setInputCloud(tmp_mcs_ptr);
  voxel_grid.filter(*subsampled_mcs_ptr);

  published_map_pointcloud_ptr_.swap(subsampled_mcs_ptr);

  // Process the new keyframes and frames into the paths
  for (auto it = keyframes_.begin() + published_keyframes; it != keyframes_.end(); ++it) {
    std::shared_ptr<Frame> keyframe = *it;
    Eigen::Matrix4d pose_matrix = keyframe->pose_.cast<double>();
    Eigen::Isometry3d pose_isometry(pose_matrix);
    geometry_msgs::msg::PoseStamped pose_msg;
    visualization_msgs::msg::Marker keyframe_marker;
    pose_msg.header = keyframe->header_;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = tf2::toMsg(pose_isometry);
    published_keyframes_path_.poses.push_back(pose_msg);

    keyframe_marker.header = keyframe->header_;
    keyframe_marker.header.frame_id = map_frame_;
    keyframe_marker.ns = "keyframe_id";
    keyframe_marker.id = keyframe->keyframe_id_;
    keyframe_marker.color.r = 1.0;
    keyframe_marker.color.g = 1.0;
    keyframe_marker.color.b = 1.0;
    keyframe_marker.color.a = 1.0;
    keyframe_marker.scale.x = 0.03;

    keyframe_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    keyframe_marker.text = std::to_string(keyframe->keyframe_id_);
    keyframe_marker.scale.z = 0.3;
    keyframe_marker.pose = pose_msg.pose;
    keyframe_marker.pose.position.z += 0.1;
    published_keyframes_markers_.markers.push_back(keyframe_marker);
  }

  for (auto it = processed_frames_.begin() + published_frames; it != processed_frames_.end(); ++it) {
    std::shared_ptr<Frame> frame = *it;
    Eigen::Matrix4d pose_matrix = frame->pose_.cast<double>();
    Eigen::Isometry3d pose_isometry(pose_matrix);
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = frame->header_;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = tf2::toMsg(pose_isometry);
    published_frames_path_.poses.push_back(pose_msg);
  }

  published_frames = processed_frames_.size();
  published_keyframes = keyframes_.size();

  // Publish the data
  sensor_msgs::msg::PointCloud2 published_map_msg;
  pcl::toROSMsg(*published_map_pointcloud_ptr_, published_map_msg);

  published_map_msg.header = *lidar_header_;
  published_keyframes_path_.header = *lidar_header_;
  published_frames_path_.header = *lidar_header_;

  published_map_msg.header.frame_id = map_frame_;
  published_keyframes_path_.header.frame_id = map_frame_;
  published_frames_path_.header.frame_id = map_frame_;

  map_pub_->publish(published_map_msg);
  keyframe_path_pub_->publish(published_keyframes_path_);
  frame_path_pub_->publish(published_frames_path_);
  keyframe_markers_pub_->publish(published_keyframes_markers_);

  return;
}

void ExtrinsicMappingBasedCalibrator::keyFrameCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::KeyFrameMap::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::KeyFrameMap::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_);

  int requested_keyframe_id = request->id;

  if (requested_keyframe_id < 0 || requested_keyframe_id >= int(keyframes_.size())) {
    RCLCPP_ERROR(get_logger(), "Invalid keyframe id");
    return;
  }

  RCLCPP_INFO(get_logger(), "keyFrameCallback callback. Processing keyframe=%d", requested_keyframe_id);

  int min_keyframe_id = std::max<int>(0, requested_keyframe_id - calibration_num_keyframes_);
  int max_keyframe_id = std::max<int>(keyframes_.size() - 1, requested_keyframe_id + calibration_num_keyframes_);

  int min_frame_id = keyframes_[min_keyframe_id]->frame_id_;
  int max_frame_id = keyframes_[max_keyframe_id]->frame_id_;

  const std::shared_ptr<Frame> & keyframe = keyframes_[requested_keyframe_id];
  auto map_keframe_pose = keyframe->pose_;
  auto keyframe_map_pose = keyframe->pose_.inverse();

  // Sum all frames in the map coordinate
  PointcloudType::Ptr tmp_kcs_ptr(new PointcloudType());
  PointcloudType::Ptr subsampled_kcs_ptr(new PointcloudType());

  for (int i = min_frame_id; i <= max_frame_id; i++) {
    std::shared_ptr<Frame> frame = processed_frames_[i];
    PointcloudType::Ptr frame_kcs_ptr(new PointcloudType());

    auto map_frame_pose = frame->pose_;
    auto keyframe_frame_pose = keyframe_map_pose * map_frame_pose;

    pcl::transformPointCloud(*frame->pointcloud_raw_, *frame_kcs_ptr, keyframe_frame_pose);
    *tmp_kcs_ptr += *frame_kcs_ptr;
  }

  PointcloudType::Ptr cropped_kcd_ptr = cropPointCloud(tmp_kcs_ptr, 50.0);
  PointcloudType::Ptr cropped_scan_kcd_ptr = cropPointCloud(keyframe->pointcloud_raw_, 50.0);

  pcl::VoxelGridTriplets<PointType> voxel_grid;
  voxel_grid.setLeafSize(leaf_size_dense_map_, leaf_size_dense_map_, leaf_size_dense_map_);
  voxel_grid.setInputCloud(cropped_kcd_ptr);
  voxel_grid.filter(*subsampled_kcs_ptr);

  RCLCPP_INFO(get_logger(), "keyFrameCallback callback. map points=%lu", subsampled_kcs_ptr->size());

  // Publish the data
  sensor_msgs::msg::PointCloud2 map_msg, scan_msg;
  cropped_scan_kcd_ptr->width = cropped_scan_kcd_ptr->points.size();
  cropped_scan_kcd_ptr->height = 1;
  cropped_kcd_ptr->width = cropped_kcd_ptr->points.size();
  cropped_kcd_ptr->height = 1;
  pcl::toROSMsg(*cropped_kcd_ptr, map_msg);
  pcl::toROSMsg(*cropped_scan_kcd_ptr, scan_msg);

  map_msg.header = keyframe->header_;
  scan_msg.header = keyframe->header_;
  RCLCPP_INFO(get_logger(), "keyFrameCallback callback. pc frame=%s", keyframe->header_.frame_id.c_str());

  keyframe_map_pub_->publish(map_msg);
  keyframe_pub_->publish(scan_msg);

  pcl::io::savePCDFileASCII("keyframe_scan.pcd", *cropped_scan_kcd_ptr);
  pcl::io::savePCDFileASCII("keyframe_map.pcd", *cropped_kcd_ptr);

  RCLCPP_INFO(get_logger(), "keyFrameCallback callback. Published");
}

PointcloudType::Ptr ExtrinsicMappingBasedCalibrator::cropPointCloud(PointcloudType::Ptr & pointcloud, double max_range)
{
  PointcloudType::Ptr tmp_ptr(new PointcloudType());
  tmp_ptr->reserve(pointcloud->size());
  for (const auto & p : pointcloud->points) {

    if (std::sqrt(p.x*p.x + p.y*p.y) < max_range) {
      tmp_ptr->points.push_back(p);
    }
  }

  return tmp_ptr;
}
