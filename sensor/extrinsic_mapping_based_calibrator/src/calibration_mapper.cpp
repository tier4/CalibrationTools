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

#include <extrinsic_mapping_based_calibrator/calibration_mapper.hpp>
#include <extrinsic_mapping_based_calibrator/utils.hpp>
#include <extrinsic_mapping_based_calibrator/voxel_grid_filter_wrapper.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

using namespace std::chrono_literals;

CalibrationMapper::CalibrationMapper(
  MappingParameters::Ptr & parameters, MappingData::Ptr & mapping_data,
  PointPublisher::SharedPtr & map_pub,
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & frame_path_pub,
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & frame_predicted_path_pub,
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & keyframe_path_pub,
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & keyframe_markers_pub,
  rclcpp::Client<rosbag2_interfaces::srv::Pause>::SharedPtr & rosbag2_pause_client,
  rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedPtr & rosbag2_resume_client,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
: parameters_(parameters),
  data_(mapping_data),
  map_pub_(map_pub),
  frame_path_pub_(frame_path_pub),
  frame_predicted_path_pub_(frame_predicted_path_pub),
  keyframe_path_pub_(keyframe_path_pub),
  keyframe_markers_pub_(keyframe_markers_pub),
  rosbag2_pause_client_(rosbag2_pause_client),
  rosbag2_resume_client_(rosbag2_resume_client),
  tf_buffer_(tf_buffer),
  bag_paused_(false),
  bag_pause_requested_(false),
  bag_resume_requested_(false),
  stopped_(false)
{
  published_map_pointcloud_ptr_.reset(new PointcloudType());

  assert(parameters_);
  ndt_.setResolution(parameters_->ndt_resolution_);
  ndt_.setStepSize(parameters_->ndt_step_size_);
  ndt_.setMaximumIterations(parameters_->ndt_max_iterations_);
  ndt_.setTransformationEpsilon(parameters_->ndt_epsilon_);
  ndt_.setNeighborhoodSearchMethod(pclomp::DIRECT7);

  if (parameters_->ndt_num_threads_ > 0) {
    ndt_.setNumThreads(parameters_->ndt_num_threads_);
  }

  for (auto & calibration_frame_name : data_->calibration_lidar_frame_names_) {
    data_->last_unmatched_keyframe_map_[calibration_frame_name] =
      parameters_->calibration_skip_keyframes_;
  }

  for (auto & calibration_frame_name : data_->calibration_camera_optical_link_frame_names) {
    data_->last_unmatched_keyframe_map_[calibration_frame_name] =
      parameters_->calibration_skip_keyframes_;
  }

  std::thread thread = std::thread(&CalibrationMapper::mappingThreadWorker, this);
  thread.detach();
}

bool CalibrationMapper::stopped()
{
  std::unique_lock<std::mutex> lock(data_->mutex_);
  return stopped_;
}

void CalibrationMapper::stop() { stopped_ = true; }

void CalibrationMapper::calibrationCameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg, const std::string & frame_name)
{
  if (stopped_) {
    std::unique_lock<std::mutex> lock(data_->mutex_);
    RCLCPP_WARN(
      rclcpp::get_logger("calibration_mapper"),
      "Reveived a calibration camera info while not mapping. Ignoring it");
    return;
  }

  latest_calibration_camera_infos_map_[frame_name] = msg;
}

void CalibrationMapper::calibrationImageCallback(
  const sensor_msgs::msg::CompressedImage::SharedPtr msg, const std::string & frame_name)
{
  if (stopped_) {
    std::unique_lock<std::mutex> lock(data_->mutex_);
    RCLCPP_WARN(
      rclcpp::get_logger("calibration_mapper"),
      "Reveived a calibration image while not mapping. Ignoring it");
    return;
  }

  calibration_images_list_map_[frame_name].push_back(msg);
}

void CalibrationMapper::calibrationPointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string & frame_name)
{
  if (stopped_) {
    std::unique_lock<std::mutex> lock(data_->mutex_);
    RCLCPP_WARN(
      rclcpp::get_logger("calibration_mapper"),
      "Reveived a calibration pc while not mapping. Ignoring it");
    return;
  }

  calibration_pointclouds_list_map_[frame_name].push_back(msg);
}

void CalibrationMapper::mappingPointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (stopped_) {
    std::unique_lock<std::mutex> lock(data_->mutex_);
    RCLCPP_WARN(
      rclcpp::get_logger("calibration_mapper"),
      "Reveived a mapping pc while not mapping. Ignoring it");
    return;
  }

  if (!mapping_lidar_header_) {
    mapping_lidar_header_ = std::make_shared<std_msgs::msg::Header>(msg->header);
  }

  PointcloudType::Ptr pc_ptr(new PointcloudType());
  pcl::fromROSMsg(*msg, *pc_ptr);

  if (static_cast<int>(pc_ptr->size()) < parameters_->min_pointcloud_size_) {
    RCLCPP_WARN(
      rclcpp::get_logger("calibration_mapper"),
      "Received a unusually small  mapping pc (size=%lu). Skipping it", pc_ptr->size());
    return;
  }

  transformPointcloud<PointcloudType>(
    msg->header.frame_id, data_->mapping_lidar_frame_, pc_ptr, *tf_buffer_);

  std::unique_lock<std::mutex> lock(data_->mutex_);
  auto frame = std::make_shared<Frame>();
  frame->header_ = msg->header;
  frame->pointcloud_raw_ = pc_ptr;

  if (
    rclcpp::Time(msg->header.stamp) < rclcpp::Time(mapping_lidar_header_->stamp) ||
    static_cast<int>(data_->processed_frames_.size()) >= parameters_->mapping_max_frames_) {
    stop();
    RCLCPP_WARN(
      rclcpp::get_logger("calibration_mapper"),
      "Stopping mapper due to enough frames being collected");
    return;
  }

  if (
    parameters_->use_rosbag_ && !bag_paused_ && !bag_pause_requested_ &&
    data_->unprocessed_frames_.size() > 0) {
    auto cb = [&](rclcpp::Client<rosbag2_interfaces::srv::Pause>::SharedFuture response_client) {
      auto res = response_client.get();
      (void)res;
      std::unique_lock<std::mutex> lock(data_->mutex_);
      bag_paused_ = true;
      bag_pause_requested_ = false;
    };
    auto request = std::make_shared<rosbag2_interfaces::srv::Pause::Request>();
    rosbag2_pause_client_->async_send_request(request, cb);
    bag_pause_requested_ = true;
  }

  data_->unprocessed_frames_.emplace_back(frame);
  RCLCPP_INFO(
    rclcpp::get_logger("calibration_mapper"),
    "ROS: New pointcloud. Unprocessed=%lu Frames=%lu Keyframes=%lu",
    data_->unprocessed_frames_.size(), data_->processed_frames_.size(), data_->keyframes_.size());
}

void CalibrationMapper::mappingThreadWorker()
{
  Eigen::Matrix4f last_pose =
    Eigen::Matrix4f::Identity();  // not necessarily assciated with a frame
  builtin_interfaces::msg::Time last_stamp;

  while (rclcpp::ok() && !stopped_) {
    Frame::Ptr frame, prev_frame, prev_frame_not_last;

    float prev_distance = 0.f;

    // Locked section
    {
      std::unique_lock<std::mutex> lock(data_->mutex_);

      if (data_->unprocessed_frames_.size() == 0) {
        if (parameters_->use_rosbag_ && bag_paused_ && !bag_resume_requested_) {
          auto cb =
            [&](rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedFuture response_client) {
              auto res = response_client.get();
              (void)res;
              std::unique_lock<std::mutex> lock(data_->mutex_);
              bag_paused_ = false;
              bag_resume_requested_ = false;
              RCLCPP_WARN(rclcpp::get_logger("calibration_mapper"), "Received resume response");
            };

          RCLCPP_WARN(rclcpp::get_logger("calibration_mapper"), "Sending resume call");
          auto request = std::make_shared<rosbag2_interfaces::srv::Resume::Request>();
          rosbag2_resume_client_->async_send_request(request, cb);
          bag_resume_requested_ = true;
        }

        lock.unlock();
        rclcpp::sleep_for(10ms);
        continue;
      }

      frame = data_->unprocessed_frames_.front();
      data_->unprocessed_frames_.pop_front();

      if (data_->processed_frames_.size() > 0) {
        prev_frame = data_->processed_frames_.back();
        prev_frame_not_last = data_->processed_frames_.back();
        prev_distance = prev_frame->distance_;

        for (auto it = data_->processed_frames_.rbegin(); it != data_->processed_frames_.rend();
             it++) {
          if ((*it)->header_.stamp != last_stamp) {
            prev_frame_not_last = *it;
            break;
          }
        }
      }
    }

    if (!prev_frame) {
      initLocalMap(frame);
    }

    VoxelGridWrapper<PointType> voxel_grid;
    frame->pointcloud_subsampled_ = PointcloudType::Ptr(new PointcloudType());
    PointcloudType::Ptr aligned_cloud_ptr(new PointcloudType());
    PointcloudType::Ptr cropped_cloud_ptr =
      cropPointCloud<PointcloudType>(frame->pointcloud_raw_, parameters_->mapping_max_range_);

    voxel_grid.setLeafSize(
      parameters_->leaf_size_input_, parameters_->leaf_size_input_, parameters_->leaf_size_input_);
    voxel_grid.setInputCloud(cropped_cloud_ptr);
    voxel_grid.filter(*frame->pointcloud_subsampled_);

    // Register the frame to the map
    ndt_.setInputTarget(data_->local_map_ptr_);
    ndt_.setInputSource(frame->pointcloud_subsampled_);

    Eigen::Matrix4f guess = last_pose;
    double dt_since_last =
      (rclcpp::Time(frame->header_.stamp) - rclcpp::Time(last_stamp)).seconds();

    if (!prev_frame) {
      frame->pose_ = Eigen::Matrix4f::Identity();
      last_stamp = frame->header_.stamp;
    } else {
      if (prev_frame->lost_) {
        RCLCPP_WARN(
          rclcpp::get_logger("calibration_mapper"), "Using last pose as guess since it is lost");
        guess = last_pose;
      }
      if (prev_frame->stopped_) {
        RCLCPP_INFO(
          rclcpp::get_logger("calibration_mapper"), "Using last pose as guess since it is stopped");
        guess = last_pose;
      } else if (prev_frame->dt_ > 0) {
        // We can improve the guess with extrapolation
        double dt1 =
          (rclcpp::Time(last_stamp) - rclcpp::Time(prev_frame_not_last->header_.stamp)).seconds();
        double dt2 =
          (rclcpp::Time(frame->header_.stamp) - rclcpp::Time(prev_frame_not_last->header_.stamp))
            .seconds();
        Eigen::Matrix4f interpolated_pose = poseInterpolation(
          dt2, 0, dt1, Eigen::Matrix4f::Identity(),
          prev_frame_not_last->pose_.inverse() * last_pose);
        guess = prev_frame_not_last->pose_ * interpolated_pose;
      }

      ndt_.align(*aligned_cloud_ptr, guess);
      last_pose = ndt_.getFinalTransformation();

      float innovation = Eigen::Affine3f(guess.inverse() * last_pose).translation().norm();
      float score = ndt_.getFitnessScore();

      RCLCPP_INFO(
        rclcpp::get_logger("calibration_mapper"), "NDT Innovation=%.2f. Score=%.2f", innovation,
        score);
      last_stamp = frame->header_.stamp;
    }

    std::unique_lock<std::mutex> lock(data_->mutex_);

    // Fill the frame information
    frame->pose_ = last_pose;
    frame->predicted_pose_ = guess;
    frame->frame_id_ = data_->processed_frames_.size();
    frame->processed_ = true;
    frame->distance_ = prev_distance;

    if (prev_frame) {
      frame->dt_ =
        (rclcpp::Time(frame->header_.stamp) - rclcpp::Time(prev_frame->header_.stamp)).seconds();
      frame->delta_translation_ =
        Eigen::Affine3f(prev_frame->pose_.inverse() * last_pose).translation();
      frame->distance_ += frame->delta_translation_.norm();
      frame->rough_speed_ = Eigen::Vector3f(frame->delta_translation_) / frame->dt_;
    }

    // We record the whole trajectory
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = frame->header_;
    pose_msg.header.frame_id = data_->map_frame_;
    pose_msg.pose = tf2::toMsg(Eigen::Affine3d(frame->pose_.cast<double>()));
    data_->trajectory_.push_back(pose_msg);

    if (!checkFrameLost(prev_frame, frame, dt_since_last) && shouldDropFrame(prev_frame, frame)) {
      continue;
    }

    data_->processed_frames_.push_back(frame);
    checkKeyframe(frame);

    RCLCPP_INFO(
      rclcpp::get_logger("calibration_mapper"),
      "New frame (id=%d | kid=%d). Distance=%.2f Delta_distance%.2f Delta_time%.2f. "
      "Unprocessed=%lu Frames=%lu Keyframes=%lu",
      frame->frame_id_, frame->keyframe_id_, frame->distance_, frame->delta_translation_.norm(),
      frame->dt_, data_->unprocessed_frames_.size(), data_->processed_frames_.size(),
      data_->keyframes_.size());
  }

  RCLCPP_WARN(rclcpp::get_logger("calibration_mapper"), "Mapping thread is exiting");
}

void CalibrationMapper::initLocalMap(Frame::Ptr frame)
{
  data_->local_map_ptr_.reset(new PointcloudType());
  PointcloudType::Ptr cropped_cloud_ptr =
    cropPointCloud<PointcloudType>(frame->pointcloud_raw_, parameters_->mapping_max_range_);
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setLeafSize(
    parameters_->leaf_size_local_map_, parameters_->leaf_size_local_map_,
    parameters_->leaf_size_local_map_);
  voxel_grid.setInputCloud(cropped_cloud_ptr);
  voxel_grid.filter(*data_->local_map_ptr_);
}

void CalibrationMapper::checkKeyframe(Frame::Ptr frame)
{
  if (
    data_->keyframes_.size() == 0 ||
    frame->distance_ >=
      data_->keyframes_.back()->distance_ + parameters_->new_keyframe_min_distance_ ||
    frame->lost_) {
    data_->keyframes_.push_back(frame);
    data_->keyframes_and_stopped_.push_back(frame);
    frame->is_key_frame_ = true;
    frame->keyframe_id_ = data_->keyframes_.size();
    checkKeyframeLost(frame);
    recalculateLocalMap();
  }
}

void CalibrationMapper::checkKeyframeLost(Frame::Ptr keyframe)
{
  assert(keyframe->is_key_frame_);

  if (keyframe->keyframe_id_ <= 1) {
    return;
  }

  const Frame::Ptr & two_left_frame = data_->keyframes_[keyframe->keyframe_id_ - 2];
  const Frame::Ptr & left_frame = data_->keyframes_[keyframe->keyframe_id_ - 1];
  Frame::Ptr & right_frame = keyframe;

  Eigen::Affine3f dpose1(two_left_frame->pose_.inverse() * left_frame->pose_);
  Eigen::Affine3f dpose2(left_frame->pose_.inverse() * right_frame->pose_);
  Eigen::Affine3f left_pose(left_frame->pose_);
  Eigen::Affine3f right_pose(right_frame->pose_);

  auto d1 = dpose1.translation().normalized();
  auto d2 = dpose2.translation().normalized();

  float trans_angle_diff = (180.0 / M_PI) * std::acos(d1.dot(d2));
  trans_angle_diff =
    dpose2.translation().norm() > parameters_->new_keyframe_min_distance_ ? trans_angle_diff : 0.0;

  float rot_angle_diff =
    (180.0 / M_PI) * std::acos(std::min(
                       1.0, 0.5 * ((dpose1.rotation().inverse() * dpose2.rotation()).trace() -
                                   1.0)));  // Tr(R) = 1 + 2*cos(theta)

  if (
    std::abs(trans_angle_diff) > parameters_->lost_frame_max_angle_diff_ ||
    std::abs(trans_angle_diff) > parameters_->lost_frame_max_angle_diff_) {
    keyframe->lost_ = true;

    RCLCPP_WARN(
      rclcpp::get_logger("calibration_mapper"),
      "Mapping failed. Angle between keyframes is too high. a1=%.2f (deg) a2=%.2f (deg) "
      "theshold=%.2f (deg)",
      trans_angle_diff, rot_angle_diff, parameters_->lost_frame_max_angle_diff_);

    return;
  }

  for (int i = left_frame->frame_id_; i < right_frame->frame_id_; i++) {
    Frame::Ptr & mid_frame = data_->processed_frames_[i];

    // Interpolate the pose
    Eigen::Matrix4f interpolated_pose_matrix = poseInterpolation(
      (rclcpp::Time(mid_frame->header_.stamp) - rclcpp::Time(left_frame->header_.stamp)).seconds(),
      0.f,
      (rclcpp::Time(right_frame->header_.stamp) - rclcpp::Time(left_frame->header_.stamp))
        .seconds(),
      left_frame->pose_, right_frame->pose_);

    Eigen::Affine3f interpolated_pose(interpolated_pose_matrix);
    Eigen::Affine3f frame_pose(mid_frame->pose_);

    float trans_diff = (interpolated_pose.inverse() * frame_pose).translation().norm();
    float rot_angle_diff =
      (180.0 / M_PI) *
      std::acos(std::min(
        1.0, 0.5 * ((interpolated_pose.rotation().inverse() * frame_pose.rotation()).trace() -
                    1.0)));  // Tr(R) = 1 + 2*cos(theta)

    if (
      (!left_frame->stopped_ && !right_frame->stopped_) &&
      (trans_diff > parameters_->lost_frame_interpolation_error_ ||
       std::abs(rot_angle_diff) > parameters_->lost_frame_max_angle_diff_)) {
      keyframe->lost_ = true;

      RCLCPP_WARN(
        rclcpp::get_logger("calibration_mapper"),
        "Mapping failed. Interpolation error is too high. d=%.2f (m) a=%.2f (deg)", trans_diff,
        rot_angle_diff);

      return;
    }

    float frame_left_distance = (frame_pose.translation() - left_pose.translation()).norm();
    float interpolated_left_distance =
      (interpolated_pose.translation() - left_pose.translation()).norm();
    float line_dist_error = std::sqrt(
      frame_left_distance * frame_left_distance +
      interpolated_left_distance * interpolated_left_distance);

    if (line_dist_error > parameters_->lost_frame_interpolation_error_) {
      keyframe->lost_ = true;

      RCLCPP_WARN(
        rclcpp::get_logger("calibration_mapper"),
        "Mapping failed. Interpolation error is too high. line_dist_error=%.2f (m)",
        line_dist_error);

      return;
    }
  }
}

bool CalibrationMapper::checkFrameLost(const Frame::Ptr & prev_frame, Frame::Ptr & frame, float dt)
{
  if (!prev_frame || prev_frame->lost_) {
    return false;
  }

  if (dt >= parameters_->mapping_lost_timeout_) {
    RCLCPP_WARN(
      rclcpp::get_logger("calibration_mapper"), "Mapping failed. sensor timeout dt=%.2fs", dt);
    frame->lost_ = true;
  }

  float acceleration = (frame->rough_speed_ - prev_frame->rough_speed_).norm() / frame->dt_;

  if (acceleration > parameters_->lost_frame_max_acceleration_) {
    RCLCPP_WARN(
      rclcpp::get_logger("calibration_mapper"),
      "Mapping failed. Acceleration is too high. acc=%.2f (m/s2)", acceleration);
    frame->lost_ = true;
  }

  return frame->lost_;
}

bool CalibrationMapper::shouldDropFrame(const Frame::Ptr & prev_frame, Frame::Ptr & frame)
{
  if (!prev_frame || frame->delta_translation_.norm() > parameters_->new_frame_min_distance_) {
    return false;
  }

  if (
    std::abs(frame->delta_translation_.norm() - prev_frame->aux_delta_translation_.norm()) <
    parameters_->frame_stopped_distance_) {
    // When the vehicle is stopped, we may either skip the frame, record it as a normal frame,
    // or even save it for calibration

    prev_frame->frames_since_stop_ += 1;
    prev_frame->aux_delta_translation_ = frame->delta_translation_;

    frame->frames_since_stop_ = prev_frame->frames_since_stop_;
    frame->stopped_ = true;

    if (prev_frame->frames_since_stop_ == parameters_->frames_since_stop_force_frame_) {
      RCLCPP_WARN(rclcpp::get_logger("calibration_mapper"), "Added a keyframe_and_stopped frame");
      data_->keyframes_and_stopped_.push_back(frame);
      return false;
    } else if (
      prev_frame->stopped_ &&
      std::abs(prev_frame->frames_since_stop_ - parameters_->frames_since_stop_force_frame_) > 1) {
      RCLCPP_INFO(rclcpp::get_logger("calibration_mapper"), "Dropped stopped frame");
      return true;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("calibration_mapper"),
    "Dropped frame. Unprocessed=%lu Frames=%lu Keyframes=%lu", data_->unprocessed_frames_.size(),
    data_->processed_frames_.size(), data_->keyframes_.size());

  return true;
}

void CalibrationMapper::recalculateLocalMap()
{
  data_->local_map_ptr_->clear();

  PointcloudType::Ptr tmp_mcs_ptr(new PointcloudType());

  for (int i = 0;
       i < parameters_->local_map_num_keyframes_ && i < static_cast<int>(data_->keyframes_.size());
       i++) {
    const auto & keyframe = data_->keyframes_[data_->keyframes_.size() - i - 1];

    if (keyframe->lost_ && i != 0) {
      break;
    }

    PointcloudType::Ptr keyframe_mcs_ptr(new PointcloudType());
    pcl::transformPointCloud(*keyframe->pointcloud_subsampled_, *keyframe_mcs_ptr, keyframe->pose_);
    *tmp_mcs_ptr += *keyframe_mcs_ptr;
  }

  VoxelGridWrapper<PointType> voxel_grid;
  voxel_grid.setLeafSize(
    parameters_->leaf_size_local_map_, parameters_->leaf_size_local_map_,
    parameters_->leaf_size_local_map_);
  voxel_grid.setInputCloud(tmp_mcs_ptr);
  voxel_grid.filter(*data_->local_map_ptr_);
}

#pragma GCC push_options
#pragma GCC optimize("O0")

void CalibrationMapper::publisherTimerCallback()
{
  static int published_frames = 0;
  static int published_keyframes = 0;

  std::unique_lock<std::mutex> lock(data_->mutex_);

  if (static_cast<int>(data_->keyframes_.size()) == published_keyframes) {
    return;
  }

  auto processed_frames_it = data_->processed_frames_.begin() + published_keyframes;

  while (processed_frames_it != data_->processed_frames_.end()) {
    // Process the new keyframes into the map
    PointcloudType::Ptr tmp_mcs_ptr(new PointcloudType());
    PointcloudType::Ptr subsampled_mcs_ptr(new PointcloudType());
    *tmp_mcs_ptr += *published_map_pointcloud_ptr_;

    for (int i = 0; i < 100;
         i++) {  // arbitrary number since large unprocessed frames lead to gigantic memory usages
      Frame::Ptr frame = *processed_frames_it;
      PointcloudType::Ptr frame_mcs_ptr(new PointcloudType());
      pcl::transformPointCloud(*frame->pointcloud_subsampled_, *frame_mcs_ptr, frame->pose_);
      *tmp_mcs_ptr += *frame_mcs_ptr;

      processed_frames_it++;
      if (processed_frames_it == data_->processed_frames_.end()) {
        break;
      }
    }

    pcl::transformPointCloud(
      *tmp_mcs_ptr, *tmp_mcs_ptr, data_->processed_frames_.back()->pose_.inverse());
    tmp_mcs_ptr = cropPointCloud<PointcloudType>(tmp_mcs_ptr, parameters_->viz_max_range_);
    pcl::transformPointCloud(*tmp_mcs_ptr, *tmp_mcs_ptr, data_->processed_frames_.back()->pose_);

    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setLeafSize(
      parameters_->mapping_viz_leaf_size_, parameters_->mapping_viz_leaf_size_,
      parameters_->mapping_viz_leaf_size_);
    voxel_grid.setInputCloud(tmp_mcs_ptr);
    voxel_grid.filter(*subsampled_mcs_ptr);

    published_map_pointcloud_ptr_.swap(subsampled_mcs_ptr);
  }

  // Process the new keyframes and frames into the paths
  for (auto it = data_->keyframes_.begin() + published_keyframes; it != data_->keyframes_.end();
       ++it) {
    Frame::Ptr keyframe = *it;
    Eigen::Matrix4d pose_matrix = keyframe->pose_.cast<double>();
    Eigen::Isometry3d pose_isometry(pose_matrix);
    geometry_msgs::msg::PoseStamped pose_msg;
    visualization_msgs::msg::Marker keyframe_marker;
    pose_msg.header = keyframe->header_;
    pose_msg.header.frame_id = data_->map_frame_;
    pose_msg.pose = tf2::toMsg(pose_isometry);
    published_keyframes_path_.poses.push_back(pose_msg);

    keyframe_marker.header = keyframe->header_;
    keyframe_marker.header.frame_id = data_->map_frame_;
    keyframe_marker.ns = "keyframe_id";
    keyframe_marker.id = keyframe->keyframe_id_;
    keyframe_marker.color.r = 1.0;
    keyframe_marker.color.g = keyframe->lost_ ? 0.0 : 1.0;
    keyframe_marker.color.b = keyframe->lost_ ? 0.0 : 1.0;
    keyframe_marker.color.a = 1.0;
    keyframe_marker.scale.x = keyframe->lost_ ? 0.06 : 0.03;

    keyframe_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    keyframe_marker.text = std::to_string(keyframe->keyframe_id_);
    keyframe_marker.scale.z = 0.3;
    keyframe_marker.pose = pose_msg.pose;
    keyframe_marker.pose.position.z += 0.1;
    published_keyframes_markers_.markers.push_back(keyframe_marker);
  }

  for (auto it = data_->processed_frames_.begin() + published_frames;
       it != data_->processed_frames_.end(); ++it) {
    Frame::Ptr frame = *it;
    Eigen::Isometry3d pose_isometry(frame->pose_.cast<double>());
    Eigen::Isometry3d predicted_pose_isometry(frame->predicted_pose_.cast<double>());
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = frame->header_;
    pose_msg.header.frame_id = data_->map_frame_;
    pose_msg.pose = tf2::toMsg(pose_isometry);
    published_frames_path_.poses.push_back(pose_msg);

    pose_msg.pose = tf2::toMsg(predicted_pose_isometry);

    if (frame->predicted_pose_.determinant() > 0.0) {
      published_frames_predicted_path_.poses.push_back(pose_msg);
    }

    visualization_msgs::msg::Marker frame_marker;

    frame_marker.header = frame->header_;
    frame_marker.header.frame_id = data_->map_frame_;
    frame_marker.ns = "frame_id";
    frame_marker.id = frame->frame_id_;
    frame_marker.color.r = 1.0;
    frame_marker.color.g = frame->lost_ ? 0.0 : 1.0;
    frame_marker.color.b = 1.0;
    frame_marker.color.a = 1.0;
    frame_marker.scale.x = frame->lost_ ? 0.03 : 0.005;

    frame_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    frame_marker.text = std::to_string(frame->frame_id_);
    frame_marker.scale.z = frame->lost_ ? 0.1 : 0.03;
    frame_marker.pose = pose_msg.pose;
    frame_marker.pose.position.z += 0.1;
    published_keyframes_markers_.markers.push_back(frame_marker);
  }

  published_frames = data_->processed_frames_.size();
  published_keyframes = data_->keyframes_.size();

  // Publish the data
  sensor_msgs::msg::PointCloud2 published_map_msg;
  pcl::toROSMsg(*published_map_pointcloud_ptr_, published_map_msg);

  if (mapping_lidar_header_) {
    published_map_msg.header.stamp = mapping_lidar_header_->stamp;
    published_keyframes_path_.header.stamp = mapping_lidar_header_->stamp;
    published_frames_path_.header.stamp = mapping_lidar_header_->stamp;
    published_frames_predicted_path_.header.stamp = mapping_lidar_header_->stamp;
  }

  published_map_msg.header.frame_id = data_->map_frame_;
  published_keyframes_path_.header.frame_id = data_->map_frame_;
  published_frames_path_.header.frame_id = data_->map_frame_;
  published_frames_predicted_path_.header.frame_id = data_->map_frame_;

  map_pub_->publish(published_map_msg);
  keyframe_path_pub_->publish(published_keyframes_path_);
  frame_path_pub_->publish(published_frames_path_);
  frame_predicted_path_pub_->publish(published_frames_predicted_path_);
  keyframe_markers_pub_->publish(published_keyframes_markers_);

  return;
}

#pragma GCC pop_options

void CalibrationMapper::dataMatchingTimerCallback()
{
  for (const auto & frame_name : data_->calibration_camera_optical_link_frame_names) {
    mappingCalibrationDatamatching<sensor_msgs::msg::CompressedImage>(
      frame_name, calibration_images_list_map_[frame_name],
      std::bind(
        &CalibrationMapper::addNewCameraCalibrationFrame, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3));
  }

  for (const auto & frame_name : data_->calibration_lidar_frame_names_) {
    mappingCalibrationDatamatching<sensor_msgs::msg::PointCloud2>(
      frame_name, calibration_pointclouds_list_map_[frame_name],
      std::bind(
        &CalibrationMapper::addNewLidarCalibrationFrame, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3));
  }
}

template <class MsgType>
void CalibrationMapper::mappingCalibrationDatamatching(
  const std::string & calibration_frame_name,
  std::list<typename MsgType::SharedPtr> & calibration_msg_list,
  std::function<bool(const std::string &, typename MsgType::SharedPtr &, CalibrationFrame &)>
    add_frame_function)
{
  std::unique_lock<std::mutex> lock(data_->mutex_);
  auto & last_unmatched_keyframe = data_->last_unmatched_keyframe_map_[calibration_frame_name];

  while (last_unmatched_keyframe < static_cast<int>(data_->keyframes_and_stopped_.size()) - 1) {
    Frame::Ptr keyframe = data_->keyframes_and_stopped_[last_unmatched_keyframe];
    auto keyframe_stamp = rclcpp::Time(keyframe->header_.stamp);

    if (
      calibration_msg_list.size() < 3 ||
      rclcpp::Time(calibration_msg_list.back()->header.stamp) < keyframe_stamp) {
      return;
    }

    // We need there to be a frame after the keyframe since we may wanto to interpolate in that
    // direction
    if (keyframe->frame_id_ + 1 >= static_cast<int>(data_->processed_frames_.size())) {
      return;
    }

    // Iterate for all frames between the last keyframe and this one looking for a stopped frame,
    // which we want to use as a calibration frame
    auto msg_left = calibration_msg_list.front();
    calibration_msg_list.pop_front();
    auto msg_right = calibration_msg_list.front();

    while (rclcpp::Time(msg_right->header.stamp) < keyframe_stamp) {
      msg_left = msg_right;
      calibration_msg_list.pop_front();
      msg_right = calibration_msg_list.front();
    }

    double dt_left = (keyframe_stamp - rclcpp::Time(msg_left->header.stamp)).seconds();
    double dt_right = (rclcpp::Time(msg_right->header.stamp) - keyframe_stamp).seconds();

    typename MsgType::SharedPtr msg;
    Frame::Ptr frame_left, frame_right, frame_aux;
    double interpolated_distance;
    double interpolated_time;
    double interpolated_speed;
    double interpolated_accel;

    // Compute first and second order derivates with finite differences
    if (dt_left < dt_right) {
      msg = msg_left;

      frame_left = data_->processed_frames_[keyframe->frame_id_ - 1];
      frame_right = keyframe;
      frame_aux = data_->processed_frames_[keyframe->frame_id_ + 1];

      double dt1 =
        (rclcpp::Time(frame_right->header_.stamp) - rclcpp::Time(frame_left->header_.stamp))
          .seconds();
      double dt2 =
        (rclcpp::Time(frame_aux->header_.stamp) - rclcpp::Time(frame_right->header_.stamp))
          .seconds();
      interpolated_speed = std::abs((frame_aux->distance_ - frame_left->distance_) / (dt1 + dt2));
      interpolated_accel = 2 * std::abs(
                                 ((frame_aux->distance_ - frame_right->distance_) / dt2 -
                                  (frame_right->distance_ - frame_left->distance_) / dt1) /
                                 (dt1 + dt2));

      interpolated_time = dt_left;
    } else {
      msg = msg_right;

      frame_aux = data_->processed_frames_[keyframe->frame_id_ - 1];
      frame_left = keyframe;
      frame_right = data_->processed_frames_[keyframe->frame_id_ + 1];

      double dt1 =
        (rclcpp::Time(frame_left->header_.stamp) - rclcpp::Time(frame_aux->header_.stamp))
          .seconds();
      double dt2 =
        (rclcpp::Time(frame_right->header_.stamp) - rclcpp::Time(frame_left->header_.stamp))
          .seconds();
      interpolated_speed = std::abs((frame_right->distance_ - frame_aux->distance_) / (dt1 + dt2));
      interpolated_accel = std::abs(
        ((frame_right->distance_ - frame_left->distance_) / dt2 -
         (frame_left->distance_ - frame_aux->distance_) / dt1) /
        (dt1 + dt2));

      interpolated_time = dt_right;
    }

    interpolated_distance =
      (frame_right->distance_ - frame_left->distance_) * interpolated_time /
      (rclcpp::Time(frame_right->header_.stamp) - rclcpp::Time(frame_left->header_.stamp))
        .seconds();

    // Interpolate the pose
    Eigen::Matrix4f interpolated_pose = poseInterpolation(
      (rclcpp::Time(msg->header.stamp) - rclcpp::Time(frame_left->header_.stamp)).seconds(), 0.f,
      (rclcpp::Time(frame_right->header_.stamp) - rclcpp::Time(frame_left->header_.stamp))
        .seconds(),
      frame_left->pose_, frame_right->pose_);

    // Compute interpolation angle
    // trR = 1 + 2cos(theta)
    Eigen::Matrix4f aux_pose = keyframe->pose_.inverse() * interpolated_pose;
    Eigen::Affine3f aux_affine(aux_pose);
    double interpolated_angle =
      180.0 *
      std::abs(std::acos(std::min(std::max(0.5 * aux_affine.linear().trace() - 0.5, -1.0), 1.0))) /
      3.1416;

    CalibrationFrame calibration_frame;

    calibration_frame.source_header_ = msg->header;
    calibration_frame.source_header_.frame_id = calibration_frame_name;

    calibration_frame.target_frame_ = keyframe;
    calibration_frame.local_map_pose_ = interpolated_pose;

    calibration_frame.interpolated_distance_ = interpolated_distance;
    calibration_frame.interpolated_angle_ = interpolated_angle;
    calibration_frame.interpolated_time_ = interpolated_time;
    calibration_frame.estimated_speed_ = interpolated_speed;
    calibration_frame.estimated_accel_ = interpolated_accel;
    calibration_frame.stopped_ = keyframe->stopped_;

    add_frame_function(calibration_frame_name, msg, calibration_frame);

    last_unmatched_keyframe += 1;
  }
}

bool CalibrationMapper::addNewCameraCalibrationFrame(
  const std::string & calibration_frame_name, sensor_msgs::msg::CompressedImage::SharedPtr & msg,
  CalibrationFrame & calibration_frame)
{
  calibration_frame.source_camera_info =
    latest_calibration_camera_infos_map_[calibration_frame_name];
  calibration_frame.source_image = msg;
  data_->camera_calibration_frames_map_[calibration_frame_name].emplace_back(calibration_frame);

  return true;
}

bool CalibrationMapper::addNewLidarCalibrationFrame(
  const std::string & calibration_frame_name, sensor_msgs::msg::PointCloud2::SharedPtr & msg,
  CalibrationFrame & calibration_frame)
{
  PointcloudType::Ptr pc_ptr(new PointcloudType());
  pcl::fromROSMsg(*msg, *pc_ptr);
  transformPointcloud<PointcloudType>(
    msg->header.frame_id, calibration_frame_name, pc_ptr, *tf_buffer_);

  calibration_frame.source_pointcloud_ = pc_ptr;

  if (static_cast<int>(pc_ptr->size()) >= parameters_->min_pointcloud_size_) {
    data_->lidar_calibration_frames_map_[calibration_frame_name].emplace_back(calibration_frame);
    return true;
  }

  return false;
}

template void CalibrationMapper::mappingCalibrationDatamatching<sensor_msgs::msg::CompressedImage>(
  const std::string & calibration_frame,
  std::list<sensor_msgs::msg::CompressedImage::SharedPtr> & calibration_msg_list,
  std::function<
    bool(const std::string &, sensor_msgs::msg::CompressedImage::SharedPtr &, CalibrationFrame &)>
    add_frame_function);
template void CalibrationMapper::mappingCalibrationDatamatching<sensor_msgs::msg::PointCloud2>(
  const std::string & calibration_frame,
  std::list<sensor_msgs::msg::PointCloud2::SharedPtr> & calibration_msg_list,
  std::function<
    bool(const std::string &, sensor_msgs::msg::PointCloud2::SharedPtr &, CalibrationFrame &)>
    add_frame_function);
