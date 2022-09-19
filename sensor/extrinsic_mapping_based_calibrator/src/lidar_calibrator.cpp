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

#include <extrinsic_mapping_based_calibrator/lidar_calibrator.hpp>
#include <extrinsic_mapping_based_calibrator/utils.hpp>
#include <extrinsic_mapping_based_calibrator/voxel_grid_filter_wrapper.hpp>

#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <pcl/common/pca.h>

#define UNUSED(x) (void)x;

LidarCalibrator::LidarCalibrator(
  const std::string & calibration_lidar_frame, LidarCalibrationParameters::Ptr & parameters,
  MappingData::Ptr & mapping_data, std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  PointPublisher::SharedPtr & initial_source_aligned_map_pub,
  PointPublisher::SharedPtr & calibrated_source_aligned_map_pub,
  PointPublisher::SharedPtr & target_map_pub)
: calibration_lidar_frame_(calibration_lidar_frame),
  calibrator_name_("lidar_calibrator(" + calibration_lidar_frame + ")"),
  parameters_(parameters),
  data_(mapping_data),
  tf_buffer_(tf_buffer),
  initial_source_aligned_map_pub_(initial_source_aligned_map_pub),
  calibrated_source_aligned_map_pub_(calibrated_source_aligned_map_pub),
  target_map_pub_(target_map_pub)
{
  // Calibration configuration
  correspondence_estimator_ =
    pcl::make_shared<pcl::registration::CorrespondenceEstimation<PointType, PointType>>();

  calibration_ndt_ = pcl::make_shared<pclomp::NormalDistributionsTransform<PointType, PointType>>();
  calibration_gicp_ =
    pcl::make_shared<pcl::GeneralizedIterativeClosestPoint<PointType, PointType>>();
  calibration_icp_coarse_ = pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();
  calibration_icp_fine_ = pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();
  calibration_icp_ultrafine_ = pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();

  calibration_registrators_ = {
    calibration_ndt_, calibration_gicp_, calibration_icp_coarse_, calibration_icp_fine_,
    calibration_icp_ultrafine_};

  calibration_batch_icp_coarse_ =
    pcl::make_shared<pcl::JointIterativeClosestPointExtended<PointType, PointType>>();
  calibration_batch_icp_fine_ =
    pcl::make_shared<pcl::JointIterativeClosestPointExtended<PointType, PointType>>();
  calibration_batch_icp_ultrafine_ =
    pcl::make_shared<pcl::JointIterativeClosestPointExtended<PointType, PointType>>();
  calibration_batch_registrators_ = {
    calibration_batch_icp_coarse_, calibration_batch_icp_fine_, calibration_batch_icp_ultrafine_};

  configureCalibrators();
}

void LidarCalibrator::configureCalibrators()
{
  calibration_gicp_->setMaxCorrespondenceDistance(parameters_->max_corr_dist_coarse_);
  calibration_icp_coarse_->setMaxCorrespondenceDistance(parameters_->max_corr_dist_coarse_);
  calibration_icp_fine_->setMaxCorrespondenceDistance(parameters_->max_corr_dist_fine_);
  calibration_icp_ultrafine_->setMaxCorrespondenceDistance(parameters_->max_corr_dist_ultrafine_);

  for (auto & calibrator : calibration_registrators_) {
    calibrator->setMaximumIterations(parameters_->solver_iterations_);
  }

  calibration_batch_icp_coarse_->setMaxCorrespondenceDistance(parameters_->max_corr_dist_coarse_);
  calibration_batch_icp_fine_->setMaxCorrespondenceDistance(parameters_->max_corr_dist_fine_);
  calibration_batch_icp_ultrafine_->setMaxCorrespondenceDistance(
    parameters_->max_corr_dist_ultrafine_);

  for (auto & calibrator : calibration_batch_registrators_) {
    calibrator->setMaximumIterations(parameters_->solver_iterations_);
  }
}

void LidarCalibrator::setUpCalibrators(
  PointcloudType::Ptr & source_pointcloud_ptr, PointcloudType::Ptr & target_pointcloud_ptr)
{
  for (auto & calibrator : calibration_registrators_) {
    calibrator->setInputSource(source_pointcloud_ptr);
    calibrator->setInputTarget(target_pointcloud_ptr);
  }
}

PointcloudType::Ptr LidarCalibrator::getDensePointcloudFromMap(
  const Eigen::Matrix4f & pose, Frame::Ptr & frame, double resolution, double max_range)
{
  int frame_id = frame->frame_id_;

  // Find the closest keyframe to the requested keyframe
  Frame::Ptr keyframe_left, keyframe_right, keyframe;

  for (auto it = data_->processed_frames_.begin() + frame_id;
       it != data_->processed_frames_.begin(); it--) {
    if ((*it)->is_key_frame_) {
      keyframe_left = *it;
      break;
    }
  }

  for (auto it = data_->processed_frames_.begin() + frame_id; it != data_->processed_frames_.end();
       it++) {
    if ((*it)->is_key_frame_) {
      keyframe_right = *it;
      break;
    }
  }

  if (keyframe_left && keyframe_right) {
    keyframe =
      (keyframe_right->frame_id_ - frame->frame_id_ < frame->frame_id_ - keyframe_left->frame_id_)
        ? keyframe_right
        : keyframe_left;
  } else if (keyframe_left) {
    keyframe = keyframe_left;
  } else if (keyframe_right) {
    keyframe = keyframe_right;
  } else {
    assert(false);
  }

  int min_keyframe_id =
    std::max<int>(0, keyframe->keyframe_id_ - parameters_->dense_pointcloud_num_keyframes_);
  int max_keyframe_id = std::min<int>(
    data_->keyframes_.size() - 1,
    keyframe->keyframe_id_ + parameters_->dense_pointcloud_num_keyframes_);

  int min_frame_id = data_->keyframes_[min_keyframe_id]->frame_id_;
  int max_frame_id = data_->keyframes_[max_keyframe_id]->frame_id_;

  auto target_map_pose = pose.inverse();

  // Sum all frames in the target coordinate system (tcs)
  PointcloudType::Ptr tmp_tcs_ptr(new PointcloudType());
  PointcloudType::Ptr subsampled_tcs_ptr(new PointcloudType());

  for (int i = min_frame_id; i <= max_frame_id; i++) {
    Frame::Ptr frame = data_->processed_frames_[i];
    PointcloudType::Ptr frame_tcs_ptr(new PointcloudType());

    auto map_frame_pose = frame->pose_;
    auto target_frame_pose = target_map_pose * map_frame_pose;

    pcl::transformPointCloud(*frame->pointcloud_raw_, *frame_tcs_ptr, target_frame_pose);
    *tmp_tcs_ptr += *frame_tcs_ptr;
  }

  PointcloudType::Ptr cropped_tcd_ptr = cropPointCloud<PointcloudType>(tmp_tcs_ptr, max_range);

  pcl::VoxelGridTriplets<PointType> voxel_grid;
  voxel_grid.setLeafSize(resolution, resolution, resolution);
  voxel_grid.setInputCloud(cropped_tcd_ptr);
  voxel_grid.filter(*subsampled_tcs_ptr);

  return subsampled_tcs_ptr;
}

std::vector<CalibrationFrame> LidarCalibrator::filterCalibrationFramesByLostState(
  const std::vector<CalibrationFrame> & calibration_frames)
{
  std::vector<CalibrationFrame> filtered_frames;
  std::vector<int> deleted_keyframe_ids;

  std::vector<int> invalid_keyframe_ids;
  std::unordered_set<int> invalid_keyframe_ids_map;

  int last_keyframe_id = -1;
  bool lost = false;

  // Find all keyframes that are either "lost" or have a frame "lost" nearby
  for (const auto & frame : data_->processed_frames_) {
    lost |= frame->lost_;

    if (frame->is_key_frame_ && lost) {
      if (
        last_keyframe_id != -1 &&
        invalid_keyframe_ids_map.find(last_keyframe_id) != invalid_keyframe_ids_map.end()) {
        invalid_keyframe_ids.push_back(last_keyframe_id);
        invalid_keyframe_ids_map.insert(last_keyframe_id);
      }

      invalid_keyframe_ids.push_back(frame->keyframe_id_);
      invalid_keyframe_ids_map.insert(frame->keyframe_id_);

      lost = false;
      last_keyframe_id = frame->keyframe_id_;
    }
  }

  int left_invalid_keyframe_id = 0;

  // Find and separate frames that close enought to "lost" keyframes
  for (const auto & frame : calibration_frames) {
    // Check closest keyframe
    int keyframe_id = frame.target_frame_->keyframe_id_;

    if (!frame.target_frame_->is_key_frame_) {
      for (int i = 0;; i++) {
        int left_frame_id = frame.target_frame_->frame_id_ - i;
        int right_frame_id = frame.target_frame_->frame_id_ + i;
        if (left_frame_id >= 0 && data_->processed_frames_[left_frame_id]->is_key_frame_) {
          keyframe_id = data_->processed_frames_[left_frame_id]->keyframe_id_;
          break;
        }
        if (
          right_frame_id < static_cast<int>(data_->processed_frames_.size()) &&
          data_->processed_frames_[right_frame_id]->is_key_frame_) {
          keyframe_id = data_->processed_frames_[right_frame_id]->keyframe_id_;
          break;
        }
      }
    }

    while (left_invalid_keyframe_id < static_cast<int>(invalid_keyframe_ids.size()) - 1 &&
           invalid_keyframe_ids[left_invalid_keyframe_id] <
             keyframe_id + parameters_->dense_pointcloud_num_keyframes_) {
      left_invalid_keyframe_id++;
    }

    if (
      left_invalid_keyframe_id < static_cast<int>(invalid_keyframe_ids.size()) - 1 &&
      std::abs(
        invalid_keyframe_ids[left_invalid_keyframe_id + 1] - keyframe_id <=
        parameters_->dense_pointcloud_num_keyframes_)) {
      deleted_keyframe_ids.push_back(keyframe_id);
    } else {
      filtered_frames.push_back(frame);
    }
  }

  std::stringstream ss;
  ss << "Invalid keyframes due to being associated with 'lost' frames: ";

  for (const auto & id : deleted_keyframe_ids) {
    ss << id << " ";
  }

  if (deleted_keyframe_ids.size() > 0) {
    RCLCPP_WARN(rclcpp::get_logger(calibrator_name_), "%s\n", ss.str().c_str());
  }

  return filtered_frames;
}

std::vector<CalibrationFrame> LidarCalibrator::filterCalibrationFramesByDynamics(
  const std::vector<CalibrationFrame> & calibration_frames)
{
  std::vector<CalibrationFrame> filtered_frames;

  std::stringstream ss;
  ss << "Accepted kyframes due to dynamics & interpolation: ";

  for (auto & frame : calibration_frames) {
    RCLCPP_INFO(
      rclcpp::get_logger(calibrator_name_),
      "Attempting to add keyframe id=%d to the calibration list",
      frame.target_frame_->keyframe_id_);
    RCLCPP_INFO(
      rclcpp::get_logger(calibrator_name_), "\t - stopped: %s", frame.stopped_ ? " true" : "false");
    RCLCPP_INFO(
      rclcpp::get_logger(calibrator_name_), "\t - interpolated time: %.4f s (%s)",
      frame.interpolated_time_,
      frame.interpolated_time_ < parameters_->max_allowed_interpolated_time_ ? "accepted"
                                                                             : "rejected");
    RCLCPP_INFO(
      rclcpp::get_logger(calibrator_name_), "\t - interpolated distance: %.4f m (%s)",
      frame.interpolated_distance_,
      frame.interpolated_distance_ < parameters_->max_allowed_interpolated_distance_ ? "accepted"
                                                                                     : "rejected");
    RCLCPP_INFO(
      rclcpp::get_logger(calibrator_name_), "\t - interpolated angle: %.4f deg (%s)",
      frame.interpolated_angle_,
      frame.interpolated_angle_ < parameters_->max_allowed_interpolated_angle_ ? "accepted"
                                                                               : "rejected");
    RCLCPP_INFO(
      rclcpp::get_logger(calibrator_name_), "\t - interpolated speed: %.4f m/s (%s)",
      frame.estimated_speed_,
      frame.estimated_speed_ < parameters_->max_allowed_interpolated_speed_ ? "accepted"
                                                                            : "rejected");
    RCLCPP_INFO(
      rclcpp::get_logger(calibrator_name_), "\t - interpolated accel: %.4f m/s2 (%s)",
      frame.estimated_accel_,
      frame.estimated_accel_ < parameters_->max_allowed_interpolated_accel_ ? "accepted"
                                                                            : "rejected");

    bool standard_criteria =
      frame.interpolated_time_ < parameters_->max_allowed_interpolated_time_ &&
      frame.interpolated_distance_ < parameters_->max_allowed_interpolated_distance_ &&
      frame.interpolated_angle_ < parameters_->max_allowed_interpolated_angle_ &&
      frame.estimated_speed_ < parameters_->max_allowed_interpolated_speed_ &&
      frame.estimated_accel_ < parameters_->max_allowed_interpolated_accel_;

    bool straight_criteria =
      frame.interpolated_time_ < parameters_->max_allowed_interpolated_time_ &&
      frame.interpolated_distance_ < parameters_->max_allowed_interpolated_distance_straight_ &&
      frame.interpolated_angle_ < parameters_->max_allowed_interpolated_angle_straight_ &&
      frame.estimated_speed_ < parameters_->max_allowed_interpolated_speed_straight_ &&
      frame.estimated_accel_ < parameters_->max_allowed_interpolated_accel_straight_;

    RCLCPP_INFO(
      rclcpp::get_logger(calibrator_name_), "\t - standard criteria: %s",
      standard_criteria ? "accepted" : "rejected");
    RCLCPP_INFO(
      rclcpp::get_logger(calibrator_name_), "\t - straight criteria: %s",
      straight_criteria ? "accepted" : "rejected");

    if (
      (standard_criteria || straight_criteria) &&
      (frame.stopped_ || !parameters_->calibration_use_only_stopped_)) {
      filtered_frames.emplace_back(frame);
      ss << frame.target_frame_->frame_id_ << "/" << frame.target_frame_->keyframe_id_ << " ";
    }
  }

  if (filtered_frames.size() > 0) {
    RCLCPP_INFO(rclcpp::get_logger(calibrator_name_), "%s\n", ss.str().c_str());
  }

  return filtered_frames;
}

std::vector<CalibrationFrame> LidarCalibrator::selectBestKCalibrationFrames(
  const std::vector<CalibrationFrame> & calibration_frames, int num_frames)
{
  std::vector<CalibrationFrame> filtered_frames;

  std::vector<std::pair<float, std::size_t>> pca_coeff_calibration_id_pairs;

  for (std::size_t i = 0; i < calibration_frames.size(); i++) {
    auto & frame = calibration_frames[i];

    pcl::PCA<PointType> pca;
    pca.setInputCloud(frame.source_pointcloud_);

    float pca_coefficient = std::sqrt(std::abs(pca.getEigenValues().z()));

    RCLCPP_INFO(
      rclcpp::get_logger(calibrator_name_), "\t - pca coeff: %.4f (%s)", pca_coefficient,
      pca_coefficient >= parameters_->calibration_min_pca_eigenvalue_ ? "accepted" : "rejected");

    if (pca_coefficient >= parameters_->calibration_min_pca_eigenvalue_) {
      pca_coeff_calibration_id_pairs.push_back(std::make_pair<>(pca_coefficient, i));
    }
  }

  std::sort(
    pca_coeff_calibration_id_pairs.begin(), pca_coeff_calibration_id_pairs.end(),
    [](auto & lhs, auto & rhs) { return lhs.first > rhs.first; });

  std::stringstream ss;
  ss << "Final selected keyframes: ";

  for (auto & pair : pca_coeff_calibration_id_pairs) {
    bool accepted = true;
    for (auto & accepted_frame : filtered_frames) {
      if (
        std::abs(
          calibration_frames[pair.second].target_frame_->distance_ -
          accepted_frame.target_frame_->distance_) <
        parameters_->calibration_min_distance_between_frames_) {
        accepted = false;
        break;
      }
    }

    if (accepted) {
      auto & accepted_frame = calibration_frames[pair.second];
      filtered_frames.push_back(accepted_frame);
      ss << accepted_frame.target_frame_->frame_id_ << "/"
         << accepted_frame.target_frame_->keyframe_id_ << " ";
    }

    if (static_cast<int>(filtered_frames.size()) == num_frames) {
      break;
    }
  }

  if (filtered_frames.size() > 0) {
    RCLCPP_INFO(rclcpp::get_logger(calibrator_name_), "%s\n", ss.str().c_str());
  }

  return filtered_frames;
}

void LidarCalibrator::singleLidarCalibrationCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response)
{
  std::unique_lock<std::mutex> lock(data_->mutex_);

  Eigen::Matrix4f initial_calibration_transform;
  float initial_distance;

  // Get the tf between frames
  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    geometry_msgs::msg::Transform initial_target_to_source_msg;
    Eigen::Affine3d initial_target_to_source_affine;

    initial_target_to_source_msg =
      tf_buffer_->lookupTransform(data_->mapping_lidar_frame_, calibration_lidar_frame_, t, timeout)
        .transform;

    initial_target_to_source_affine = tf2::transformToEigen(initial_target_to_source_msg);
    initial_distance = initial_target_to_source_affine.translation().norm();
    initial_calibration_transform = initial_target_to_source_affine.matrix().cast<float>();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(rclcpp::get_logger(calibrator_name_), "could not get initial tf. %s", ex.what());
    return;
  }

  auto & calibration_frames = data_->calibration_frames_map_[calibration_lidar_frame_];
  // auto & initial_source_aligned_map_pub =
  //   initial_source_aligned_map_pub_map_[calibration_lidar_frame_];
  // auto & calibrated_source_aligned_map_pub =
  //   calibrated_source_aligned_map_pub_map_[calibration_lidar_frame_];
  // auto & target_map_pub = target_map_pub_map_[calibration_lidar_frame_];

  std::vector<CalibrationFrame> filtered_calibration_frames =
    filterCalibrationFramesByDynamics(calibration_frames);

  if (request->id >= static_cast<int>(filtered_calibration_frames.size())) {
    RCLCPP_WARN(
      rclcpp::get_logger(calibrator_name_), "Invalid requested calibration frame. size=%lu",
      filtered_calibration_frames.size());
    return;
  }

  CalibrationFrame & calibration_frame = filtered_calibration_frames[request->id];
  PointcloudType::Ptr source_pc_ptr = cropPointCloud<PointcloudType>(
    calibration_frame.source_pointcloud_, parameters_->max_calibration_range_);

  PointcloudType::Ptr target_dense_pc_ptr = getDensePointcloudFromMap(
    calibration_frame.target_frame_->pose_, calibration_frame.target_frame_,
    parameters_->leaf_size_dense_map_, parameters_->max_calibration_range_ + initial_distance);
  PointcloudType::Ptr target_thin_pc_ptr = getDensePointcloudFromMap(
    calibration_frame.target_frame_->pose_, calibration_frame.target_frame_,
    parameters_->calibration_viz_leaf_size_,
    parameters_->max_calibration_range_ + initial_distance);

  PointcloudType::Ptr initial_source_aligned_pc_ptr(new PointcloudType());
  pcl::transformPointCloud(
    *source_pc_ptr, *initial_source_aligned_pc_ptr, initial_calibration_transform);

  // Evaluate the initial calibration
  setUpCalibrators(source_pc_ptr, target_dense_pc_ptr);

  // Crop unused areas of the target pointcloud to save processing time
  cropTargetPointcloud<PointType>(
    initial_source_aligned_pc_ptr, target_dense_pc_ptr, initial_distance);
  cropTargetPointcloud<PointType>(
    initial_source_aligned_pc_ptr, target_thin_pc_ptr, initial_distance);

  correspondence_estimator_->setInputSource(initial_source_aligned_pc_ptr);
  correspondence_estimator_->setInputTarget(target_dense_pc_ptr);
  double initial_score = sourceTargetDistance(*correspondence_estimator_);

  RCLCPP_WARN(
    rclcpp::get_logger(calibrator_name_),
    "Initial calibration score = %.4f (avg.squared.dist) | sqrt.score = %.4f m | discretization = "
    "%.4f m",
    initial_score, std::sqrt(initial_score), parameters_->leaf_size_dense_map_);

  // Find best calibration using an "ensemble" of calibrators
  std::vector<Eigen::Matrix4f> candidate_transforms = {initial_calibration_transform};
  Eigen::Matrix4f best_transform;
  float best_score;

  findBestTransform<pcl::Registration<PointType, PointType>, PointType>(
    candidate_transforms, calibration_registrators_, parameters_->calibration_verbose_,
    best_transform, best_score);

  PointcloudType::Ptr calibrated_source_aligned_pc_ptr(new PointcloudType());
  pcl::transformPointCloud(*source_pc_ptr, *calibrated_source_aligned_pc_ptr, best_transform);

  RCLCPP_WARN(
    rclcpp::get_logger(calibrator_name_),
    "Best calibration score = %.4f (avg.squared.dist) | sqrt.score = %.4f m | discretization = "
    "%.4f m",
    best_score, std::sqrt(best_score), parameters_->leaf_size_dense_map_);

  PointcloudType::Ptr test_aligned_pc_ptr(new PointcloudType());
  pcl::transformPointCloud(*source_pc_ptr, *test_aligned_pc_ptr, best_transform);

  correspondence_estimator_->setInputSource(test_aligned_pc_ptr);
  correspondence_estimator_->setInputTarget(target_dense_pc_ptr);
  double test_score = sourceTargetDistance(*correspondence_estimator_);

  RCLCPP_WARN(
    rclcpp::get_logger(calibrator_name_),
    "Test calibration score = %.4f (avg.squared.dist) | sqrt.score = %.4f m | discretization = "
    "%.4f m",
    test_score, std::sqrt(test_score), parameters_->leaf_size_dense_map_);

  // Output ROS data
  PointcloudType::Ptr initial_source_aligned_map_ptr(new PointcloudType());
  PointcloudType::Ptr calibrated_source_aligned_map_ptr(new PointcloudType());
  PointcloudType::Ptr target_thin_map_ptr(new PointcloudType());
  pcl::transformPointCloud(
    *initial_source_aligned_pc_ptr, *initial_source_aligned_map_ptr,
    calibration_frame.target_frame_->pose_);
  pcl::transformPointCloud(
    *calibrated_source_aligned_pc_ptr, *calibrated_source_aligned_map_ptr,
    calibration_frame.target_frame_->pose_);
  pcl::transformPointCloud(
    *target_thin_pc_ptr, *target_thin_map_ptr, calibration_frame.target_frame_->pose_);

  sensor_msgs::msg::PointCloud2 initial_source_aligned_map_msg, calibrated_source_aligned_map_msg,
    target_map_msg;
  initial_source_aligned_pc_ptr->width = initial_source_aligned_pc_ptr->points.size();
  initial_source_aligned_pc_ptr->height = 1;
  calibrated_source_aligned_pc_ptr->width = calibrated_source_aligned_pc_ptr->points.size();
  calibrated_source_aligned_pc_ptr->height = 1;
  target_thin_pc_ptr->width = target_thin_pc_ptr->points.size();
  target_thin_pc_ptr->height = 1;

  pcl::toROSMsg(*initial_source_aligned_map_ptr, initial_source_aligned_map_msg);
  pcl::toROSMsg(*calibrated_source_aligned_map_ptr, calibrated_source_aligned_map_msg);
  pcl::toROSMsg(*target_thin_map_ptr, target_map_msg);

  initial_source_aligned_map_msg.header = calibration_frame.source_header_;
  initial_source_aligned_map_msg.header.frame_id = data_->map_frame_;
  calibrated_source_aligned_map_msg.header = calibration_frame.source_header_;
  calibrated_source_aligned_map_msg.header.frame_id = data_->map_frame_;
  target_map_msg.header = calibration_frame.target_frame_->header_;
  target_map_msg.header.frame_id = data_->map_frame_;

  initial_source_aligned_map_pub_->publish(initial_source_aligned_map_msg);
  calibrated_source_aligned_map_pub_->publish(calibrated_source_aligned_map_msg);
  target_map_pub_->publish(target_map_msg);

  response->success = true;
}

void LidarCalibrator::multipleLidarCalibrationCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response)
{
  UNUSED(request);
  std::unique_lock<std::mutex> lock(data_->mutex_);

  Eigen::Matrix4f initial_calibration_transform;
  float initial_distance;

  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "Calibrating frame: %s",
    calibration_lidar_frame_.c_str());
  RCLCPP_INFO(rclcpp::get_logger(calibrator_name_), "Obtaining initial calibration...");

  // Get the tf between frames
  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    geometry_msgs::msg::Transform initial_target_to_source_msg;
    Eigen::Affine3d initial_target_to_source_affine;

    initial_target_to_source_msg =
      tf_buffer_->lookupTransform(data_->mapping_lidar_frame_, calibration_lidar_frame_, t, timeout)
        .transform;

    initial_target_to_source_affine = tf2::transformToEigen(initial_target_to_source_msg);
    initial_distance = initial_target_to_source_affine.translation().norm();
    initial_calibration_transform = initial_target_to_source_affine.matrix().cast<float>();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(rclcpp::get_logger(calibrator_name_), "could not get initial tf. %s", ex.what());
    response->success = false;
    return;
  }

  auto & calibration_frames = data_->calibration_frames_map_[calibration_lidar_frame_];

  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "Original calibration frames: %ld",
    calibration_frames.size());

  std::vector<CalibrationFrame> filtered_calibration_frames_1 =
    filterCalibrationFramesByLostState(calibration_frames);

  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "Filtered calibration frames by dynamics: %ld",
    filtered_calibration_frames_1.size());

  std::vector<CalibrationFrame> filtered_calibration_frames_2 =
    filterCalibrationFramesByDynamics(filtered_calibration_frames_1);

  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "Filtered calibration frames by dynamics: %ld",
    filtered_calibration_frames_2.size());

  std::vector<CalibrationFrame> accepted_calibration_frames = selectBestKCalibrationFrames(
    filtered_calibration_frames_2, parameters_->calibration_max_frames_);

  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "Accepted calibration frames: %ld",
    accepted_calibration_frames.size());

  std::vector<pcl::PointCloud<PointType>::Ptr> sources, targets, targets_thin;
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_),
    "Preparing dense calibration pointclouds from the map...");

  // Prepare pointclouds for calibration
  for (auto & calibration_frame : accepted_calibration_frames) {
    PointcloudType::Ptr source_pc_ptr = cropPointCloud<PointcloudType>(
      calibration_frame.source_pointcloud_, parameters_->max_calibration_range_);

    PointcloudType::Ptr target_pc_ptr = getDensePointcloudFromMap(
      calibration_frame.target_frame_->pose_, calibration_frame.target_frame_,
      parameters_->leaf_size_dense_map_, parameters_->max_calibration_range_ + initial_distance);

    PointcloudType::Ptr target_thin_pc_ptr = getDensePointcloudFromMap(
      calibration_frame.target_frame_->pose_, calibration_frame.target_frame_,
      parameters_->calibration_viz_leaf_size_,
      parameters_->max_calibration_range_ + initial_distance);

    // Transfor the source to target frame to crop it later
    PointcloudType::Ptr initial_source_aligned_pc_ptr(new PointcloudType());
    pcl::transformPointCloud(
      *source_pc_ptr, *initial_source_aligned_pc_ptr, initial_calibration_transform);

    // Crop unused areas of the target pointcloud to save processing time
    cropTargetPointcloud<PointType>(initial_source_aligned_pc_ptr, target_pc_ptr, initial_distance);
    cropTargetPointcloud<PointType>(
      initial_source_aligned_pc_ptr, target_thin_pc_ptr, initial_distance);

    sources.push_back(source_pc_ptr);
    targets.push_back(target_pc_ptr);
    targets_thin.push_back(target_thin_pc_ptr);
  }

  // Set all the registrators with the pointclouds
  for (auto & calibrator : calibration_batch_registrators_) {
    calibrator->clearInputSources();
    calibrator->clearInputTargets();

    for (std::size_t i = 0; i < sources.size(); i++) {
      calibrator->addInputSource(sources[i]);
      calibrator->addInputTarget(targets[i]);
    }
  }

  // Perform single-frame calibration for all frames
  std::vector<float> initial_calibration_single_frame_score(accepted_calibration_frames.size());
  std::vector<float> single_frame_calibration_multi_frame_score(accepted_calibration_frames.size());
  std::vector<float> single_frame_calibration_single_frame_score(
    accepted_calibration_frames.size());
  std::vector<Eigen::Matrix4f> best_single_frame_transforms(accepted_calibration_frames.size());

  RCLCPP_INFO(rclcpp::get_logger(calibrator_name_), "Calibration using single frames...");
  for (std::size_t i = 0; i < accepted_calibration_frames.size(); i++) {
    // start

    PointcloudType::Ptr initial_source_aligned_pc_ptr(new PointcloudType());
    pcl::transformPointCloud(
      *sources[i], *initial_source_aligned_pc_ptr, initial_calibration_transform);

    // Evaluate the initial calibration
    // setUpCalibrators(sources[i], targets[i]);
    for (auto & calibrator : calibration_registrators_) {
      calibrator->setInputSource(sources[i]);
      calibrator->setInputTarget(targets[i]);
    }

    correspondence_estimator_->setInputSource(initial_source_aligned_pc_ptr);
    correspondence_estimator_->setInputTarget(targets[i]);
    double initial_score = sourceTargetDistance(*correspondence_estimator_);

    // Find best calibration using an "ensemble" of calibrators
    std::vector<Eigen::Matrix4f> candidate_transforms = {initial_calibration_transform};
    Eigen::Matrix4f best_single_frame_transform;
    float best_single_frame_transform_score;

    findBestTransform<pcl::Registration<PointType, PointType>, PointType>(
      candidate_transforms, calibration_registrators_, parameters_->calibration_verbose_,
      best_single_frame_transform, best_single_frame_transform_score);

    float best_single_frame_transform_multi_frame_score =
      sourceTargetDistance<PointType>(sources, targets, best_single_frame_transform);

    initial_calibration_single_frame_score[i] = initial_score;
    single_frame_calibration_multi_frame_score[i] = best_single_frame_transform_multi_frame_score;
    single_frame_calibration_single_frame_score[i] = best_single_frame_transform_score;
    best_single_frame_transforms[i] = best_single_frame_transform;

    RCLCPP_INFO(
      rclcpp::get_logger(calibrator_name_),
      "Calibration frame=%ld Keyframe=%d Initial single-frame score= %.4f Single-frame score=%.4f "
      "Multi-frame score=%.4f",
      i, accepted_calibration_frames[i].target_frame_->keyframe_id_, initial_score,
      best_single_frame_transform_score, best_single_frame_transform_multi_frame_score);
    // finish
  }

  // Choose the best sigle-frame calibration
  std::vector<float>::iterator best_single_frame_calibration_multi_frame_score_it =
    std::min_element(
      std::begin(single_frame_calibration_multi_frame_score),
      std::end(single_frame_calibration_multi_frame_score));
  int best_single_frame_calibration_multi_frame_score_index = std::distance(
    std::begin(single_frame_calibration_multi_frame_score),
    best_single_frame_calibration_multi_frame_score_it);

  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_),
    "Single-frame calibration results:\n  Best Calibration frame=%d Keyframe=%d Multi-frame "
    "score=%.4f",
    best_single_frame_calibration_multi_frame_score_index,
    accepted_calibration_frames[best_single_frame_calibration_multi_frame_score_index]
      .target_frame_->keyframe_id_,
    *best_single_frame_calibration_multi_frame_score_it);

  RCLCPP_INFO(rclcpp::get_logger(calibrator_name_), "Calibration using multiple frames...");
  std::vector<Eigen::Matrix4f> candidate_transforms = {
    initial_calibration_transform,
    best_single_frame_transforms[best_single_frame_calibration_multi_frame_score_index]};
  Eigen::Matrix4f best_multi_frame_calibration_transform;
  float best_multi_frame_calibration_multi_frame_score;

  findBestTransform<pcl::JointIterativeClosestPointExtended<PointType, PointType>, PointType>(
    candidate_transforms, calibration_batch_registrators_, parameters_->calibration_verbose_,
    best_multi_frame_calibration_transform, best_multi_frame_calibration_multi_frame_score);

  float initial_score =
    sourceTargetDistance<PointType>(sources, targets, initial_calibration_transform);
  float final_score =
    sourceTargetDistance<PointType>(sources, targets, best_multi_frame_calibration_transform);

  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_),
    "Initial calibration score = %.4f (avg.squared.dist) | sqrt.score = %.4f m | discretization = "
    "%.4f m",
    initial_score, std::sqrt(initial_score), parameters_->leaf_size_dense_map_);
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_),
    "Best calibration score = %.4f (avg.squared.dist) | sqrt.score = %.4f m | discretization = "
    "%.4f m",
    final_score, std::sqrt(final_score), parameters_->leaf_size_dense_map_);
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_),
    "Test calibration score = %.4f (avg.squared.dist) | sqrt.score = %.4f m | discretization = "
    "%.4f m",
    best_multi_frame_calibration_multi_frame_score,
    std::sqrt(best_multi_frame_calibration_multi_frame_score), parameters_->leaf_size_dense_map_);

  // Publish ROS data
  PointcloudType::Ptr initial_source_aligned_map_ptr(new PointcloudType());
  PointcloudType::Ptr calibrated_source_aligned_map_ptr(new PointcloudType());
  PointcloudType::Ptr target_thin_map_ptr(new PointcloudType());

  for (std::size_t i = 0; i < accepted_calibration_frames.size(); i++) {
    PointcloudType::Ptr initial_tmp_ptr(new PointcloudType());
    PointcloudType::Ptr calibrated_tmp_ptr(new PointcloudType());
    PointcloudType::Ptr target_thin_tmp_ptr(new PointcloudType());

    pcl::transformPointCloud(
      *sources[i], *initial_tmp_ptr,
      accepted_calibration_frames[i].target_frame_->pose_ * initial_calibration_transform);
    pcl::transformPointCloud(
      *sources[i], *calibrated_tmp_ptr,
      accepted_calibration_frames[i].target_frame_->pose_ * best_multi_frame_calibration_transform);

    pcl::transformPointCloud(
      *targets_thin[i], *target_thin_tmp_ptr, accepted_calibration_frames[i].target_frame_->pose_);

    *initial_source_aligned_map_ptr += *initial_tmp_ptr;
    *calibrated_source_aligned_map_ptr += *calibrated_tmp_ptr;
    *target_thin_map_ptr += *target_thin_tmp_ptr;
  }

  sensor_msgs::msg::PointCloud2 initial_source_aligned_map_msg, calibrated_source_aligned_map_msg,
    target_map_msg;
  initial_source_aligned_map_ptr->width = initial_source_aligned_map_ptr->points.size();
  initial_source_aligned_map_ptr->height = 1;
  calibrated_source_aligned_map_ptr->width = calibrated_source_aligned_map_ptr->points.size();
  calibrated_source_aligned_map_ptr->height = 1;
  target_thin_map_ptr->width = target_thin_map_ptr->points.size();
  target_thin_map_ptr->height = 1;

  pcl::toROSMsg(*initial_source_aligned_map_ptr, initial_source_aligned_map_msg);
  pcl::toROSMsg(*calibrated_source_aligned_map_ptr, calibrated_source_aligned_map_msg);
  pcl::toROSMsg(*target_thin_map_ptr, target_map_msg);

  initial_source_aligned_map_msg.header = accepted_calibration_frames[0].source_header_;
  initial_source_aligned_map_msg.header.frame_id = data_->map_frame_;
  calibrated_source_aligned_map_msg.header = accepted_calibration_frames[0].source_header_;
  calibrated_source_aligned_map_msg.header.frame_id = data_->map_frame_;
  target_map_msg.header = accepted_calibration_frames[0].source_header_;
  target_map_msg.header.frame_id = data_->map_frame_;

  initial_source_aligned_map_pub_->publish(initial_source_aligned_map_msg);
  calibrated_source_aligned_map_pub_->publish(calibrated_source_aligned_map_msg);
  target_map_pub_->publish(target_map_msg);

  response->success = false;
}
