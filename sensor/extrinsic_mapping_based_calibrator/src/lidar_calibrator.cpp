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

#include <extrinsic_mapping_based_calibrator/filters/best_frames_filter.hpp>
#include <extrinsic_mapping_based_calibrator/filters/dynamics_filter.hpp>
#include <extrinsic_mapping_based_calibrator/filters/lost_state_filter.hpp>
#include <extrinsic_mapping_based_calibrator/filters/object_detection_filter.hpp>
#include <extrinsic_mapping_based_calibrator/filters/sequential_filter.hpp>
#include <extrinsic_mapping_based_calibrator/lidar_calibrator.hpp>
#include <extrinsic_mapping_based_calibrator/utils.hpp>
#include <extrinsic_mapping_based_calibrator/voxel_grid_filter_wrapper.hpp>

#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

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
  // Filter configuration
  std::shared_ptr<Filter> lost_state_filter(new LostStateFilter(calibrator_name_, parameters));
  std::shared_ptr<Filter> dynamics_filter(new DynamicsFilter(calibrator_name_, parameters));
  std::shared_ptr<Filter> best_frames_filter(new BestFramesFilter(calibrator_name_, parameters));
  std::shared_ptr<Filter> object_detection_filter(
    new ObjectDetectionFilter(calibrator_name_, parameters, tf_buffer_));
  std::vector<std::shared_ptr<Filter>> filters =
    parameters_->filter_detections_
      ? std::vector<std::shared_ptr<
          Filter>>{lost_state_filter, dynamics_filter, best_frames_filter, object_detection_filter}
      : std::vector<std::shared_ptr<Filter>>{
          lost_state_filter, dynamics_filter, best_frames_filter};

  filter_.reset(new SequentialFilter(calibrator_name_, parameters, filters));

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
  const Eigen::Matrix4f & pose, const Frame::Ptr & frame, double resolution, double max_range)
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

  std::vector<CalibrationFrame> filtered_calibration_frames =
    filter_->filter(calibration_frames, data_);

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
  double initial_score = sourceTargetDistance(
    *correspondence_estimator_, parameters_->calibration_eval_max_corr_distance_);

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
    candidate_transforms, calibration_registrators_,
    parameters_->calibration_eval_max_corr_distance_, parameters_->calibration_verbose_,
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
  double test_score = sourceTargetDistance(
    *correspondence_estimator_, parameters_->calibration_eval_max_corr_distance_);

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

  Eigen::Matrix4f result;
  float score;
  response->success = calibrate(result, score);
}

bool LidarCalibrator::calibrate(Eigen::Matrix4f & best_transform, float & best_score)
{
  std::unique_lock<std::mutex> lock(data_->mutex_);

  Eigen::Matrix4f initial_calibration_transform;
  float initial_distance;
  std::string map_frame = data_->map_frame_;

  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "Calibrating frame: %s",
    calibration_lidar_frame_.c_str());
  RCLCPP_INFO(rclcpp::get_logger(calibrator_name_), "Obtaining initial calibration...");

  // Get the initial tf between mapping and calibration lidars
  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    auto initial_target_to_source_affine = tf2::transformToEigen(
      tf_buffer_->lookupTransform(data_->mapping_lidar_frame_, calibration_lidar_frame_, t, timeout)
        .transform);

    initial_calibration_transform = initial_target_to_source_affine.matrix().cast<float>();
    initial_distance = initial_target_to_source_affine.translation().norm();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(rclcpp::get_logger(calibrator_name_), "could not get initial tf. %s", ex.what());

    return false;
  }

  // Filter calibration frames using several criteria and select the best ones suited for
  // calibration
  std::vector<CalibrationFrame> calibration_frames =
    filter_->filter(data_->calibration_frames_map_[calibration_lidar_frame_], data_);

  if (static_cast<int>(calibration_frames.size()) < parameters_->calibration_min_frames_) {
    RCLCPP_WARN(rclcpp::get_logger(calibrator_name_), "Insufficient calibration frames. aborting.");
    return false;
  }

  // Prepate augmented calibration pointclouds
  std::vector<pcl::PointCloud<PointType>::Ptr> sources, targets, targets_thin;
  prepareCalibrationData(
    calibration_frames, initial_distance, initial_calibration_transform, sources, targets,
    targets_thin);

  // We no lnoger used the shared data
  lock.unlock();

  // Set all the registrators with the pointclouds
  for (auto & calibrator : calibration_batch_registrators_) {
    calibrator->clearInputSources();
    calibrator->clearInputTargets();

    for (std::size_t i = 0; i < sources.size(); i++) {
      calibrator->addInputSource(sources[i]);
      calibrator->addInputTarget(targets[i]);
    }
  }

  // Single-frame calibration for all frames
  std::vector<float> initial_calibration_single_frame_score(calibration_frames.size());
  std::vector<float> single_frame_calibration_multi_frame_score(calibration_frames.size());
  std::vector<float> single_frame_calibration_single_frame_score(calibration_frames.size());
  std::vector<Eigen::Matrix4f> best_single_frame_transforms(calibration_frames.size());

  RCLCPP_INFO(rclcpp::get_logger(calibrator_name_), "Calibration using single frames...");
  for (std::size_t i = 0; i < calibration_frames.size(); i++) {
    PointcloudType::Ptr initial_source_aligned_pc_ptr(new PointcloudType());
    pcl::transformPointCloud(
      *sources[i], *initial_source_aligned_pc_ptr, initial_calibration_transform);

    for (auto & calibrator : calibration_registrators_) {
      calibrator->setInputSource(sources[i]);
      calibrator->setInputTarget(targets[i]);
    }

    correspondence_estimator_->setInputSource(initial_source_aligned_pc_ptr);
    correspondence_estimator_->setInputTarget(targets[i]);
    double initial_score = sourceTargetDistance(
      *correspondence_estimator_, parameters_->calibration_eval_max_corr_distance_);

    // Find best calibration using an "ensemble" of calibrators
    std::vector<Eigen::Matrix4f> candidate_transforms = {initial_calibration_transform};
    Eigen::Matrix4f best_single_frame_transform;
    float best_single_frame_transform_score;

    findBestTransform<pcl::Registration<PointType, PointType>, PointType>(
      candidate_transforms, calibration_registrators_,
      parameters_->calibration_eval_max_corr_distance_, parameters_->calibration_verbose_,
      best_single_frame_transform, best_single_frame_transform_score);

    float best_single_frame_transform_multi_frame_score = sourceTargetDistance<PointType>(
      sources, targets, best_single_frame_transform,
      parameters_->calibration_eval_max_corr_distance_);

    initial_calibration_single_frame_score[i] = initial_score;
    single_frame_calibration_multi_frame_score[i] = best_single_frame_transform_multi_frame_score;
    single_frame_calibration_single_frame_score[i] = best_single_frame_transform_score;
    best_single_frame_transforms[i] = best_single_frame_transform;

    RCLCPP_INFO(
      rclcpp::get_logger(calibrator_name_),
      "Calibration frame=%ld Keyframe=%d Initial single-frame score= %.4f Single-frame score=%.4f "
      "Multi-frame score=%.4f",
      i, calibration_frames[i].target_frame_->keyframe_id_, initial_score,
      best_single_frame_transform_score, best_single_frame_transform_multi_frame_score);
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
    calibration_frames[best_single_frame_calibration_multi_frame_score_index]
      .target_frame_->keyframe_id_,
    *best_single_frame_calibration_multi_frame_score_it);

  // Multi-frame calibration using all frames
  RCLCPP_INFO(rclcpp::get_logger(calibrator_name_), "Calibration using multiple frames...");
  std::vector<Eigen::Matrix4f> candidate_transforms = {
    initial_calibration_transform,
    best_single_frame_transforms[best_single_frame_calibration_multi_frame_score_index]};
  Eigen::Matrix4f best_multi_frame_calibration_transform;
  float best_multi_frame_calibration_multi_frame_score;

  findBestTransform<pcl::JointIterativeClosestPointExtended<PointType, PointType>, PointType>(
    candidate_transforms, calibration_batch_registrators_,
    parameters_->calibration_eval_max_corr_distance_, parameters_->calibration_verbose_,
    best_multi_frame_calibration_transform, best_multi_frame_calibration_multi_frame_score);

  float initial_score = sourceTargetDistance<PointType>(
    sources, targets, initial_calibration_transform,
    parameters_->calibration_eval_max_corr_distance_);
  float final_score = sourceTargetDistance<PointType>(
    sources, targets, best_multi_frame_calibration_transform,
    parameters_->calibration_eval_max_corr_distance_);

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

  // Publish the calbiraton resullts
  publishResults(
    calibration_frames, sources, targets_thin, initial_calibration_transform,
    best_multi_frame_calibration_transform, map_frame);

  best_transform = best_multi_frame_calibration_transform;
  best_score = std::sqrt(best_multi_frame_calibration_multi_frame_score);

  return true;
}

void LidarCalibrator::prepareCalibrationData(
  const std::vector<CalibrationFrame> & calibration_frames, const float initial_distance,
  const Eigen::Matrix4f & initial_calibration_transform,
  std::vector<pcl::PointCloud<PointType>::Ptr> & sources,
  std::vector<pcl::PointCloud<PointType>::Ptr> & targets,
  std::vector<pcl::PointCloud<PointType>::Ptr> & targets_thin)
{
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_),
    "Preparing dense calibration pointclouds from the map...");

  // Prepare pointclouds for calibration
  for (auto & calibration_frame : calibration_frames) {
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
}

void LidarCalibrator::publishResults(
  const std::vector<CalibrationFrame> & calibration_frames,
  const std::vector<pcl::PointCloud<PointType>::Ptr> & sources,
  const std::vector<pcl::PointCloud<PointType>::Ptr> & targets,
  const Eigen::Matrix4f & initial_transform, const Eigen::Matrix4f & calibrated_transform,
  const std::string & map_frame)
{
  // Publish ROS data
  PointcloudType::Ptr initial_source_aligned_map_ptr(new PointcloudType());
  PointcloudType::Ptr calibrated_source_aligned_map_ptr(new PointcloudType());
  PointcloudType::Ptr target_thin_map_ptr(new PointcloudType());

  for (std::size_t i = 0; i < calibration_frames.size(); i++) {
    PointcloudType::Ptr initial_tmp_ptr(new PointcloudType());
    PointcloudType::Ptr calibrated_tmp_ptr(new PointcloudType());
    PointcloudType::Ptr target_thin_tmp_ptr(new PointcloudType());

    pcl::transformPointCloud(
      *sources[i], *initial_tmp_ptr,
      calibration_frames[i].target_frame_->pose_ * initial_transform);
    pcl::transformPointCloud(
      *sources[i], *calibrated_tmp_ptr,
      calibration_frames[i].target_frame_->pose_ * calibrated_transform);

    pcl::transformPointCloud(
      *targets[i], *target_thin_tmp_ptr, calibration_frames[i].target_frame_->pose_);

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

  initial_source_aligned_map_msg.header = calibration_frames[0].source_header_;
  initial_source_aligned_map_msg.header.frame_id = map_frame;
  calibrated_source_aligned_map_msg.header = calibration_frames[0].source_header_;
  calibrated_source_aligned_map_msg.header.frame_id = map_frame;
  target_map_msg.header = calibration_frames[0].source_header_;
  target_map_msg.header.frame_id = map_frame;

  initial_source_aligned_map_pub_->publish(initial_source_aligned_map_msg);
  calibrated_source_aligned_map_pub_->publish(calibrated_source_aligned_map_msg);
  target_map_pub_->publish(target_map_msg);
}
