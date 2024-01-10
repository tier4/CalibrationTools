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

#include <extrinsic_mapping_based_calibrator/filters/best_frames_filter.hpp>
#include <extrinsic_mapping_based_calibrator/filters/dynamics_filter.hpp>
#include <extrinsic_mapping_based_calibrator/filters/lost_state_filter.hpp>
#include <extrinsic_mapping_based_calibrator/filters/object_detection_filter.hpp>
#include <extrinsic_mapping_based_calibrator/filters/sequential_filter.hpp>
#include <extrinsic_mapping_based_calibrator/lidar_calibrator.hpp>
#include <extrinsic_mapping_based_calibrator/utils.hpp>
#include <extrinsic_mapping_based_calibrator/voxel_grid_filter_wrapper.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl_conversions/pcl_conversions.h>

LidarCalibrator::LidarCalibrator(
  const std::string & calibration_lidar_frame, CalibrationParameters::Ptr & parameters,
  MappingData::Ptr & mapping_data, std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  PointPublisher::SharedPtr & initial_source_aligned_map_pub,
  PointPublisher::SharedPtr & calibrated_source_aligned_map_pub,
  PointPublisher::SharedPtr & target_map_pub)
: SensorCalibrator(
    calibration_lidar_frame, "lidar_calibrator(" + calibration_lidar_frame + ")", parameters,
    mapping_data, tf_buffer),
  initial_source_aligned_map_pub_(initial_source_aligned_map_pub),
  calibrated_source_aligned_map_pub_(calibrated_source_aligned_map_pub),
  target_map_pub_(target_map_pub)
{
  // Filter configuration
  std::shared_ptr<Filter> lost_state_filter(
    new LostStateFilter(Filter::FilterType::LidarFilter, calibrator_name_, parameters));
  std::shared_ptr<Filter> dynamics_filter(
    new DynamicsFilter(Filter::FilterType::LidarFilter, calibrator_name_, parameters));
  std::shared_ptr<Filter> best_frames_filter(
    new BestFramesFilter(Filter::FilterType::LidarFilter, calibrator_name_, parameters));
  std::shared_ptr<Filter> object_detection_filter(new ObjectDetectionFilter(
    Filter::FilterType::LidarFilter, calibrator_name_, parameters, tf_buffer_));
  std::vector<std::shared_ptr<Filter>> filters =
    parameters_->filter_detections_
      ? std::vector<std::shared_ptr<
          Filter>>{lost_state_filter, dynamics_filter, best_frames_filter, object_detection_filter}
      : std::vector<std::shared_ptr<Filter>>{
          lost_state_filter, dynamics_filter, best_frames_filter};

  filter_.reset(
    new SequentialFilter(Filter::FilterType::LidarFilter, calibrator_name_, parameters, filters));

  // Calibration configuration
  correspondence_estimator_ =
    pcl::make_shared<pcl::registration::CorrespondenceEstimation<PointType, PointType>>();

  // cSpell:ignore pclomp
  calibration_ndt_ = pcl::make_shared<pclomp::NormalDistributionsTransform<PointType, PointType>>();
  calibration_gicp_ =
    pcl::make_shared<pcl::GeneralizedIterativeClosestPoint<PointType, PointType>>();
  calibration_icp_coarse_ = pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();
  calibration_icp_fine_ = pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();
  calibration_icp_ultra_fine_ =
    pcl::make_shared<pcl::IterativeClosestPoint<PointType, PointType>>();

  calibration_registrators_ = {
    calibration_ndt_, calibration_gicp_, calibration_icp_coarse_, calibration_icp_fine_,
    calibration_icp_ultra_fine_};

  calibration_batch_icp_coarse_ =
    pcl::make_shared<pcl::JointIterativeClosestPointExtended<PointType, PointType>>();
  calibration_batch_icp_fine_ =
    pcl::make_shared<pcl::JointIterativeClosestPointExtended<PointType, PointType>>();
  calibration_batch_icp_ultra_fine_ =
    pcl::make_shared<pcl::JointIterativeClosestPointExtended<PointType, PointType>>();
  calibration_batch_registrators_ = {
    calibration_batch_icp_coarse_, calibration_batch_icp_fine_, calibration_batch_icp_ultra_fine_};

  configureCalibrators();
}

void LidarCalibrator::configureCalibrators()
{
  calibration_gicp_->setMaxCorrespondenceDistance(parameters_->max_corr_dist_coarse_);
  calibration_icp_coarse_->setMaxCorrespondenceDistance(parameters_->max_corr_dist_coarse_);
  calibration_icp_fine_->setMaxCorrespondenceDistance(parameters_->max_corr_dist_fine_);
  calibration_icp_ultra_fine_->setMaxCorrespondenceDistance(parameters_->max_corr_dist_ultra_fine_);

  for (auto & calibrator : calibration_registrators_) {
    calibrator->setMaximumIterations(parameters_->solver_iterations_);
  }

  calibration_batch_icp_coarse_->setMaxCorrespondenceDistance(parameters_->max_corr_dist_coarse_);
  calibration_batch_icp_fine_->setMaxCorrespondenceDistance(parameters_->max_corr_dist_fine_);
  calibration_batch_icp_ultra_fine_->setMaxCorrespondenceDistance(
    parameters_->max_corr_dist_ultra_fine_);

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

std::tuple<bool, Eigen::Matrix4d, float> LidarCalibrator::calibrate()
{
  std::unique_lock<std::recursive_mutex> lock(data_->mutex_);

  Eigen::Matrix4f initial_calibration_transform;
  float initial_distance;
  std::string map_frame = data_->map_frame_;

  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "Calibrating frame: %s",
    calibrator_sensor_frame_.c_str());
  RCLCPP_INFO(rclcpp::get_logger(calibrator_name_), "Obtaining initial calibration...");

  // Get the initial tf between mapping and calibration lidars
  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    auto initial_target_to_source_affine = tf2::transformToEigen(
      tf_buffer_->lookupTransform(data_->mapping_lidar_frame_, calibrator_sensor_frame_, t, timeout)
        .transform);

    initial_calibration_transform = initial_target_to_source_affine.matrix().cast<float>();
    initial_distance = initial_target_to_source_affine.translation().norm();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(rclcpp::get_logger(calibrator_name_), "could not get initial tf. %s", ex.what());

    return std::make_tuple<>(false, Eigen::Matrix4d::Identity(), 0.f);
  }

  // Filter calibration frames using several criteria and select the best ones suited for
  // calibration
  std::vector<CalibrationFrame> calibration_frames =
    filter_->filter(data_->lidar_calibration_frames_map_[calibrator_sensor_frame_], data_);

  if (static_cast<int>(calibration_frames.size()) < parameters_->lidar_calibration_min_frames_) {
    RCLCPP_WARN(
      rclcpp::get_logger(calibrator_name_), "Insufficient calibration frames (%lu / %d). aborting.",
      calibration_frames.size(), parameters_->lidar_calibration_min_frames_);
    return std::make_tuple<>(false, Eigen::Matrix4d::Identity(), 0.f);
  }

  // Prepare augmented calibration pointclouds
  std::vector<pcl::PointCloud<PointType>::Ptr> sources, targets, targets_thin;
  prepareCalibrationData(
    calibration_frames, initial_distance, initial_calibration_transform, sources, targets,
    targets_thin);

  // We no longer used the shared data
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

  // Choose the best single-frame calibration
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

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(calibrator_name_), "Initial calibration matrix =\n"
                                            << initial_calibration_transform);

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(calibrator_name_), "Best calibration matrix =\n"
                                            << best_multi_frame_calibration_transform);

  auto best_multi_frame_calibration_tf =
    tf2::eigenToTransform(Eigen::Affine3d(best_multi_frame_calibration_transform.cast<double>()));

  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "Calibration result as a tf main lidar -> %s",
    calibrator_name_.c_str());
  RCLCPP_INFO(rclcpp::get_logger(calibrator_name_), "\ttranslation:");
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "\t\tx: %f",
    best_multi_frame_calibration_tf.transform.translation.x);
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "\t\ty: %f",
    best_multi_frame_calibration_tf.transform.translation.y);
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "\t\tz: %f",
    best_multi_frame_calibration_tf.transform.translation.z);
  RCLCPP_INFO(rclcpp::get_logger(calibrator_name_), "\trotation:");
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "\t\tx: %f",
    best_multi_frame_calibration_tf.transform.rotation.x);
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "\t\ty: %f",
    best_multi_frame_calibration_tf.transform.rotation.y);
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "\t\tz: %f",
    best_multi_frame_calibration_tf.transform.rotation.z);
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_), "\t\tw: %f",
    best_multi_frame_calibration_tf.transform.rotation.w);

  // Publish the calibration results
  publishResults(
    calibration_frames, sources, targets_thin, initial_calibration_transform,
    best_multi_frame_calibration_transform, map_frame);

  return std::make_tuple<>(
    true, best_multi_frame_calibration_transform.cast<double>(),
    std::sqrt(best_multi_frame_calibration_multi_frame_score));
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
      calibration_frame.source_pointcloud_, parameters_->min_calibration_range_,
      parameters_->max_calibration_range_);

    PointcloudType::Ptr target_pc_ptr = getDensePointcloudFromMap(
      calibration_frame.local_map_pose_, calibration_frame.target_frame_,
      parameters_->leaf_size_dense_map_, parameters_->min_calibration_range_,
      parameters_->max_calibration_range_ + initial_distance);

    PointcloudType::Ptr target_thin_pc_ptr = getDensePointcloudFromMap(
      calibration_frame.local_map_pose_, calibration_frame.target_frame_,
      parameters_->calibration_viz_leaf_size_, parameters_->min_calibration_range_,
      parameters_->max_calibration_range_ + initial_distance);

    // Transform the source to target frame to crop it later
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
      *sources[i], *initial_tmp_ptr, calibration_frames[i].local_map_pose_ * initial_transform);
    pcl::transformPointCloud(
      *sources[i], *calibrated_tmp_ptr,
      calibration_frames[i].local_map_pose_ * calibrated_transform);

    pcl::transformPointCloud(
      *targets[i], *target_thin_tmp_ptr, calibration_frames[i].local_map_pose_);

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
