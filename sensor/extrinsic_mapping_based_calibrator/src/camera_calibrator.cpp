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

#include <extrinsic_mapping_based_calibrator/camera_calibrator.hpp>
#include <extrinsic_mapping_based_calibrator/filters/best_frames_filter.hpp>
#include <extrinsic_mapping_based_calibrator/filters/dynamics_filter.hpp>
#include <extrinsic_mapping_based_calibrator/filters/lost_state_filter.hpp>
#include <extrinsic_mapping_based_calibrator/filters/object_detection_filter.hpp>
#include <extrinsic_mapping_based_calibrator/filters/sequential_filter.hpp>
#include <tier4_pcl_extensions/voxel_grid_triplets.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#define UNUSED(x) (void)x;

CameraCalibrator::CameraCalibrator(
  const std::string & calibration_camera_optical_link_frame,
  CalibrationParameters::Ptr & parameters, MappingData::Ptr & mapping_data,
  std::shared_ptr<tf2_ros::Buffer> & tf_buffer, PointPublisher::SharedPtr & target_map_pub)
: SensorCalibrator(
    calibration_camera_optical_link_frame,
    "camera_calibrator(" + calibration_camera_optical_link_frame + ")", parameters, mapping_data,
    tf_buffer),
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
      ? std::vector<std::shared_ptr<Filter>>{lost_state_filter, dynamics_filter, best_frames_filter}
      : std::vector<std::shared_ptr<Filter>>{
          lost_state_filter, dynamics_filter, best_frames_filter};

  filter_.reset(new SequentialFilter(calibrator_name_, parameters, filters));

  configureCalibrators();
}

void CameraCalibrator::configureCalibrators() {}

bool CameraCalibrator::calibrate(Eigen::Matrix4f & best_transform, float & best_score)
{
  std::unique_lock<std::mutex> lock(data_->mutex_);

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

    return false;
  }

  // Filter calibration frames using several criteria and select the best ones suited for
  // calibration
  std::vector<CalibrationFrame> calibration_frames =
    filter_->filter(data_->camera_calibration_frames_map_[calibrator_sensor_frame_], data_);

  if (static_cast<int>(calibration_frames.size()) < parameters_->calibration_min_frames_) {
    RCLCPP_WARN(rclcpp::get_logger(calibrator_name_), "Insufficient calibration frames. aborting.");
    return false;
  }

  // Prepate augmented calibration pointclouds
  std::vector<pcl::PointCloud<PointType>::Ptr> targets;
  prepareCalibrationData(calibration_frames, initial_distance, targets);

  // We no lnoger used the shared data
  lock.unlock();

  // Publish the calbiraton resullts
  publishResults(calibration_frames, targets, map_frame);

  best_transform = Eigen::Matrix4f::Identity();
  best_score = 0.f;

  return true;
}

void CameraCalibrator::prepareCalibrationData(
  const std::vector<CalibrationFrame> & calibration_frames, const float initial_distance,
  std::vector<pcl::PointCloud<PointType>::Ptr> & targets)
{
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_),
    "Preparing dense calibration pointclouds from the map...");

  // Prepare pointclouds for calibration
  for (auto & calibration_frame : calibration_frames) {
    PointcloudType::Ptr target_pc_ptr = getDensePointcloudFromMap(
      calibration_frame.target_frame_->pose_, calibration_frame.target_frame_,
      parameters_->leaf_size_dense_map_, parameters_->max_calibration_range_ + initial_distance);

    targets.push_back(target_pc_ptr);
  }
}

void CameraCalibrator::publishResults(
  const std::vector<CalibrationFrame> & calibration_frames,
  const std::vector<pcl::PointCloud<PointType>::Ptr> & targets, const std::string & map_frame)
{
  // Publish ROS data
  PointcloudType::Ptr target_map_ptr(new PointcloudType());
  PointcloudType::Ptr target_thin_map_ptr(new PointcloudType());

  for (std::size_t i = 0; i < calibration_frames.size(); i++) {
    PointcloudType::Ptr target_tmp_ptr(new PointcloudType());

    pcl::transformPointCloud(
      *targets[i], *target_tmp_ptr, calibration_frames[i].target_frame_->pose_);

    *target_map_ptr += *target_tmp_ptr;
  }

  pcl::VoxelGridTriplets<PointType> voxel_grid;
  voxel_grid.setLeafSize(
    parameters_->calibration_viz_leaf_size_, parameters_->calibration_viz_leaf_size_,
    parameters_->calibration_viz_leaf_size_);
  voxel_grid.setInputCloud(target_map_ptr);
  voxel_grid.filter(*target_thin_map_ptr);

  sensor_msgs::msg::PointCloud2 target_map_msg;
  target_thin_map_ptr->width = target_thin_map_ptr->points.size();
  target_thin_map_ptr->height = 1;

  pcl::toROSMsg(*target_thin_map_ptr, target_map_msg);

  target_map_msg.header = calibration_frames[0].source_header_;
  target_map_msg.header.frame_id = map_frame;

  target_map_pub_->publish(target_map_msg);
}

void CameraCalibrator::singleSensorCalibrationCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response)
{
  UNUSED(request);
  UNUSED(response);
}

void CameraCalibrator::multipleSensorCalibrationCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response)
{
  UNUSED(request);
  UNUSED(response);

  Eigen::Matrix4f result;
  float score;
  response->success = calibrate(result, score);
}
