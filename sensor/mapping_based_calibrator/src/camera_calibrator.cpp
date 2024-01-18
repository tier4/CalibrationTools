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

#include <mapping_based_calibrator/camera_calibrator.hpp>
#include <mapping_based_calibrator/filters/best_frames_filter.hpp>
#include <mapping_based_calibrator/filters/dynamics_filter.hpp>
#include <mapping_based_calibrator/filters/lost_state_filter.hpp>
#include <mapping_based_calibrator/filters/object_detection_filter.hpp>
#include <mapping_based_calibrator/filters/sequential_filter.hpp>
#include <opencv2/core.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tier4_calibration_pcl_extensions/voxel_grid_triplets.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl_conversions/pcl_conversions.h>

CameraCalibrator::CameraCalibrator(
  const std::string & calibration_camera_optical_link_frame,
  CalibrationParameters::Ptr & parameters, MappingData::Ptr & mapping_data,
  std::shared_ptr<tf2_ros::Buffer> & tf_buffer, PointPublisher::SharedPtr & target_map_pub,
  MarkersPublisher::SharedPtr & target_markers_pub)
: SensorCalibrator(
    calibration_camera_optical_link_frame,
    "camera_calibrator(" + calibration_camera_optical_link_frame + ")", parameters, mapping_data,
    tf_buffer),
  target_map_pub_(target_map_pub),
  target_markers_pub_(target_markers_pub)
{
  // Filter configuration
  std::shared_ptr<Filter> lost_state_filter(
    new LostStateFilter(Filter::FilterType::CameraFilter, parameters));
  std::shared_ptr<Filter> dynamics_filter(
    new DynamicsFilter(Filter::FilterType::CameraFilter, calibrator_name_, parameters));
  std::shared_ptr<Filter> best_frames_filter(
    new BestFramesFilter(Filter::FilterType::CameraFilter, calibrator_name_, parameters));
  std::shared_ptr<Filter> object_detection_filter(new ObjectDetectionFilter(
    Filter::FilterType::CameraFilter, calibrator_name_, parameters, tf_buffer_));
  std::vector<std::shared_ptr<Filter>> filters =
    parameters_->filter_detections_
      ? std::vector<std::shared_ptr<Filter>>{lost_state_filter, dynamics_filter, best_frames_filter}
      : std::vector<std::shared_ptr<Filter>>{
          lost_state_filter, dynamics_filter, best_frames_filter};

  filter_.reset(
    new SequentialFilter(Filter::FilterType::CameraFilter, calibrator_name_, parameters, filters));

  configureCalibrators();
}

void CameraCalibrator::configureCalibrators() {}

std::tuple<bool, Eigen::Matrix4d, float> CameraCalibrator::calibrate()
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
    filter_->filter(data_->camera_calibration_frames_map_[calibrator_sensor_frame_], data_);

  if (static_cast<int>(calibration_frames.size()) < parameters_->camera_calibration_min_frames_) {
    RCLCPP_WARN(rclcpp::get_logger(calibrator_name_), "Insufficient calibration frames. aborting.");
    return std::make_tuple<>(false, Eigen::Matrix4d::Identity(), 0.f);
  }

  // Prepare augmented calibration pointclouds
  std::vector<pcl::PointCloud<PointType>::Ptr> targets;
  prepareCalibrationData(
    calibration_frames, initial_calibration_transform, initial_distance, targets);

  // We no longer used the shared data
  lock.unlock();

  // Publish the calibration results
  publishResults(calibration_frames, targets, map_frame, initial_calibration_transform);

  return std::make_tuple<>(true, Eigen::Matrix4d::Identity(), 0.f);
  ;
}

void CameraCalibrator::prepareCalibrationData(
  const std::vector<CalibrationFrame> & calibration_frames,
  const Eigen::Matrix4f & initial_calibration_transform, const float initial_distance,
  std::vector<pcl::PointCloud<PointType>::Ptr> & targets)
{
  RCLCPP_INFO(
    rclcpp::get_logger(calibrator_name_),
    "Preparing dense calibration pointclouds from the map...");

  // Time frustum-ing the last pointcloud or all the pointclouds in between
  auto & camera_info = calibration_frames.front().source_camera_info_;
  float fx = camera_info->p[0];
  float fy = camera_info->p[5];
  float fov_x = (180.f / CV_PI) * 2 * std::atan(0.5f * camera_info->width / fx);
  float fov_y = (180.f / CV_PI) * 2 * std::atan(0.5f * camera_info->height / fy);

  pcl::FrustumCulling<PointType> filter;
  filter.setNearPlaneDistance(parameters_->pc_features_min_distance_);
  filter.setFarPlaneDistance(parameters_->pc_features_max_distance_);
  filter.setHorizontalFOV(fov_x);
  filter.setVerticalFOV(fov_y);

  Eigen::Matrix4f adapter_matrix = Eigen::Matrix4f::Zero();
  adapter_matrix(2, 0) = 1;
  adapter_matrix(1, 1) = -1;
  adapter_matrix(0, 2) = 1;
  adapter_matrix(3, 3) = 1;

  // Prepare pointclouds for calibration
  for (auto & calibration_frame : calibration_frames) {
    PointcloudType::Ptr target_pc_ptr = getDensePointcloudFromMap(
      calibration_frame.local_map_pose_, calibration_frame.target_frame_,
      parameters_->leaf_size_dense_map_, parameters_->min_calibration_range_,
      parameters_->max_calibration_range_ + initial_distance);

    filter.setCameraPose(initial_calibration_transform * adapter_matrix);
    filter.setInputCloud(target_pc_ptr);
    filter.filter(*target_pc_ptr);

    targets.push_back(target_pc_ptr);
  }
}

void CameraCalibrator::publishResults(
  const std::vector<CalibrationFrame> & calibration_frames,
  const std::vector<pcl::PointCloud<PointType>::Ptr> & targets, const std::string & map_frame,
  const Eigen::Matrix4f & initial_calibration_transform)
{
  image_geometry::PinholeCameraModel pinhole_camera_model_;
  pinhole_camera_model_.fromCameraInfo(calibration_frames.front().source_camera_info_);

  auto size = pinhole_camera_model_.fullResolution();
  cv::Point3d corner1 = parameters_->pc_features_max_distance_ *
                        pinhole_camera_model_.projectPixelTo3dRay(cv::Point2d(0.0, 0.0));
  cv::Point3d corner2 = parameters_->pc_features_max_distance_ *
                        pinhole_camera_model_.projectPixelTo3dRay(cv::Point2d(size.width, 0.0));
  cv::Point3d corner3 =
    parameters_->pc_features_max_distance_ *
    pinhole_camera_model_.projectPixelTo3dRay(cv::Point2d(size.width, size.height));
  cv::Point3d corner4 = parameters_->pc_features_max_distance_ *
                        pinhole_camera_model_.projectPixelTo3dRay(cv::Point2d(0.0, size.height));

  std::array<Eigen::Vector4f, 4> corners_ccs{
    Eigen::Vector4f(corner1.x, corner1.y, corner1.z, 1.f),
    Eigen::Vector4f(corner2.x, corner2.y, corner2.z, 1.f),
    Eigen::Vector4f(corner3.x, corner3.y, corner3.z, 1.f),
    Eigen::Vector4f(corner4.x, corner4.y, corner4.z, 1.f)};

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;

  marker.header = calibration_frames.front().source_header_;
  marker.header.frame_id = map_frame;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  // marker.lifetime = rclcpp::Duration::from_seconds(5.0);
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.ns = "camera_fov";
  marker.scale.x = 0.05;

  for (std::size_t calibration_frame_id = 0; calibration_frame_id < calibration_frames.size();
       calibration_frame_id++) {
    Eigen::Matrix4f map_camera_transform =
      calibration_frames[calibration_frame_id].local_map_pose_ * initial_calibration_transform;
    Eigen::Vector4f camera_center_mcs = map_camera_transform.col(3);

    std::array<geometry_msgs::msg::Point, 4> corners_mcs;
    geometry_msgs::msg::Point center_mcs;
    center_mcs.x = camera_center_mcs.x();
    center_mcs.y = camera_center_mcs.y();
    center_mcs.z = camera_center_mcs.z();

    for (int corner_id = 0; corner_id < 4; corner_id++) {
      auto p = map_camera_transform * corners_ccs[corner_id];
      corners_mcs[corner_id].x = p.x();
      corners_mcs[corner_id].y = p.y();
      corners_mcs[corner_id].z = p.z();
    }

    marker.id = calibration_frame_id;
    marker.points.clear();
    marker.points.push_back(center_mcs);

    // Center to corners marker
    for (int corner_id = 0; corner_id < 4; corner_id++) {
      marker.points.push_back(corners_mcs[corner_id]);
      marker.points.push_back(center_mcs);
    }

    // Corner to corner marker
    for (int corner_id = 0; corner_id < 5; corner_id++) {
      marker.points.push_back(corners_mcs[corner_id % 4]);
    }

    markers.markers.push_back(marker);
  }

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
  target_markers_pub_->publish(markers);
}
