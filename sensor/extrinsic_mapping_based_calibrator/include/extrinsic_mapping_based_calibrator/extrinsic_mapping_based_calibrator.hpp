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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR_EXTRINSIC_MAPPING_BASED_CALIBRATOR_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR_EXTRINSIC_MAPPING_BASED_CALIBRATOR_HPP_

#include <Eigen/Dense>
#include <extrinsic_mapping_based_calibrator/types.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <rosbag2_interfaces/srv/pause.hpp>
#include <rosbag2_interfaces/srv/resume.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tier4_pcl_extensions/joint_icp_extended.hpp>

#include <tier4_calibration_msgs/srv/calibration_database.hpp>
#include <tier4_calibration_msgs/srv/extrinsic_calibrator.hpp>
#include <tier4_calibration_msgs/srv/frame.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <list>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
class ExtrinsicMappingBasedCalibrator : public rclcpp::Node
{
public:
  using PointPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
  using PointSubscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>;
  using FrameService = rclcpp::Service<tier4_calibration_msgs::srv::Frame>;

  ExtrinsicMappingBasedCalibrator(const rclcpp::NodeOptions & options);

protected:
  // void requestReceivedCallback(
  //  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
  //  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);

  /*!
   * Callback to set parameters using the ROS interface
   * @param[in] parameters vector of new parameters
   */
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  void keyFrameCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response);
  void singleLidarCalibrationCallback(
    const std::string & calibration_frame_name,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response);
  void multipleLidarCalibrationCallback(
    const std::string & calibration_frame_name,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::Frame::Response> response);
  void loadDatabaseCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Response> response);
  void saveDatabaseCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Response> response);

  /*!
   * Message callback for calibration pointclouds (pointclouds in the frame to calibrate)
   * @param[in] pc Calibration pointcloud msg
   */
  void calibrationPointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr pc, const std::string & frame_name);

  /*!
   * Message callback for mapping pointclouds (pointclouds used for the map used as a target during
   * calibration)
   * @param[in] pc Calibration pointcloud msg
   */
  void mappingPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);

  /*!
   * Timer callback. Publishes a 'sparse' map and the trajectory of the mapping
   */
  void publisherTimerCallback();

  /*!
   * Timer callback. Matches mapping keyframes with calibration lidars
   */
  void dataMatchingTimerCallback();

  /*!
   * Matches mapping keyframes with a singular calibration lidar
   */
  void mappingCalibrationDatamatching(const std::string & calibration_frame);

  // Mapping methods

  /*!
   * Dedicated thread to do mapping
   */
  void mappingThreadWorker();

  /*!
   * Initialize a map using a Frame
   * @param[in] frame Source pointcloud
   */
  void initLocalMap(Frame::Ptr frame);

  /*!
   * Check whether a frame is considered a Keyframe
   * @param[in] frame Keyframe candidate
   */
  void checkKeyframe(Frame::Ptr frame);

  /*!
   * Check whether a keyframe is lost
   * @param[in] frame Keyframe candidate
   */
  void checkKeyframeLost(Frame::Ptr frame);

  /*!
   * Recalculate the mapping local map based on the latest keyframes
   */
  void recalculateLocalMap();

  // Calibration methods

  /*!
   * Configure the calibrator parameters
   */
  void configureCalibrators();

  /*!
   * Prepare calibrators for a specific pair of pointclouds
   * @param[in] source_pointcloud_ptr Source pointcloud
   * @param[in] target_pointcloud_ptr Target pointcloud
   */
  void setUpCalibrators(
    PointcloudType::Ptr & source_pointcloud_ptr, PointcloudType::Ptr & target_pointcloud_ptr);

  // General methods

  /*!
   * Compute a map with a user-defined resolution around a certain frame centered in `pose`
   * @param[in] pose The pose in map coordinates the pointcloud should be centered in
   * @param[in] frame Frame to use as a center for constructing the pointcloud
   * @param[in] resolution Max resolution of the resulting pointcloud
   * @param[in] max_range Max range of the resulting pointcloud
   * @retval Source to distance pointcloud distance
   */
  PointcloudType::Ptr getDensePointcloudFromMap(
    const Eigen::Matrix4f & pose, Frame::Ptr & frame, double resolution, double max_range);

  /*!
   * Filter calibration frames that are close to frames where the robot is assumes to be lost
   * @param[in] calibration_frames The raw calibrated frames
   */
  std::vector<CalibrationFrame> filterCalibrationFramesByLostState(
    const std::vector<CalibrationFrame> & calibration_frames);

  /*!
   * Filter calibration frames to avoid high speed, acceleration, interpolation, etc
   * @param[in] calibration_frames The raw calibrated frames
   */
  std::vector<CalibrationFrame> filterCalibrationFramesByDynamics(
    const std::vector<CalibrationFrame> & calibration_frames);

  /*!
   * Select best K calibration frames based on information and spatial correlation
   * @param[in] calibration_frames The raw calibrated frames
   */
  std::vector<CalibrationFrame> selectBestKCalibrationFrames(
    const std::vector<CalibrationFrame> & calibration_frames, int num_frames);

  // Parameters
  std::string base_frame_;
  std::string sensor_kit_frame_;  // calibration parent frame
  std::string lidar_base_frame_;  // calibration child frame
  std::string map_frame_;         // isolated frame to visualize the mapping
  std::vector<std::string>
    calibration_frame_names_;  // calibration_source frame. needs to be a parameter since the
                               // pointcloud may come transformed due to the lidar's pipeline
  std::vector<std::string> calibration_pointcloud_topics_;
  std::string mapping_lidar_frame_;  // calibration_target frame. needs to be a parameter since the
                                     // pointcloud may come transformed due to the lidar's pipeline

  struct Params
  {
    bool use_rosbag_;
    bool verbose_;
    int max_frames_;
    int local_map_num_keyframes_;
    int calibration_num_keyframes_;  // Num of keyframes to use when creating a dense point cloud
    double max_pointcloud_range_;
    double ndt_resolution_;
    double ndt_step_size_;
    int ndt_max_iterations_;
    int ndt_num_threads_;
    double leaf_size_viz_;
    double leaf_size_input_;
    double leaf_size_local_map_;
    double leaf_size_dense_map_;
    double new_keyframe_min_distance_;
    double new_frame_min_distance_;
    double frame_stopped_distance_;
    double frame_nonstopped_distance_;
    int frames_since_stop_force_frame_;
    int calibration_skip_keyframes_;
    int calibration_max_frames_;  // num of max calibration frames to use

    // Calibration preprocessing
    double calibration_max_interpolated_time_;
    double calibration_max_interpolated_distance_;
    double calibration_max_interpolated_angle_;
    double calibration_max_interpolated_speed_;
    double calibration_max_interpolated_accel_;

    double calibration_max_interpolated_distance_straight_;
    double calibration_max_interpolated_angle_straight_;
    double calibration_max_interpolated_speed_straight_;
    double calibration_max_interpolated_accel_straight_;

    double lost_frame_max_angle_diff_;
    double lost_frame_interpolation_error_;

    bool calibration_use_only_stopped_;
    double max_calibration_range_;
    double calibration_min_pca_eigenvalue_;
    double calibration_min_distance_between_frames_;

    // Calibration parameters
    int calibration_solver_iterations_;
    double calibration_max_corr_dist_coarse_;
    double calibration_max_corr_dist_fine_;
    double calibration_max_corr_dist_ultrafine_;
  } params_;

  // ROS Interface
  tf2_ros::StaticTransformBroadcaster tf_broascaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr srv_callback_group_;

  PointPublisher::SharedPtr map_pub_;
  PointPublisher::SharedPtr keyframe_map_pub_;
  PointPublisher::SharedPtr keyframe_pub_;

  std::map<std::string, PointPublisher::SharedPtr> initial_source_aligned_map_pub_map_;
  std::map<std::string, PointPublisher::SharedPtr> calibrated_source_aligned_map_pub_map_;
  std::map<std::string, PointPublisher::SharedPtr> target_map_pub_map_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frame_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr keyframe_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keyframe_markers_pub_;

  std::map<std::string, PointSubscription::SharedPtr> calibration_pointcloud_subs_;
  PointSubscription::SharedPtr mapping_pointcloud_sub_;

  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr service_server_;
  rclcpp::Service<tier4_calibration_msgs::srv::Frame>::SharedPtr keyframe_map_server_;
  std::map<std::string, FrameService::SharedPtr> single_lidar_calibration_server_map_;
  std::map<std::string, FrameService::SharedPtr> multiple_lidar_calibration_server_map_;
  rclcpp::Service<tier4_calibration_msgs::srv::CalibrationDatabase>::SharedPtr
    load_database_server_;
  rclcpp::Service<tier4_calibration_msgs::srv::CalibrationDatabase>::SharedPtr
    save_database_server_;

  rclcpp::Client<rosbag2_interfaces::srv::Pause>::SharedPtr rosbag2_pause_client_;
  rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedPtr rosbag2_resume_client_;

  rclcpp::TimerBase::SharedPtr publisher_timer_;
  rclcpp::TimerBase::SharedPtr data_matching_timer_;

  pclomp::NormalDistributionsTransform<PointType, PointType> ndt_;

  // Threading, sync, and result
  std::mutex mutex_;

  // ROS Data
  std_msgs::msg::Header::SharedPtr mapping_lidar_header_;

  // ROS Publishers buffers
  PointcloudType::Ptr published_map_pointcloud_ptr_;
  nav_msgs::msg::Path published_frames_path_;
  nav_msgs::msg::Path published_keyframes_path_;
  visualization_msgs::msg::MarkerArray published_keyframes_markers_;

  // Calibration matching data
  std::map<std::string, std::list<sensor_msgs::msg::PointCloud2::SharedPtr>>
    calibration_pointclouds_list_map_;
  std::map<std::string, std_msgs::msg::Header::SharedPtr> calibration_lidar_header_map_;
  std::map<std::string, int> last_unmatched_keyframe_map_;

  // Calibration
  std::map<std::string, std::vector<CalibrationFrame>> calibration_frames_map_;
  std::vector<pcl::Registration<PointType, PointType>::Ptr> calibration_registrators_;
  std::vector<pcl::JointIterativeClosestPointExtended<PointType, PointType>::Ptr>
    calibration_batch_registrators_;
  pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr calibration_ndt_;
  pcl::GeneralizedIterativeClosestPoint<PointType, PointType>::Ptr calibration_gicp_;
  pcl::IterativeClosestPoint<PointType, PointType>::Ptr calibration_icp_coarse_;
  pcl::IterativeClosestPoint<PointType, PointType>::Ptr calibration_icp_fine_;
  pcl::IterativeClosestPoint<PointType, PointType>::Ptr calibration_icp_ultrafine_;
  pcl::registration::CorrespondenceEstimation<PointType, PointType>::Ptr correspondence_estimator_;

  pcl::JointIterativeClosestPointExtended<PointType, PointType>::Ptr calibration_batch_icp_coarse_;
  pcl::JointIterativeClosestPointExtended<PointType, PointType>::Ptr calibration_batch_icp_fine_;
  pcl::JointIterativeClosestPointExtended<PointType, PointType>::Ptr
    calibration_batch_icp_ultrafine_;

  // Mapping data
  Eigen::Matrix4f delta_pose_;
  int selected_keyframe_;
  int n_processed_frames_;
  std::list<Frame::Ptr> unprocessed_frames_;
  std::vector<Frame::Ptr> processed_frames_;
  std::vector<Frame::Ptr> keyframes_;
  std::vector<Frame::Ptr> keyframes_and_stopped_;
  pcl::PointCloud<PointType>::Ptr local_map_ptr_;
  std::vector<geometry_msgs::msg::PoseStamped> trajectory_;

  // Rosbag interface
  bool bag_paused_;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR_EXTRINSIC_MAPPING_BASED_CALIBRATOR_HPP_
