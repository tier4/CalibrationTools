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

#include <extrinsic_tag_based_pnp_calibrator/tag_calibrator_visualizer.hpp>

#include <limits>

TagCalibratorVisualizer::TagCalibratorVisualizer(
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub)
: valid_base_lidar_transform_(false),
  valid_camera_lidar_transform_(false),
  valid_transforms_(false),
  pub_(pub)
{
}

void TagCalibratorVisualizer::setTagSizes(
  std::vector<int64_t> & tag_ids, std::vector<double> & tag_sizes)
{
  assert(tag_ids.size() == tag_sizes.size());

  for (std::size_t i = 0; i < tag_ids.size(); ++i) {
    tag_sizes_map_[tag_ids[i]] = tag_sizes[i];
  }
}

void TagCalibratorVisualizer::setBaseFrame(const std::string & base_frame)
{
  base_frame_ = base_frame;
}

void TagCalibratorVisualizer::setCameraFrame(const std::string & camera_frame)
{
  camera_frame_ = camera_frame;
}

void TagCalibratorVisualizer::setLidarFrame(const std::string & lidar_frame)
{
  lidar_frame_ = lidar_frame;
}

void TagCalibratorVisualizer::setCameraModel(const sensor_msgs::msg::CameraInfo & camera_info)
{
  camera_info_ = camera_info;
  pinhole_camera_model_.fromCameraInfo(camera_info);
}

void TagCalibratorVisualizer::setCameraLidarTransform(
  const cv::Matx31d & translation, const cv::Matx33d & rotation)
{
  cv::Vec3d v(translation(0), translation(1), translation(2));
  camera_lidar_transform_ = cv::Affine3d(rotation, v);
  computeTransforms();

  valid_camera_lidar_transform_ = true;
  valid_transforms_ = valid_base_lidar_transform_ && valid_camera_lidar_transform_;
}

void TagCalibratorVisualizer::setBaseLidarTransform(
  const cv::Matx31d & translation, const cv::Matx33d & rotation)
{
  cv::Vec3d v(translation(0), translation(1), translation(2));
  base_lidar_transform_ = cv::Affine3d(rotation, v);
  computeTransforms();

  valid_base_lidar_transform_ = true;
  valid_transforms_ = valid_base_lidar_transform_ && valid_camera_lidar_transform_;
}

void TagCalibratorVisualizer::drawCalibrationStatus(
  const CalibrationEstimator & estimator, rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray marker_array;
  circle_diameter_ = estimator.getNewHypothesisDistance();
  // In the lidar frame
  // Draw all current lidartags. Corners, ids
  // Draw all converged lidartags

  const auto & active_lidartag_hypotheses = estimator.getActiveLidartagHypotheses();
  const auto & converged_lidartag_hypotheses = estimator.getConvergedLidartagHypotheses();

  drawLidartagHypotheses(active_lidartag_hypotheses, HypothesisType::Active, stamp, marker_array);
  drawLidartagHypotheses(
    converged_lidartag_hypotheses, HypothesisType::Converged, stamp, marker_array);

  // In the image frame
  // Project the apriltags detections in 3d and draw them

  const auto & active_apriltag_hypotheses = estimator.getActiveApriltagHypotheses();
  const auto & converged_apriltag_hypotheses = estimator.getConvergedApriltagHypotheses();

  drawApriltagHypotheses(active_apriltag_hypotheses, HypothesisType::Active, stamp, marker_array);
  drawApriltagHypotheses(
    converged_apriltag_hypotheses, HypothesisType::Converged, stamp, marker_array);

  // We also need the lidar-base tf
  // We draw the FOV of the camera in base, and draw how much have we used the thing
  // Based on the camera info an the current transform

  drawCalibrationZone(stamp, marker_array);

  drawCalibrationStatusText(estimator, stamp, marker_array);

  pub_->publish(marker_array);
}

void TagCalibratorVisualizer::drawLidartagHypotheses(
  const std::vector<std::shared_ptr<tier4_tag_utils::LidartagHypothesis>> & h_vector,
  HypothesisType type, rclcpp::Time & stamp, visualization_msgs::msg::MarkerArray & marker_array)
{
  std::string s = type == HypothesisType::Active ? "active" : "converged";
  double alpha = type == HypothesisType::Active ? 0.2 : 1.0;

  cv::Matx33d base_lidar_rotation_matrix = base_lidar_transform_.rotation();
  cv::Matx31d base_lidar_translation_vector = base_lidar_transform_.translation();

  for (std::size_t i = 0; i < h_vector.size(); i++) {
    const auto & h = h_vector[i];
    const auto & corners = h->getFilteredPoints();
    const auto & center = h->getCenter();
    int id = h->getId();

    cv::Matx31d center_base(center.x, center.y, center.z);
    center_base = base_lidar_rotation_matrix * center_base + base_lidar_translation_vector;

    assert(corners.size() == 4);

    visualization_msgs::msg::Marker line_strip_lcs, id_marker_lcs, center_marker_bcs;

    line_strip_lcs.id = i;
    line_strip_lcs.header.frame_id = lidar_frame_;
    line_strip_lcs.header.stamp = stamp;
    line_strip_lcs.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip_lcs.lifetime = rclcpp::Duration::from_seconds(5.0);
    line_strip_lcs.color.r = 0.0;
    line_strip_lcs.color.g = 1.0;
    line_strip_lcs.color.b = 0.0;
    line_strip_lcs.color.a = alpha;
    line_strip_lcs.ns = s + "_lidartag_frame";
    line_strip_lcs.scale.x = 0.03;

    id_marker_lcs = line_strip_lcs;
    id_marker_lcs.ns = s + "_lidartag_id";
    id_marker_lcs.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    id_marker_lcs.text = std::to_string(id);
    id_marker_lcs.scale.z = 0.3;
    id_marker_lcs.pose.position.x = center.x;
    id_marker_lcs.pose.position.y = center.y;
    id_marker_lcs.pose.position.z = center.z;
    id_marker_lcs.pose.orientation.w = 1.0;
    marker_array.markers.push_back(id_marker_lcs);

    center_marker_bcs = id_marker_lcs;
    center_marker_bcs.header.frame_id = base_frame_;
    center_marker_bcs.ns = s + "_center";
    center_marker_bcs.type = visualization_msgs::msg::Marker::CYLINDER;
    center_marker_bcs.scale.x = circle_diameter_;
    center_marker_bcs.scale.y = circle_diameter_;
    center_marker_bcs.scale.z = 0.01;
    center_marker_bcs.pose.position.x = center_base(0);
    center_marker_bcs.pose.position.y = center_base(1);
    center_marker_bcs.pose.position.z = 0.0;
    center_marker_bcs.pose.orientation.w = 1.0;
    marker_array.markers.push_back(center_marker_bcs);

    for (std::size_t i = 0; i < corners.size(); ++i) {
      geometry_msgs::msg::Point p;
      p.x = corners[i].x;
      p.y = corners[i].y;
      p.z = corners[i].z;

      line_strip_lcs.points.push_back(p);
    }

    line_strip_lcs.points.push_back(line_strip_lcs.points.front());
    marker_array.markers.push_back(line_strip_lcs);
  }
}

void TagCalibratorVisualizer::drawApriltagHypotheses(
  const std::vector<std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>> & h_vector,
  HypothesisType type, rclcpp::Time & stamp, visualization_msgs::msg::MarkerArray & marker_array)
{
  if (!valid_camera_lidar_transform_) {
    return;
  }

  std::string s = type == HypothesisType::Active ? "active" : "converged";
  double alpha = type == HypothesisType::Active ? 0.2 : 1.0;

  cv::Matx33d lidar_camera_rotation_matrix = lidar_camera_transform_.rotation();
  cv::Matx31d lidar_camera_translation_vector = lidar_camera_transform_.translation();

  for (std::size_t i = 0; i < h_vector.size(); i++) {
    const auto & h = h_vector[i];
    const auto & corners = h->getFilteredPoints3d();
    const auto & center = h->getCenter3d();
    int id = h->getId();

    cv::Matx31d center_lcs(center.x, center.y, center.z);
    center_lcs = lidar_camera_rotation_matrix * center_lcs + lidar_camera_translation_vector;

    assert(corners.size() == 4);

    visualization_msgs::msg::Marker line_strip_lcs, id_marker_lcs;

    line_strip_lcs.id = i;
    line_strip_lcs.header.frame_id = lidar_frame_;
    line_strip_lcs.header.stamp = stamp;
    line_strip_lcs.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip_lcs.lifetime = rclcpp::Duration::from_seconds(5.0);
    line_strip_lcs.color.r = 1.0;
    line_strip_lcs.color.g = 1.0;
    line_strip_lcs.color.b = 0.0;
    line_strip_lcs.color.a = alpha;
    line_strip_lcs.ns = s + "_apriltag_frame";
    line_strip_lcs.scale.x = 0.03;

    id_marker_lcs = line_strip_lcs;
    id_marker_lcs.ns = s + "_apriltag_id";
    id_marker_lcs.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    id_marker_lcs.text = std::to_string(id);
    id_marker_lcs.scale.z = 0.3;
    id_marker_lcs.pose.position.x = center_lcs(0);
    id_marker_lcs.pose.position.y = center_lcs(1);
    id_marker_lcs.pose.position.z = center_lcs(2);
    id_marker_lcs.pose.orientation.w = 1.0;
    marker_array.markers.push_back(id_marker_lcs);

    for (const cv::Point3d & corner : corners) {
      cv::Matx31d corner_lcs(corner.x, corner.y, corner.z);
      corner_lcs = lidar_camera_rotation_matrix * corner_lcs + lidar_camera_translation_vector;

      geometry_msgs::msg::Point p;
      p.x = corner_lcs(0);
      p.y = corner_lcs(1);
      p.z = corner_lcs(2);

      line_strip_lcs.points.push_back(p);
    }

    line_strip_lcs.points.push_back(line_strip_lcs.points.front());
    marker_array.markers.push_back(line_strip_lcs);
  }
}

void TagCalibratorVisualizer::drawCalibrationZone(
  rclcpp::Time & stamp, visualization_msgs::msg::MarkerArray & marker_array)
{
  if (!valid_transforms_) {
    return;
  }

  double width = pinhole_camera_model_.fullResolution().width;

  cv::Point3d v_left =
    pinhole_camera_model_.projectPixelTo3dRay(cv::Point2d(0, pinhole_camera_model_.cy()));
  cv::Point3d v_right =
    pinhole_camera_model_.projectPixelTo3dRay(cv::Point2d(width, pinhole_camera_model_.cy()));
  // v = v / cv::norm(v);

  cv::Matx31d v_l(v_left.x, v_left.y, v_left.z);
  cv::Matx31d v_r(v_right.x, v_right.y, v_right.z);

  cv::Matx31d base_camera_translation = base_camera_transform_.translation();
  cv::Matx33d base_camera_rotation = base_camera_transform_.rotation();

  // Second way to get the min distance
  double min_tag_size = std::numeric_limits<double>::max();
  for (auto it = tag_sizes_map_.begin(); it != tag_sizes_map_.end(); it++) {
    min_tag_size = std::min(min_tag_size, it->second);
  }

  double min_calib_dist = 0.0;
  double max_calib_dist = 12.0;  // this is inly for visualization purposes
  double min_angle = std::atan2(v_l(0), v_l(2));
  double max_angle = std::atan2(v_r(0), v_r(2));

  visualization_msgs::msg::Marker line_strip;

  line_strip.id = 0;
  line_strip.header.frame_id = base_frame_;
  line_strip.header.stamp = stamp;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip.lifetime = rclcpp::Duration::from_seconds(5.0);
  line_strip.color.r = 1.0;
  line_strip.color.g = 1.0;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  line_strip.ns = "calibration_zone";
  line_strip.scale.x = 0.01;  // 0.003

  auto camera_to_base_msg = [](
                              double z, double x, const cv::Matx31d & translation,
                              const cv::Matx33d & rotation, geometry_msgs::msg::Point & p_msg) {
    cv::Matx31d p_cv;
    p_cv(0) = x;
    p_cv(1) = 0;
    p_cv(2) = z;
    p_cv = rotation * p_cv + translation;

    p_msg.x = p_cv(0);
    p_msg.y = p_cv(1);
    p_msg.z = 0.0;
  };

  geometry_msgs::msg::Point p_msg_bcs;
  cv::Matx31d p_cv_ccs, p_cv_bcs;
  double angle = min_angle;

  camera_to_base_msg(
    min_calib_dist * std::cos(angle), min_calib_dist * std::sin(angle), base_camera_translation,
    base_camera_rotation, p_msg_bcs);
  line_strip.points.push_back(p_msg_bcs);

  camera_to_base_msg(
    max_calib_dist * std::cos(angle), max_calib_dist * std::sin(angle), base_camera_translation,
    base_camera_rotation, p_msg_bcs);
  line_strip.points.push_back(p_msg_bcs);

  const int num_points = 30;

  for (int i = 0; i < num_points; ++i) {
    angle = min_angle + (i / static_cast<float>(num_points - 1)) * (max_angle - min_angle);

    camera_to_base_msg(
      max_calib_dist * std::cos(angle), max_calib_dist * std::sin(angle), base_camera_translation,
      base_camera_rotation, p_msg_bcs);
    line_strip.points.push_back(p_msg_bcs);

    line_strip.points.push_back(p_msg_bcs);
  }

  angle = max_angle;
  camera_to_base_msg(
    max_calib_dist * std::cos(angle), max_calib_dist * std::sin(angle), base_camera_translation,
    base_camera_rotation, p_msg_bcs);
  line_strip.points.push_back(p_msg_bcs);
  line_strip.points.push_back(p_msg_bcs);

  angle = max_angle;
  camera_to_base_msg(
    min_calib_dist * std::cos(angle), min_calib_dist * std::sin(angle), base_camera_translation,
    base_camera_rotation, p_msg_bcs);
  line_strip.points.push_back(p_msg_bcs);

  for (int i = 0; i < num_points; ++i) {
    angle = max_angle + (i / static_cast<float>(num_points - 1)) * (min_angle - max_angle);
    camera_to_base_msg(
      min_calib_dist * std::cos(angle), min_calib_dist * std::sin(angle), base_camera_translation,
      base_camera_rotation, p_msg_bcs);

    line_strip.points.push_back(p_msg_bcs);
  }

  marker_array.markers.push_back(line_strip);
}

void TagCalibratorVisualizer::drawCalibrationStatusText(
  const CalibrationEstimator & estimator, rclcpp::Time & stamp,
  visualization_msgs::msg::MarkerArray & marker_array)
{
  if (!valid_base_lidar_transform_) {
    return;
  }

  auto to_string_with_precision = [](const double value, const int n = 2) -> std::string {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << value;
    return out.str();
  };

  const auto & h_vector = estimator.getActiveLidartagHypotheses();

  cv::Matx33d base_lidar_rotation_matrix = base_lidar_transform_.rotation();
  cv::Matx31d base_lidar_translation_vector = base_lidar_transform_.translation();

  visualization_msgs::msg::Marker text_marker;

  text_marker.id = 0;
  text_marker.header.frame_id = base_frame_;
  text_marker.header.stamp = stamp;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.lifetime = rclcpp::Duration::from_seconds(5.0);
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.ns = "calibration_status";
  text_marker.scale.z = 0.3;

  text_marker.text =
    "pairs=" + std::to_string(estimator.getCurrentCalibrationPairsNumber()) +
    "\ncoverage=" + to_string_with_precision(estimator.getCalibrationCoveragePercentage()) +
    "\ncrossval_reproj_error=" +
    to_string_with_precision(estimator.getCrossValidationReprojError());

  text_marker.pose.position.x = base_lidar_translation_vector(0);
  text_marker.pose.position.y = base_lidar_translation_vector(1);
  text_marker.pose.position.z = base_lidar_translation_vector(2) + 1.0;
  text_marker.pose.orientation.w = 1.0;

  marker_array.markers.push_back(text_marker);

  for (std::size_t i = 0; i < h_vector.size(); i++) {
    const auto & h = h_vector[i];
    const auto & center = h->getCenter();
    int id = h->getId();
    const double time_since_first_observation = h->timeSinceFirstObservation(stamp);
    const double time_since_last_observation = h->timeSinceLastObservation(stamp);

    const double trans_cov = h->getTransCov();
    const double rot_cov = h->getRotCov();
    const double speed = h->getSpeed();

    cv::Matx31d center_base(center.x, center.y, center.z);
    center_base = base_lidar_rotation_matrix * center_base + base_lidar_translation_vector;

    text_marker.id = i;
    text_marker.ns = "active_lidartag_status";
    text_marker.scale.z = 0.1;

    text_marker.text = "id=" + std::to_string(id) +
                       "\nt_total=" + to_string_with_precision(time_since_first_observation) + "/" +
                       to_string_with_precision(min_convergence_time_) +
                       "\nt_since_last=" + to_string_with_precision(time_since_last_observation) +
                       "/" + to_string_with_precision(max_no_observation_time_) +
                       "\ntrans_cov=" + to_string_with_precision(trans_cov, 3) + "/" +
                       to_string_with_precision(lidartag_convergence_transl_, 3) +
                       "\nrot_cov=" + to_string_with_precision(rot_cov, 3) + "/" +
                       to_string_with_precision(lidartag_convergence_rot_, 3) +
                       "\nspeed=" + to_string_with_precision(speed, 3) + "/" +
                       to_string_with_precision(lidartag_convergence_transl_dot_, 3) +
                       to_string_with_precision(lidartag_convergence_rot_dot_, 3);

    text_marker.pose.position.x = center_base(0);
    text_marker.pose.position.y = center_base(1);
    text_marker.pose.position.z = center_base(2) + 1.0;

    marker_array.markers.push_back(text_marker);
  }
}

void TagCalibratorVisualizer::drawAprilTagDetections(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr & apriltag_detection_array)
{
  rclcpp::Time stamp = rclcpp::Time(apriltag_detection_array->header.stamp);

  float fx = camera_info_.k[3 * 0 + 0];
  float fy = camera_info_.k[3 * 1 + 1];
  float cx = camera_info_.k[3 * 0 + 2];
  float cy = camera_info_.k[3 * 1 + 2];

  visualization_msgs::msg::MarkerArray marker_array;

  for (auto & detection : apriltag_detection_array->detections) {
    auto & v = detection.corners;
    int v_size = detection.corners.size();

    cv::Point p1_cv(v[0].x, v[0].y), p2_cv(v[1].x, v[1].y), p3_cv(v[2].x, v[2].y),
      p4_cv(v[3].x, v[3].y);
    cv::Point center_cv(
      0.25f * (p1_cv.x + p2_cv.x + p3_cv.x + p4_cv.x),
      0.25f * (p1_cv.y + p2_cv.y + p3_cv.y + p4_cv.y));
    cv::Point offset_cv(0 * 10.f, 0 * 10.f);

    std::vector<cv::Point3d> object_points = get3dpoints(detection);

    if (object_points.size() == 0) {
      continue;
    }

    visualization_msgs::msg::Marker line_strip_ics, id_marker_ics, corners_id_marker_ics;
    visualization_msgs::msg::Marker line_strip_ccs, /*id_marker_ccs, */ corners_id_marker_ccs;

    line_strip_ics.id = detection.id;
    line_strip_ics.header = camera_info_.header;
    line_strip_ics.header.stamp = stamp;
    line_strip_ics.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip_ics.lifetime = rclcpp::Duration::from_seconds(5.0);
    line_strip_ics.color.r = 0.0;
    line_strip_ics.color.g = 1.0;
    line_strip_ics.color.b = 1.0;
    line_strip_ics.color.a = 1.0;
    line_strip_ccs = line_strip_ics;

    line_strip_ics.ns = "apriltag_ics";
    line_strip_ics.scale.x = 0.01;  // 0.003

    line_strip_ccs.ns = "apriltag_ccs";
    line_strip_ccs.scale.x = 0.003;

    id_marker_ics = line_strip_ics;
    id_marker_ics.ns = "apriltag_id_ics";
    id_marker_ics.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    id_marker_ics.text = std::to_string(detection.id);
    id_marker_ics.scale.z = 0.08;

    corners_id_marker_ics = id_marker_ics;
    corners_id_marker_ics.ns = "apriltag_" + std::to_string(detection.id) + "_corner_id_ics";
    corners_id_marker_ics.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    corners_id_marker_ics.lifetime = rclcpp::Duration::from_seconds(5.0);
    corners_id_marker_ics.scale.z = 0.08;

    corners_id_marker_ccs = corners_id_marker_ics;
    corners_id_marker_ccs.ns = "apriltag_" + std::to_string(detection.id) + "_corner_id_ccs";
    corners_id_marker_ccs.scale.z = 0.3;

    geometry_msgs::msg::Point center;

    for (int i = 0; i < v_size; ++i) {
      geometry_msgs::msg::Point p_ics;
      p_ics.x = (v[i].x - cx) / fx;  // Assumes no distortion -> use image_rect to viz
      p_ics.y = (v[i].y - cy) / fy;
      p_ics.z = 1.f;

      center.x += p_ics.x;
      center.y += p_ics.y;
      center.z += p_ics.z;

      geometry_msgs::msg::Point p_ccs;
      p_ccs.x = object_points[i].x;
      p_ccs.y = object_points[i].y;
      p_ccs.z = object_points[i].z;

      corners_id_marker_ics.id = i;
      corners_id_marker_ics.text = std::to_string(i);
      corners_id_marker_ics.pose.position = p_ics;
      corners_id_marker_ics.pose.orientation.w = 1.0;
      marker_array.markers.push_back(corners_id_marker_ics);

      corners_id_marker_ccs.id = i;
      corners_id_marker_ccs.text = std::to_string(i);
      corners_id_marker_ccs.pose.position = p_ccs;
      corners_id_marker_ccs.pose.orientation.w = 1.0;
      marker_array.markers.push_back(corners_id_marker_ccs);

      line_strip_ics.points.push_back(p_ics);
      line_strip_ccs.points.push_back(p_ccs);
    }

    center.x /= v_size;
    center.y /= v_size;
    center.z /= v_size;
    id_marker_ics.pose.position = center;
    id_marker_ics.pose.orientation.w = 1.0;

    marker_array.markers.push_back(id_marker_ics);

    line_strip_ics.points.push_back(line_strip_ics.points.front());
    line_strip_ccs.points.push_back(line_strip_ccs.points.front());
    marker_array.markers.push_back(line_strip_ics);
    marker_array.markers.push_back(line_strip_ccs);
  }

  pub_->publish(marker_array);
}

void TagCalibratorVisualizer::drawLidarTagDetections(
  const lidartag_msgs::msg::LidarTagDetectionArray::SharedPtr & lidartag_detections_array)
{
  rclcpp::Time stamp = rclcpp::Time(lidartag_detections_array->header.stamp);

  float fx = camera_info_.k[3 * 0 + 0];
  float cx = camera_info_.k[3 * 0 + 2];
  float cy = camera_info_.k[3 * 1 + 2];

  visualization_msgs::msg::MarkerArray marker_array;

  for (auto & detection : lidartag_detections_array->detections) {
    std::vector<cv::Point2d> projected_points;
    std::vector<cv::Point3d> source_points;

    visualization_msgs::msg::Marker line_strip_ccs, line_strip_ics, line_strip_lcs;
    visualization_msgs::msg::Marker id_marker_ccs, corners_id_marker_ccs, id_marker_ics,
      corners_id_marker_ics;

    line_strip_ccs.header = camera_info_.header;
    line_strip_ccs.header.stamp = stamp;
    line_strip_ccs.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip_ccs.lifetime = rclcpp::Duration::from_seconds(5.0);
    line_strip_ccs.id = detection.id;
    line_strip_ics = line_strip_ccs;
    line_strip_lcs = line_strip_ccs;

    line_strip_lcs.header.frame_id = lidar_frame_;

    line_strip_ccs.ns = "lidartag_ccs";
    line_strip_ccs.color.r = 1.0;
    line_strip_ccs.color.g = 1.0;
    line_strip_ccs.color.b = 0.0;
    line_strip_ccs.color.a = 1.0;
    line_strip_ccs.scale.x = 0.03;  // 0.01

    line_strip_ics.ns = "lidartag_ics";
    line_strip_ics.color.r = 1.0;
    line_strip_ics.color.g = 0.0;
    line_strip_ics.color.b = 1.0;
    line_strip_ics.color.a = 1.0;
    line_strip_ics.scale.x = 0.003;

    line_strip_lcs.ns = "lidartag_lcs";
    line_strip_lcs.color.r = 0.0;
    line_strip_lcs.color.g = 1.0;
    line_strip_lcs.color.b = 0.0;
    line_strip_lcs.color.a = 1.0;
    line_strip_lcs.scale.x = 0.03;  // 0.01

    id_marker_ccs = line_strip_ccs;
    id_marker_ccs.ns = "lidartag_ccs_id";
    id_marker_ccs.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    id_marker_ccs.text = std::to_string(detection.id);
    id_marker_ccs.scale.z = 0.3;

    corners_id_marker_ccs = line_strip_ccs;
    corners_id_marker_ccs.ns = "lidartag_ccs_" + std::to_string(detection.id) + "_corner_id";
    corners_id_marker_ccs.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    corners_id_marker_ccs.scale.z = 0.3;

    id_marker_ics = line_strip_ics;
    id_marker_ics.ns = "lidartag_ics_id";
    id_marker_ics.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    id_marker_ics.text = std::to_string(detection.id);
    id_marker_ics.scale.z = 0.08;

    corners_id_marker_ics = line_strip_ics;
    corners_id_marker_ics.ns = "lidartag_ics_" + std::to_string(detection.id) + "_corner_id";
    corners_id_marker_ics.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    corners_id_marker_ics.scale.z = 0.08;

    geometry_msgs::msg::Point center_ccs, center_ics;

    cv::Matx33d camera_lidar_rotation_matrix = camera_lidar_transform_.rotation();
    cv::Matx31d camera_lidar_translation_vector = camera_lidar_transform_.translation();

    for (std::size_t i = 0; i < detection.vertices.size(); i++) {
      const auto & corner = detection.vertices[i];

      cv::Mat p_lcs = (cv::Mat_<double>(3, 1) << corner.x, corner.y, corner.z);
      cv::Mat p_ccs = camera_lidar_rotation_matrix * p_lcs + camera_lidar_translation_vector;

      cv::Point3d p3d_ccs =
        cv::Point3d(p_ccs.at<double>(0, 0), p_ccs.at<double>(0, 1), p_ccs.at<double>(0, 2));

      cv::Point2d p2d_ics = pinhole_camera_model_.project3dToPixel(p3d_ccs);

      geometry_msgs::msg::Point p_msg_ccs, p_msg_ics;

      p_msg_ccs.x = p3d_ccs.x;
      p_msg_ccs.y = p3d_ccs.y;
      p_msg_ccs.z = p3d_ccs.z;

      p_msg_ics.x = (p2d_ics.x - cx) / fx;
      p_msg_ics.y = (p2d_ics.y - cy) / fx;
      p_msg_ics.z = 1.f;

      center_ccs.x += p_msg_ccs.x;
      center_ccs.y += p_msg_ccs.y;
      center_ccs.z += p_msg_ccs.z;

      center_ics.x += p_msg_ics.x;
      center_ics.y += p_msg_ics.y;
      center_ics.z += p_msg_ics.z;

      corners_id_marker_ccs.id = i;
      corners_id_marker_ccs.text = std::to_string(i);
      corners_id_marker_ccs.pose.position = p_msg_ccs;
      corners_id_marker_ccs.pose.orientation.w = 1.0;
      marker_array.markers.push_back(corners_id_marker_ccs);

      corners_id_marker_ics.id = i;
      corners_id_marker_ics.text = std::to_string(i);
      corners_id_marker_ics.pose.position = p_msg_ics;
      corners_id_marker_ics.pose.orientation.w = 1.0;
      marker_array.markers.push_back(corners_id_marker_ics);

      line_strip_lcs.points.push_back(corner);
      line_strip_ccs.points.push_back(p_msg_ccs);
      line_strip_ics.points.push_back(p_msg_ics);
    }

    int num_corners = detection.vertices.size();

    center_ccs.x /= num_corners;
    center_ccs.y /= num_corners;
    center_ccs.z /= num_corners;
    id_marker_ccs.pose.position = center_ccs;
    id_marker_ccs.pose.orientation.w = 1.0;

    marker_array.markers.push_back(id_marker_ccs);

    center_ics.x /= num_corners;
    center_ics.y /= num_corners;
    center_ics.z /= num_corners;
    id_marker_ics.pose.position = center_ics;
    id_marker_ics.pose.orientation.w = 1.0;

    marker_array.markers.push_back(id_marker_ccs);
    marker_array.markers.push_back(id_marker_ics);

    line_strip_lcs.points.push_back(line_strip_lcs.points.front());
    line_strip_ccs.points.push_back(line_strip_ccs.points.front());
    line_strip_ics.points.push_back(line_strip_ics.points.front());

    marker_array.markers.push_back(line_strip_lcs);
    marker_array.markers.push_back(line_strip_ccs);
    marker_array.markers.push_back(line_strip_ics);
  }

  pub_->publish(marker_array);
}

void TagCalibratorVisualizer::displayImagePoints(
  const std::vector<cv::Point2d> & points, const rclcpp::Time & stamp)
{
  float fx = camera_info_.k[3 * 0 + 0];
  float fy = camera_info_.k[3 * 1 + 1];
  float cx = camera_info_.k[3 * 0 + 2];
  float cy = camera_info_.k[3 * 1 + 2];

  visualization_msgs::msg::MarkerArray marker_array;

  for (std::size_t i = 0; i < points.size(); i++) {
    const cv::Point & p_img = points[i];
    cv::Point offset(30, 30);

    visualization_msgs::msg::Marker circle_marker, id_marker;
    circle_marker.id = i;
    circle_marker.header = camera_info_.header;
    circle_marker.header.stamp = stamp;
    circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
    circle_marker.lifetime = rclcpp::Duration::from_seconds(5.0);
    circle_marker.ns = "manual_image_points";
    circle_marker.color.r = 1.0;
    circle_marker.color.g = 0.0;
    circle_marker.color.b = 0.0;
    circle_marker.color.a = 1.0;
    circle_marker.scale.x = 0.03;
    circle_marker.scale.y = 0.03;
    circle_marker.scale.z = 0.03;

    id_marker = circle_marker;
    id_marker.ns = "manual_image_points_id";
    id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    id_marker.text = std::to_string(i);
    id_marker.scale.z = 0.1;

    geometry_msgs::msg::Point p_circle, p_text;

    p_circle.x = (p_img.x - cx) / fx;
    p_circle.y = (p_img.y - cy) / fy;
    p_circle.z = 1.f;

    p_text.x = (p_img.x + offset.x - cx) / fx;
    p_text.y = (p_img.y + offset.y - cy) / fy;
    p_text.z = 1.f;

    circle_marker.pose.position = p_circle;
    circle_marker.pose.orientation.w = 1.0;

    id_marker.pose.position = p_text;
    id_marker.pose.orientation.w = 1.0;

    marker_array.markers.push_back(circle_marker);
    marker_array.markers.push_back(id_marker);
  }

  pub_->publish(marker_array);
}

void TagCalibratorVisualizer::displayObjectPoints(
  const std::vector<cv::Point3d> & points, const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (std::size_t i = 0; i < points.size(); i++) {
    const cv::Point3d & p_obj = points[i];
    cv::Point offset(0.05, 0.05);

    visualization_msgs::msg::Marker circle_marker, id_marker;
    circle_marker.id = i;
    circle_marker.header.frame_id = lidar_frame_;
    circle_marker.header.stamp = stamp;
    circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
    circle_marker.lifetime = rclcpp::Duration::from_seconds(5.0);
    circle_marker.ns = "manual_object_points";
    circle_marker.color.r = 0.0;
    circle_marker.color.g = 1.0;
    circle_marker.color.b = 0.0;
    circle_marker.color.a = 1.0;
    circle_marker.scale.x = 0.1;
    circle_marker.scale.y = 0.1;
    circle_marker.scale.z = 0.1;

    id_marker = circle_marker;
    id_marker.ns = "manual_object_points_id";
    id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    id_marker.text = std::to_string(i);
    id_marker.scale.z = 0.3;

    geometry_msgs::msg::Point p_circle, p_text;

    p_circle.x = p_obj.x;
    p_circle.y = p_obj.y;
    p_circle.z = p_obj.z;

    p_text.x = p_obj.x + offset.x;
    p_text.y = p_obj.y + offset.y;
    p_text.z = p_obj.z;

    circle_marker.pose.position = p_circle;
    circle_marker.pose.orientation.w = 1.0;

    id_marker.pose.position = p_text;
    id_marker.pose.orientation.w = 1.0;

    marker_array.markers.push_back(circle_marker);
    marker_array.markers.push_back(id_marker);
  }

  pub_->publish(marker_array);
}

std::vector<cv::Point3d> TagCalibratorVisualizer::get3dpoints(
  apriltag_msgs::msg::AprilTagDetection & detection)
{
  std::vector<cv::Point2d> image_points;
  std::vector<cv::Point3d> object_points;

  for (auto & corner : detection.corners) {
    image_points.push_back(cv::Point2d(corner.x, corner.y));
  }

  if (tag_sizes_map_.count(detection.id) == 0) {
    return object_points;
  }

  double board_size = tag_sizes_map_[detection.id];

  std::vector<cv::Point3d> apriltag_template_points = {
    cv::Point3d(-0.5 * board_size, 0.5 * board_size, 0.0),
    cv::Point3d(0.5 * board_size, 0.5 * board_size, 0.0),
    cv::Point3d(0.5 * board_size, -0.5 * board_size, 0.0),
    cv::Point3d(-0.5 * board_size, -0.5 * board_size, 0.0)};

  cv::Mat rvec, tvec;

  bool success = cv::solvePnP(
    apriltag_template_points, image_points, pinhole_camera_model_.intrinsicMatrix(),
    pinhole_camera_model_.distortionCoeffs(), rvec, tvec, false, cv::SOLVEPNP_SQPNP);

  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("teir4_tag_utils"), "PNP failed");
    return object_points;
  }

  cv::Matx31d translation_vector = tvec;
  cv::Matx33d rotation_matrix;

  translation_vector = tvec;
  cv::Rodrigues(rvec, rotation_matrix);

  for (cv::Point3d & template_point : apriltag_template_points) {
    cv::Matx31d p = rotation_matrix * cv::Matx31d(template_point) + translation_vector;
    object_points.push_back(cv::Point3d(p(0), p(1), p(2)));
  }

  return object_points;
}

void TagCalibratorVisualizer::computeTransforms()
{
  lidar_camera_transform_ = camera_lidar_transform_.inv();
  lidar_base_transform_ = base_lidar_transform_.inv();
  base_camera_transform_ = base_lidar_transform_ * lidar_camera_transform_;
  camera_base_transform_ = base_camera_transform_.inv();
}

void TagCalibratorVisualizer::setMinConvergenceTime(double convergence_time)
{
  min_convergence_time_ = convergence_time;
}

void TagCalibratorVisualizer::setMaxNoObservationTime(double time)
{
  max_no_observation_time_ = time;
}

void TagCalibratorVisualizer::setLidartagMaxConvergenceThreshold(
  double transl, double transl_dot, double rot, double rot_dot)
{
  lidartag_convergence_transl_ = transl;
  lidartag_convergence_transl_dot_ = transl_dot;
  lidartag_convergence_rot_ = CV_PI * rot / 180.0;
  lidartag_convergence_rot_dot_ = CV_PI * rot_dot / 180.0;
}

void TagCalibratorVisualizer::setApriltagMaxConvergenceThreshold(double transl)
{
  apriltag_convergence_transl_ = transl;
}
