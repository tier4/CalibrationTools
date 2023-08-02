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
#include <Eigen/Geometry>
#include <extrinsic_tag_based_base_calibrator/apriltag_detection.hpp>
#include <extrinsic_tag_based_base_calibrator/math.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <tier4_tag_utils/cv/sqpnp.hpp>

#include <limits>

namespace extrinsic_tag_based_base_calibrator
{

LidartagDetection LidartagDetection::fromLidartagDetectionMsg(
  const lidartag_msgs::msg::LidarTagDetection & msg, double scale_factor)
{
  // Lidartag and apriltag poses have different directions (orientation) so we need to choose one
  // and adapt the other We choose to use the coordinate system defined by apriltag and convert the
  // lidartag poses to match the apriltag definitions E.j corner^{1}_{apriltag} = (-hsize, hsize, 0)
  // <=> corner^{1}_{lidartag} = (0, -hsize, -hsize) corner^{2}_{apriltag} = (hsize, hsize, 0)   <=>
  // corner^{2}_{lidartag} = (0, hsize, -hsize) corner^{3}_{apriltag} = (hsize, -hsize, 0)  <=>
  // corner^{3}_{lidartag} = (0, hsize, hsize) corner^{4}_{apriltag} = (-hsize, -hsize, 0) <=>
  // corner^{4}_{lidartag} = (0, -hsize, hsize) corner_{lidartag} = R * corner_{apriltag} R = [[0,
  // 0, -1],
  //      [1,  0, 0],
  //      [0, -1, 0]]
  // Rot_{apriltag} = R^{T} * Rot_{lidartag}

  LidartagDetection detection;
  detection.id = msg.id;
  detection.size = msg.size * scale_factor;

  Eigen::Isometry3d pose_eigen;
  tf2::fromMsg(msg.pose, pose_eigen);

  Eigen::Vector3d translation_eigen = pose_eigen.translation();
  Eigen::Matrix3d rotation_eigen = pose_eigen.rotation();

  Eigen::Matrix3d apriltag_to_lidartag_rot;
  apriltag_to_lidartag_rot << 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0;

  rotation_eigen = rotation_eigen * apriltag_to_lidartag_rot;

  cv::Vec3d translation_cv;
  cv::Matx33d rotation_cv;
  cv::eigen2cv(translation_eigen, translation_cv);
  cv::eigen2cv(rotation_eigen, rotation_cv);

  detection.computeTemplateCorners();
  detection.pose = cv::Affine3d(rotation_cv, translation_cv);
  detection.computeObjectCorners();

  return detection;
}

void LidartagDetection::computeTemplateCorners()
{
  assert(size > 0.0);
  double hsize = 0.5 * size;

  template_corners = {
    {-hsize, hsize, 0.0}, {hsize, hsize, 0.0}, {hsize, -hsize, 0.0}, {-hsize, -hsize, 0.0}};
}

void LidartagDetection::computeObjectCorners()
{
  object_corners.resize(template_corners.size());
  for (std::size_t corner_index = 0; corner_index < template_corners.size(); corner_index++) {
    cv::Point3d p = pose * template_corners[corner_index];
    object_corners[corner_index] = p;  // cv::Point3d(p(0), p(1), p(2));
  }
}

ApriltagDetection ApriltagDetection::fromApriltagDetectionMsg(
  const apriltag_msgs::msg::AprilTagDetection & msg, const IntrinsicParameters & intrinsics,
  double size)
{
  ApriltagDetection detection;

  detection.id = msg.id;
  detection.center = cv::Point2d(msg.centre.x, msg.centre.y);
  detection.family = msg.family;

  std::transform(
    msg.corners.begin(), msg.corners.end(), std::back_inserter(detection.image_corners),
    [](const apriltag_msgs::msg::Point & p) { return cv::Point2d(p.x, p.y); });

  detection.size = size;
  detection.computeTemplateCorners();
  detection.computeObjectCorners();
  double reprojection_error = detection.computePose(intrinsics);
  CV_UNUSED(reprojection_error);

  return detection;
}

double ApriltagDetection::computePose(const IntrinsicParameters & intrinsics)
{
  std::vector<cv::Point2d> undistorted_points;

  cv::undistortPoints(
    image_corners, undistorted_points, intrinsics.camera_matrix, intrinsics.dist_coeffs);

  assert(template_corners.size() > 0);
  assert(template_corners.size() == undistorted_points.size());

  cv::sqpnp::PoseSolver solver;
  std::vector<cv::Mat> rvec_vec, tvec_vec;
  solver.solve(template_corners, undistorted_points, rvec_vec, tvec_vec);

  if (tvec_vec.size() == 0) {
    assert(false);
    return std::numeric_limits<double>::infinity();
  }

  assert(rvec_vec.size() == 1);
  cv::Mat rvec = rvec_vec[0];
  cv::Mat tvec = tvec_vec[0];

  // cv::Matx31d translation_vector = tvec;
  // cv::Matx33d rotation_matrix;

  // translation_vector = tvec;
  // cv::Rodrigues(rvec, rotation_matrix);

  pose = cv::Affine3d(rvec, tvec);
  computeObjectCorners();

  return computeReprojError(intrinsics);
}

double ApriltagDetection::computeReprojError(const IntrinsicParameters & intrinsics) const
{
  return computeReprojError(
    intrinsics.undistorted_camera_matrix(0, 2), intrinsics.undistorted_camera_matrix(1, 2),
    intrinsics.undistorted_camera_matrix(0, 0), intrinsics.undistorted_camera_matrix(1, 1));
}

double ApriltagDetection::computeReprojError(double cx, double cy, double fx, double fy) const
{
  assert(object_corners.size() == image_corners.size());

  double error = 0;
  for (std::size_t corner_index = 0; corner_index < object_corners.size(); corner_index++) {
    const auto & object_corner = object_corners[corner_index];
    const auto & image_corner = image_corners[corner_index];
    double prx = cx + fx * (object_corner.x / object_corner.z);
    double pry = cy + fy * (object_corner.y / object_corner.z);
    double errx = std::abs(prx - image_corner.x);
    double erry = std::abs(pry - image_corner.y);
    error += std::sqrt(errx * errx + erry * erry);
  }

  return error / object_corners.size();
}

double ApriltagDetection::detectionDiagonalRatio() const
{
  assert(image_corners.size() == 4);
  cv::Point2d ac = image_corners[3] - image_corners[1];
  cv::Point2d bd = image_corners[2] - image_corners[0];
  return std::min(cv::norm(ac), cv::norm(bd)) / std::max(cv::norm(ac), cv::norm(bd));
}

void ApriltagGridDetection::computeTemplateCorners(const TagParameters & tag_parameters)
{
  template_corners.clear();

  for (auto & sub_detection : sub_detections) {
    int offset = sub_detection.id - id;
    assert(offset >= 0 && offset < rows * cols);

    double factor = tag_parameters.size * (1.0 + tag_parameters.spacing);

    int row = offset / cols;
    int col = offset % cols;
    double corner_offset_x = (col - 0.5 * (cols - 1)) * factor;
    double corner_offset_y = -1.0 * (row - 0.5 * (rows - 1)) * factor;
    cv::Point3d corner_offset(corner_offset_x, corner_offset_y, 0.0);

    sub_detection.computeTemplateCorners();

    for (auto & template_corner : sub_detection.template_corners) {
      template_corner = template_corner + corner_offset;
    }
    template_corners.insert(
      template_corners.end(), sub_detection.template_corners.begin(),
      sub_detection.template_corners.end());
  }
}

void ApriltagGridDetection::computeObjectCorners()
{
  object_corners.resize(template_corners.size());
  for (std::size_t corner_index = 0; corner_index < template_corners.size(); corner_index++) {
    cv::Point3d p = pose * template_corners[corner_index];
    object_corners[corner_index] = p;  // cv::Point3d(p(0), p(1), p(2));
  }

  for (auto & sub_detection : sub_detections) {
    for (std::size_t corner_index = 0; corner_index < sub_detection.template_corners.size();
         corner_index++) {
      cv::Point3d p = pose * sub_detection.template_corners[corner_index];
      sub_detection.object_corners[corner_index] = p;  // cv::Point3d(p(0), p(1), p(2));
    }
  }
}

double ApriltagGridDetection::recomputeFromSubDetections(const TagParameters & tag_parameters)
{
  image_corners.clear();
  template_corners.clear();
  object_corners.clear();

  for (auto & sub_detection : sub_detections) {
    int offset = sub_detection.id - id;
    assert(offset >= 0 && offset < rows * cols);

    double factor = tag_parameters.size * (1.0 + tag_parameters.spacing);

    int row = offset / cols;
    int col = offset % cols;
    double corner_offset_x = (col - 0.5 * (cols - 1)) * factor;
    double corner_offset_y = -1.0 * (row - 0.5 * (rows - 1)) * factor;
    cv::Point3d corner_offset(corner_offset_x, corner_offset_y, 0.0);

    for (auto & template_corner : sub_detection.template_corners) {
      template_corner = template_corner + corner_offset;
    }

    image_corners.insert(
      image_corners.end(), sub_detection.image_corners.begin(), sub_detection.image_corners.end());
    template_corners.insert(
      template_corners.end(), sub_detection.template_corners.begin(),
      sub_detection.template_corners.end());
    object_corners.insert(
      object_corners.end(), sub_detection.object_corners.begin(),
      sub_detection.object_corners.end());
  }

  center = std::accumulate(image_corners.begin(), image_corners.end(), cv::Point2d(0.0, 0.0)) /
           (4 * rows * cols);

  Eigen::Vector3d avg_translation = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector4d> quats;

  for (auto & detection : sub_detections) {
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    cv::cv2eigen(detection.pose.translation(), translation);
    cv::cv2eigen(detection.pose.rotation(), rotation);
    Eigen::Quaterniond quat(rotation);
    quats.emplace_back(quat.w(), quat.x(), quat.y(), quat.z());

    avg_translation += translation;
  }

  avg_translation /= sub_detections.size();
  Eigen::Vector4d avg_quat = quaternionAverage(quats);

  Eigen::Matrix3d avg_rotation =
    Eigen::Quaterniond(avg_quat(0), avg_quat(1), avg_quat(2), avg_quat(3)).toRotationMatrix();

  cv::Vec3d avg_pose_translation;
  cv::Matx33d avg_pose_rotation;
  cv::eigen2cv(avg_translation, avg_pose_translation);
  cv::eigen2cv(avg_rotation, avg_pose_rotation);

  pose = cv::Affine3d(avg_pose_rotation, avg_pose_translation);

  std::vector<double> center_distances;
  std::transform(
    sub_detections.begin(), sub_detections.end(), std::back_inserter(center_distances),
    [&](const ApriltagDetection & detection) {
      return cv::norm(detection.pose.translation() - pose.translation());
    });

  double max_distance = *std::max_element(center_distances.begin(), center_distances.end());

  return max_distance;
}

GroupedApriltagGridDetections ApriltagGridDetection::fromGroupedApriltagDetections(
  const GroupedApriltagDetections & grouped_detections,
  std::unordered_map<TagType, TagParameters> & tag_parameters_map)
{
  GroupedApriltagGridDetections grouped_grid_detections;

  // Group tags into grid
  for (const auto & it : grouped_detections) {
    const TagType & tag_type = it.first;
    const ApriltagDetections & detections = it.second;
    std::vector<ApriltagGridDetection> & grid_detections = grouped_grid_detections[tag_type];

    const TagParameters & tag_parameters = tag_parameters_map.at(tag_type);
    const int & rows = tag_parameters.rows;
    const int & cols = tag_parameters.cols;
    const int & size = tag_parameters.size;

    std::unordered_map<int, std::vector<ApriltagDetection>> partial_detections_map;

    // Group the detections into their respective grids
    for (const ApriltagDetection & detection : detections) {
      int base_id = detection.id / (rows * cols);
      partial_detections_map[base_id].push_back(detection);
    }

    // Create the grid detections after checking their status and consistency
    for (const auto & partial_detections_it : partial_detections_map) {
      if (partial_detections_it.second.size() != static_cast<std::size_t>(rows * cols)) {
        continue;
      }

      ApriltagGridDetection grid_detection;
      grid_detection.rows = rows;
      grid_detection.cols = cols;
      grid_detection.size = size;
      grid_detection.id = partial_detections_it.first;
      grid_detection.family = tag_parameters.family;

      grid_detection.sub_detections.insert(
        grid_detection.sub_detections.end(), partial_detections_it.second.begin(),
        partial_detections_it.second.end());

      // Recompute the corners, center, and pose
      double max_distance = grid_detection.recomputeFromSubDetections(tag_parameters);
      if (max_distance > tag_parameters.size * (1.0 + tag_parameters.spacing)) {
        RCLCPP_WARN(
          rclcpp::get_logger("apriltag_detector"),
          "There was a misdetection filtered through pose consistency");
        continue;
      }

      grid_detections.push_back(grid_detection);
    }
  }

  return grouped_grid_detections;
}

double ApriltagGridDetection::detectionDiagonalRatio() const
{
  return std::transform_reduce(
           sub_detections.cbegin(), sub_detections.cend(), 0.0, std::plus{},
           [](const auto & detection) { return detection.detectionDiagonalRatio(); }) /
         (rows * cols);
}

}  // namespace extrinsic_tag_based_base_calibrator
