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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__MATH_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__MATH_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <extrinsic_tag_based_base_calibrator/calibration_types.hpp>
#include <extrinsic_tag_based_base_calibrator/math.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace extrinsic_tag_based_base_calibrator
{

/*
 * Method to find the average of a set of rotation quaternions using Singular Value Decomposition
 * Snipped taken from https://gist.github.com/PeteBlackerThe3rd/f73e9d569e29f23e8bd828d7886636a0
 * The algorithm used is described here:
 * https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
 */
Eigen::Vector4d quaternionAverage(std::vector<Eigen::Vector4d> quaternions)
{
  if (quaternions.size() == 0) {
    std::cout << "Error trying to calculate the average quaternion of an empty set!\n";
    return Eigen::Vector4d::Zero();
  }

  // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with
  // itself
  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

  for (std::size_t q = 0; q < quaternions.size(); ++q) {
    A += quaternions[q] * quaternions[q].transpose();
  }

  // normalise with the number of quaternions
  A /= quaternions.size();

  // Compute the SVD of this 4x4 matrix
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::VectorXd singular_values = svd.singularValues();
  Eigen::MatrixXd U = svd.matrixU();

  // find the eigen vector corresponding to the largest eigen value
  int largest_eigen_value_index = -1;
  double largest_eigen_value = -std::numeric_limits<double>::max();

  for (int i = 0; i < singular_values.rows(); ++i) {
    if (singular_values(i) > largest_eigen_value) {
      largest_eigen_value = singular_values(i);
      largest_eigen_value_index = i;
    }
  }

  Eigen::Vector4d average;
  average(0) = U(0, largest_eigen_value_index);
  average(1) = U(1, largest_eigen_value_index);
  average(2) = U(2, largest_eigen_value_index);
  average(3) = U(3, largest_eigen_value_index);

  return average;
}

std::array<cv::Vec3d, 4> tagPoseToCorners(const cv::Affine3d & pose, double size)
{
  std::array<cv::Vec3d, 4> templates{
    cv::Vec3d{-1.0, 1.0, 0.0}, cv::Vec3d{1.0, 1.0, 0.0}, cv::Vec3d{1.0, -1.0, 0.0},
    cv::Vec3d{-1.0, -1.0, 0.0}};

  return std::array<cv::Vec3d, 4>{
    pose * (0.5 * size * templates[0]), pose * (0.5 * size * templates[1]),
    pose * (0.5 * size * templates[2]), pose * (0.5 * size * templates[3])};
}

bool computeGroundPlane(const std::vector<cv::Vec3d> & points, cv::Affine3d & ground_pose)
{
  int num_points = static_cast<int>(points.size());

  if (num_points == 0) {
    return false;
  }

  cv::Mat_<double> pca_input = cv::Mat_<double>(num_points, 3);

  for (int i = 0; i < num_points; i++) {
    pca_input(i, 0) = points[i](0);
    pca_input(i, 1) = points[i](1);
    pca_input(i, 2) = points[i](2);
  }

  cv::PCA pca_analysis(pca_input, cv::Mat_<double>(), cv::PCA::DATA_AS_ROW);

  cv::Matx33d rotation = cv::Matx33d(pca_analysis.eigenvectors).inv();
  cv::Vec3d translation = cv::Vec3d(pca_analysis.mean);

  auto det = cv::determinant(rotation);

  if (det < 0.0) {
    rotation(0, 2) *= -1.0;
    rotation(1, 2) *= -1.0;
    rotation(2, 2) *= -1.0;
    det = cv::determinant(rotation);
  }

  assert(std::abs(det - 1.0) < 1e5);

  ground_pose = cv::Affine3d(rotation, translation);

  // Fix the ground origin to be the projection of the origin into the ground plane
  cv::Vec3d initial_ground_to_origin = ground_pose.inv() * cv::Vec3d(0.0, 0.0, 0.0);

  cv::Vec3d aux = initial_ground_to_origin;
  aux(2) = 0.0;
  cv::Vec3d origin_to_new_ground = ground_pose * aux;

  // Because the pose of apriltags points into the tag, we want the ground pose point into the
  // ground in z
  if (initial_ground_to_origin(2) > 0) {
    // Invert z
    rotation(0, 2) *= -1.0;
    rotation(1, 2) *= -1.0;
    rotation(2, 2) *= -1.0;

    // Invert y to keep the right hand rule (det=1)
    rotation(0, 1) *= -1.0;
    rotation(1, 1) *= -1.0;
    rotation(2, 1) *= -1.0;

    auto det = cv::determinant(rotation);
    assert(std::abs(det - 1.0) < 1e5);

    if (std::abs(det - 1.0) > 1e5) {
      return false;
    }
  }

  ground_pose = cv::Affine3d(rotation, origin_to_new_ground);

  return true;
}

bool computeGroundPlane(
  const std::map<UID, std::shared_ptr<cv::Affine3d>> & poses_map, double tag_size,
  cv::Affine3d & ground_pose)
{
  std::vector<cv::Vec3d> points;

  for (const auto & pose_it : poses_map) {
    std::array<cv::Vec3d, 4> corners = tagPoseToCorners(*pose_it.second, tag_size);
    points.insert(points.end(), corners.begin(), corners.end());
  }

  return computeGroundPlane(points, ground_pose);
}

cv::Affine3d computeBaseLink(
  const cv::Affine3d & left_wheel_pose, const cv::Affine3d right_wheel_pose,
  const cv::Affine3d & ground_pose)
{
  // Compute the base link center
  cv::Vec3d tag_center = 0.5 * (left_wheel_pose.translation() + right_wheel_pose.translation());
  cv::Vec3d base_link_translation = ground_pose.inv() * tag_center;
  base_link_translation(2) = 0.0;
  base_link_translation = ground_pose * base_link_translation;

  cv::Vec3d base_link_direction = base_link_translation / cv::norm(base_link_translation);

  cv::Vec3d base_link_z_axis(ground_pose.rotation().col(2).val);
  double factor = base_link_z_axis.dot(base_link_direction) > 0.0 ? -1.0 : 1.0;
  base_link_z_axis = factor * base_link_z_axis;

  cv::Vec3d base_link_y_axis = left_wheel_pose.translation() - right_wheel_pose.translation();
  base_link_y_axis = ground_pose.rotation().inv() * base_link_y_axis;
  base_link_y_axis(2) = 0.0;
  base_link_y_axis = base_link_y_axis / cv::norm(base_link_y_axis);
  base_link_y_axis = ground_pose.rotation() * base_link_y_axis;

  cv::Vec3d base_link_x_axis = base_link_y_axis.cross(base_link_z_axis);

  auto fill_rotation_from_column = [](cv::Matx33d & rotation, const cv::Vec3d column, int index) {
    rotation(0, index) = column(0);
    rotation(1, index) = column(1);
    rotation(2, index) = column(2);
  };

  cv::Matx33d base_link_rotation;
  fill_rotation_from_column(base_link_rotation, base_link_x_axis, 0);
  fill_rotation_from_column(base_link_rotation, base_link_y_axis, 1);
  fill_rotation_from_column(base_link_rotation, base_link_z_axis, 2);

  return cv::Affine3d(base_link_rotation, base_link_translation);
}

cv::Point2d projectPoint(
  const cv::Vec3d & p, double fx, double fy, double cx, double cy, double k1, double k2)
{
  const double xp = p(0) / p(2);
  const double yp = p(1) / p(2);
  const double r2 = xp * xp + yp * yp;
  const double d = 1.0 + r2 * (k1 + k2 * r2);
  return cv::Point2d(cx + fx * d * xp, cy + fy * d * yp);
}

void estimateInitialPosesAux(
  const UID uid, std::unordered_set<UID> traversed_uids, cv::Affine3d current_pose, int depth,
  std::unordered_map<UID, std::vector<cv::Affine3d>> & raw_poses_map, CalibrationData & data,
  const int max_depth)
{
  raw_poses_map[uid].push_back(current_pose);
  traversed_uids.insert(uid);

  if (++depth > max_depth) {
    return;
  }

  for (const UID & next_uid : data.uid_connections_map[uid]) {
    if (traversed_uids.count(next_uid) > 0) {
      continue;
    }

    const cv::Affine3d rel_pose = data.detections_relative_poses_map[std::make_pair(uid, next_uid)];
    const cv::Affine3d next_pose = current_pose * rel_pose;

    estimateInitialPosesAux(
      next_uid, traversed_uids, next_pose, depth, raw_poses_map, data, max_depth);
  }
}

void estimateInitialPoses(
  CalibrationData & data, const UID & main_sensor_uid, UID & left_wheel_uid, UID & right_wheel_uid,
  int max_depth)
{
  std::unordered_map<UID, std::vector<cv::Affine3d>> raw_poses_map;

  cv::Affine3d identity = cv::Affine3d::Identity();
  estimateInitialPosesAux(
    main_sensor_uid, std::unordered_set<UID>(), identity, 0, raw_poses_map, data, max_depth);

  for (const auto & it : raw_poses_map) {
    const UID & uid = it.first;
    const std::vector<cv::Affine3d> & poses = it.second;

    Eigen::Vector3d avg_translation = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector4d> quats;

    for (auto & pose : poses) {
      Eigen::Vector3d translation;
      Eigen::Matrix3d rotation;
      cv::cv2eigen(pose.translation(), translation);
      cv::cv2eigen(pose.rotation(), rotation);
      Eigen::Quaterniond quat(rotation);
      quats.emplace_back(quat.w(), quat.x(), quat.y(), quat.z());

      avg_translation += translation;
    }

    avg_translation /= poses.size();
    Eigen::Vector4d avg_quat = quaternionAverage(quats);

    Eigen::Matrix3d avg_rotation =
      Eigen::Quaterniond(avg_quat(0), avg_quat(1), avg_quat(2), avg_quat(3)).toRotationMatrix();

    cv::Vec3d avg_pose_translation;
    cv::Matx33d avg_pose_rotation;
    cv::eigen2cv(avg_translation, avg_pose_translation);
    cv::eigen2cv(avg_rotation, avg_pose_rotation);

    auto initial_pose = std::make_shared<cv::Affine3d>(avg_pose_rotation, avg_pose_translation);

    if (uid.tag_type != TagType::Unknown) {
      data.initial_tag_poses_map[uid] = initial_pose;
    } else if (uid.sensor_type != SensorType::Unknown) {
      data.initial_sensor_poses_map[uid] = initial_pose;
    } else {
      throw std::domain_error("Invalid UID");
    }

    if (uid.tag_type == TagType::GroundTag) {
      data.initial_ground_tag_poses_map[uid] = initial_pose;
    }

    if (uid == left_wheel_uid) {
      data.initial_left_wheel_tag_pose = initial_pose;
    }

    if (uid == right_wheel_uid) {
      data.initial_right_wheel_tag_pose = initial_pose;
    }
  }

  data.optimized_camera_intrinsics_map = data.initial_camera_intrinsics_map;
  data.optimized_ground_tag_poses_map = data.initial_ground_tag_poses_map;
  data.optimized_left_wheel_tag_pose = data.initial_left_wheel_tag_pose;
  data.optimized_right_wheel_tag_pose = data.initial_right_wheel_tag_pose;
  data.optimized_sensor_poses_map = data.initial_sensor_poses_map;
  data.optimized_tag_poses_map = data.initial_tag_poses_map;
}

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__MATH_HPP_
