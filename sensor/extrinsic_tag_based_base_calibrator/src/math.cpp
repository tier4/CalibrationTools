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

#include <Eigen/SVD>
#include <extrinsic_tag_based_base_calibrator/calibration_types.hpp>
#include <extrinsic_tag_based_base_calibrator/math.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>

#include <iostream>
#include <limits>

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
  const std::vector<std::shared_ptr<cv::Affine3d>> & poses, double tag_size,
  cv::Affine3d & ground_pose)
{
  std::vector<cv::Vec3d> points;

  for (const auto & pose : poses) {
    std::array<cv::Vec3d, 4> corners = tagPoseToCorners(*pose, tag_size);
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

void estimateInitialPoses(CalibrationData & data, const UID & main_sensor_uid)
{
  // Input : the scenes
  //          the main sensor

  // Keep a dictionary of all the UIDs in the scenes, which tracks if they were added or not
  // Keep a dictionary from all the poses from the main sensor

  // Iterate all scenes all sensors
  //  For the detections of said sensors, check if one of them already has an entry in the pose
  //  dictionary If there is compute the pose of said sensor. Compute the pose of all the remaining
  //  sensors without a pose Mark the iteration as successful Stop when the iteration has no
  //  successes

  // Check that all the elements in the diationary have valid poses, otherwise pose an error

  // Based on th

  assert(false);
  CV_UNUSED(data);
  CV_UNUSED(main_sensor_uid);

  return;

  /*
  // Estimate the the initial poses for all the tags
  std::map<UID, std::map<UID, cv::Affine3d>> uid_poses_map; // TODO(knzo25): implement the hash
  method to UID to enable the use of unordered maps std::map<UID, bool> uid_status_map;

  // First get all the UIDs  in the dict and set them to false
  for (const CalibrationScene & scene : scenes) {

    //std::vector<LidartagDetection> calibration_lidar_detections;
    //std::map<TagType, std::vector<ApriltagGridDetection>> calibration_camera_detections;
    //std::vector<ExternalCameraFrame> external_camera_frames;
    for(const LidartagDetection & lidar_detection : scene.calibration_lidar_detections) {
      const int & id = lidar_detection.id;
      const UID = UID::makeWaypointUID(scene_id, id);
    }
  }

  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    const CalibrationScene & scene = data_->scenes[scene_index];

    // Add the waypoints seen from the calibration sensor
    for (auto & detection : scene.calibration_camera_detections) {
      UID waypoint_uid = UID::makeWaypointUID(scene_index, detection.id);
      poses_vector_map[waypoint_uid].push_back(detection.pose);
    }

    for (auto & detection : scene.calibration_lidar_detections) {
      UID waypoint_uid = UID::makeWaypointUID(scene_index, detection.id);
      poses_vector_map[waypoint_uid].push_back(detection.pose);
    }

    // Add the remaining tags and poses when possib;e
    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      auto & frame = scene.external_camera_frames[frame_id];
      std::vector<ApriltagDetection> waypoint_detections;
      std::vector<ApriltagDetection> wheel_detections;
      std::vector<ApriltagDetection> ground_detections;

      std::copy_if(
        frame.detections.begin(), frame.detections.end(), std::back_inserter(waypoint_detections),
        [this](const ApriltagDetection & detection) {
          return waypoint_tag_ids_set_.count(detection.id) > 0;
        });

      std::copy_if(
        frame.detections.begin(), frame.detections.end(), std::back_inserter(wheel_detections),
        [this](const ApriltagDetection & detection) {
          return wheel_tag_ids_set_.count(detection.id) > 0;
        });

      std::copy_if(
        frame.detections.begin(), frame.detections.end(), std::back_inserter(ground_detections),
        [this](const ApriltagDetection & detection) {
          return ground_tag_ids_set_.count(detection.id) > 0;
        });

      for (const auto & waypoint_detection : waypoint_detections) {
        UID waypoint_uid = UID::makeWaypointUID(scene_index, waypoint_detection.id);
        cv::Affine3d sensor_to_waypoint_pose = poses_vector_map[waypoint_uid].back();

        const cv::Affine3d & external_camera_to_waypoint_pose = waypoint_detection.pose;

        cv::Affine3d sensor_to_external_camera_pose =
          sensor_to_waypoint_pose * external_camera_to_waypoint_pose.inv();

        UID external_camera_uid = UID::makeExternalCameraUID(scene_index, frame_id);

        poses_vector_map[external_camera_uid].push_back(sensor_to_external_camera_pose);

        for (const auto & wheel_detection : wheel_detections) {
          const cv::Affine3d & external_camera_to_wheel_pose = wheel_detection.pose;

          cv::Affine3d sensor_to_wheel_pose =
            sensor_to_external_camera_pose * external_camera_to_wheel_pose;

          UID wheel_tag_uid = UID::makeWheelTagUID(wheel_detection.id);
          poses_vector_map[wheel_tag_uid].push_back(sensor_to_wheel_pose);
        }

        for (const auto & ground_detection : ground_detections) {
          cv::Affine3d external_camera_to_ground_pose = ground_detection.pose;
          cv::Affine3d sensor_to_ground_pose = sensor_to_waypoint_pose *
                                               external_camera_to_waypoint_pose.inv() *
                                               external_camera_to_ground_pose;

          UID ground_tag_uid = UID::makeGroundTagUID(ground_detection.id);
          poses_vector_map[ground_tag_uid].push_back(sensor_to_ground_pose);
        }
      }
    }
  }

  // Some external cameras are not conected to the waypoints, so we make another pass
  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    const CalibrationScene & scene = data_->scenes[scene_index];

    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      // Need to make sure all the cameras are in the map
      UID external_camera_uid = UID::makeExternalCameraUID(scene_index, frame_id);

      auto & frame = scene.external_camera_frames[frame_id];
      std::vector<ApriltagDetection> linked_detections;
      std::vector<ApriltagDetection> unlinked_wheel_detections;
      std::vector<ApriltagDetection> unlinked_ground_detections;

      std::copy_if(
        frame.detections.begin(), frame.detections.end(), std::back_inserter(linked_detections),
        [this, &poses_vector_map](const ApriltagDetection & detection) {
          UID wheel_tag_uid = UID::makeWheelTagUID(detection.id);
          UID ground_tag_uid = UID::makeGroundTagUID(detection.id);
          return poses_vector_map.count(wheel_tag_uid) > 0 ||
                 poses_vector_map.count(ground_tag_uid) > 0;
        });

      std::copy_if(
        frame.detections.begin(), frame.detections.end(),
        std::back_inserter(unlinked_wheel_detections),
        [this, &poses_vector_map](const ApriltagDetection & detection) {
          UID wheel_tag_uid = UID::makeWheelTagUID(detection.id);
          return wheel_tag_ids_set_.count(detection.id) > 0 &&
                 poses_vector_map.count(wheel_tag_uid) == 0;
        });

      std::copy_if(
        frame.detections.begin(), frame.detections.end(),
        std::back_inserter(unlinked_ground_detections),
        [this, &poses_vector_map](const ApriltagDetection & detection) {
          UID ground_tag_uid = UID::makeGroundTagUID(detection.id);
          return ground_tag_ids_set_.count(detection.id) > 0 &&
                 poses_vector_map.count(ground_tag_uid) == 0;
        });

      assert(linked_detections.size() > 0);

      if (poses_vector_map.count(external_camera_uid) == 0) {
        auto & linked_detection = linked_detections.front();

        UID wheel_tag_uid = UID::makeWheelTagUID(linked_detection.id);
        UID ground_tag_uid = UID::makeGroundTagUID(linked_detection.id);
        cv::Affine3d sensor_to_linked_tag = poses_vector_map.count(wheel_tag_uid) > 0
                                              ? poses_vector_map[wheel_tag_uid].front()
                                              : poses_vector_map[ground_tag_uid].front();

        const cv::Affine3d & external_camera_to_linked_tag_affine = linked_detection.pose;
        poses_vector_map[external_camera_uid].push_back(
          sensor_to_linked_tag * external_camera_to_linked_tag_affine.inv());
      }

      for (auto & unlinked_wheel_detection : unlinked_wheel_detections) {
        auto & linked_detection = linked_detections.front();

        UID unlinked_wheel_tag_uid = UID::makeWheelTagUID(unlinked_wheel_detection.id);
        UID linked_wheel_tag_uid = UID::makeWheelTagUID(linked_detection.id);
        UID linked_ground_tag_uid = UID::makeGroundTagUID(linked_detection.id);
        cv::Affine3d sensor_to_linked_tag = poses_vector_map.count(linked_wheel_tag_uid) > 0
                                              ? poses_vector_map[linked_wheel_tag_uid].front()
                                              : poses_vector_map[linked_ground_tag_uid].front();

        const cv::Affine3d & external_camera_to_linked_tag_affine = linked_detection.pose;
        const cv::Affine3d & external_camera_to_unlinked_tag_affine = unlinked_wheel_detection.pose;

        poses_vector_map[unlinked_wheel_tag_uid].push_back(
          sensor_to_linked_tag * external_camera_to_linked_tag_affine.inv() *
          external_camera_to_unlinked_tag_affine);
      }

      for (auto & unlinked_ground_detection : unlinked_ground_detections) {
        auto & linked_detection = linked_detections.front();

        UID unlinked_ground_tag_uid = UID::makeGroundTagUID(unlinked_ground_detection.id);
        UID linked_wheel_tag_uid = UID::makeWheelTagUID(linked_detection.id);
        UID linked_ground_tag_uid = UID::makeGroundTagUID(linked_detection.id);

        cv::Affine3d sensor_to_linked_tag = poses_vector_map.count(linked_wheel_tag_uid) > 0
                                              ? poses_vector_map[linked_wheel_tag_uid].front()
                                              : poses_vector_map[linked_ground_tag_uid].front();

        const cv::Affine3d & external_camera_to_linked_tag_affine = linked_detection.pose;
        const cv::Affine3d & external_camera_to_unlinked_tag_affine =
          unlinked_ground_detection.pose;

        poses_vector_map[unlinked_ground_tag_uid].push_back(
          sensor_to_linked_tag * external_camera_to_linked_tag_affine.inv() *
          external_camera_to_unlinked_tag_affine);
      }
    }
  }

  std::array<double, CalibrationData::INTRINSICS_DIM> initial_intrinsics;
  initial_intrinsics[0] = external_camera_intrinsics_.undistorted_camera_matrix(0, 2);
  initial_intrinsics[1] = external_camera_intrinsics_.undistorted_camera_matrix(1, 2);
  initial_intrinsics[2] = external_camera_intrinsics_.undistorted_camera_matrix(0, 0);
  initial_intrinsics[3] = external_camera_intrinsics_.undistorted_camera_matrix(1, 1);
  initial_intrinsics[4] = 0.0;
  initial_intrinsics[5] = 0.0;

  // Obtain the initial poses from averages
  for (auto it = poses_vector_map.begin(); it != poses_vector_map.end(); it++) {
    const UID & uid = it->first;
    auto poses = it->second;

    RCLCPP_INFO(
      this->get_logger(), "UID: %s \tposes: %lu", it->first.to_string().c_str(), it->second.size());

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
    (void)initial_pose;

    if (uid.is_tag) {
      data_->initial_tag_poses_map[uid] = initial_pose;

      if (uid.is_waypoint_tag) {
        data_->initial_waypoint_tag_poses.push_back(initial_pose);
      } else if (uid.is_ground_tag) {
        data_->initial_ground_tag_poses.push_back(initial_pose);
      } else if (uid.is_wheel_tag && uid.tag_id == left_wheel_tag_id_) {
        data_->initial_left_wheel_tag_pose = initial_pose;
      } else if (uid.is_wheel_tag && uid.tag_id == right_wheel_tag_id_) {
        data_->initial_right_wheel_tag_pose = initial_pose;
      }
    } else if (uid.is_camera) {
      data_->initial_external_camera_poses[uid] = initial_pose;
      data_->initial_external_camera_intrinsics[uid] =
        std::make_shared<std::array<double, CalibrationData::INTRINSICS_DIM>>(initial_intrinsics);
    }
  }
  */
}

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__MATH_HPP_
