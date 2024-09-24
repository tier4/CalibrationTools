// Copyright 2024 TIER IV, Inc.
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
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tag_based_sfm_calibrator/ceres/calibration_problem.hpp>
#include <tag_based_sfm_calibrator/ceres/camera_residual.hpp>
#include <tag_based_sfm_calibrator/ceres/lidar_residual.hpp>
#include <tag_based_sfm_calibrator/math.hpp>
#include <tag_based_sfm_calibrator/visualization.hpp>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <numeric>
#include <optional>

namespace tag_based_sfm_calibrator
{

void CalibrationProblem::setOptimizeIntrinsics(bool ba_optimize_intrinsics)
{
  optimize_intrinsics_ = ba_optimize_intrinsics;
}

void CalibrationProblem::setShareIntrinsics(bool ba_share_intrinsics)
{
  share_intrinsics_ = ba_share_intrinsics;
}

void CalibrationProblem::setForceSharedGroundPlane(bool ba_force_shared_ground_plane)
{
  force_shared_ground_plane_ = ba_force_shared_ground_plane;
}

void CalibrationProblem::setFixedSharedGroundPlane(
  bool ba_fixed_ground_plane_model, Eigen::Vector4d ground_model)
{
  force_fixed_ground_plane_ = ba_fixed_ground_plane_model;

  // This normal points "upwards" or facing the lidar
  Eigen::Vector3d n(ground_model(0), ground_model(1), ground_model(2));
  n.normalize();

  // Our coordinate system, like the apriltag detections, points "inside" the ground
  Eigen::Vector3d x0 = -n * ground_model(3);

  // To create a real pose we need to invent a basis
  Eigen::Vector3d base_x, base_y, base_z;
  base_z = -n;

  Eigen::Vector3d c1 = Eigen::Vector3d(1.0, 0.0, 0.0).cross(n);
  Eigen::Vector3d c2 = Eigen::Vector3d(0.0, 1.0, 0.0).cross(n);
  Eigen::Vector3d c3 = Eigen::Vector3d(0.0, 0.0, 1.0).cross(n);

  // Any non-zero would work but we use the one with the highest norm (there has to be a non zero)
  if (c1.norm() > c2.norm() && c1.norm() > c3.norm()) {
    base_x = c1;
  } else if (c2.norm() > c3.norm()) {
    base_x = c2;
  } else {
    base_x = c3;
  }

  base_y = base_z.cross(base_x);

  Eigen::Matrix3d rot;
  rot.col(0) = base_x.normalized();
  rot.col(1) = base_y.normalized();
  rot.col(2) = base_z.normalized();

  cv::Matx33d cv_rotation;
  cv::Vec3d cv_translation;

  cv::eigen2cv(x0, cv_translation);
  cv::eigen2cv(rot, cv_rotation);

  fixed_ground_pose_ = cv::Affine3d(cv_rotation, cv_translation);
}

void CalibrationProblem::setOptimizationWeights(
  double calibration_camera_weight, double calibration_lidar_weight, double external_camera_weight)
{
  double scale = calibration_camera_weight + calibration_lidar_weight + external_camera_weight;
  calibration_camera_optimization_weight_ = calibration_camera_weight / scale;
  calibration_lidar_optimization_weight_ = calibration_lidar_weight / scale;
  external_camera_optimization_weight_ = external_camera_weight / scale;
}

void CalibrationProblem::setExternalCameraIntrinsics(IntrinsicParameters & intrinsics)
{
  external_camera_intrinsics_ = intrinsics;
}

void CalibrationProblem::setCalibrationLidarIntrinsics(double calibration_lidar_virtual_f)
{
  calibration_lidar_intrinsics_ = calibration_lidar_virtual_f;
}

void CalibrationProblem::setWheelTagUIDs(UID left_wheel_tag_uid, UID right_wheel_tag_uid)
{
  left_wheel_tag_uid_ = left_wheel_tag_uid;
  right_wheel_tag_uid_ = right_wheel_tag_uid;
}

void CalibrationProblem::setData(CalibrationData::Ptr & data) { data_ = data; }

void CalibrationProblem::dataToPlaceholders()
{
  // Compute the initial ground plane !
  std::optional<cv::Affine3d> ground_pose;

  if (force_fixed_ground_plane_) {
    ground_pose = fixed_ground_pose_;
  } else {
    ground_pose = computeGroundPlane(data_->initial_ground_tag_poses_map, 0.0);
    assert(ground_pose.has_value());
  }

  // Prepare the placeholders

  // Sensor placeholders
  for (auto it = data_->initial_sensor_poses_map.begin();
       it != data_->initial_sensor_poses_map.end(); it++) {
    const UID & sensor_uid = it->first;
    const auto & pose = it->second;

    if (sensor_uid.sensor_type == SensorType::CalibrationCamera) {
      pose3dToPlaceholder(*pose, pose_opt_map[sensor_uid], true);
      placeholderToPose3d(
        pose_opt_map[sensor_uid], data_->optimized_sensor_poses_map[sensor_uid], true);

      intrinsics_opt_map[sensor_uid] = *data_->initial_camera_intrinsics_map[sensor_uid];
    } else if (sensor_uid.sensor_type == SensorType::CalibrationLidar) {
      pose3dToPlaceholder(*pose, pose_opt_map[sensor_uid], true);
      placeholderToPose3d(
        pose_opt_map[sensor_uid], data_->optimized_sensor_poses_map[sensor_uid], true);
    } else if (sensor_uid.sensor_type == SensorType::ExternalCamera) {
      pose3dToPlaceholder(*pose, pose_opt_map[sensor_uid], true);

      if (share_intrinsics_) {
        shared_intrinsics_opt = *data_->initial_camera_intrinsics_map[sensor_uid];
      } else {
        intrinsics_opt_map[sensor_uid] = *data_->initial_camera_intrinsics_map[sensor_uid];
      }

      placeholderToPose3d(
        pose_opt_map[sensor_uid], data_->optimized_sensor_poses_map[sensor_uid], true);

      data_->optimized_camera_intrinsics_map[sensor_uid] =
        std::make_shared<std::array<double, INTRINSICS_DIM>>();

      if (share_intrinsics_) {
        *data_->optimized_camera_intrinsics_map[sensor_uid] = shared_intrinsics_opt;
      } else {
        *data_->optimized_camera_intrinsics_map[sensor_uid] = intrinsics_opt_map[sensor_uid];
      }
    } else {
      throw std::domain_error("Invalid UID");
    }
  }

  // Tag poses
  for (auto it = data_->initial_tag_poses_map.begin(); it != data_->initial_tag_poses_map.end();
       it++) {
    const UID & uid = it->first;
    const auto & pose = it->second;

    if (uid.tag_type != TagType::GroundTag || !force_shared_ground_plane_) {
      pose3dToPlaceholder(*pose, pose_opt_map[uid], false);
      placeholderToPose3d(pose_opt_map[uid], data_->optimized_tag_poses_map[uid], false);
    } else {
      pose3dToGroundTagPlaceholder(
        uid, *pose, ground_pose.value(), shrd_ground_tag_pose_opt,
        indep_ground_tag_pose_opt_map[uid]);  // cSpell:ignore shrd
      groundTagPlaceholderToPose3d(
        shrd_ground_tag_pose_opt, indep_ground_tag_pose_opt_map[uid],
        data_->optimized_tag_poses_map[uid]);
    }
  }
}

void CalibrationProblem::placeholdersToData()
{
  for (auto it = data_->optimized_sensor_poses_map.begin();
       it != data_->optimized_sensor_poses_map.end(); it++) {
    const UID & sensor_uid = it->first;
    auto & pose = it->second;

    if (
      sensor_uid.sensor_type == SensorType::CalibrationCamera ||
      sensor_uid.sensor_type == SensorType::CalibrationLidar) {
      placeholderToPose3d(pose_opt_map[sensor_uid], pose, true);
    } else if (sensor_uid.sensor_type == SensorType::ExternalCamera) {
      placeholderToPose3d(pose_opt_map[sensor_uid], pose, true);

      if (share_intrinsics_) {
        *data_->optimized_camera_intrinsics_map[sensor_uid] = shared_intrinsics_opt;
      } else {
        *data_->optimized_camera_intrinsics_map[sensor_uid] = intrinsics_opt_map[sensor_uid];
      }
    } else {
      throw std::domain_error("Invalid UID");
    }
  }

  for (auto it = data_->optimized_tag_poses_map.begin(); it != data_->optimized_tag_poses_map.end();
       it++) {
    const UID & tag_uid = it->first;
    auto & pose = data_->optimized_tag_poses_map[tag_uid];

    if (tag_uid.tag_type != TagType::GroundTag || !force_shared_ground_plane_) {
      placeholderToPose3d(pose_opt_map[tag_uid], pose, false);
    } else {
      groundTagPlaceholderToPose3d(
        shrd_ground_tag_pose_opt, indep_ground_tag_pose_opt_map[tag_uid], pose);
    }

    if (tag_uid.tag_type == TagType::GroundTag) {
      data_->optimized_ground_tag_poses_map[tag_uid] = pose;
    } else if (tag_uid == left_wheel_tag_uid_) {
      data_->optimized_left_wheel_tag_pose = pose;
    } else if (tag_uid == right_wheel_tag_uid_) {
      data_->optimized_right_wheel_tag_pose = pose;
    }
  }
}

void CalibrationProblem::evaluate()
{
  std::map<UID, std::vector<double>> sensor_reprojection_errors_map;
  std::map<TagType, std::vector<double>> tag_reprojection_errors_map;

  auto identity = std::make_shared<std::array<double, POSE_OPT_DIM>>(
    std::array<double, POSE_OPT_DIM>{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    CalibrationScene & scene = data_->scenes[scene_index];

    // Calibration camera
    for (const auto & single_camera_detections : scene.calibration_cameras_detections) {
      const int & calibration_camera_id = single_camera_detections.calibration_camera_id;
      UID calibration_camera_uid =
        UID::makeSensorUID(SensorType::CalibrationCamera, calibration_camera_id);

      for (const auto & group_detections : single_camera_detections.grouped_detections) {
        const TagType tag_type = group_detections.first;

        for (const auto & grid_detection : group_detections.second) {
          const int tag_id = grid_detection.id;
          UID detection_uid = UID::makeTagUID(tag_type, scene_index, tag_id);

          bool camera_status = pose_opt_map.count(calibration_camera_uid) > 0;
          bool detection_status = pose_opt_map.count(detection_uid) > 0 ||
                                  indep_ground_tag_pose_opt_map.count(detection_uid) > 0;
          bool pair_status = data_->invalid_pairs_set.count(
                               std::make_pair(calibration_camera_uid, detection_uid)) == 0;

          if (!camera_status || !detection_status || !pair_status) {
            RCLCPP_ERROR(
              rclcpp::get_logger("calibration_problem"),
              "Skipping %s <-> %s since there are no poses available or was deemed an outlier. "
              "camera_status=%d. detection_status=%d. pair_status=%d",
              calibration_camera_uid.toString().c_str(), detection_uid.toString().c_str(),
              camera_status, detection_status, pair_status);
            continue;
          }

          double sum_res = 0.0;
          const int res_size = RESIDUAL_DIM * grid_detection.cols * grid_detection.rows;

          for (const auto & detection : grid_detection.sub_detections) {
            std::array<double, RESIDUAL_DIM> residuals;

            if (
              detection_uid.tag_type == TagType::AuxiliarTag ||
              detection_uid.tag_type == TagType::WaypointTag ||
              detection_uid.tag_type == TagType::WheelTag ||
              (detection_uid.tag_type == TagType::GroundTag && !force_shared_ground_plane_)) {
              double * calibration_camera_pose_op = pose_opt_map.at(calibration_camera_uid).data();

              auto f = CameraResidual(
                calibration_camera_uid,
                data_->calibration_camera_intrinsics_map_.at(calibration_camera_uid), detection,
                pose_opt_map.at(calibration_camera_uid),
                std::array<double, CalibrationData::SHRD_GROUND_TAG_POSE_DIM>(), false, false,
                false, false);

              f(calibration_camera_pose_op, pose_opt_map.at(detection_uid).data(),
                residuals.data());
            } else if (detection_uid.tag_type == TagType::GroundTag && force_shared_ground_plane_) {
              double * calibration_camera_pose_op = pose_opt_map.at(calibration_camera_uid).data();
              double * shrd_ground_pose_op = shrd_ground_tag_pose_opt.data();
              double * indep_ground_pose_op =
                indep_ground_tag_pose_opt_map.at(detection_uid).data();

              auto f = CameraResidual(
                calibration_camera_uid,
                data_->calibration_camera_intrinsics_map_.at(calibration_camera_uid), detection,
                pose_opt_map.at(calibration_camera_uid),
                std::array<double, CalibrationData::SHRD_GROUND_TAG_POSE_DIM>(), false, false,
                false, true);

              f(calibration_camera_pose_op, shrd_ground_pose_op, indep_ground_pose_op,
                residuals.data());
            } else {
              RCLCPP_ERROR(
                rclcpp::get_logger("calibration_problem"), "%s <-> %s error",
                calibration_camera_uid.toString().c_str(), detection_uid.toString().c_str());
              throw std::domain_error("Invalid residual");
            }

            sum_res += std::transform_reduce(
              residuals.begin(), residuals.end(), sum_res, std::plus{},
              [](auto v) { return std::abs(v); });
          }

          double avg_reprojection_error = sum_res / res_size;
          tag_reprojection_errors_map[tag_type].push_back(avg_reprojection_error);
          sensor_reprojection_errors_map[calibration_camera_uid].push_back(avg_reprojection_error);

          RCLCPP_INFO(
            rclcpp::get_logger("calibration_problem"), "%s <-> %s error: %.2f",
            calibration_camera_uid.toString().c_str(), detection_uid.toString().c_str(),
            avg_reprojection_error);
        }
      }
    }

    // Calibration lidar
    for (const auto & single_lidar_detections : scene.calibration_lidars_detections) {
      const int & lidar_id = single_lidar_detections.calibration_lidar_id;
      UID calibration_lidar_uid = UID::makeSensorUID(SensorType::CalibrationLidar, lidar_id);

      for (auto & detection : single_lidar_detections.detections) {
        const TagType tag_type = TagType::WaypointTag;
        const int tag_id = detection.id;

        UID detection_uid = UID::makeTagUID(tag_type, scene_index, tag_id);

        std::array<double, RESIDUAL_DIM> residuals;

        bool lidar_status = pose_opt_map.count(calibration_lidar_uid) > 0;
        bool detection_status = pose_opt_map.count(detection_uid) > 0 ||
                                indep_ground_tag_pose_opt_map.count(detection_uid) > 0;
        bool pair_status =
          data_->invalid_pairs_set.count(std::make_pair(calibration_lidar_uid, detection_uid)) == 0;

        if (!lidar_status || !detection_status || !pair_status) {
          RCLCPP_ERROR(
            rclcpp::get_logger("calibration_problem"),
            "Skipping %s <-> %s since there are no poses available or was deemed an outlier. "
            "lidar_status=%d. detection_status=%d. pair_status=%d",
            calibration_lidar_uid.toString().c_str(), detection_uid.toString().c_str(),
            lidar_status, detection_status, pair_status);
          continue;
        }

        auto f = LidarResidual(
          calibration_lidar_uid, calibration_lidar_intrinsics_, detection,
          pose_opt_map.at(calibration_lidar_uid), false);

        f(pose_opt_map.at(calibration_lidar_uid).data(), pose_opt_map.at(detection_uid).data(),
          residuals.data());

        double sum_res = std::transform_reduce(
          residuals.begin(), residuals.end(), 0.0, std::plus{}, [](auto v) { return std::abs(v); });

        tag_reprojection_errors_map[tag_type].push_back(sum_res / residuals.size());
        sensor_reprojection_errors_map[calibration_lidar_uid].push_back(sum_res / residuals.size());

        RCLCPP_INFO(
          rclcpp::get_logger("calibration_problem"), "%s <-> %s error: %.2f",
          calibration_lidar_uid.toString().c_str(), detection_uid.toString().c_str(),
          sum_res / residuals.size());
      }
    }

    // External camera
    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      UID external_camera_uid =
        UID::makeSensorUID(SensorType::ExternalCamera, scene_index, frame_id);

      auto & external_camera_frame = scene.external_camera_frames[frame_id];
      // const int & camera_id = frame_id;

      for (auto & group_detections : external_camera_frame.detections) {
        const TagType tag_type = group_detections.first;

        for (auto & grid_detection : group_detections.second) {
          const int tag_id = grid_detection.id;
          UID detection_uid = UID::makeTagUID(tag_type, scene_index, tag_id);

          bool external_camera_status = pose_opt_map.count(external_camera_uid) > 0;
          bool detection_status = pose_opt_map.count(detection_uid) > 0 ||
                                  indep_ground_tag_pose_opt_map.count(detection_uid) > 0;
          bool pair_status =
            data_->invalid_pairs_set.count(std::make_pair(external_camera_uid, detection_uid)) == 0;

          if (!external_camera_status || !detection_status || !pair_status) {
            RCLCPP_ERROR(
              rclcpp::get_logger("calibration_problem"),
              "Skipping %s <-> %s since there are no poses available or was deemed an outlier. "
              "external_camera_status=%d. detection_status=%d. pair_status=%d",
              external_camera_uid.toString().c_str(), detection_uid.toString().c_str(),
              external_camera_status, detection_status, pair_status);
            continue;
          }

          double sum_res = 0.0;
          const int res_size = RESIDUAL_DIM * grid_detection.cols * grid_detection.rows;

          for (const auto & detection : grid_detection.sub_detections) {
            std::array<double, RESIDUAL_DIM> residuals;

            if (
              detection_uid.tag_type == TagType::AuxiliarTag ||
              detection_uid.tag_type == TagType::WaypointTag ||
              detection_uid.tag_type == TagType::WheelTag ||
              (detection_uid.tag_type == TagType::GroundTag && !force_shared_ground_plane_)) {
              double * external_camera_pose_op = pose_opt_map.at(external_camera_uid).data();
              double * external_camera_intrinsics_op =
                share_intrinsics_ ? shared_intrinsics_opt.data()
                                  : intrinsics_opt_map[external_camera_uid].data();

              if (optimize_intrinsics_) {
                auto f = CameraResidual(
                  external_camera_uid, external_camera_intrinsics_, detection,
                  pose_opt_map.at(external_camera_uid),
                  std::array<double, CalibrationData::SHRD_GROUND_TAG_POSE_DIM>(), false, false,
                  true, false);

                f(external_camera_pose_op, external_camera_intrinsics_op,
                  pose_opt_map.at(detection_uid).data(), residuals.data());

              } else {
                auto f = CameraResidual(
                  external_camera_uid, external_camera_intrinsics_, detection,
                  pose_opt_map.at(external_camera_uid),
                  std::array<double, CalibrationData::SHRD_GROUND_TAG_POSE_DIM>(), false, false,
                  false, false);

                f(external_camera_pose_op, pose_opt_map.at(detection_uid).data(), residuals.data());
              }
            } else if (detection_uid.tag_type == TagType::GroundTag && force_shared_ground_plane_) {
              double * external_camera_pose_op = pose_opt_map.at(external_camera_uid).data();
              double * external_camera_intrinsics_op =
                share_intrinsics_ ? shared_intrinsics_opt.data()
                                  : intrinsics_opt_map[external_camera_uid].data();
              double * shrd_ground_pose_op = shrd_ground_tag_pose_opt.data();
              double * indep_ground_pose_op =
                indep_ground_tag_pose_opt_map.at(detection_uid).data();

              if (optimize_intrinsics_) {
                auto f = CameraResidual(
                  external_camera_uid, external_camera_intrinsics_, detection,
                  pose_opt_map.at(external_camera_uid),
                  std::array<double, CalibrationData::SHRD_GROUND_TAG_POSE_DIM>(), false, false,
                  true, true);

                f(external_camera_pose_op, external_camera_intrinsics_op, shrd_ground_pose_op,
                  indep_ground_pose_op, residuals.data());

              } else {
                auto f = CameraResidual(
                  external_camera_uid, external_camera_intrinsics_, detection,
                  pose_opt_map.at(external_camera_uid),
                  std::array<double, CalibrationData::SHRD_GROUND_TAG_POSE_DIM>(), false, false,
                  false, true);

                f(external_camera_pose_op, shrd_ground_pose_op, indep_ground_pose_op,
                  residuals.data());
              }
            } else {
              RCLCPP_ERROR(
                rclcpp::get_logger("calibration_problem"), "%s <-> %s error",
                external_camera_uid.toString().c_str(), detection_uid.toString().c_str());
              throw std::domain_error("Invalid residual");
            }

            sum_res += std::transform_reduce(
              residuals.begin(), residuals.end(), sum_res, std::plus{},
              [](auto v) { return std::abs(v); });
          }

          double avg_reprojection_error = sum_res / res_size;
          UID common_external_camera_uid = external_camera_uid;
          common_external_camera_uid.scene_id = 0;
          common_external_camera_uid.frame_id = 0;
          tag_reprojection_errors_map[tag_type].push_back(avg_reprojection_error);
          sensor_reprojection_errors_map[common_external_camera_uid].push_back(
            avg_reprojection_error);

          RCLCPP_INFO(
            rclcpp::get_logger("calibration_problem"), "%s <-> %s error: %.2f",
            external_camera_uid.toString().c_str(), detection_uid.toString().c_str(),
            avg_reprojection_error);
        }
      }
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("calibration_problem"), "Reprojection error statistics per tag type");

  for (const auto & [tag_type, reprojection_errors] : tag_reprojection_errors_map) {
    double min_reprojection_error =
      *std::min_element(reprojection_errors.begin(), reprojection_errors.end());
    double max_reprojection_error =
      *std::max_element(reprojection_errors.begin(), reprojection_errors.end());
    double mean_reprojection_error =
      std::accumulate(reprojection_errors.begin(), reprojection_errors.end(), 0.0) /
      reprojection_errors.size();

    std::string tag_type_str = tag_type == TagType::WaypointTag   ? "WaypointTag"
                               : tag_type == TagType::GroundTag   ? "GroundTag"
                               : tag_type == TagType::WheelTag    ? "WheelTag"
                               : tag_type == TagType::AuxiliarTag ? "AuxiliarTag"
                                                                  : "Unknown";

    RCLCPP_INFO(
      rclcpp::get_logger("calibration_problem"),
      "\t%s reprojection errors:  mean=%.2f min=%.2f max=%.2f observations=%lu",
      tag_type_str.c_str(), mean_reprojection_error, min_reprojection_error, max_reprojection_error,
      reprojection_errors.size());
  }

  RCLCPP_INFO(
    rclcpp::get_logger("calibration_problem"), "Reprojection error statistics per sensor");
  for (const auto & [sensor_uid, reprojection_errors] : sensor_reprojection_errors_map) {
    double min_reprojection_error =
      *std::min_element(reprojection_errors.begin(), reprojection_errors.end());
    double max_reprojection_error =
      *std::max_element(reprojection_errors.begin(), reprojection_errors.end());
    double mean_reprojection_error =
      std::accumulate(reprojection_errors.begin(), reprojection_errors.end(), 0.0) /
      reprojection_errors.size();
    std::string sensor_name = sensor_uid.sensor_type == SensorType::ExternalCamera
                                ? "external_camera"
                                : sensor_uid.toString();

    data_->optimized_sensor_residuals_map[sensor_uid] = mean_reprojection_error;

    RCLCPP_INFO(
      rclcpp::get_logger("calibration_problem"),
      "\t%s reprojection errors:  mean=%.2f min=%.2f max=%.2f observations=%lu",
      sensor_name.c_str(), mean_reprojection_error, min_reprojection_error, max_reprojection_error,
      reprojection_errors.size());
  }
}

void CalibrationProblem::solve()
{
  ceres::Problem problem;

  // Compute the optimization weights
  std::size_t num_calibration_camera_detections = 0;
  std::size_t num_calibration_lidar_detections = 0;
  std::size_t num_external_camera_detections = 0;

  for (const auto & scene : data_->scenes) {
    // Calibration camera residuals
    for (const auto & single_camera_detections : scene.calibration_cameras_detections) {
      for (const auto & group_detections : single_camera_detections.grouped_detections) {
        for (auto & grid_detection : group_detections.second) {
          num_calibration_camera_detections += grid_detection.sub_detections.size();
        }
      }
    }

    // Calibration lidar residuals
    for (const auto & single_lidar_detections : scene.calibration_lidars_detections) {
      num_calibration_lidar_detections += single_lidar_detections.detections.size();
    }

    // External camera-related residuals
    for (const auto & external_camera_frame : scene.external_camera_frames) {
      for (const auto & group_detections : external_camera_frame.detections) {
        for (auto & grid_detection : group_detections.second) {
          num_external_camera_detections += grid_detection.sub_detections.size();
        }
      }
    }
  }

  std::size_t num_total_detections = num_calibration_camera_detections +
                                     num_calibration_lidar_detections +
                                     num_external_camera_detections;
  double calibration_camera_residual_weight = calibration_camera_optimization_weight_ *
                                              num_total_detections /
                                              num_calibration_camera_detections;
  double calibration_lidar_residual_weight = calibration_lidar_optimization_weight_ *
                                             num_total_detections /
                                             num_calibration_lidar_detections;
  double external_camera_residual_weight =
    external_camera_optimization_weight_ * num_total_detections / num_external_camera_detections;

  RCLCPP_INFO(
    rclcpp::get_logger("calibration_problem"), "Total observations: %lu", num_total_detections);
  RCLCPP_INFO(
    rclcpp::get_logger("calibration_problem"),
    "\t - calibration camera: weight=%.2f observations=%lu", calibration_camera_residual_weight,
    num_calibration_camera_detections);
  RCLCPP_INFO(
    rclcpp::get_logger("calibration_problem"),
    "\t - calibration lidar: weight=%.2f observations=%lu", calibration_lidar_residual_weight,
    num_calibration_lidar_detections);
  RCLCPP_INFO(
    rclcpp::get_logger("calibration_problem"),
    "\t - external; camera: weight=%.2f observations=%lu", external_camera_residual_weight,
    num_external_camera_detections);

  // Build the optimization problem
  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    CalibrationScene & scene = data_->scenes[scene_index];

    // Calibration camera residuals
    for (const auto & single_camera_detections : scene.calibration_cameras_detections) {
      const int & camera_id = single_camera_detections.calibration_camera_id;
      UID calibration_camera_uid = UID::makeSensorUID(SensorType::CalibrationCamera, camera_id);
      bool fix_sensor_pose = data_->main_calibration_sensor_uid == calibration_camera_uid;

      for (const auto & group_detections : single_camera_detections.grouped_detections) {
        const TagType tag_type = group_detections.first;

        for (auto & grid_detection : group_detections.second) {
          const int tag_id = grid_detection.id;
          UID detection_uid = UID::makeTagUID(tag_type, scene_index, tag_id);

          bool camera_status = pose_opt_map.count(calibration_camera_uid) > 0;
          bool detection_status = pose_opt_map.count(detection_uid) > 0 ||
                                  indep_ground_tag_pose_opt_map.count(detection_uid) > 0;
          bool pair_status = data_->invalid_pairs_set.count(
                               std::make_pair(calibration_camera_uid, detection_uid)) == 0;

          if (!camera_status || !detection_status || !pair_status) {
            RCLCPP_ERROR(
              rclcpp::get_logger("calibration_problem"),
              "Skipping %s <-> %s since there are no poses available or was deemed an outlier. "
              "camera_status=%d. detection_status=%d. pair_status=%d",
              calibration_camera_uid.toString().c_str(), detection_uid.toString().c_str(),
              camera_status, detection_status, pair_status);
            continue;
          }

          for (const auto & detection : grid_detection.sub_detections) {
            if (
              detection_uid.tag_type == TagType::AuxiliarTag ||
              detection_uid.tag_type == TagType::WaypointTag ||
              (detection_uid.tag_type == TagType::GroundTag && !force_shared_ground_plane_)) {
              ceres::CostFunction * res = CameraResidual::createTagResidual(
                calibration_camera_uid,
                data_->calibration_camera_intrinsics_map_.at(calibration_camera_uid), detection,
                pose_opt_map.at(calibration_camera_uid), fix_sensor_pose, false);

              if (fix_sensor_pose) {
                problem.AddResidualBlock(
                  res,
                  new ceres::ScaledLoss(
                    nullptr, calibration_camera_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
                  pose_opt_map.at(detection_uid).data());
              } else {
                problem.AddResidualBlock(
                  res,
                  new ceres::ScaledLoss(
                    nullptr, calibration_camera_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
                  pose_opt_map.at(calibration_camera_uid).data(),
                  pose_opt_map.at(detection_uid).data());
              }
            } else if (detection_uid.tag_type == TagType::GroundTag && force_shared_ground_plane_) {
              double * shrd_ground_pose_op = shrd_ground_tag_pose_opt.data();
              double * indep_ground_pose_op =
                indep_ground_tag_pose_opt_map.at(detection_uid).data();

              ceres::CostFunction * res = CameraResidual::createGroundTagResidual(
                calibration_camera_uid,
                data_->calibration_camera_intrinsics_map_.at(calibration_camera_uid), detection,
                pose_opt_map.at(calibration_camera_uid), shrd_ground_tag_pose_opt, fix_sensor_pose,
                force_fixed_ground_plane_, false);

              if (fix_sensor_pose && !force_fixed_ground_plane_) {
                problem.AddResidualBlock(
                  res,
                  new ceres::ScaledLoss(
                    nullptr, calibration_camera_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
                  shrd_ground_pose_op, indep_ground_pose_op);
              } else if (!fix_sensor_pose && !force_fixed_ground_plane_) {
                problem.AddResidualBlock(
                  res,
                  new ceres::ScaledLoss(
                    nullptr, calibration_camera_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
                  pose_opt_map.at(calibration_camera_uid).data(), shrd_ground_pose_op,
                  indep_ground_pose_op);
              } else if (fix_sensor_pose && force_fixed_ground_plane_) {
                problem.AddResidualBlock(
                  res,
                  new ceres::ScaledLoss(
                    nullptr, calibration_camera_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
                  indep_ground_pose_op);
              } else if (!fix_sensor_pose && force_fixed_ground_plane_) {
                problem.AddResidualBlock(
                  res,
                  new ceres::ScaledLoss(
                    nullptr, calibration_camera_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
                  pose_opt_map.at(calibration_camera_uid).data(), indep_ground_pose_op);
              }
            } else {
              RCLCPP_ERROR(
                rclcpp::get_logger("calibration_problem"), "%s <-> %s error",
                calibration_camera_uid.toString().c_str(), detection_uid.toString().c_str());
              throw std::domain_error("Invalid residual");
            }
          }
        }
      }
    }

    // Calibration lidar residuals
    for (const auto & single_lidar_detections : scene.calibration_lidars_detections) {
      const int & lidar_id = single_lidar_detections.calibration_lidar_id;
      UID calibration_lidar_uid = UID::makeSensorUID(SensorType::CalibrationLidar, lidar_id);
      bool fix_sensor_pose = data_->main_calibration_sensor_uid == calibration_lidar_uid;

      for (const auto & detection : single_lidar_detections.detections) {
        const int tag_id = detection.id;
        UID detection_uid = UID::makeTagUID(TagType::WaypointTag, scene_index, tag_id);

        bool lidar_status = pose_opt_map.count(calibration_lidar_uid) > 0;
        bool detection_status = pose_opt_map.count(detection_uid) > 0 ||
                                indep_ground_tag_pose_opt_map.count(detection_uid) > 0;
        bool pair_status =
          data_->invalid_pairs_set.count(std::make_pair(calibration_lidar_uid, detection_uid)) == 0;

        if (!lidar_status || !detection_status || !pair_status) {
          RCLCPP_ERROR(
            rclcpp::get_logger("calibration_problem"),
            "Skipping %s <-> %s since there are no poses available or was deemed an outlier. "
            "lidar_status=%d. detection_status=%d. pair_status=%d",
            calibration_lidar_uid.toString().c_str(), detection_uid.toString().c_str(),
            lidar_status, detection_status, pair_status);
          continue;
        }

        ceres::CostFunction * res = LidarResidual::createTagResidual(
          calibration_lidar_uid, calibration_lidar_intrinsics_, detection,
          pose_opt_map.at(calibration_lidar_uid), fix_sensor_pose);

        if (fix_sensor_pose) {
          problem.AddResidualBlock(
            res,
            new ceres::ScaledLoss(
              nullptr, calibration_lidar_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
            pose_opt_map.at(detection_uid).data());
        } else {
          problem.AddResidualBlock(
            res,
            new ceres::ScaledLoss(
              nullptr, calibration_lidar_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
            pose_opt_map.at(calibration_lidar_uid).data(), pose_opt_map.at(detection_uid).data());
        }
      }
    }

    // External camera-related residuals
    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      UID external_camera_uid =
        UID::makeSensorUID(SensorType::ExternalCamera, scene_index, frame_id);

      const auto & external_camera_frame = scene.external_camera_frames[frame_id];
      // const int & camera_id = frame_id;

      for (const auto & group_detections : external_camera_frame.detections) {
        const TagType tag_type = group_detections.first;

        for (auto & grid_detection : group_detections.second) {
          const int tag_id = grid_detection.id;
          UID detection_uid = UID::makeTagUID(tag_type, scene_index, tag_id);

          bool external_camera_status = pose_opt_map.count(external_camera_uid) > 0;
          bool detection_status = pose_opt_map.count(detection_uid) > 0 ||
                                  indep_ground_tag_pose_opt_map.count(detection_uid) > 0;
          bool pair_status =
            data_->invalid_pairs_set.count(std::make_pair(external_camera_uid, detection_uid)) == 0;

          if (!external_camera_status || !detection_status || !pair_status) {
            RCLCPP_ERROR(
              rclcpp::get_logger("calibration_problem"),
              "Skipping %s <-> %s since there are no poses available or was deemed an outlier. "
              "external_camera_status=%d. detection_status=%d. pair_status=%d",
              external_camera_uid.toString().c_str(), detection_uid.toString().c_str(),
              external_camera_status, detection_status, pair_status);
            continue;
          }

          for (const auto & detection : grid_detection.sub_detections) {
            if (
              detection_uid.tag_type == TagType::AuxiliarTag ||
              detection_uid.tag_type == TagType::WaypointTag ||
              detection_uid.tag_type == TagType::WheelTag ||
              (detection_uid.tag_type == TagType::GroundTag && !force_shared_ground_plane_)) {
              double * external_camera_intrinsics_op =
                share_intrinsics_ ? shared_intrinsics_opt.data()
                                  : intrinsics_opt_map[external_camera_uid].data();

              if (optimize_intrinsics_) {
                ceres::CostFunction * res = CameraResidual::createTagResidual(
                  external_camera_uid, external_camera_intrinsics_, detection,
                  pose_opt_map.at(external_camera_uid), false, true);

                problem.AddResidualBlock(
                  res,
                  new ceres::ScaledLoss(
                    nullptr, external_camera_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
                  pose_opt_map.at(external_camera_uid).data(), external_camera_intrinsics_op,
                  pose_opt_map.at(detection_uid).data());
              } else {
                ceres::CostFunction * res = CameraResidual::createTagResidual(
                  external_camera_uid, external_camera_intrinsics_, detection,
                  pose_opt_map.at(external_camera_uid), false, false);

                problem.AddResidualBlock(
                  res,
                  new ceres::ScaledLoss(
                    nullptr, external_camera_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
                  pose_opt_map.at(external_camera_uid).data(),
                  pose_opt_map.at(detection_uid).data());
              }
            } else if (detection_uid.tag_type == TagType::GroundTag && force_shared_ground_plane_) {
              double * external_camera_intrinsics_op =
                share_intrinsics_ ? shared_intrinsics_opt.data()
                                  : intrinsics_opt_map[external_camera_uid].data();
              double * shrd_ground_pose_op = shrd_ground_tag_pose_opt.data();
              double * indep_ground_pose_op =
                indep_ground_tag_pose_opt_map.at(detection_uid).data();

              ceres::CostFunction * res = CameraResidual::createGroundTagResidual(
                external_camera_uid, external_camera_intrinsics_, detection,
                pose_opt_map.at(external_camera_uid), shrd_ground_tag_pose_opt, false,
                force_fixed_ground_plane_, optimize_intrinsics_);

              if (optimize_intrinsics_ && !force_fixed_ground_plane_) {
                problem.AddResidualBlock(
                  res,
                  new ceres::ScaledLoss(
                    nullptr, external_camera_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
                  pose_opt_map.at(external_camera_uid).data(), external_camera_intrinsics_op,
                  shrd_ground_pose_op, indep_ground_pose_op);
              } else if (!optimize_intrinsics_ && !force_fixed_ground_plane_) {
                problem.AddResidualBlock(
                  res,
                  new ceres::ScaledLoss(
                    nullptr, external_camera_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
                  pose_opt_map.at(external_camera_uid).data(), shrd_ground_pose_op,
                  indep_ground_pose_op);
              } else if (optimize_intrinsics_ && force_fixed_ground_plane_) {
                problem.AddResidualBlock(
                  res,
                  new ceres::ScaledLoss(
                    nullptr, external_camera_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
                  pose_opt_map.at(external_camera_uid).data(), external_camera_intrinsics_op,
                  indep_ground_pose_op);
              } else if (!optimize_intrinsics_ && force_fixed_ground_plane_) {
                problem.AddResidualBlock(
                  res,
                  new ceres::ScaledLoss(
                    nullptr, external_camera_residual_weight, ceres::TAKE_OWNERSHIP),  // L2
                  pose_opt_map.at(external_camera_uid).data(), indep_ground_pose_op);
              }

            } else {
              RCLCPP_ERROR(
                rclcpp::get_logger("calibration_problem"), "%s <-> %s error",
                external_camera_uid.toString().c_str(), detection_uid.toString().c_str());
              throw std::domain_error("Invalid residual");
            }
          }
        }
      }
    }
  }

  double initial_cost = 0.0;
  std::vector<double> residuals;
  ceres::Problem::EvaluateOptions eval_opt;
  eval_opt.num_threads = 1;
  problem.GetResidualBlocks(&eval_opt.residual_blocks);
  problem.Evaluate(eval_opt, &initial_cost, &residuals, nullptr, nullptr);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("calibration_problem"), "Initial cost: " << initial_cost);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;  // cSpell:ignore schur
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 500;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("calibration_problem"), "Report: " << summary.FullReport());
}

void CalibrationProblem::writeDebugImage(
  std::size_t scene_index, const UID & sensor_uid, cv::Mat undistorted_img,
  const GroupedApriltagGridDetections & grouped_detections, const std::string & output_image_name)
{
  for (const auto & group_detections : grouped_detections) {
    const TagType tag_type = group_detections.first;

    for (const auto & grid_detection : group_detections.second) {
      const int tag_id = grid_detection.id;
      UID detection_uid = UID::makeTagUID(tag_type, scene_index, tag_id);

      bool sensor_status = pose_opt_map.count(sensor_uid) > 0;
      bool detection_status = pose_opt_map.count(detection_uid) > 0 ||
                              indep_ground_tag_pose_opt_map.count(detection_uid) > 0;
      bool pair_status =
        data_->invalid_pairs_set.count(std::make_pair(sensor_uid, detection_uid)) == 0;

      if (!sensor_status || !detection_status || !pair_status) {
        RCLCPP_ERROR(
          rclcpp::get_logger("calibration_problem"),
          "Skipping %s <-> %s since there are no poses available or was deemed an outlier. "
          "camera_status=%d. detection_status=%d. pair_status=%d",
          sensor_uid.toString().c_str(), detection_uid.toString().c_str(), sensor_status,
          detection_status, pair_status);
        continue;
      }

      for (const auto & detection : grid_detection.sub_detections) {
        cv::Affine3d initial_camera_pose = *data_->initial_sensor_poses_map[sensor_uid];
        cv::Affine3d initial_tag_pose = *data_->initial_tag_poses_map[detection_uid];

        cv::Affine3d optimized_camera_pose = *data_->optimized_sensor_poses_map[sensor_uid];
        cv::Affine3d optimized_tag_pose = *data_->optimized_tag_poses_map[detection_uid];

        ApriltagDetection initial_detection = detection;
        ApriltagDetection optimized_detection = detection;

        auto project_corners = [this, &sensor_uid](
                                 ApriltagDetection & detection, const cv::Affine3d & camera_pose,
                                 const cv::Affine3d & tag_pose, bool use_optimized_intrinsics) {
          // Note: wcs=world coordinate system. ccs=camera coordinate system
          std::vector<cv::Vec3d> corners_wcs{
            tag_pose * detection.template_corners[0], tag_pose * detection.template_corners[1],
            tag_pose * detection.template_corners[2], tag_pose * detection.template_corners[3]};
          std::vector<cv::Vec3d> corners_ccs{
            camera_pose.inv() * corners_wcs[0], camera_pose.inv() * corners_wcs[1],
            camera_pose.inv() * corners_wcs[2], camera_pose.inv() * corners_wcs[3]};

          const auto & intrinsics = use_optimized_intrinsics
                                      ? *data_->optimized_camera_intrinsics_map.at(sensor_uid)
                                      : *data_->initial_camera_intrinsics_map.at(sensor_uid);

          detection.image_corners = std::vector<cv::Point2d>{
            projectPoint(corners_ccs[0], intrinsics), projectPoint(corners_ccs[1], intrinsics),
            projectPoint(corners_ccs[2], intrinsics), projectPoint(corners_ccs[3], intrinsics)};
        };

        project_corners(initial_detection, initial_camera_pose, initial_tag_pose, false);
        project_corners(optimized_detection, optimized_camera_pose, optimized_tag_pose, true);

        const auto & intrinsics = *data_->optimized_camera_intrinsics_map[sensor_uid];
        cv::Affine3d initial_camera_to_tag_pose = initial_camera_pose.inv() * initial_tag_pose;
        cv::Affine3d optimized_camera_to_tag_pose =
          optimized_camera_pose.inv() * optimized_tag_pose;

        drawDetection(undistorted_img, detection, cv::Scalar(255, 0, 255));
        drawDetection(undistorted_img, initial_detection, cv::Scalar(0, 0, 255));
        drawDetection(undistorted_img, optimized_detection, cv::Scalar(0, 255, 0));
        drawAxes(undistorted_img, initial_detection, initial_camera_to_tag_pose, intrinsics);
        drawAxes(
          undistorted_img, optimized_detection, optimized_camera_to_tag_pose, intrinsics, 2.f);
      }
    }
  }

  cv::imwrite(output_image_name, undistorted_img);
}

void CalibrationProblem::writeDebugImages()
{
  RCLCPP_INFO(
    rclcpp::get_logger("calibration_problem"),
    "Drawing the optimized poses on the external camera images");

  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    CalibrationScene & scene = data_->scenes[scene_index];

    for (std::size_t camera_id = 0; camera_id < scene.calibration_cameras_detections.size();
         camera_id++) {
      // Need to make sure all the cameras are in the map
      UID calibration_camera_uid = UID::makeSensorUID(SensorType::CalibrationCamera, camera_id);

      if (!scene.calibration_cameras_detections[camera_id].calibration_image) {
        RCLCPP_ERROR(
          rclcpp::get_logger("calibration_problem"), "scene=%lu camera_id=%lu is invalid",
          scene_index, camera_id);
        continue;
      }

      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(
        *scene.calibration_cameras_detections[camera_id].calibration_image,
        sensor_msgs::image_encodings::BGR8);

      cv::Mat undistorted_img = cv_ptr->image;  // we assume we use the undistorted image

      writeDebugImage(
        scene_index, calibration_camera_uid, undistorted_img,
        scene.calibration_cameras_detections[camera_id].grouped_detections,
        "s" + std::to_string(scene_index) + "_" + calibration_camera_uid.toString() + "_debug.jpg");
    }

    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      // Need to make sure all the cameras are in the map
      UID external_camera_uid =
        UID::makeSensorUID(SensorType::ExternalCamera, scene_index, frame_id);
      std::string file_name = scene.external_camera_frames[frame_id].image_filename;

      cv::Mat distorted_img =
        cv::imread(file_name, cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION);
      cv::Mat undistorted_img;
      cv::undistort(
        distorted_img, undistorted_img, external_camera_intrinsics_.camera_matrix,
        external_camera_intrinsics_.dist_coeffs,
        external_camera_intrinsics_.undistorted_camera_matrix);

      writeDebugImage(
        scene_index, external_camera_uid, undistorted_img,
        scene.external_camera_frames[frame_id].detections,
        external_camera_uid.toString() + "_debug.jpg");
    }
  }
}

void CalibrationProblem::pose3dToPlaceholder(
  cv::Affine3d pose, std::array<double, POSE_OPT_DIM> & placeholder, bool invert)
{
  if (invert) {
    pose = pose.inv();
  }

  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;
  cv::cv2eigen(pose.translation(), translation);
  cv::cv2eigen(pose.rotation(), rotation);
  Eigen::Quaterniond quat(rotation);

  std::fill(placeholder.begin(), placeholder.end(), 0);
  placeholder[ROTATION_W_INDEX] = quat.w();
  placeholder[ROTATION_X_INDEX] = quat.x();
  placeholder[ROTATION_Y_INDEX] = quat.y();
  placeholder[ROTATION_Z_INDEX] = quat.z();
  placeholder[TRANSLATION_X_INDEX] = translation.x();
  placeholder[TRANSLATION_Y_INDEX] = translation.y();
  placeholder[TRANSLATION_Z_INDEX] = translation.z();
}

void CalibrationProblem::placeholderToPose3d(
  const std::array<double, POSE_OPT_DIM> & placeholder, std::shared_ptr<cv::Affine3d> & pose,
  bool invert)
{
  const double scale = 1.0 / std::sqrt(
                               placeholder[0] * placeholder[0] + placeholder[1] * placeholder[1] +
                               placeholder[2] * placeholder[2] + placeholder[3] * placeholder[3]);

  Eigen::Quaterniond quat = Eigen::Quaterniond(
    scale * placeholder[ROTATION_W_INDEX], scale * placeholder[ROTATION_X_INDEX],
    scale * placeholder[ROTATION_Y_INDEX], scale * placeholder[ROTATION_Z_INDEX]);

  Eigen::Vector3d translation = Eigen::Vector3d(
    placeholder[TRANSLATION_X_INDEX], placeholder[TRANSLATION_Y_INDEX],
    placeholder[TRANSLATION_Z_INDEX]);

  Eigen::Matrix3d rotation = quat.toRotationMatrix();

  cv::Matx33d cv_rotation;
  cv::Vec3d cv_translation;
  cv::eigen2cv(translation, cv_translation);
  cv::eigen2cv(rotation, cv_rotation);

  pose = std::make_shared<cv::Affine3d>(cv_rotation, cv_translation);

  if (invert) {
    *pose = pose->inv();
  }
}

void CalibrationProblem::pose3dToGroundTagPlaceholder(
  const UID & uid, cv::Affine3d tag_pose, cv::Affine3d ground_pose,
  std::array<double, SHRD_GROUND_TAG_POSE_DIM> & shrd_placeholder,
  std::array<double, INDEP_GROUND_TAG_POSE_DIM> & indep_placeholder)
{
  // We define a coordinate system in the ground plane where the calibration sensor has only
  // component in the z direction
  cv::Vec3d aux = ground_pose.inv() * cv::Vec3d(0.0, 0.0, 0.0);
  aux(2) = 0.0;
  aux = ground_pose * aux;
  cv::Affine3d projected_ground_pose = cv::Affine3d(ground_pose.rotation(), aux);

  // Pose of the calibration sensor from the projected ground plane
  cv::Affine3d projected_ground_pose_inv = projected_ground_pose.inv();
  double d = projected_ground_pose_inv.translation()(2);

  // Pose of the tag seen from the projected ground
  cv::Affine3d tag_pose_aux = projected_ground_pose_inv * tag_pose;

  Eigen::Vector3d tag_translation;
  Eigen::Matrix3d ground_rotation;
  cv::cv2eigen(tag_pose_aux.translation(), tag_translation);
  cv::cv2eigen(projected_ground_pose_inv.rotation(), ground_rotation);

  Eigen::Quaterniond ground_quat(ground_rotation);

  std::fill(shrd_placeholder.begin(), shrd_placeholder.end(), 0);
  shrd_placeholder[ROTATION_W_INDEX] = ground_quat.w();
  shrd_placeholder[ROTATION_X_INDEX] = ground_quat.x();
  shrd_placeholder[ROTATION_Y_INDEX] = ground_quat.y();
  shrd_placeholder[ROTATION_Z_INDEX] = ground_quat.z();
  shrd_placeholder[GROUND_TAG_D_INDEX] = d;

  if (tag_pose_aux.rotation()(2, 2) < 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("calibration_problem"),
      "Invalid pose rotation (%.2f) for UID=%s. Ignoring", tag_pose_aux.rotation()(2, 2),
      uid.toString().c_str());
    return;
  }

  std::fill(indep_placeholder.begin(), indep_placeholder.end(), 0);
  indep_placeholder[GROUND_TAG_YAW_INDEX] =
    std::atan2(tag_pose_aux.rotation()(1, 0), tag_pose_aux.rotation()(0, 0));
  indep_placeholder[GROUND_TAG_X_INDEX] = tag_translation.x();
  indep_placeholder[GROUND_TAG_Y_INDEX] = tag_translation.y();
}

void CalibrationProblem::groundTagPlaceholderToPose3d(
  const std::array<double, SHRD_GROUND_TAG_POSE_DIM> & shrd_placeholder,
  const std::array<double, INDEP_GROUND_TAG_POSE_DIM> & indep_placeholder,
  std::shared_ptr<cv::Affine3d> & pose)
{
  const double scale =
    1.0 / std::sqrt(
            shrd_placeholder[ROTATION_W_INDEX] * shrd_placeholder[ROTATION_W_INDEX] +
            shrd_placeholder[ROTATION_X_INDEX] * shrd_placeholder[ROTATION_X_INDEX] +
            shrd_placeholder[ROTATION_Y_INDEX] * shrd_placeholder[ROTATION_Y_INDEX] +
            shrd_placeholder[ROTATION_Z_INDEX] * shrd_placeholder[ROTATION_Z_INDEX]);

  // cSpell:ignore WXYZ
  // Eigen's Quaternion constructor is in the WXYZ order but the internal data is in the XYZW format
  Eigen::Quaterniond quat = Eigen::Quaterniond(
    scale * shrd_placeholder[ROTATION_W_INDEX], scale * shrd_placeholder[ROTATION_X_INDEX],
    scale * shrd_placeholder[ROTATION_Y_INDEX], scale * shrd_placeholder[ROTATION_Z_INDEX]);

  double d = shrd_placeholder[GROUND_TAG_D_INDEX];

  double yaw = indep_placeholder[GROUND_TAG_YAW_INDEX];
  double x = indep_placeholder[GROUND_TAG_X_INDEX];
  double y = indep_placeholder[GROUND_TAG_Y_INDEX];

  const double cos = std::cos(yaw);
  const double sin = std::sin(yaw);
  Eigen::Matrix3d rotation2d;
  rotation2d << cos, -sin, 0.0, sin, cos, 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix4d pose2d_matrix;
  pose2d_matrix.setIdentity();
  pose2d_matrix.block<3, 3>(0, 0) = rotation2d;
  pose2d_matrix.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, 0.0);

  Eigen::Matrix4d pose3d_matrix;
  Eigen::Matrix3d rotation3d = quat.toRotationMatrix();
  pose3d_matrix.setIdentity();
  pose3d_matrix.block<3, 3>(0, 0) = rotation3d;
  pose3d_matrix.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, d);  // not z but d

  Eigen::Matrix4d pose_matrix = pose3d_matrix.inverse() * pose2d_matrix;
  Eigen::Matrix3d pose_rotation = pose_matrix.block<3, 3>(0, 0);
  Eigen::Vector3d pose_translation = pose_matrix.block<3, 1>(0, 3);

  cv::Matx33d cv_rotation;
  cv::Vec3d cv_translation;
  cv::eigen2cv(pose_translation, cv_translation);
  cv::eigen2cv(pose_rotation, cv_rotation);

  pose = std::make_shared<cv::Affine3d>(cv_rotation, cv_translation);
}

void CalibrationProblem::printCalibrationResults()
{
  for (auto & [sensor_uid, sensor_pose] : data_->optimized_sensor_poses_map) {
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    cv::cv2eigen(sensor_pose->translation(), translation);
    cv::cv2eigen(sensor_pose->rotation(), rotation);
    Eigen::Quaterniond quat(rotation);

    RCLCPP_INFO(
      rclcpp::get_logger("calibration_problem"),
      "sensor_uid=%s\toptimized pose:\n\ttranslation: [%.5f, %.5f, %.5f]\n\tquaternion: [%.5f, "
      "%.5f, %.5f, %.5f]",
      sensor_uid.toString().c_str(), translation.x(), translation.y(), translation.z(), quat.x(),
      quat.y(), quat.z(), quat.w());
  }
}

}  // namespace tag_based_sfm_calibrator
