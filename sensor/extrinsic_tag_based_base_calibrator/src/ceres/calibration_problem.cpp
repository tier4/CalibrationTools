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
#include <extrinsic_tag_based_base_calibrator/ceres/calibration_problem.hpp>
#include <extrinsic_tag_based_base_calibrator/ceres/camera_residual.hpp>
#include <extrinsic_tag_based_base_calibrator/ceres/lidar_residual.hpp>
#include <extrinsic_tag_based_base_calibrator/math.hpp>
#include <extrinsic_tag_based_base_calibrator/visualization.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ceres/ceres.h>

#include <algorithm>
#include <numeric>

namespace extrinsic_tag_based_base_calibrator
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

void CalibrationProblem::setExternalCameraIntrinsics(IntrinsicParameters & intrinsics)
{
  external_camera_intrinsics_ = intrinsics;
}

void CalibrationProblem::setCalibrationLidarIntrinsics(double calibration_lidar_virtual_f)
{
  calibration_lidar_intrinsics_ = calibration_lidar_virtual_f;
}

void CalibrationProblem::setData(CalibrationData::Ptr & data) { data_ = data; }

void CalibrationProblem::dataToPlaceholders()
{
  // Compute the initial ground plane !
  cv::Affine3d ground_pose;
  computeGroundPlane(data_->initial_ground_tag_poses, 0.0, ground_pose);

  // Prepare the placeholders

  // Calibration camera poses
  for (auto it = data_->initial_calibration_camera_poses.begin();
       it != data_->initial_calibration_camera_poses.end(); it++) {
    const UID & calibration_camera_uid = it->first;
    const auto & pose = it->second;

    pose3dToPlaceholder(*pose, pose_opt_map[calibration_camera_uid], true);
    placeholderToPose3d(
      pose_opt_map[calibration_camera_uid],
      data_->optimized_external_camera_poses[calibration_camera_uid], true);
  }

  // Calibration lidar poses
  for (auto it = data_->initial_calibration_lidar_poses.begin();
       it != data_->initial_calibration_lidar_poses.end(); it++) {
    const UID & calibration_lidar_uid = it->first;
    const auto & pose = it->second;

    pose3dToPlaceholder(*pose, pose_opt_map[calibration_lidar_uid], true);
    placeholderToPose3d(
      pose_opt_map[calibration_lidar_uid],
      data_->optimized_external_camera_poses[calibration_lidar_uid], true);
  }

  // External camera poses
  for (auto it = data_->initial_external_camera_poses.begin();
       it != data_->initial_external_camera_poses.end(); it++) {
    const UID & external_camera_uid = it->first;
    const auto & pose = it->second;

    pose3dToPlaceholder(*pose, pose_opt_map[external_camera_uid], true);

    if (share_intrinsics_) {
      shared_intrinsics_opt = *data_->initial_camera_intrinsics[external_camera_uid];
    } else {
      intrinsics_opt_map[external_camera_uid] =
        *data_->initial_camera_intrinsics[external_camera_uid];
    }

    placeholderToPose3d(
      pose_opt_map[external_camera_uid],
      data_->optimized_external_camera_poses[external_camera_uid], true);

    data_->optimized_camera_intrinsics[external_camera_uid] =
      std::make_shared<std::array<double, INTRINSICS_DIM>>();

    if (share_intrinsics_) {
      *data_->optimized_camera_intrinsics[external_camera_uid] = shared_intrinsics_opt;
    } else {
      *data_->optimized_camera_intrinsics[external_camera_uid] =
        intrinsics_opt_map[external_camera_uid];
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
        *pose, ground_pose, shrd_ground_tag_pose_opt, indep_ground_tag_pose_opt_map[uid]);
      groundTagPlaceholderToPose3d(
        shrd_ground_tag_pose_opt, indep_ground_tag_pose_opt_map[uid],
        data_->optimized_tag_poses_map[uid]);
    }
  }
}

void CalibrationProblem::placeholdersToData()
{
  for (auto it = data_->optimized_external_camera_poses.begin();
       it != data_->optimized_external_camera_poses.end(); it++) {
    const UID & uid = it->first;

    placeholderToPose3d(pose_opt_map[uid], data_->optimized_external_camera_poses[uid], true);

    if (share_intrinsics_) {
      *data_->optimized_camera_intrinsics[uid] = shared_intrinsics_opt;
    } else {
      *data_->optimized_camera_intrinsics[uid] = intrinsics_opt_map[uid];
    }
  }

  for (auto it = data_->optimized_tag_poses_map.begin(); it != data_->optimized_tag_poses_map.end();
       it++) {
    const UID & uid = it->first;
    auto & pose = data_->optimized_tag_poses_map[uid];

    if (uid.tag_type != TagType::GroundTag || !force_shared_ground_plane_) {
      placeholderToPose3d(pose_opt_map[uid], pose, false);
    } else {
      groundTagPlaceholderToPose3d(
        shrd_ground_tag_pose_opt, indep_ground_tag_pose_opt_map[uid], pose);
    }

    if (uid.tag_type == TagType::WaypointTag) {
      data_->optimized_waypoint_tag_poses.push_back(pose);
    } else if (uid.tag_type == TagType::GroundTag) {
      data_->optimized_ground_tag_poses.push_back(pose);
    } else if (uid.tag_type == TagType::WheelTag && uid.tag_id == left_wheel_tag_id_) {
      data_->optimized_left_wheel_tag_pose = pose;
    } else if (uid.tag_type == TagType::WheelTag && uid.tag_id == right_wheel_tag_id_) {
      data_->optimized_right_wheel_tag_pose = pose;
    } else {
      throw std::domain_error("Invalid UID");
    }
  }
}

void CalibrationProblem::evaluate()
{
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

          double sum_res = 0;
          const int res_size = RESIDUAL_DIM * grid_detection.cols * grid_detection.rows;

          for (const auto & detection : grid_detection.sub_detections) {
            std::array<double, RESIDUAL_DIM> residuals;

            auto f = CameraResidual(
              calibration_camera_uid,
              data_->calibration_camera_intrinsics_map_[calibration_camera_uid], detection,
              pose_opt_map[calibration_camera_uid], false, false, false);

            f(pose_opt_map[detection_uid].data(), residuals.data());

            sum_res = std::transform_reduce(
              residuals.begin(), residuals.end(), sum_res, std::plus{},
              [](auto v) { return std::abs(v); });
          }

          RCLCPP_INFO(
            rclcpp::get_logger("calibration_problem"), "%s <-> %s error: %.2f",
            calibration_camera_uid.toString().c_str(), detection_uid.toString().c_str(),
            sum_res / res_size);
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

        auto f = LidarResidual(
          calibration_lidar_uid, calibration_lidar_intrinsics_, detection,
          pose_opt_map[calibration_lidar_uid], false);

        f(pose_opt_map[calibration_lidar_uid].data(), pose_opt_map[detection_uid].data(),
          residuals.data());

        double sum_res = std::transform_reduce(
          residuals.begin(), residuals.end(), 0.0, std::plus{}, [](auto v) { return std::abs(v); });

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

          double sum_res = 0.0;
          const int res_size = RESIDUAL_DIM * grid_detection.cols * grid_detection.rows;

          for (const auto & detection : grid_detection.sub_detections) {
            std::array<double, RESIDUAL_DIM> residuals;

            if (
              detection_uid.tag_type == TagType::WaypointTag ||
              detection_uid.tag_type == TagType::WheelTag ||
              (detection_uid.tag_type == TagType::GroundTag && !force_shared_ground_plane_)) {
              double * external_camera_pose_op = pose_opt_map[external_camera_uid].data();
              double * external_camera_intrinsics_op =
                share_intrinsics_ ? shared_intrinsics_opt.data()
                                  : intrinsics_opt_map[external_camera_uid].data();

              if (optimize_intrinsics_) {
                auto f = CameraResidual(
                  external_camera_uid, external_camera_intrinsics_, detection,
                  pose_opt_map[external_camera_uid], false, true, false);

                f(external_camera_pose_op, external_camera_intrinsics_op,
                  pose_opt_map[detection_uid].data(), residuals.data());

              } else {
                auto f = CameraResidual(
                  external_camera_uid, external_camera_intrinsics_, detection,
                  pose_opt_map[external_camera_uid], false, false, false);

                f(external_camera_pose_op, pose_opt_map[detection_uid].data(), residuals.data());
              }
            } else if (detection_uid.tag_type == TagType::GroundTag && force_shared_ground_plane_) {
              double * external_camera_pose_op = pose_opt_map[external_camera_uid].data();
              double * external_camera_intrinsics_op =
                share_intrinsics_ ? shared_intrinsics_opt.data()
                                  : intrinsics_opt_map[external_camera_uid].data();
              double * shrd_ground_pose_op = shrd_ground_tag_pose_opt.data();
              double * indep_ground_pose_op = indep_ground_tag_pose_opt_map[detection_uid].data();

              if (optimize_intrinsics_) {
                auto f = CameraResidual(
                  external_camera_uid, external_camera_intrinsics_, detection,
                  pose_opt_map[external_camera_uid], false, true, true);

                f(external_camera_pose_op, external_camera_intrinsics_op, shrd_ground_pose_op,
                  indep_ground_pose_op, residuals.data());

              } else {
                auto f = CameraResidual(
                  external_camera_uid, external_camera_intrinsics_, detection,
                  pose_opt_map[external_camera_uid], false, false, true);

                f(external_camera_pose_op, shrd_ground_pose_op, indep_ground_pose_op,
                  residuals.data());
              }
            } else {
              throw std::domain_error("Invalid residual");
            }

            sum_res = std::transform_reduce(
              residuals.begin(), residuals.end(), sum_res, std::plus{},
              [](auto v) { return std::abs(v); });
          }

          RCLCPP_INFO(
            rclcpp::get_logger("calibration_problem"), "%s <-> %s error: %.2f",
            external_camera_uid.toString().c_str(), detection_uid.toString().c_str(),
            sum_res / res_size);
        }
      }
    }
  }
}

void CalibrationProblem::solve()
{
  ceres::Problem problem;

  // auto identity = std::make_shared<std::array<double, POSE_OPT_DIM>>(
  //   std::array<double, POSE_OPT_DIM>{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

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

          for (const auto & detection : grid_detection.sub_detections) {
            if (
              detection_uid.tag_type == TagType::WaypointTag ||
              (detection_uid.tag_type == TagType::GroundTag && !force_shared_ground_plane_)) {
              ceres::CostFunction * res = CameraResidual::createTagResidual(
                calibration_camera_uid,
                data_->calibration_camera_intrinsics_map_[calibration_camera_uid], detection,
                pose_opt_map[calibration_camera_uid], fix_sensor_pose, false);

              if (fix_sensor_pose) {
                problem.AddResidualBlock(
                  res,
                  nullptr,  // L2
                  pose_opt_map[detection_uid].data());
              } else {
                problem.AddResidualBlock(
                  res,
                  nullptr,  // L2
                  pose_opt_map[calibration_camera_uid].data(), pose_opt_map[detection_uid].data());
              }
            } else if (detection_uid.tag_type == TagType::GroundTag && force_shared_ground_plane_) {
              double * shrd_ground_pose_op = shrd_ground_tag_pose_opt.data();
              double * indep_ground_pose_op = indep_ground_tag_pose_opt_map[detection_uid].data();

              ceres::CostFunction * res = CameraResidual::createGroundTagResidual(
                calibration_camera_uid,
                data_->calibration_camera_intrinsics_map_[calibration_camera_uid], detection,
                pose_opt_map[calibration_camera_uid], fix_sensor_pose, false);

              if (fix_sensor_pose) {
                problem.AddResidualBlock(
                  res,
                  nullptr,  // L2
                  shrd_ground_pose_op, indep_ground_pose_op);
              } else {
                problem.AddResidualBlock(
                  res,
                  nullptr,  // L2
                  pose_opt_map[calibration_camera_uid].data(), shrd_ground_pose_op,
                  indep_ground_pose_op);
              }
            } else {
              throw std::domain_error("Invalid residual");
            }
          }
        }
      }
    }

    // Calibration lidar residuals
    for (const auto & single_lidar_detections : scene.calibration_lidars_detections) {
      const int & lidar_id = single_lidar_detections.calibration_lidar_id;
      UID lidar_uid = UID::makeSensorUID(SensorType::CalibrationLidar, lidar_id);
      bool fix_sensor_pose = data_->main_calibration_sensor_uid == lidar_uid;

      for (const auto & detection : single_lidar_detections.detections) {
        const int tag_id = detection.id;
        UID detection_uid = UID::makeTagUID(TagType::WaypointTag, scene_index, tag_id);

        ceres::CostFunction * res = LidarResidual::createTagResidual(
          lidar_uid, calibration_lidar_intrinsics_, detection, pose_opt_map[lidar_uid],
          fix_sensor_pose);

        problem.AddResidualBlock(
          res,
          nullptr,  // L2
          pose_opt_map[lidar_uid].data(), pose_opt_map[detection_uid].data());
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

          for (const auto & detection : grid_detection.sub_detections) {
            if (
              detection_uid.tag_type == TagType::WaypointTag ||
              detection_uid.tag_type == TagType::WheelTag ||
              (detection_uid.tag_type == TagType::GroundTag && !force_shared_ground_plane_)) {
              double * external_camera_intrinsics_op =
                share_intrinsics_ ? shared_intrinsics_opt.data()
                                  : intrinsics_opt_map[external_camera_uid].data();

              if (optimize_intrinsics_) {
                ceres::CostFunction * res = CameraResidual::createTagResidual(
                  external_camera_uid, external_camera_intrinsics_, detection,
                  pose_opt_map[external_camera_uid], false, true);

                problem.AddResidualBlock(
                  res,
                  nullptr,  // L2
                  pose_opt_map[external_camera_uid].data(), external_camera_intrinsics_op,
                  pose_opt_map[detection_uid].data());
              } else {
                ceres::CostFunction * res = CameraResidual::createTagResidual(
                  external_camera_uid, external_camera_intrinsics_, detection,
                  pose_opt_map[external_camera_uid], false, false);

                problem.AddResidualBlock(
                  res,
                  nullptr,  // L2
                  pose_opt_map[external_camera_uid].data(), pose_opt_map[detection_uid].data());
              }
            } else if (detection_uid.tag_type == TagType::GroundTag && force_shared_ground_plane_) {
              double * external_camera_intrinsics_op =
                share_intrinsics_ ? shared_intrinsics_opt.data()
                                  : intrinsics_opt_map[external_camera_uid].data();
              double * shrd_ground_pose_op = shrd_ground_tag_pose_opt.data();
              double * indep_ground_pose_op = indep_ground_tag_pose_opt_map[detection_uid].data();

              ceres::CostFunction * res = CameraResidual::createGroundTagResidual(
                external_camera_uid, external_camera_intrinsics_, detection,
                pose_opt_map[external_camera_uid], false, optimize_intrinsics_);

              if (optimize_intrinsics_) {
                problem.AddResidualBlock(
                  res,
                  nullptr,  // L2
                  pose_opt_map[external_camera_uid].data(), external_camera_intrinsics_op,
                  shrd_ground_pose_op, indep_ground_pose_op);
              } else {
                problem.AddResidualBlock(
                  res,
                  nullptr,  // L2
                  pose_opt_map[external_camera_uid].data(), shrd_ground_pose_op,
                  indep_ground_pose_op);
              }
            } else {
              throw std::domain_error("Invalid residual");
            }
          }
        }
      }
    }
  }

  std::vector<double> residuals;
  ceres::Problem::EvaluateOptions eval_opt;
  eval_opt.num_threads = 1;
  problem.GetResidualBlocks(&eval_opt.residual_blocks);
  problem.Evaluate(eval_opt, nullptr, &residuals, nullptr, nullptr);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 500;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("calibration_problem"), "Report: " << summary.FullReport());
}

void CalibrationProblem::writeDebugImages()
{
  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    CalibrationScene & scene = data_->scenes[scene_index];

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

      for (const auto & group_detections : scene.external_camera_frames[frame_id].detections) {
        const TagType tag_type = group_detections.first;

        for (const auto & grid_detection : group_detections.second) {
          for (const auto & detection : grid_detection.sub_detections) {
            const int tag_id = grid_detection.id + detection.id;
            UID detection_uid = UID::makeTagUID(tag_type, scene_index, tag_id);

            cv::Affine3d initial_camera_pose =
              *data_->initial_external_camera_poses[external_camera_uid];
            cv::Affine3d initial_tag_pose = *data_->initial_tag_poses_map[detection_uid];

            cv::Affine3d optimized_camera_pose =
              *data_->optimized_external_camera_poses[external_camera_uid];
            cv::Affine3d optimized_tag_pose = *data_->optimized_tag_poses_map[detection_uid];

            ApriltagDetection initial_detection = detection;
            ApriltagDetection optimized_detection = detection;

            auto project_corners = [this, &external_camera_uid](
                                     ApriltagDetection & detection,
                                     const cv::Affine3d & camera_pose,
                                     const cv::Affine3d & tag_pose, bool use_optimized_intrinsics) {
              cv::Vec3d template_corners[4] = {
                {-1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, -1.0, 0.0}, {-1.0, -1.0, 0.0}};

              for (int j = 0; j < 4; ++j) {
                template_corners[j] *= 0.5 * detection.size;
              }

              std::vector<cv::Vec3d> corners_wcs{
                tag_pose * template_corners[0], tag_pose * template_corners[1],
                tag_pose * template_corners[2], tag_pose * template_corners[3]};
              std::vector<cv::Vec3d> corners_ccs{
                camera_pose.inv() * corners_wcs[0], camera_pose.inv() * corners_wcs[1],
                camera_pose.inv() * corners_wcs[2], camera_pose.inv() * corners_wcs[3]};

              const auto & intrinsics = use_optimized_intrinsics
                                          ? *data_->optimized_camera_intrinsics[external_camera_uid]
                                          : *data_->initial_camera_intrinsics[external_camera_uid];
              double cx = intrinsics[INTRINSICS_CX_INDEX];
              double cy = intrinsics[INTRINSICS_CY_INDEX];
              double fx = intrinsics[INTRINSICS_FX_INDEX];
              double fy = intrinsics[INTRINSICS_FY_INDEX];
              double k1 = intrinsics[INTRINSICS_K1_INDEX];
              double k2 = intrinsics[INTRINSICS_K2_INDEX];

              detection.image_corners = std::vector<cv::Point2d>{
                projectPoint(corners_ccs[0], fx, fy, cx, cy, k1, k2),
                projectPoint(corners_ccs[1], fx, fy, cx, cy, k1, k2),
                projectPoint(corners_ccs[2], fx, fy, cx, cy, k1, k2),
                projectPoint(corners_ccs[3], fx, fy, cx, cy, k1, k2)};
            };

            project_corners(initial_detection, initial_camera_pose, initial_tag_pose, false);
            project_corners(optimized_detection, optimized_camera_pose, optimized_tag_pose, true);

            const auto & intrinsics = *data_->optimized_camera_intrinsics[external_camera_uid];
            double cx = intrinsics[INTRINSICS_CX_INDEX];
            double cy = intrinsics[INTRINSICS_CY_INDEX];
            double fx = intrinsics[INTRINSICS_FX_INDEX];
            double fy = intrinsics[INTRINSICS_FY_INDEX];
            double k1 = intrinsics[INTRINSICS_K1_INDEX];
            double k2 = intrinsics[INTRINSICS_K2_INDEX];

            cv::Affine3d camera_to_tag_pose = optimized_camera_pose.inv() * optimized_tag_pose;

            cv::Vec3d px3d =
              camera_to_tag_pose * cv::Vec3d(0.5 * optimized_detection.size, 0.0, 0.0);
            cv::Vec3d py3d =
              camera_to_tag_pose * cv::Vec3d(0.0, 0.5 * optimized_detection.size, 0.0);
            cv::Vec3d pz3d =
              camera_to_tag_pose * cv::Vec3d(0.0, 0.0, 0.5 * optimized_detection.size);
            cv::Vec3d center3d = camera_to_tag_pose.translation();
            cv::Point2d px2d = projectPoint(px3d, fx, fy, cx, cy, k1, k2);
            cv::Point2d py2d = projectPoint(py3d, fx, fy, cx, cy, k1, k2);
            cv::Point2d pz2d = projectPoint(pz3d, fx, fy, cx, cy, k1, k2);
            cv::Point2d center2d = projectPoint(center3d, fx, fy, cx, cy, k1, k2);

            drawDetection(undistorted_img, detection, cv::Scalar(255, 0, 255));
            drawDetection(undistorted_img, initial_detection, cv::Scalar(0, 0, 255));
            drawDetection(undistorted_img, optimized_detection, cv::Scalar(0, 255, 0));
            drawAxes(undistorted_img, optimized_detection, center2d, px2d, py2d, pz2d);
          }
        }
      }

      std::string output_name = external_camera_uid.toString() + "_debug.jpg";
      cv::imwrite(output_name, undistorted_img);
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

  cv::Matx33d cv_rot;
  cv::Vec3d cv_transl;
  cv::eigen2cv(translation, cv_transl);
  cv::eigen2cv(rotation, cv_rot);

  pose = std::make_shared<cv::Affine3d>(cv_rot, cv_transl);

  if (invert) {
    *pose = pose->inv();
  }
}

void CalibrationProblem::pose3dToGroundTagPlaceholder(
  cv::Affine3d tag_pose, cv::Affine3d ground_pose,
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

  assert(tag_pose_aux.rotation()(2, 2) > 0);

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

  cv::Matx33d cv_rot;
  cv::Vec3d cv_transl;
  cv::eigen2cv(pose_translation, cv_transl);
  cv::eigen2cv(pose_rotation, cv_rot);

  pose = std::make_shared<cv::Affine3d>(cv_rot, cv_transl);
}

}  // namespace extrinsic_tag_based_base_calibrator
