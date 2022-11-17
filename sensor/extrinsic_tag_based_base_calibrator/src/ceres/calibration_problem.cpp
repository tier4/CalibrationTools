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

#include <Eigen/Core>
#include <extrinsic_tag_based_base_calibrator/ceres/calibration_problem.hpp>
#include <extrinsic_tag_based_base_calibrator/ceres/tag_reprojection_error.hpp>
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

namespace extrinsic_tag_based_base_calibrator
{

void CalibrationProblem::setTagIds(
  std::vector<int> & waypoint_tag_ids, std::vector<int> & ground_tag_ids, int left_wheel_tag_id,
  int right_wheel_tag_id)
{
  for (auto id : waypoint_tag_ids) {
    waypoint_tag_ids_set_.insert(id);
  }

  for (auto id : ground_tag_ids) {
    ground_tag_ids_set_.insert(id);
  }

  wheel_tag_ids_set_.insert(left_wheel_tag_id);
  wheel_tag_ids_set_.insert(right_wheel_tag_id);

  left_wheel_tag_id_ = left_wheel_tag_id;
  right_wheel_tag_id_ = right_wheel_tag_id;
}

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

void CalibrationProblem::setCalibrationSensorIntrinsics(IntrinsicParameters & intrinsics)
{
  calibration_sensor_intrinsics_ = intrinsics;
}

void CalibrationProblem::setData(CalibrationData::Ptr & data) { data_ = data; }

void CalibrationProblem::dataToPlaceholders()
{
  // Prepare the placeholders
  for (auto it = data_->initial_external_camera_poses.begin();
       it != data_->initial_external_camera_poses.end(); it++) {
    const UID & uid = it->first;
    const auto & pose = it->second;

    pose3dToPlaceholder(*pose, pose_opt_map[uid], true);

    if (share_intrinsics_) {
      shared_intrinsics_opt = *data_->initial_external_camera_intrinsics[uid];
    } else {
      intrinsics_opt_map[uid] = *data_->initial_external_camera_intrinsics[uid];
    }

    placeholderToPose3d(pose_opt_map[uid], data_->optimized_external_camera_poses[uid], true);

    data_->optimized_external_camera_intrinsics[uid] =
      std::make_shared<std::array<double, INTRINSICS_DIM>>();

    if (share_intrinsics_) {
      *data_->optimized_external_camera_intrinsics[uid] = shared_intrinsics_opt;
    } else {
      *data_->optimized_external_camera_intrinsics[uid] = intrinsics_opt_map[uid];
    }
  }

  for (auto it = data_->initial_tag_poses_map.begin(); it != data_->initial_tag_poses_map.end();
       it++) {
    const UID & uid = it->first;
    const auto & pose = it->second;

    if (!uid.is_ground_tag || !force_shared_ground_plane_) {
      pose3dToPlaceholder(*pose, pose_opt_map[uid], false);
      placeholderToPose3d(pose_opt_map[uid], data_->optimized_tag_poses_map[uid], false);
    } else {
      pose3dToGroundTagPlaceholder(
        *pose, shrd_ground_tag_pose_opt, indep_ground_tag_pose_opt_map[uid], false);
      groundTagPlaceholderToPose3d(
        shrd_ground_tag_pose_opt, indep_ground_tag_pose_opt_map[uid],
        data_->optimized_tag_poses_map[uid], false);
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
      *data_->optimized_external_camera_intrinsics[uid] = shared_intrinsics_opt;
    } else {
      *data_->optimized_external_camera_intrinsics[uid] = intrinsics_opt_map[uid];
    }
  }

  for (auto it = data_->optimized_tag_poses_map.begin(); it != data_->optimized_tag_poses_map.end();
       it++) {
    const UID & uid = it->first;
    auto & pose = data_->optimized_tag_poses_map[uid];

    if (!uid.is_ground_tag || !force_shared_ground_plane_) {
      placeholderToPose3d(pose_opt_map[uid], pose, false);
    } else {
      groundTagPlaceholderToPose3d(
        shrd_ground_tag_pose_opt, indep_ground_tag_pose_opt_map[uid], pose, false);
    }

    if (uid.is_waypoint_tag) {
      data_->optimized_waypoint_tag_poses.push_back(pose);
    } else if (uid.is_ground_tag) {
      data_->optimized_ground_tag_poses.push_back(pose);
    } else if (uid.is_wheel_tag && uid.tag_id == left_wheel_tag_id_) {
      data_->optimized_left_wheel_tag_pose = pose;
    } else if (uid.is_wheel_tag && uid.tag_id == right_wheel_tag_id_) {
      data_->optimized_right_wheel_tag_pose = pose;
    }
  }
}

void CalibrationProblem::evaluate()
{
  auto identity = std::make_shared<std::array<double, POSE_OPT_DIM>>(
    std::array<double, POSE_OPT_DIM>{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    CalibrationScene & scene = data_->scenes[scene_index];

    for (auto detection : scene.calibration_sensor_detections) {
      UID sensor_uid = UID::makeCameraUID(scene_index, -1);
      UID detection_uid = UID::makeWaypointUID(scene_index, detection.id);

      std::array<double, RESIDUAL_DIM> residuals;

      auto f = TagReprojectionError(
        sensor_uid, calibration_sensor_intrinsics_, detection, identity, nullptr, true, false,
        false, false);

      f(pose_opt_map[detection_uid].data(), residuals.data());

      double sum_res = std::transform_reduce(
        residuals.begin(), residuals.end(), 0.0, std::plus{}, [](auto v) { return std::abs(v); });

      std::cout << sensor_uid.to_string() << " <-> " << detection_uid.to_string()
                << " error: " << sum_res << std::endl;
    }

    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      UID external_camera_uid = UID::makeCameraUID(scene_index, frame_id);

      auto & frame = scene.external_camera_frames[frame_id];

      for (const auto & detection : frame.detections) {
        UID detection_uid = waypoint_tag_ids_set_.count(detection.id) > 0
                              ? UID::makeWaypointUID(scene_index, detection.id)
                            : wheel_tag_ids_set_.count(detection.id)
                              ? UID::makeWheelTagUID(detection.id)
                              : UID::makeGroundTagUID(detection.id);

        std::array<double, RESIDUAL_DIM> residuals;

        if (!detection_uid.is_ground_tag || !force_shared_ground_plane_) {
          double * external_camera_pose_op = pose_opt_map[external_camera_uid].data();
          double * external_camera_intrinsics_op =
            share_intrinsics_ ? shared_intrinsics_opt.data()
                              : intrinsics_opt_map[external_camera_uid].data();
          double * detection_pose_op = pose_opt_map[detection_uid].data();

          if (optimize_intrinsics_) {
            auto f = TagReprojectionError(
              external_camera_uid, external_camera_intrinsics_, detection, nullptr, nullptr, false,
              true, false, false);

            f(external_camera_pose_op, external_camera_intrinsics_op, detection_pose_op,
              residuals.data());

          } else {
            auto f = TagReprojectionError(
              external_camera_uid, external_camera_intrinsics_, detection, nullptr, nullptr, false,
              false, false, false);

            f(external_camera_pose_op, pose_opt_map[detection_uid].data(), residuals.data());
          }
        } else {
          double * external_camera_pose_op = pose_opt_map[external_camera_uid].data();
          double * external_camera_intrinsics_op =
            share_intrinsics_ ? shared_intrinsics_opt.data()
                              : intrinsics_opt_map[external_camera_uid].data();
          double * shrd_ground_pose_op = shrd_ground_tag_pose_opt.data();
          double * indep_ground_pose_op = indep_ground_tag_pose_opt_map[detection_uid].data();

          if (optimize_intrinsics_) {
            auto f = TagReprojectionError(
              external_camera_uid, external_camera_intrinsics_, detection, nullptr, nullptr, false,
              true, false, true);

            f(external_camera_pose_op, external_camera_intrinsics_op, shrd_ground_pose_op,
              indep_ground_pose_op, residuals.data());

          } else {
            auto f = TagReprojectionError(
              external_camera_uid, external_camera_intrinsics_, detection, nullptr, nullptr, false,
              false, false, true);

            f(external_camera_pose_op, shrd_ground_pose_op, indep_ground_pose_op, residuals.data());
          }
        }

        double sum_res = std::transform_reduce(
          residuals.begin(), residuals.end(), 0.0, std::plus{}, [](auto v) { return std::abs(v); });

        std::cout << external_camera_uid.to_string() << " <-> " << detection_uid.to_string()
                  << " error: " << sum_res / residuals.size() << std::endl;
      }
    }
  }
}

void CalibrationProblem::solve()
{
  ceres::Problem problem;

  auto identity = std::make_shared<std::array<double, POSE_OPT_DIM>>(
    std::array<double, POSE_OPT_DIM>{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Build the optimization problem
  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    CalibrationScene & scene = data_->scenes[scene_index];

    // Calibration sensor-related residuals
    for (auto detection : scene.calibration_sensor_detections) {
      UID sensor_uid = UID::makeCameraUID(scene_index, -1);
      UID detection_uid = UID::makeWaypointUID(scene_index, detection.id);

      ceres::CostFunction * res = TagReprojectionError::createTagResidual(
        sensor_uid, calibration_sensor_intrinsics_, detection, identity, identity, true, false,
        false);

      problem.AddResidualBlock(
        res,
        nullptr,  // L2
        pose_opt_map[detection_uid].data());
    }

    // External camera-related residuals
    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      UID external_camera_uid = UID::makeCameraUID(scene_index, frame_id);

      auto & frame = scene.external_camera_frames[frame_id];

      for (const auto & detection : frame.detections) {
        UID detection_uid = waypoint_tag_ids_set_.count(detection.id) > 0
                              ? UID::makeWaypointUID(scene_index, detection.id)
                            : wheel_tag_ids_set_.count(detection.id)
                              ? UID::makeWheelTagUID(detection.id)
                              : UID::makeGroundTagUID(detection.id);

        if (!detection_uid.is_ground_tag || !force_shared_ground_plane_) {
          double * external_camera_pose_op = pose_opt_map[external_camera_uid].data();
          double * external_camera_intrinsics_op =
            share_intrinsics_ ? shared_intrinsics_opt.data()
                              : intrinsics_opt_map[external_camera_uid].data();
          double * detection_pose_op = pose_opt_map[detection_uid].data();

          if (optimize_intrinsics_) {
            ceres::CostFunction * res = TagReprojectionError::createTagResidual(
              external_camera_uid, external_camera_intrinsics_, detection, identity, identity,
              false, true, false);

            problem.AddResidualBlock(
              res,
              nullptr,  // L2
              external_camera_pose_op, external_camera_intrinsics_op, detection_pose_op);
          } else {
            ceres::CostFunction * res = TagReprojectionError::createTagResidual(
              external_camera_uid, external_camera_intrinsics_, detection, identity, identity,
              false, false, false);

            problem.AddResidualBlock(
              res,
              nullptr,  // L2
              external_camera_pose_op, detection_pose_op);
          }
        } else {
          double * external_camera_pose_op = pose_opt_map[external_camera_uid].data();
          double * external_camera_intrinsics_op =
            share_intrinsics_ ? shared_intrinsics_opt.data()
                              : intrinsics_opt_map[external_camera_uid].data();
          double * shrd_ground_pose_op = shrd_ground_tag_pose_opt.data();
          double * indep_ground_pose_op = indep_ground_tag_pose_opt_map[detection_uid].data();

          if (optimize_intrinsics_) {
            ceres::CostFunction * res = TagReprojectionError::createGroundTagResidual(
              external_camera_uid, external_camera_intrinsics_, detection, true);

            problem.AddResidualBlock(
              res,
              nullptr,  // L2
              external_camera_pose_op, external_camera_intrinsics_op, shrd_ground_pose_op,
              indep_ground_pose_op);
          } else {
            ceres::CostFunction * res = TagReprojectionError::createGroundTagResidual(
              external_camera_uid, external_camera_intrinsics_, detection, false);

            problem.AddResidualBlock(
              res,
              nullptr,  // L2
              external_camera_pose_op, shrd_ground_pose_op, indep_ground_pose_op);
          }
        }
      }
    }
  }

  int numresidualsblocks = problem.NumResidualBlocks();
  int numresiduals = problem.NumResiduals();

  (void)numresidualsblocks;
  (void)numresiduals;

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
  std::cout << summary.FullReport() << "\n" << std::flush;
}

void CalibrationProblem::writeDebugImages()
{
  for (std::size_t scene_index = 0; scene_index < data_->scenes.size(); scene_index++) {
    CalibrationScene & scene = data_->scenes[scene_index];

    for (std::size_t frame_id = 0; frame_id < scene.external_camera_frames.size(); frame_id++) {
      // Need to make sure all the cameras are in the map
      UID external_camera_uid = UID::makeCameraUID(scene_index, frame_id);
      std::string file_name = scene.external_camera_frames[frame_id].image_filename;

      cv::Mat distorted_img = cv::imread(file_name, cv::IMREAD_COLOR);
      cv::Mat undistorted_img;
      cv::undistort(
        distorted_img, undistorted_img, external_camera_intrinsics_.camera_matrix,
        external_camera_intrinsics_.dist_coeffs,
        external_camera_intrinsics_.undistorted_camera_matrix);

      for (auto & detection : scene.external_camera_frames[frame_id].detections) {
        UID detection_uid = waypoint_tag_ids_set_.count(detection.id) > 0
                              ? UID::makeWaypointUID(scene_index, detection.id)
                            : wheel_tag_ids_set_.count(detection.id)
                              ? UID::makeWheelTagUID(detection.id)
                              : UID::makeGroundTagUID(detection.id);

        cv::Affine3f initial_camera_pose =
          *data_->initial_external_camera_poses[external_camera_uid];
        cv::Affine3f initial_tag_pose = *data_->initial_tag_poses_map[detection_uid];

        cv::Affine3f optimized_camera_pose =
          *data_->optimized_external_camera_poses[external_camera_uid];
        cv::Affine3f optimized_tag_pose = *data_->optimized_tag_poses_map[detection_uid];

        ApriltagDetection initial_detection = detection;
        ApriltagDetection optimized_detection = detection;

        auto project_corners = [this, &external_camera_uid](
                                 ApriltagDetection & detection, const cv::Affine3f & camera_pose,
                                 const cv::Affine3f & tag_pose, bool use_optimized_intrinsics) {
          cv::Vec3f template_corners[4] = {
            {-1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, -1.0, 0.0}, {-1.0, -1.0, 0.0}};

          for (int j = 0; j < 4; ++j) {
            template_corners[j] *= 0.5 * detection.size;
          }

          std::vector<cv::Vec3f> corners_wcs{
            tag_pose * template_corners[0], tag_pose * template_corners[1],
            tag_pose * template_corners[2], tag_pose * template_corners[3]};
          std::vector<cv::Vec3f> corners_ccs{
            camera_pose.inv() * corners_wcs[0], camera_pose.inv() * corners_wcs[1],
            camera_pose.inv() * corners_wcs[2], camera_pose.inv() * corners_wcs[3]};

          const auto & intrinsics =
            use_optimized_intrinsics
              ? *data_->optimized_external_camera_intrinsics[external_camera_uid]
              : *data_->initial_external_camera_intrinsics[external_camera_uid];
          float cx = intrinsics[INTRINSICS_CX_INDEX];
          float cy = intrinsics[INTRINSICS_CY_INDEX];
          float fx = intrinsics[INTRINSICS_FX_INDEX];
          float fy = intrinsics[INTRINSICS_FY_INDEX];
          float k1 = intrinsics[INTRINSICS_K1_INDEX];
          float k2 = intrinsics[INTRINSICS_K2_INDEX];

          auto camera_ccs_to_image = [&fx, &fy, &cx, &cy, &k1, &k2](cv::Vec3f & p) {
            const float xp = p(0) / p(2);
            const float yp = p(1) / p(2);
            const float r2 = xp * xp + yp * yp;
            const float d = 1.0 + r2 * (k1 + k2 * r2);
            return cv::Point2f(cx + fx * d * xp, cy + fy * d * yp);
          };

          detection.corners = std::vector<cv::Point2f>{
            camera_ccs_to_image(corners_ccs[0]), camera_ccs_to_image(corners_ccs[1]),
            camera_ccs_to_image(corners_ccs[2]), camera_ccs_to_image(corners_ccs[3])};
        };

        project_corners(initial_detection, initial_camera_pose, initial_tag_pose, false);
        project_corners(optimized_detection, optimized_camera_pose, optimized_tag_pose, true);

        drawDetection(undistorted_img, detection, cv::Scalar(255, 0, 255));
        drawDetection(undistorted_img, initial_detection, cv::Scalar(0, 0, 255));
        drawDetection(undistorted_img, optimized_detection, cv::Scalar(0, 255, 0));
      }

      std::string output_name = external_camera_uid.to_string() + "_debug.jpg";
      cv::imwrite(output_name, undistorted_img);
    }
  }
}

void CalibrationProblem::getMarkers() {}

void CalibrationProblem::pose3dToPlaceholder(
  cv::Affine3f pose, std::array<double, POSE_OPT_DIM> & placeholder, bool invert)
{
  if (invert) {
    pose = pose.inv();
  }

  Eigen::Vector3f translation;
  Eigen::Matrix3f rotation;
  cv::cv2eigen(pose.translation(), translation);
  cv::cv2eigen(pose.rotation(), rotation);
  Eigen::Quaternionf quat(rotation);

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
  const std::array<double, POSE_OPT_DIM> & placeholder, std::shared_ptr<cv::Affine3f> & pose,
  bool invert)
{
  const float scale = 1.f / std::sqrt(
                              placeholder[0] * placeholder[0] + placeholder[1] * placeholder[1] +
                              placeholder[2] * placeholder[2] + placeholder[3] * placeholder[3]);

  Eigen::Quaternionf quat =
    Eigen::Quaterniond(
      scale * placeholder[ROTATION_W_INDEX], scale * placeholder[ROTATION_X_INDEX],
      scale * placeholder[ROTATION_Y_INDEX], scale * placeholder[ROTATION_Z_INDEX])
      .cast<float>();

  Eigen::Vector3f translation =
    Eigen::Vector3d(
      placeholder[TRANSLATION_X_INDEX], placeholder[TRANSLATION_Y_INDEX],
      placeholder[TRANSLATION_Z_INDEX])
      .cast<float>();

  Eigen::Matrix3f rotation = quat.toRotationMatrix();

  cv::Matx33f cv_rot;
  cv::Vec3f cv_transl;
  cv::eigen2cv(translation, cv_transl);
  cv::eigen2cv(rotation, cv_rot);

  pose = std::make_shared<cv::Affine3f>(cv_rot, cv_transl);

  if (invert) {
    *pose = pose->inv();
  }
}

void CalibrationProblem::pose3dToGroundTagPlaceholder(
  cv::Affine3f pose, std::array<double, SHRD_GROUND_TAG_POSE_DIM> & shrd_placeholder,
  std::array<double, INDEP_GROUND_TAG_POSE_DIM> & indep_placeholder, bool invert)
{
  if (invert) {
    pose = pose.inv();
  }

  Eigen::Vector3f translation;
  Eigen::Matrix3f rotation;
  cv::cv2eigen(pose.translation(), translation);
  cv::cv2eigen(pose.rotation(), rotation);
  Eigen::Quaternionf quat(rotation);

  std::fill(shrd_placeholder.begin(), shrd_placeholder.end(), 0);
  shrd_placeholder[ROTATION_W_INDEX] = quat.w();
  shrd_placeholder[ROTATION_X_INDEX] = quat.x();
  shrd_placeholder[ROTATION_Y_INDEX] = quat.y();
  shrd_placeholder[ROTATION_Z_INDEX] = quat.z();
  shrd_placeholder[GROUND_TAG_Z_INDEX] = translation.z();

  std::fill(indep_placeholder.begin(), indep_placeholder.end(), 0);
  indep_placeholder[GROUND_TAG_X_INDEX] = translation.x();
  indep_placeholder[GROUND_TAG_Y_INDEX] = translation.y();
}

void CalibrationProblem::groundTagPlaceholderToPose3d(
  const std::array<double, SHRD_GROUND_TAG_POSE_DIM> & shrd_placeholder,
  const std::array<double, INDEP_GROUND_TAG_POSE_DIM> & indep_placeholder,
  std::shared_ptr<cv::Affine3f> & pose, bool invert)
{
  const float scale =
    1.f / std::sqrt(
            shrd_placeholder[ROTATION_W_INDEX] * shrd_placeholder[ROTATION_W_INDEX] +
            shrd_placeholder[ROTATION_X_INDEX] * shrd_placeholder[ROTATION_X_INDEX] +
            shrd_placeholder[ROTATION_Y_INDEX] * shrd_placeholder[ROTATION_Y_INDEX] +
            shrd_placeholder[ROTATION_Z_INDEX] * shrd_placeholder[ROTATION_Z_INDEX]);

  Eigen::Quaternionf quat =
    Eigen::Quaterniond(
      scale * shrd_placeholder[ROTATION_W_INDEX], scale * shrd_placeholder[ROTATION_X_INDEX],
      scale * shrd_placeholder[ROTATION_Y_INDEX], scale * shrd_placeholder[ROTATION_Z_INDEX])
      .cast<float>();

  float z = shrd_placeholder[GROUND_TAG_Z_INDEX];

  float yaw = indep_placeholder[GROUND_TAG_YAW_INDEX];
  float x = indep_placeholder[GROUND_TAG_X_INDEX];
  float y = indep_placeholder[GROUND_TAG_Y_INDEX];

  const float cos = std::cos(yaw);
  const float sin = std::sin(yaw);
  Eigen::Matrix3f rotation2d;
  rotation2d << cos, -sin, 0.0, sin, cos, 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix4f pose2d_matrix;
  pose2d_matrix.setIdentity();
  pose2d_matrix.block<3, 3>(0, 0) = rotation2d;
  pose2d_matrix.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, 0.f);

  Eigen::Matrix4f pose3d_matrix;
  Eigen::Matrix3f rotation3d = quat.toRotationMatrix();
  pose3d_matrix.setIdentity();
  pose3d_matrix.block<3, 3>(0, 0) = rotation3d;
  pose3d_matrix.block<3, 1>(0, 3) = Eigen::Vector3f(0.0, 0.0, z);

  Eigen::Matrix4f pose_matrix = pose3d_matrix * pose2d_matrix;
  Eigen::Matrix3f pose_rotation = pose_matrix.block<3, 3>(0, 0);
  Eigen::Vector3f pose_translation = pose_matrix.block<3, 1>(0, 3);

  cv::Matx33f cv_rot;
  cv::Vec3f cv_transl;
  cv::eigen2cv(pose_translation, cv_transl);
  cv::eigen2cv(pose_rotation, cv_rot);

  pose = std::make_shared<cv::Affine3f>(cv_rot, cv_transl);

  if (invert) {
    *pose = pose->inv();
  }
}

}  // namespace extrinsic_tag_based_base_calibrator
