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

#ifndef TAG_BASED_SFM_CALIBRATOR__CERES__LIDAR_RESIDUAL_HPP_
#define TAG_BASED_SFM_CALIBRATOR__CERES__LIDAR_RESIDUAL_HPP_

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <tag_based_sfm_calibrator/calibration_types.hpp>
#include <tag_based_sfm_calibrator/ceres/sensor_residual.hpp>
#include <tag_based_sfm_calibrator/types.hpp>

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <array>
#include <memory>

namespace tag_based_sfm_calibrator
{

struct LidarResidual : public SensorResidual
{
  LidarResidual(
    const UID & lidar_uid, const double & virtual_f, const LidartagDetection & detection,
    const std::array<double, CalibrationData::POSE_OPT_DIM> & fixed_lidar_pose_inv,
    bool fix_lidar_pose)
  : lidar_uid_(lidar_uid),
    virtual_f_(virtual_f),
    detection_(detection),
    fix_lidar_pose_(fix_lidar_pose)
  {
    fixed_lidar_rotation_inv_ = Eigen::Map<const Vector4<double>>(fixed_lidar_pose_inv.data());
    fixed_lidar_translation_inv_ =
      Eigen::Map<const Vector3<double>>(&fixed_lidar_pose_inv[TRANSLATION_X_INDEX]);

    // Instead of relying on the lidar coordinate system, we make a new one
    // based on the same origin but pointing towards the detection
    Eigen::Matrix3d detection_rotation_eigen;
    Eigen::Vector3d detection_translation_eigen;

    cv::cv2eigen(detection.pose.rotation(), detection_rotation_eigen);
    cv::cv2eigen(detection.pose.translation(), detection_translation_eigen);

    Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
    pose_matrix.block<3, 3>(0, 0) = detection_rotation_eigen;
    pose_matrix.block<3, 1>(0, 3) = detection_translation_eigen;

    // Compute z hat as the direction of the translation component
    Eigen::Vector3d z_hat = detection_translation_eigen.normalized();

    auto point_to_eigen = [](const cv::Point3d & p) -> Eigen::Vector3d {
      return Eigen::Vector3d(p.x, p.y, p.z);
    };

    auto vec_to_eigen = [](const cv::Vec3d & v) -> Eigen::Vector3d {
      return Eigen::Vector3d(v(0), v(1), v(2));
    };

    Eigen::Vector3d v1 = point_to_eigen(detection.object_corners[0]);
    Eigen::Vector3d v2 = point_to_eigen(detection.object_corners[1]);

    Eigen::Vector3d x_hat = v2 - v1;
    x_hat = (x_hat - x_hat.dot(z_hat) * z_hat).normalized();

    Eigen::Vector3d y_hat = z_hat.cross(x_hat);
    assert(std::abs(1.0 - y_hat.norm()) < 1e-5);

    assert(std::abs(x_hat.dot(y_hat)) < 1e-5);
    assert(std::abs(x_hat.dot(z_hat)) < 1e-5);

    tag_centric_rotation_.row(0) = x_hat;
    tag_centric_rotation_.row(1) = y_hat;
    tag_centric_rotation_.row(2) = z_hat;

    Eigen::Vector3d center = vec_to_eigen(detection.pose.translation());
    Eigen::Vector3d rotated_center = tag_centric_rotation_ * center;
    CV_Assert(std::abs(rotated_center.x()) < 1e-5);
    CV_Assert(std::abs(rotated_center.y()) < 1e-5);

    assert(static_cast<int>(detection.object_corners.size()) == NUM_CORNERS);
    for (std::size_t corner_index = 0; corner_index < detection.object_corners.size();
         corner_index++) {
      Eigen::Vector3d rotated_corner =
        tag_centric_rotation_ * point_to_eigen(detection.object_corners[corner_index]);
      observed_corners_[corner_index] = Eigen::Vector2d(
        virtual_f_ * rotated_corner.x() / rotated_corner.z(),
        virtual_f_ * rotated_corner.y() / rotated_corner.z());
    }
  }

  /*!
   * The implementation of the cost function
   * @param[in] camera_pose_inv The pose from the camera to the origin
   * @param[in] camera_intrinsics The camera intrinsics
   * @param[in] tag_pose The pose of the tag to project into the camera
   * @param[in] tag_rotation_z The pose from the ground plane to the origin (only used when using
   * ground tags)
   * @param[in] tag_pose_2d The pose from ground plane to the tag (only used when using ground tags)
   * @param[in] residuals The residual error of projecting the tag into the camera
   * @returns success status
   */
  template <typename T>
  bool impl(const T * const lidar_pose_inv, const T * const tag_pose, T * residuals) const
  {
    Vector3<T> template_corners[NUM_CORNERS] = {
      {T(detection_.template_corners[0].x), T(detection_.template_corners[0].y),
       T(detection_.template_corners[0].z)},
      {T(detection_.template_corners[1].x), T(detection_.template_corners[1].y),
       T(detection_.template_corners[1].z)},
      {T(detection_.template_corners[2].x), T(detection_.template_corners[2].y),
       T(detection_.template_corners[2].z)},
      {T(detection_.template_corners[3].x), T(detection_.template_corners[3].y),
       T(detection_.template_corners[3].z)}};

    Vector3<T> corners_wcs[NUM_CORNERS];
    Vector3<T> corners_lcs[NUM_CORNERS];
    Vector3<T> corners_lrcs[NUM_CORNERS];  // cSpell:ignore lrcs

    auto transform_corners =
      [](auto & quaternion, auto & translation, auto & input_corners, auto & output_corners) {
        for (int i = 0; i < NUM_CORNERS; i++) {
          output_corners[i] = quaternion * input_corners[i] + translation;
        }
      };

    auto rotate_corners = [](auto & rotation, auto & input_corners, auto & output_corners) {
      for (int i = 0; i < NUM_CORNERS; i++) {
        output_corners[i] = rotation * input_corners[i];
      }
    };

    // Template corners to World coordinate system (wcs)
    const Eigen::Map<const Vector4<T>> tag_rotation_map(tag_pose);
    const Eigen::Map<const Vector3<T>> tag_translation_map(&tag_pose[TRANSLATION_X_INDEX]);

    Eigen::Quaternion<T> tag_quaternion = {
      tag_rotation_map(ROTATION_W_INDEX), tag_rotation_map(ROTATION_X_INDEX),
      tag_rotation_map(ROTATION_Y_INDEX), tag_rotation_map(ROTATION_Z_INDEX)};

    tag_quaternion = tag_quaternion.normalized();

    transform_corners(tag_quaternion, tag_translation_map, template_corners, corners_wcs);

    // World corners to lidar coordinate system (lcs)
    if (fix_lidar_pose_) {
      const Eigen::Map<const Eigen::Vector4d> fixed_lidar_rotation_inv_map(
        fixed_lidar_rotation_inv_.data());
      const Eigen::Map<const Eigen::Vector3d> fixed_lidar_translation_inv_map(
        fixed_lidar_translation_inv_.data());

      Eigen::Quaternion<T> fixed_lidar_rotation_inv_quaternion = {
        T(1) * fixed_lidar_rotation_inv_map(ROTATION_W_INDEX),
        T(1) * fixed_lidar_rotation_inv_map(ROTATION_X_INDEX),
        T(1) * fixed_lidar_rotation_inv_map(ROTATION_Y_INDEX),
        T(1) * fixed_lidar_rotation_inv_map(ROTATION_Z_INDEX)};

      fixed_lidar_rotation_inv_quaternion = fixed_lidar_rotation_inv_quaternion.normalized();
      transform_corners(
        fixed_lidar_rotation_inv_quaternion, fixed_lidar_translation_inv_map, corners_wcs,
        corners_lcs);
    } else {
      const Eigen::Map<const Vector4<T>> lidar_rotation_inv_map(lidar_pose_inv);
      const Eigen::Map<const Vector3<T>> lidar_translation_inv_map(
        &lidar_pose_inv[TRANSLATION_X_INDEX]);

      Eigen::Quaternion<T> lidar_rotation_inv_quaternion = {
        lidar_rotation_inv_map(ROTATION_W_INDEX), lidar_rotation_inv_map(ROTATION_X_INDEX),
        lidar_rotation_inv_map(ROTATION_Y_INDEX), lidar_rotation_inv_map(ROTATION_Z_INDEX)};

      lidar_rotation_inv_quaternion = lidar_rotation_inv_quaternion.normalized();
      transform_corners(
        lidar_rotation_inv_quaternion, lidar_translation_inv_map, corners_wcs, corners_lcs);
    }

    Eigen::Matrix<T, 3, 3> tag_centric_rotation = tag_centric_rotation_.cast<T>();

    rotate_corners(tag_centric_rotation, corners_lcs, corners_lrcs);

    // Compute the reprojection error residuals
    auto compute_reprojection_error_point =
      [&](auto & predicted_ccs, auto observed_ics, auto * residuals) {
        const T f = T(virtual_f_);

        const T xp = predicted_ccs.x() / predicted_ccs.z();
        const T yp = predicted_ccs.y() / predicted_ccs.z();
        const T predicted_ics_x = f * xp;
        const T predicted_ics_y = f * yp;

        residuals[0] = predicted_ics_x - observed_ics.x();
        residuals[1] = predicted_ics_y - observed_ics.y();
      };

    for (int i = 0; i < NUM_CORNERS; i++) {
      compute_reprojection_error_point(corners_lrcs[i], observed_corners_[i], residuals + 2 * i);
    }

    return true;
  }

  /*!
   * The cost function wrapper for one of the following cases:
   *   - the basic case of a normal tag with the intrinsics not being optimized
   *   - the case of a fixed tag with intrinsics being optimized
   * @param[in] camera_pose_inv The pose from the camera to the origin
   * @param[in] tag_pose The pose of the tag
   * @param[in] residuals The residual error of projecting the tag into the camera
   * @returns success status
   */
  template <typename T>
  bool operator()(const T * const lidar_pose_inv, const T * const tag_pose, T * residuals) const
  {
    assert(fix_lidar_pose_ == false);

    return impl(lidar_pose_inv, tag_pose, residuals);
  }

  /*!
   * The cost function wrapper for the case where the camera has a fixed pose and intrinsics
   * @param[in] tag_pose The pose of the tag
   * @param[in] residuals The residual error of projecting the tag into the camera
   * @returns success status
   */
  template <typename T>
  bool operator()(const T * const tag_pose, T * residuals) const
  {
    assert(fix_lidar_pose_ == true);

    return impl(static_cast<T *>(nullptr), tag_pose, residuals);
  }

  /*!
   * Residual factory function of a normal tag & camera residual without exposing much templates
   * @param[in] camera_uid The detection's uid
   * @param[in] intrinsics The camera intrinsics
   * @param[in] detection The tag detection
   * @param[in] fixed_camera_pose_inv The fixed pose of the camera
   * @param[in] fixed_tag_pose The fixed tag of the pose
   * @param[in] fix_camera_pose Whether the camera pose should be fixed during optimization
   * @param[in] optimize_intrinsics Whether the intrinsics should be optimized
   * @returns the ceres residual
   */
  static ceres::CostFunction * createTagResidual(
    const UID & camera_uid, const double virtual_f, const LidartagDetection & detection,
    std::array<double, CalibrationData::POSE_OPT_DIM> & fixed_lidar_pose_inv, bool fix_lidar_pose)
  {
    auto f =
      new LidarResidual(camera_uid, virtual_f, detection, fixed_lidar_pose_inv, fix_lidar_pose);

    if (fix_lidar_pose) {
      return (new ceres::AutoDiffCostFunction<
              LidarResidual,
              RESIDUAL_DIM,   // 4 corners x 2 residuals
              POSE_OPT_DIM>(  // 7 tag pose parameters
        f));
    } else {
      return (new ceres::AutoDiffCostFunction<
              LidarResidual,
              RESIDUAL_DIM,   // 4 corners x 2 residuals
              POSE_OPT_DIM,   // 7 camera pose parameters
              POSE_OPT_DIM>(  // 7 tag pose parameters
        f));
    }
  }

  Eigen::Vector2d observed_corners_[NUM_CORNERS];

  UID lidar_uid_;
  double virtual_f_;
  LidartagDetection detection_;
  Eigen::Vector4d fixed_lidar_rotation_inv_;
  Eigen::Vector3d fixed_lidar_translation_inv_;
  Eigen::Vector4d fixed_tag_rotation_;
  Eigen::Vector3d fixed_tag_translation_;
  bool fix_lidar_pose_;

  Eigen::Matrix3d tag_centric_rotation_;
};

}  // namespace tag_based_sfm_calibrator

#endif  // TAG_BASED_SFM_CALIBRATOR__CERES__LIDAR_RESIDUAL_HPP_
