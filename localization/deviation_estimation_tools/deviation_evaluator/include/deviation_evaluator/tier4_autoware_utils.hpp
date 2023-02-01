// Copyright 2018-2019 Autoware Foundation
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

#ifndef DEVIATION_EVALUATOR__TIER4_AUTOWARE_UTILS_HPP_
#define DEVIATION_EVALUATOR__TIER4_AUTOWARE_UTILS_HPP_

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/constants.hpp"
#include "tier4_autoware_utils/math/normalization.hpp"

#include <tf2/utils.h>

// ToDo (kminoda): Replace these functions with the one from tier4_autoware_utils.
// Currently these functions are declared here since this tool has to be compatible with older
// version of Autoware.

template <class Pose1, class Pose2>
bool isDrivingForward(const Pose1 & src_pose, const Pose2 & dst_pose)
{
  // check the first point direction
  const double src_yaw = tf2::getYaw(tier4_autoware_utils::getPose(src_pose).orientation);
  const double pose_direction_yaw = tier4_autoware_utils::calcAzimuthAngle(
    tier4_autoware_utils::getPoint(src_pose), tier4_autoware_utils::getPoint(dst_pose));
  return std::fabs(tier4_autoware_utils::normalizeRadian(src_yaw - pose_direction_yaw)) <
         tier4_autoware_utils::pi / 2.0;
}

/**
 * @brief Calculate a point by linear interpolation.
 * @param src source point
 * @param dst destination point
 * @param ratio interpolation ratio, which should be [0.0, 1.0]
 * @return interpolated point
 */
template <class Point1, class Point2>
geometry_msgs::msg::Point calcInterpolatedPoint(
  const Point1 & src, const Point2 & dst, const double ratio)
{
  const auto src_point = tier4_autoware_utils::getPoint(src);
  const auto dst_point = tier4_autoware_utils::getPoint(dst);

  tf2::Vector3 src_vec;
  src_vec.setX(src_point.x);
  src_vec.setY(src_point.y);
  src_vec.setZ(src_point.z);

  tf2::Vector3 dst_vec;
  dst_vec.setX(dst_point.x);
  dst_vec.setY(dst_point.y);
  dst_vec.setZ(dst_point.z);

  // Get pose by linear interpolation
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  const auto & vec = tf2::lerp(src_vec, dst_vec, clamped_ratio);

  geometry_msgs::msg::Point point;
  point.x = vec.x();
  point.y = vec.y();
  point.z = vec.z();

  return point;
}

/**
 * @brief Calculate a pose by linear interpolation.
 * Note that if dist(src_pose, dst_pose)<=0.01
 * the orientation of the output pose is same as the orientation
 * of the dst_pose
 * @param src source point
 * @param dst destination point
 * @param ratio interpolation ratio, which should be [0.0, 1.0]
 * @param set_orientation_from_position_direction set position by spherical interpolation if false
 * @return interpolated point
 */
template <class Pose1, class Pose2>
geometry_msgs::msg::Pose calcInterpolatedPose(
  const Pose1 & src_pose, const Pose2 & dst_pose, const double ratio,
  const bool set_orientation_from_position_direction = true)
{
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);

  geometry_msgs::msg::Pose output_pose;
  output_pose.position = calcInterpolatedPoint(
    tier4_autoware_utils::getPoint(src_pose), tier4_autoware_utils::getPoint(dst_pose),
    clamped_ratio);

  if (set_orientation_from_position_direction) {
    const double input_poses_dist = tier4_autoware_utils::calcDistance2d(
      tier4_autoware_utils::getPoint(src_pose), tier4_autoware_utils::getPoint(dst_pose));
    const bool is_driving_forward = isDrivingForward(src_pose, dst_pose);

    // Get orientation from interpolated point and src_pose
    if ((is_driving_forward && clamped_ratio > 1.0 - (1e-6)) || input_poses_dist < 1e-3) {
      output_pose.orientation = tier4_autoware_utils::getPose(dst_pose).orientation;
    } else if (!is_driving_forward && clamped_ratio < 1e-6) {
      output_pose.orientation = tier4_autoware_utils::getPose(src_pose).orientation;
    } else {
      const auto & base_pose = is_driving_forward ? dst_pose : src_pose;
      const double pitch = tier4_autoware_utils::calcElevationAngle(
        tier4_autoware_utils::getPoint(output_pose), tier4_autoware_utils::getPoint(base_pose));
      const double yaw = tier4_autoware_utils::calcAzimuthAngle(
        tier4_autoware_utils::getPoint(output_pose), tier4_autoware_utils::getPoint(base_pose));
      output_pose.orientation = tier4_autoware_utils::createQuaternionFromRPY(0.0, pitch, yaw);
    }
  } else {
    // Get orientation by spherical linear interpolation
    tf2::Transform src_tf;
    tf2::Transform dst_tf;
    tf2::fromMsg(tier4_autoware_utils::getPose(src_pose), src_tf);
    tf2::fromMsg(tier4_autoware_utils::getPose(dst_pose), dst_tf);
    const auto & quaternion = tf2::slerp(src_tf.getRotation(), dst_tf.getRotation(), clamped_ratio);
    output_pose.orientation = tf2::toMsg(quaternion);
  }

  return output_pose;
}

#endif  // DEVIATION_EVALUATOR__TIER4_AUTOWARE_UTILS_HPP_
