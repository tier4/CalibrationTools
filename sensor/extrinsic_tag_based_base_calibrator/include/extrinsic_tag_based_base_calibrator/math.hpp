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
#include <extrinsic_tag_based_base_calibrator/calibration_types.hpp>
#include <extrinsic_tag_based_base_calibrator/scene_types.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>

#include <memory>
#include <vector>

namespace extrinsic_tag_based_base_calibrator
{

/*
 * Method to find the average of a set of rotation quaternions using Singular Value Decomposition
 * Snippet taken from https://gist.github.com/PeteBlackerThe3rd/f73e9d569e29f23e8bd828d7886636a0
 * The algorithm used is described here:
 * https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
 */
Eigen::Vector4d quaternionAverage(std::vector<Eigen::Vector4d> quaternions);

/*!
 * Computes an array of corners from a tag pose and its size
 * @param pose the calibrated intrinsics
 * @param size the side-to-side size of the tag
 * @returns the tags corners
 */
std::array<cv::Vec3d, 4> tagPoseToCorners(const cv::Affine3d & pose, double size);

/*!
 * Computes a pose to the ground plane by fitting a plane to a set of points and then projecting the
 * origin to said plane
 * @param[in] points the points used to calculate the ground pose from
 * @param[out] ground_pose the ground pose
 * @returns whether or not the algorithm succeeded
 */
bool computeGroundPlane(const std::vector<cv::Vec3d> & points, cv::Affine3d & ground_pose);

/*!
 * Computes a pose to the ground plane by fitting a plane to a set of tag poses and then projecting
 * the origin to said plane
 * @param[in] poses the tag poses to use to compute the ground plane
 * @param[in] tag_size the sized of the tags
 * @param[out] ground_pose the ground pose
 * @returns whether or not the algorithm succeeded
 */
bool computeGroundPlane(
  const std::vector<std::shared_ptr<cv::Affine3d>> & poses, double tag_size,
  cv::Affine3d & ground_pose);

/*!
 * Computes the base link pose by projecting the mid point between the left and right wheel poses
 * into the ground plane
 * @param[in] left_wheel_pose the left whee pose
 * @param[in] right_wheel_pose right whee pose
 * @param[in] ground_pose ground pose
 * @returns the base link pose
 */
cv::Affine3d computeBaseLink(
  const cv::Affine3d & left_wheel_pose, const cv::Affine3d right_wheel_pose,
  const cv::Affine3d & ground_pose);

/*!
 * Projects a point into an image using a distortion model
 * @param[in] p the point to be projected
 * @param[in] fx the focal point in x
 * @param[in] fy the focal point in y
 * @param[in] cx the optical center in x
 * @param[in] cy the optical center in y
 * @param[in] k1 the 1st radial distortion component
 * @param[in] k2 the 2st radial distortion component
 * @returns the base link pose
 */
cv::Point2d projectPoint(
  const cv::Vec3d & p, double fx, double fy, double cx, double cy, double k1, double k2);

void estimateInitialPoses(CalibrationData & data, const UID & main_sensor_uid);

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__MATH_HPP_