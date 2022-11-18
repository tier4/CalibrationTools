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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__MATH_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__MATH_HPP_

#include <Eigen/Core>
#include <extrinsic_tag_based_base_calibrator/types.hpp>

#include <memory>
#include <vector>

namespace extrinsic_tag_based_base_calibrator
{

/*
 * Method to find the average of a set of rotation quaternions using Singular Value Decomposition
 * Snipped taken from https://gist.github.com/PeteBlackerThe3rd/f73e9d569e29f23e8bd828d7886636a0
 * The algorithm used is described here:
 * https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
 */
Eigen::Vector4d quaternionAverage(std::vector<Eigen::Vector4d> quaternions);

std::array<cv::Vec3d, 4> tagPoseToCorners(const cv::Affine3d & pose, double size);

bool computeGroundPlane(const std::vector<cv::Vec3d> & points, cv::Affine3d & ground_pose);

bool computeGroundPlane(
  const std::vector<std::shared_ptr<cv::Affine3d>> & poses, double tag_size,
  cv::Affine3d & ground_pose);

cv::Affine3d computeBaseLink(
  const cv::Affine3d & left_wheel_pose, const cv::Affine3d right_wheel_pose,
  const cv::Affine3d & ground_pose);

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__MATH_HPP_
