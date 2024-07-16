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

#ifndef EXTRINSIC_MAP_BASED_CALIBRATOR__GRID_SEARCH_MATCHING_HPP_
#define EXTRINSIC_MAP_BASED_CALIBRATOR__GRID_SEARCH_MATCHING_HPP_

#include "extrinsic_map_based_calibrator/pointcloud_matcher.hpp"
#include "pcl/PCLPointCloud2.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.hpp"

#include "tier4_calibration_msgs/srv/extrinsic_calibrator.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

namespace extrinsic_map_base_calibrator
{

struct GridSearchConfig
{
  double x_range_min_;
  double x_range_max_;
  double x_resolution_;
  double y_range_min_;
  double y_range_max_;
  double y_resolution_;
  double z_range_min_;
  double z_range_max_;
  double z_resolution_;
  double roll_range_min_;
  double roll_range_max_;
  double roll_resolution_;
  double pitch_range_min_;
  double pitch_range_max_;
  double pitch_resolution_;
  double yaw_range_min_;
  double yaw_range_max_;
  double yaw_resolution_;
  MatchingConfig matching_config;
};

class GridSearchMatching
{
public:
  GridSearchMatching();
  void setParameter(GridSearchConfig & config) { config_ = config; }
  bool executeGridSearchMatching(
    const PointCloudT::Ptr & map_pointCloud, const PointCloudT::Ptr & sensor_pointCloud);
  matchingResult getRematchedResult() { return rematched_result_; }
  matchingResult getSearchedResult() { return searched_result_; }

private:
  matchingResult rematched_result_;
  matchingResult searched_result_;
  PointCloudMatcher matcher_;
  GridSearchConfig config_;

  matchingResult gridSearch(
    const PointCloudT::Ptr & map_pointCloud, const PointCloudT::Ptr & sensor_pointCloud);
  static Eigen::Matrix4d getMatrix4d(
    const double & x, const double & y, const double & z, const double & roll, const double & pitch,
    const double & yaw);
  static std::vector<double> generateSearchElement(
    const double & range_min, const double & range_max, const double & resolution);
};

}  // namespace extrinsic_map_base_calibrator
#endif  // EXTRINSIC_MAP_BASED_CALIBRATOR__GRID_SEARCH_MATCHING_HPP_
