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

#ifndef TAG_BASED_PNP_CALIBRATOR__BRUTE_FORCE_MATCHER_HPP_
#define TAG_BASED_PNP_CALIBRATOR__BRUTE_FORCE_MATCHER_HPP_

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;  // cSpell:ignore FPFH
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT>
  FeatureEstimationT;  // cSpell:ignore FPFH
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

bool bruteForceMatcher(
  PointCloudT::Ptr & source, PointCloudT::Ptr & target, double thresh,
  std::vector<int> & source_indexes, std::vector<int> & target_indexes, bool debug = false);

#endif  // TAG_BASED_PNP_CALIBRATOR__BRUTE_FORCE_MATCHER_HPP_
