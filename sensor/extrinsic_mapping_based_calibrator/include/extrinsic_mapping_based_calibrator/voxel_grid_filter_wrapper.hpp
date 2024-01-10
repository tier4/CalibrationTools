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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR__VOXEL_GRID_FILTER_WRAPPER_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR__VOXEL_GRID_FILTER_WRAPPER_HPP_

#include <tier4_calibration_pcl_extensions/voxel_grid_triplets_impl.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

template <typename PointType>
class VoxelGridWrapper
{
public:
  void setLeafSize(float leaf_x, float leaf_y, float leaf_z)
  {
    voxel_grid.setLeafSize(leaf_x, leaf_y, leaf_z);
    voxel_triplets.setLeafSize(leaf_x, leaf_y, leaf_z);
  }

  void setInputCloud(typename pcl::PointCloud<PointType>::Ptr input)
  {
    input_size = input->size();
    voxel_grid.setInputCloud(input);
    voxel_triplets.setInputCloud(input);
  }

  void filter(typename pcl::PointCloud<PointType> & output)
  {
    voxel_grid.filter(output);

    if (output.size() == input_size) {
      voxel_triplets.filter(output);
    }
  }

protected:
  std::size_t input_size;
  pcl::VoxelGrid<PointType> voxel_grid;
  pcl::VoxelGrid<PointType> voxel_triplets;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR__VOXEL_GRID_FILTER_WRAPPER_HPP_
