

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <tier4_pcl_extensions/voxel_grid_triplets_impl.hpp>


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

  void filter(typename pcl::PointCloud<PointType> &  output)
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
