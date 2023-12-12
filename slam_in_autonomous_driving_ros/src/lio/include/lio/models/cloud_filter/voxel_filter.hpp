/**
 * @file voxel_filter.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_VOXEL_FILTER_HPP_
#define LIO_VOXEL_FILTER_HPP_

#include <lio/models/cloud_filter/cloud_filter_interface.hpp>

#include <pcl/filters/voxel_grid.h>

namespace lio
{
class VoxelFilter : public CloudFilterInterface
{
  public:
    // VoxelFilter(const YAML::Node& node);
    VoxelFilter(const float &leaf_size_x, const float &leaf_size_y, const float &leaf_size_z);
    VoxelFilter(const float &leaf_size);
    VoxelFilter() = default;
    ~VoxelFilter() = default;

    bool Filter(const CloudPtr &input_cloud_ptr, CloudPtr &filtered_cloud_ptr) override;

  private:

    bool SetFilterParam(const float &leaf_size_x, const float &leaf_size_y, const float &leaf_size_z);
    bool SetFilterParam(const float &leaf_size);

    // double downsample_resolution_ = 0.1;
    pcl::VoxelGrid<PointType> voxel_filter_;
};



}// namespace lio
#endif//LIO_VOXEL_FILTER_HPP_