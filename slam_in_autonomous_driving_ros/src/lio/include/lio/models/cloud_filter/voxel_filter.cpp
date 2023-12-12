/**
 * @file voxel_filter.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "voxel_filter.hpp"

namespace lio
{

VoxelFilter::VoxelFilter(const float &leaf_size_x, const float &leaf_size_y, const float &leaf_size_z)
{
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(const float &leaf_size)
{
    SetFilterParam(leaf_size);
}


bool VoxelFilter::Filter(const CloudPtr &input_cloud_ptr, CloudPtr &filtered_cloud_ptr)
{

    voxel_filter_.setInputCloud(input_cloud_ptr);
    voxel_filter_.filter(*filtered_cloud_ptr);

    return true;
}


bool VoxelFilter::SetFilterParam(const float &leaf_size_x, const float &leaf_size_y, const float &leaf_size_z)
{
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    return true;
}

bool VoxelFilter::SetFilterParam(const float &leaf_size)
{
    voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
    return true;
}
} // namespace lio