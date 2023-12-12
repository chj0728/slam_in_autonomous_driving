/**
 * @file point_cloud_utils.hpp
 * @author your name (you@domain.com)
 * @brief 点云的一些常用操作
 * @version 0.1
 * @date 2023-11-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_POINT_CLOUD_UTILS_HPP_
#define LIO_POINT_CLOUD_UTILS_HPP_

#include "point_types.hpp"

#include <pcl/common/transforms.h>  //点云变换
#include <pcl/filters/voxel_grid.h> //体素滤波器
#include <pcl/io/pcd_io.h>          //点云文件读写



// ********************点云到Eigen的常用的转换函数********************

/**
 * @brief getVector3fMap()返回一个Eigen::Map<Eigen::Vector3f>对象，该对象的数据指针指向pt的x,y,z成员
 * 
 * @param pt  PointType& pt
 * @return Vec3f 
 */
inline Vec3f ToVec3f(const PointType& pt) { return pt.getVector3fMap(); }

/**
 * @brief getVector3fMap()返回一个Eigen::Map<Eigen::Vector3f>对象，该对象的数据指针指向pt的x,y,z成员
 * 
 * @param pt  PointType& pt
 * @return Vec3d 
 */
inline Vec3d ToVec3d(const PointType& pt) { return pt.getVector3fMap().cast<double>(); }

// 模板类型转换函数
/**
 * @brief 模板类型转换函数
 * 
 * @tparam T 
 * @tparam dim 
 * @param pt 
 * @return Eigen::Matrix<T, dim, 1> 
 */
template <typename T, int dim>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType& pt);

/**
 * @brief PointType转换为Eigen::Matrix<float, 2, 1>
 * 
 * @tparam  
 * @param pt PointType& pt
 * @return Eigen::Matrix<float, 2, 1> 
 */
template <>
inline Eigen::Matrix<float, 2, 1> ToEigen<float, 2>(const PointType& pt) {
    return Vec2f(pt.x, pt.y);
}

/**
 * @brief PointType转换为Eigen::Matrix<float, 3, 1>
 * 
 * @tparam  
 * @param pt 
 * @return Eigen::Matrix<float, 3, 1> 
 */
template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3>(const PointType& pt) {
    return Vec3f(pt.x, pt.y, pt.z);
}

/**
 * @brief Eigen::Matrix<S, 3, 1>转换为PointType
 * 
 * @tparam S 
 * @param pt 
 * @return PointType 
 */
template <typename S>
inline PointType ToPointType(const Eigen::Matrix<S, 3, 1>& pt) {
    PointType p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    return p;
}


// ********************点云操作********************
/**
 * @brief 移除点云中的nan点
 * 
 * @param in CloudPtr &in
 * @return CloudPtr 
 */
inline CloudPtr RemoveNanPoint(const CloudPtr &in) {
    CloudPtr out(new PointCloudType);
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*in, *out, index);
    return out;
}

/**
 * @brief 体素滤波器
 * 
 * @param in 
 * @param leaf_size 
 */
inline void VoxelFilterFunc(const CloudPtr &in, float leaf_size) {

    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.setInputCloud(in);

    CloudPtr out(new PointCloudType);
    voxel_filter.filter(*out);
    in->swap(*out);
}
/**
 * @brief 移除地面点
 * 
 * @param in CloudPtr &in
 * @param z_min 
 */
inline void RemoveGround(const CloudPtr &in, float z_min)
{
    CloudPtr out(new PointCloudType);
    for (auto &p : in->points)
    {
        if (p.z > z_min)
        {
            out->points.emplace_back(p);
        }
    }

    out->width = out->points.size();
    out->height = 1;
    out->is_dense = false;
    in->swap(*out);
    
}

template <typename Point>
float PointDistance(Point p)
{
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

// ********************点云文件读写********************
inline void SaveCloud(const std::string &file_path, const CloudPtr &cloud)
{
    pcl::io::savePCDFileASCII(file_path, *cloud);
    std::cout << "save cloud to " << file_path << " finished" << std::endl;
}

inline void ReadCloud(const std::string &file_path, CloudPtr &cloud)
{
    pcl::io::loadPCDFile(file_path, *cloud);
    std::cout << "read cloud from " << file_path << " finished" << std::endl;
}


inline void TransformPointCloud(const CloudPtr& in, CloudPtr& out, const SE3& tf) {
    pcl::transformPointCloud(*in, *out, tf.matrix().cast<float>());
}


#endif // LIO_POINT_CLOUD_UTILS_HPP_