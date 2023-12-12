/**
 * @file point_types.hpp
 * @author your name (you@domain.com)
 * @brief 自定义的点云数据类型
 * @version 0.1
 * @date 2023-10-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_POINT_TYPES_HPP
#define LIO_POINT_TYPES_HPP

#include <pcl/point_traits.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>

#include "eigen_types.hpp"


// *********************FullPointType**************************
/// 带ring, range等其他信息的全量信息点云
struct FullPointType {
    PCL_ADD_POINT4D;
    float range = 0;
    float radius = 0;
    uint8_t intensity = 0;
    uint8_t ring = 0;
    uint8_t angle = 0;
    double time = 0;
    double time_span = 0;
    double time_intervel = 0;
    float height = 0;

    inline FullPointType() {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(FullPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, range, range)(float, radius, radius)
                                  (std::uint8_t, intensity, intensity)(std::uint16_t, angle, angle)(std::uint8_t, ring, ring)
                                  (double, time, time)(double, time_span, time_span)(double, time_intervel, time_intervel)(float, height, height))

// clang-format on


// ***************************velodyne_ros******************
namespace velodyne_ros {
struct Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;
}  // namespace velodyne_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                      (float, time, time)(std::uint16_t, ring, ring))
// clang-format on



// ***************************ouster_ros******************
namespace ouster_ros {
struct Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16 ;
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                      (std::uint32_t, t, t)
                                      (std::uint16_t, reflectivity, reflectivity)
                                      (std::uint8_t, ring, ring)
                                      (std::uint16_t, ambient, ambient)
                                      (std::uint32_t, range, range)
)// clang-format off
                            

// ***************************rslidar_ros******************                           
namespace rslidar_ros{
    struct Point
    {   
        PCL_ADD_POINT4D;
        float intensity;
        double timestamp    = 0;
        uint16_t ring       = 0;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }EIGEN_ALIGN16;

}

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (double, timestamp, timestamp)
                                  (uint16_t, ring, ring)
)
// clang-format on


// 定义系统中用到的点和点云类型
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using IndexVec = std::vector<int>;


/// 全量点云的定义
using FullPointCloudType = pcl::PointCloud<FullPointType>;
using FullCloudPtr = FullPointCloudType::Ptr;

inline Vec3f ToVec3f(const FullPointType& pt) { return pt.getVector3fMap(); }
inline Vec3d ToVec3d(const FullPointType& pt) { return pt.getVector3fMap().cast<double>(); }

/// ui中的点云颜色
using UiPointType = pcl::PointXYZRGBA;
using UiPointCloudType = pcl::PointCloud<UiPointType>;
using UiCloudPtr = UiPointCloudType::Ptr;

//velodyne_ros
using VelodynePointType = velodyne_ros::Point;
using VelodynePointCloudType = pcl::PointCloud<VelodynePointType>;
using VelodyneCloudPtr = VelodynePointCloudType::Ptr;

//ouster_ros
using OusterPointType = ouster_ros::Point;
using OusterPointCloudType = pcl::PointCloud<OusterPointType>;
using OusterCloudPtr = OusterPointCloudType::Ptr;

//rslidar_ros
using RslidarPointType = rslidar_ros::Point;
using RslidarPointCloudType = pcl::PointCloud<RslidarPointType>;
using RslidarCloudPtr = RslidarPointCloudType::Ptr;


#endif  // LIO_POINT_TYPES_HPP
