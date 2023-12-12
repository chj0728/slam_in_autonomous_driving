/**
 * @file cloud_data.hpp
 * @author your name (you@domain.com)
 * @brief 自定义点云数据结构，包含当前点云的时间戳和点云数据
 * @version 0.1
 * @date 2023-11-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_CLOUD_DATA_HPP_
#define LIO_CLOUD_DATA_HPP_

#include <common/point_types.hpp>

namespace lio
{   
/**
 * @brief 点云数据结构，包含当前点云的时间戳和点云数据
 * 
 * @tparam CloudT 具体的点云类型：PointCloudType,FullPointCloudType, VelodynePointCloudType, OusterPointCloudType, RslidarPointCloudType
 * @returns using CloudData         = CloudDataTemplate<PointCloudType>; 
            using CloudDataFull     = CloudDataTemplate<FullPointCloudType>;
            using CloudDataVelodyne = CloudDataTemplate<VelodynePointCloudType>;
            using CloudDataOuster   = CloudDataTemplate<OusterPointCloudType>;
            using CloudDataRslidar  = CloudDataTemplate<RslidarPointCloudType>;
 */
template <typename CloudT>
class CloudDataTemplate
{   
public:
    CloudDataTemplate()
    : cloud_ptr(new CloudT())
    {
        // cloud_ptr = std::make_shared<CloudT>();
    }

    // std::string frame_id_ = "";
    double time = 0.0;
    typename CloudT::Ptr cloud_ptr;
};

using CloudData         = CloudDataTemplate<PointCloudType>;
using CloudDataFull     = CloudDataTemplate<FullPointCloudType>;
using CloudDataVelodyne = CloudDataTemplate<VelodynePointCloudType>;
using CloudDataOuster   = CloudDataTemplate<OusterPointCloudType>;
using CloudDataRslidar  = CloudDataTemplate<RslidarPointCloudType>;


} // namespace lio


#endif//LIO_CLOUD_DATA_HPP_