/**
 * @file cloud_filter_interface.hpp
 * @author your name (you@domain.com)
 * @brief 点云滤波接口类
 * @version 0.1
 * @date 2023-11-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_CLOUD_FILTER_INTERFACE_HPP_
#define LIO_CLOUD_FILTER_INTERFACE_HPP_

#include <lio/sensors/cloud_data.hpp>

#include <common/common.hpp>

namespace lio
{


class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const CloudPtr& input_cloud_ptr, CloudPtr& filtered_cloud_ptr) = 0;
};



}// namespace lio





#endif//LIO_CLOUD_FILTER_INTERFACE_HPP_