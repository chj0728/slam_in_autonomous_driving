/**
 * @file registration_interface.hpp
 * @author your name (you@domain.com)
 * @brief 前端点云匹配接口基类
 * @version 0.1
 * @date 2023-11-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_REGISTRATION_INTERFACE_HPP_
#define LIO_REGISTRATION_INTERFACE_HPP_


#include <lio/sensors/cloud_data.hpp>
#include <lio/sensors/imu_data.hpp>

#include <common/common.hpp>



namespace lio
{
/**
 * @brief 前端点云匹配接口基类
 * 
 */
class RegistrationInterface
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    /**
     * @brief 构造函数
     * 
     */
    RegistrationInterface() = default;

    /**
     * @brief 析构函数
     * 
     */
    virtual ~RegistrationInterface() = default;

    /**
     * @brief 设置匹配参数
     * 
     * @param config 
     */
    // virtual void SetRegistrationParam(const RegistrationParam& config) = 0;

    virtual bool SetInputTarget(const CloudPtr& input_target) = 0;

    virtual bool ScanMatch(const CloudPtr& input_cloud_ptr, 
                            const Mat4f& predict_pose, 
                            CloudPtr& result_cloud_ptr, 
                            Mat4f& result_pose) = 0;

    virtual bool ScanMatch(const CloudPtr& input_cloud_ptr, 
                            const SE3f& predict_pose, 
                            CloudPtr& result_cloud_ptr, 
                            SE3f& result_pose) = 0;                        

    /**
     * @brief 设置点云匹配的初始位姿
     * 
     * @param imu_data 
     * @param cloud_data 
     * @return Eigen::Matrix4f 
     */
    // virtual Eigen::Matrix4f Update(const IMUData& imu_data, const CloudData& cloud_data) = 0;

    virtual float GetFitnessScore() = 0;

}; 

}// namespace lio
#endif