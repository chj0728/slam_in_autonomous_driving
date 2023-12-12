/**
 * @file ndt_registration.hpp
 * @author your name (you@domain.com)
 * @brief ndt匹配算法,继承RegistrationInterface接口
 * class NDTRegistration : public RegistrationInterface
 * @version 0.1
 * @date 2023-11-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_NDT_REGISTRATION_HPP_
#define LIO_NDT_REGISTRATION_HPP_

#include "registration_interface.hpp"

// #include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <pcl/registration/ndt.h>

namespace lio
{   
/**
 * @brief ndt匹配算法,继承RegistrationInterface接口
 * 
 */
class NDTRegistration : public RegistrationInterface
{   
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        NDTRegistration(const YAML::Node& node);
        NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

        bool SetInputTarget(const CloudPtr& input_target) override;

        bool ScanMatch(const CloudPtr& input_cloud_ptr, 
                            const Mat4f& predict_pose, 
                            CloudPtr& result_cloud_ptr, 
                            Mat4f& result_pose) override;

        bool ScanMatch(const CloudPtr& input_cloud_ptr, 
                            const SE3f& predict_pose, 
                            CloudPtr& result_cloud_ptr, 
                            SE3f& result_pose) override;

        float GetFitnessScore() override;

    private:
        bool SetRegistrationParam(
            float res, 
            float step_size, 
            float trans_eps, 
            int max_iter);

        pcl::NormalDistributionsTransform<PointType, PointType>::Ptr ndt_ptr_;

};


}// end namespace lio
#endif //LIO_NDT_REGISTRATION_HPP_