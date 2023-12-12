/**
 * @file ndt_registration.cpp
 * @author your name (you@domain.com)
 * @brief  具体实现ndt匹配算法
 * @version 0.1
 * @date 2023-11-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ndt_registration.hpp"

namespace lio
{
    NDTRegistration::NDTRegistration(const YAML::Node& node):
    ndt_ptr_(new pcl::NormalDistributionsTransform<PointType, PointType>())
    {
        float res       = node["lio_mapping"]["ndt_option"]["res"].as<float>();
        float step_size = node["lio_mapping"]["ndt_option"]["step_size"].as<float>();
        float trans_eps = node["lio_mapping"]["ndt_option"]["trans_eps"].as<float>();
        int max_iter    = node["lio_mapping"]["ndt_option"]["max_iter"].as<int>();
        
        SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }

    NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
        :ndt_ptr_(new pcl::NormalDistributionsTransform<PointType, PointType>()) 
    {

        SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }
    
    bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) 
    {
        ndt_ptr_->setResolution(res);
        ndt_ptr_->setStepSize(step_size);
        ndt_ptr_->setTransformationEpsilon(trans_eps);
        ndt_ptr_->setMaximumIterations(max_iter);

        std::cout << "NDT 的匹配参数为：" << std::endl
                << "网格大小->res: " << res << ", "
                << "牛顿法优化的最大步长->step_size: " << step_size << ", "
                << "连续变化之间允许的最大误差->trans_eps: " << trans_eps << ", "
                << "最大迭代次数->max_iter: " << max_iter 
                << std::endl;

        return true;
    }

    bool NDTRegistration::SetInputTarget(const CloudPtr& input_target)
    {
        ndt_ptr_->setInputTarget(input_target);
        return true;
    }

    bool NDTRegistration::ScanMatch(const CloudPtr& input_cloud_ptr, 
                        const Mat4f& predict_pose, 
                        CloudPtr& result_cloud_ptr, 
                        Mat4f& result_pose)
    {
        ndt_ptr_->setInputSource(input_cloud_ptr);
        // CloudPtr result_cloud_ptr(new pcl::PointCloud<PointType>());
        ndt_ptr_->align(*result_cloud_ptr, predict_pose);
        result_pose = ndt_ptr_->getFinalTransformation();
        return true;
    
    }

    bool NDTRegistration::ScanMatch(const CloudPtr& input_cloud_ptr,
                        const SE3f& predict_pose,
                        CloudPtr& result_cloud_ptr,
                        SE3f& result_pose)
    {
        ndt_ptr_->setInputSource(input_cloud_ptr);

        // CloudPtr result_cloud_ptr(new pcl::PointCloud<PointType>());
        // Mat4f predict_pose_mat = predict_pose.matrix().cast<float>();

        ndt_ptr_->align(*result_cloud_ptr, predict_pose.matrix());
        result_pose.matrix() = ndt_ptr_->getFinalTransformation();
        return true;
    
    }


    float NDTRegistration::GetFitnessScore() 
    {
        return ndt_ptr_->getFitnessScore();
    }

}// end namespace lio