/**
 * @file front_end.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "front_end.hpp"

#include <glog/logging.h>

namespace lio
{
    FrontEnd::FrontEnd(const YAML::Node& node)
    {
        // 1. 初始化帧滤波器
        std::string frame_filter_method = node["lio_mapping"]["frame_filter_method"].as<std::string>();
        if(frame_filter_method == "voxel_filter")
        {
            float leaf_size = node["lio_mapping"]["frame_filter_voxel_size"].as<float>();
            frame_filter_ptr_ = std::make_shared<VoxelFilter>(leaf_size);
        }
        else
        {
            LOG(ERROR) << "Invalid frame filter method, please check your config file.";
            return;
        }

        // 2. 初始化局部地图滤波器
        std::string local_map_filter_method = node["lio_mapping"]["local_map_filter_method"].as<std::string>();
        if(local_map_filter_method == "voxel_filter")
        {
            float leaf_size = node["lio_mapping"]["local_map_filter_voxel_size"].as<float>();
            local_map_filter_ptr_ = std::make_shared<VoxelFilter>(leaf_size);
        }
        else
        {
            LOG(ERROR) << "Invalid local map filter method, please check your config file.";
            return;
        }

        // 3. 初始化配准器
        std::string registration_method = node["lio_mapping"]["registration_method"].as<std::string>();
        if(registration_method == "NDT")
        {
            registration_ptr_ = std::make_shared<NDTRegistration>(node);
        }
        else
        {
            LOG(ERROR) << "Invalid registration method, please check your config file.";
            return;
        }

        // 4. 初始化局部地图
        local_map_ptr_.reset(new PointCloudType);

        // 5. 初始化关键帧
        key_frame_distance_ = node["lio_mapping"]["kf_dist"].as<float>();
        local_frame_size_   = node["lio_mapping"]["kf_nums_in_local_map"].as<int>();
    }

    bool FrontEnd::SetInitPose(const Mat4f& init_pose)
    {
        init_pose_ = init_pose;
        return true;
    }

    bool FrontEnd::Update(const CloudData& cloud_data, Mat4f& cloud_pose)
    {
        current_frame_.cloud_data.time = cloud_data.time;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);
        
        // 1. 过滤当前帧
        CloudPtr filtered_cloud_ptr(new PointCloudType);
        frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

        // 
        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
        static Eigen::Matrix4f last_pose = init_pose_;
        static Eigen::Matrix4f predict_pose = init_pose_;
        static Eigen::Matrix4f last_key_frame_pose = init_pose_;

        // 局部地图容器中没有关键帧，代表是第一帧数据
        // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
        if (local_map_frames_.size() == 0)
        {
            current_frame_.pose = init_pose_;

            double time1 = ros::Time::now().toSec();
            UpdateWithNewFrame(current_frame_);
            double time_end1 = ros::Time::now().toSec();
            LOG(INFO) << "First UpdateWithNewFrame Time: " << time_end1 - time1;


            cloud_pose = current_frame_.pose;
            return true;
        }

        // 2. 进行配准
        CloudPtr result_cloud_ptr(new PointCloudType);
        
        double time2 = ros::Time::now().toSec();
        registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose,result_cloud_ptr, current_frame_.pose);
        double time_end2 = ros::Time::now().toSec();
        LOG(INFO) << "registration_ptr_->ScanMatch Time: " << time_end2 - time2;
        
        // 3. 更新当前帧位姿
        cloud_pose = current_frame_.pose;       

         // 更新相邻两帧的相对运动
        step_pose = last_pose.inverse() * current_frame_.pose;
        predict_pose = current_frame_.pose * step_pose;
        last_pose = current_frame_.pose;

        // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
        if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
            fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
            fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) 
        {
            UpdateWithNewFrame(current_frame_);
            last_key_frame_pose = current_frame_.pose;
        }

        return true;
    }


    bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame)
    {   
        Frame key_frame = new_key_frame;
        // 这一步的目的是为了把关键帧的点云保存下来
        // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
        // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
        // 所以这里需要重新分配内存，把点云数据复制一份
        key_frame.cloud_data.cloud_ptr.reset(new PointCloudType(*new_key_frame.cloud_data.cloud_ptr));

        CloudPtr transformed_cloud_ptr(new PointCloudType);

        // 1. 更新局部地图  
        local_map_frames_.emplace_back(key_frame);
        if (local_map_frames_.size() > local_frame_size_)
        {
            local_map_frames_.pop_front();
        }
        local_map_ptr_.reset(new PointCloudType);
        for (size_t i = 0; i < local_map_frames_.size(); ++i)
        {
            //局部地图由多帧点云组成，每一帧点云都需要根据自己的位姿进行坐标变换
            pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                    *transformed_cloud_ptr, 
                                    local_map_frames_.at(i).pose);

            *local_map_ptr_ += *transformed_cloud_ptr;
        }

        // 更新ndt匹配的目标点云
        // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
        if (local_map_frames_.size() < 25) 
        {
            registration_ptr_->SetInputTarget(local_map_ptr_);
        } 
        else 
        {
            CloudPtr filtered_local_map_ptr(new PointCloudType);
            local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
            registration_ptr_->SetInputTarget(filtered_local_map_ptr);
        }

        return true;

    }

}// namespace lio