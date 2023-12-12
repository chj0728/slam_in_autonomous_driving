/**
 * @file front_end.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef LIO_MAPPING_FRONT_END_HPP_
#define LIO_MAPPING_FRONT_END_HPP_

#include<yaml-cpp/yaml.h>

#include<common/common.hpp>

#include<lio/sensors/cloud_data.hpp>

#include<lio/models/cloud_filter/cloud_filter_interface.hpp>//基类
#include<lio/models/cloud_filter/voxel_filter.hpp>//子类 clss VoxelFilter

#include<lio/models/registration/registration_interface.hpp>//基类
#include<lio/models/registration/ndt_registration.hpp>//子类 class NDTRegistration

namespace lio
{
    class FrontEnd
    {   
       
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
            /**
             * @brief 当前点云帧和位姿
             * 
             */
            struct Frame
            {
                CloudData cloud_data;
                Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            };

            FrontEnd(const YAML::Node& node);
            
            bool SetInitPose(const Mat4f& init_pose);//设置初始位姿
            bool Update(const CloudData& cloud_data, Mat4f& cloud_pose);//更新当前帧
            bool UpdateWithNewFrame(const Frame& new_key_frame);//更新关键帧

            PointCloudType GetLocalMap()
            {
                return *local_map_ptr_;
            }
            
            CloudPtr GetLocalMapPtr()
            {
                return local_map_ptr_;
            }

        private:

            std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;//帧滤波器
            std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;//局部地图滤波器
            std::shared_ptr<RegistrationInterface> registration_ptr_;//配准器

            Frame current_frame_;//当前帧
            std::deque<Frame> local_map_frames_;//局部地图帧队列

            CloudPtr local_map_ptr_;//局部地图

            Mat4f init_pose_ = Mat4f::Identity();//初始位姿

            float key_frame_distance_ = 2.0;//关键帧距离
            float key_frame_angle_ = 0.15;//关键帧角度
            int   local_frame_size_ = 20;//局部地图帧队列大小

    };// class FrontEnd
    


}// namespace lio

#endif//LIO_MAPPING_FRONT_END_FRONT_END_HPP_