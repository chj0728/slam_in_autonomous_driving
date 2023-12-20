/**
 * @file tf2_broadcaster.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_TF2_BROADCASTER_HPP_
#define LIO_TF2_BROADCASTER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/utils.h>
#include <tf2_ros/static_transform_broadcaster.h>
// #include <tf2_ros/transform_broadcaster.h>


#include <common/common.hpp>

namespace lio
{   
    /**
     * @brief   通过ros发布tf
     * @param[in] base_frame_id 坐标系名
     * @param[in] child_frame_id 子坐标系名
     * 
     */
    class TF2BroadCaster
    {
        public:
            TF2BroadCaster(std::string &base_frame_id, std::string &child_frame_id);
            TF2BroadCaster() = default;

            void SendTF(const Mat4f &pose);
            void SendTF(const Mat4f &pose, const double &time);
            void SendTF(const Mat4f &pose, const ros::Time &time);

        protected:
            tf2_ros::StaticTransformBroadcaster br_;
            geometry_msgs::TransformStamped tf_;

    };
    
} // namespace lio



#endif //endif