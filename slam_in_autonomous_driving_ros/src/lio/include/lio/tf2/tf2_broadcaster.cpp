/**
 * @file tf2_broadcaster.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "tf2_broadcaster.hpp"

namespace lio
{
    TF2BroadCaster::TF2BroadCaster(std::string &base_frame_id, std::string &child_frame_id)
    {
        tf_.header.frame_id = base_frame_id;
        tf_.child_frame_id  = child_frame_id;

    }

    void TF2BroadCaster::SendTF(const Mat4f &pose)
    {
        SendTF(pose, ros::Time::now());
    }

    void TF2BroadCaster::SendTF(const Mat4f &pose, const double &time)
    {
        SendTF(pose, ros::Time(time));
    }

    void TF2BroadCaster::SendTF(const Mat4f &pose, const ros::Time &time)
    {
        tf_.header.stamp = time;

        Quatf qf_(pose.block<3,3>(0,0));

        tf_.transform.translation.x = pose(0,3);
        tf_.transform.translation.y = pose(1,3);
        tf_.transform.translation.z = pose(2,3);

        tf_.transform.rotation.x = qf_.x();
        tf_.transform.rotation.y = qf_.y();
        tf_.transform.rotation.z = qf_.z();
        tf_.transform.rotation.w = qf_.w();

        br_.sendTransform(tf_);
    }

    
} // namespace lio 