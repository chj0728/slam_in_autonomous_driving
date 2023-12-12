/**
 * @file imu_pre_node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>
#include "imu_pre.h"

class LIO{
public:
    LIO();
    ~LIO(){
    }
    std::shared_ptr<IMU_Pre> imu_pre_ptr_ = nullptr;

// private:

};

LIO::LIO()
{
    imu_pre_ptr_ = std::make_shared<IMU_Pre>();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_pre_node");
    // ros::NodeHandle nh("~");

    // IMU_Pre imu_pre;
    LIO lio;
    ROS_INFO_STREAM(lio.imu_pre_ptr_->imu_topic_);
    ros::spin();

    return 0;
}