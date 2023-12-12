/**
 * @file imu_node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <lio/sensors/imu_data.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>


using namespace lio;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::deque<IMUData> imu_data_buff;
    std::string imu_topic;
    nh_private.param<std::string>("imu_topic", imu_topic, "imu/data");

    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, imu_topic, 100);

    LOG(INFO)<<"IMU Topic Name-->"<<imu_sub_ptr->GetTopicName();


    ros::spin();
    // ros::Rate rate(100);
    // while (ros::ok())
    // {
    //     ros::spinOnce();

    //     imu_sub_ptr->ParseData(imu_data_buff);

    //     imu_data_buff.clear();

    //     rate.sleep();
    // }
    
    return 0;
}


