/**
 * @file cloud_publisher.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "cloud_publisher.hpp"

namespace lio
{

CloudPublisher::CloudPublisher(ros::NodeHandle& nh,std::string topic_name,std::string frame_id,size_t buff_size):nh_(nh),frame_id_(frame_id),bufffer_size_(buff_size)
{
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, bufffer_size_);
}


void CloudPublisher::Publish(const CloudPtr& cloud_ptr_input, const double &time)
{
    PublishData(cloud_ptr_input, ros::Time(time));
}

void CloudPublisher::Publish(const CloudPtr& cloud_ptr_input)
{
    PublishData(cloud_ptr_input, ros::Time::now());
}


void CloudPublisher::PublishData(const CloudPtr& cloud_ptr_input, const ros::Time &time)
{
    sensor_msgs::PointCloud2Ptr cloud_msg_ptr(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_msg_ptr);

    cloud_msg_ptr->header.stamp = time;
    cloud_msg_ptr->header.frame_id = frame_id_;

    publisher_.publish(*cloud_msg_ptr);
}

bool CloudPublisher::HasSubscribers()
{
    return publisher_.getNumSubscribers() != 0;
}

}// namespace lio