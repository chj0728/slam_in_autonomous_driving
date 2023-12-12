/**
 * @file cloud_subscriber.cpp
 * @author your name (you@domain.com)
 * @brief cloud_subscriber.hpp的实现文件
 * @version 0.1
 * @date 2023-11-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "cloud_subscriber.hpp"

#include <glog/logging.h>

namespace lio
{
//class CloudConver
//***************************************************

//转换点云：CloudData -> CloudData
//将点云中的无效点和小于4的点去除
bool CloudConver::Conver(const CloudData& input, CloudData& output)
{
    output.time = input.time;
    for (size_t i{0}; i < input.cloud_ptr->points.size(); ++i)
    {
       auto &src =  input.cloud_ptr->points.at(i);  

       if (!pcl_isfinite(src.x) || !pcl_isfinite(src.y) || !pcl_isfinite(src.z))
       {
           continue;
       }

       if(PointDistance(src) < 4.0)
       {
           continue;
       }
      
        PointType p = src;
        p.x = src.x;
        p.y = src.y;
        p.z = src.z;
        p.intensity = src.intensity;
        output.cloud_ptr->points.push_back(p);
    }   

    LOG(INFO) << input.cloud_ptr->points.size();
    LOG(INFO) << output.cloud_ptr->points.size();

    return true;
}

//转换点云：CloudDataRslidar -> CloudDataFull
bool CloudConver::Conver(const CloudDataRslidar& input, CloudDataFull& output)
{
    output.time = input.time;

    double timespan = input.cloud_ptr->points.back().timestamp - input.cloud_ptr->points.front().timestamp;

    for (size_t i{0}; i < input.cloud_ptr->points.size(); ++i)
    {
        auto &src =  input.cloud_ptr->points.at(i);  

        

        if (!pcl_isfinite(src.x) || !pcl_isfinite(src.y) || !pcl_isfinite(src.z))
        {
            continue;
        }

        if(PointDistance(src) < 4.0)
        {
            continue;
        }
      
        FullPointType p;
        p.x = src.x;
        p.y = src.y;
        p.z = src.z;
        p.intensity = src.intensity;
        p.ring = (uint8_t)src.ring;
        p.time_span = timespan;
        p.time_intervel = (src.timestamp - input.cloud_ptr->points.front().timestamp)/timespan;
        output.cloud_ptr->points.emplace_back(p);
    }
    LOG(INFO) << input.cloud_ptr->points.size();
    LOG(INFO) << output.cloud_ptr->points.size();

    return true;
}

//class CloudSubscriber
//***************************************************
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this, ros::TransportHints().tcpNoDelay());

}

//ros的pointcloud2消息类型转换为pcl的点云类型
//保存在deque<CloudData> new_cloud_data_容器中
void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    buff_mutex_.lock();
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

    //保存点云pcd文件
    // SaveCloud("/home/caohaojie/SLAM/BOOKS/slam_in_autonomous_driving_ros/src/lio/data/pcd/"+std::to_string(cloud_data.time)+".pcd", cloud_data.cloud_ptr);

    new_cloud_data_.push_back(cloud_data);
    // LOG(INFO) <<"Recevied Cloud Data: ";
    buff_mutex_.unlock();
}

//处理std::deque<CloudData> new_cloud_data_容器中的数据
//将数据保存在传入的deque_cloud_data
void CloudSubscriber::ParseData(std::deque<CloudData>& deque_cloud_data)
{
    buff_mutex_.lock();
    if (new_cloud_data_.size() > 0)
    {
        deque_cloud_data.insert(deque_cloud_data.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
    buff_mutex_.unlock();

}

//处理std::deque<CloudDataFull> new_cloud_data_full_容器中的数据
//将数据保存在传入的deque_cloud_data_full
void CloudSubscriber::ParseData(std::deque<CloudDataFull>& deque_cloud_data_full)
{
    buff_mutex_.lock();
    if (new_cloud_data_full_.size() > 0)
    {
        deque_cloud_data_full.insert(deque_cloud_data_full.end(), new_cloud_data_full_.begin(), new_cloud_data_full_.end());
        new_cloud_data_full_.clear();
    }
    buff_mutex_.unlock();

}

} // namespace lio
