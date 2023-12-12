/**
 * @file cloud_subscriber.hpp
 * @author your name (you@domain.com)
 * @brief 点云的订阅类
 * class CloudConver
 * class CloudSubscriber
 * @version 0.1
 * @date 2023-11-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef LIO_CLOUD_SUBSCRIBER_HPP_
#define LIO_CLOUD_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <lio/sensors/cloud_data.hpp>
// #include <common/point_types.hpp>

#include <common/point_cloud_utils.hpp>

namespace lio
{

class CloudConver
{
    public:
        CloudConver() = default;
        ~CloudConver() = default;

        bool Conver(const CloudData& input, CloudData& output);
        bool Conver(const CloudDataRslidar& input, CloudDataFull& output);
        // bool Conver(const CloudData& input, CloudDataVelodyne& output);

};

class   CloudSubscriber
{
    public:
        CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        CloudSubscriber() = default;
        ~CloudSubscriber() = default;

        void ParseData(std::deque<CloudData>& deque_cloud_data);
        void ParseData(std::deque<CloudDataFull>& deque_cloud_data_full);

    private:

        void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<CloudData> new_cloud_data_;
        std::deque<CloudDataFull> new_cloud_data_full_;
        // std::deque<CloudDataVelodyne> new_cloud_data_velodyne_;
        // std::deque<CloudDataOuster> new_cloud_data_ouster_;
        // std::deque<CloudDataRslidar> new_cloud_data_rslidar_;

        std::mutex buff_mutex_;


};

}
#endif//LIO_CLOUD_SUBSCRIBER_HPP_
