/**
 * @file cloud_publisher.hpp
 * @author your name (you@domain.com)
 * @brief 通过ros发布点云
 * @version 0.1
 * @date 2023-11-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_CLOUD_PUBLISHER_HPP_
#define LIO_CLOUD_PUBLISHER_HPP_

#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>

#include<common/common.hpp>
#include<pcl_conversions/pcl_conversions.h>

#include<lio/sensors/cloud_data.hpp>

namespace lio
{
class CloudPublisher 
{
    public:
        CloudPublisher(ros::NodeHandle& nh,std::string topic_name,std::string frame_id,size_t buff_size);
        CloudPublisher() = default;
        ~CloudPublisher() = default;

        void Publish(const CloudPtr& cloud_ptr_input, const double &time);
        void Publish(const CloudPtr& cloud_ptr_input);

        bool HasSubscribers();
    
    private:
        void PublishData(const CloudPtr& cloud_ptr_input, const ros::Time &time);

        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;
        size_t bufffer_size_;
};



} // namespace lio
#endif//LIO_CLOUD_PUBLISHER_HPP_