/**
 * @file odometry_publisher.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_ODOMETRY_PUBLISHER_HPP_
#define LIO_ODOMETRY_PUBLISHER_HPP_

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>

#include<common/common.hpp>

namespace lio
{
/**
 * @brief  通过ros发布里程计
 * @param[in] nh ros句柄
 * @param[in] topic_name 话题名
 * @param[in] base_frame_id 坐标系名
 * @param[in] child_frame_id 子坐标系名
 * @param[in] buff_size 缓存大小
 * 
 */
class OdometryPublisher
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
        OdometryPublisher(ros::NodeHandle& nh,
                        std::string &topic_name,
                        std::string &base_frame_id,
                        std::string &child_frame_id,
                        const size_t &buff_size);
        OdometryPublisher() = default;
        ~OdometryPublisher() = default;

        void Publish(const SE3f& pose, const double &time);
        void Publish(const SE3f& pose);
        void Publish(const Eigen::Matrix4f& transform_matrix, const double &time);
        void Publish(const Eigen::Matrix4f& transform_matrix);

        bool HasSubscribers();
    
    private:
        void PublishData(const Eigen::Matrix4f& transform_matrix,const ros::Time& time);

        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        // std::string base_frame_id_;
        // std::string child_frame_id_;
        nav_msgs::Odometry odometry_;
};

} // namespace lio
#endif//LIO_ODOMETRY_PUBLISHER_HPP_