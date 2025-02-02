/**
 * @file odometry_piblisher.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "odometry_publisher.hpp"

namespace lio
{

OdometryPublisher::OdometryPublisher(ros::NodeHandle &nh,
                    std::string &topic_name,
                    std::string &base_frame_id,
                    std::string &child_frame_id,
                    const size_t &buff_size)
:nh_(nh) 
{
    publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
    odometry_.header.frame_id = base_frame_id;
    odometry_.child_frame_id  = child_frame_id;
}

void OdometryPublisher::Publish(const SE3f& pose)
{
    ros::Time time = ros::Time::now();
    PublishData(pose.matrix(), time);
}

void OdometryPublisher::Publish(const SE3f& pose, const double &time)

{   ros::Time ros_time(time);
    PublishData(pose.matrix(), ros_time);
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix, const double &time) 
{
    ros::Time ros_time(time);
    PublishData(transform_matrix, ros_time);
    // PublishData(transform_matrix, ros::Time(time));
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix) {
    ros::Time time = ros::Time::now();
    PublishData(transform_matrix, time);
}

void OdometryPublisher::PublishData(const Eigen::Matrix4f& transform_matrix,const ros::Time &time) 
{
    odometry_.header.stamp = time;

    //set the position
    odometry_.pose.pose.position.x = transform_matrix(0,3);
    odometry_.pose.pose.position.y = transform_matrix(1,3);
    odometry_.pose.pose.position.z = transform_matrix(2,3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    publisher_.publish(odometry_);
}

bool OdometryPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}

}