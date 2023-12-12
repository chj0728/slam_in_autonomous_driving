/**
 * @file imu.hpp
 * @author chj (you@domain.com)
 * @brief  class IMUData, class IMUSubscriber
 *包含imu自定义数据结构和imu数据订阅类
 * @version 0.1
 * @date 2023-11-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef  LIO_IMU_HPP
#define  LIO_IMU_HPP


#include <common/eigen_types.hpp>

#include <deque>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace lio{


/**
 * @brief imu自定义的数据结构
 * @details imu数据结构，包含时间戳，角速度，线加速度
 * 
 */
class IMUData
{
private:
    /* data */
public:

    IMUData() = default;
    
    /**
     * @brief Construct a new IMUData object from sensor_msgs::ImuConstPtr
     * 
     * @param imu_msg 
     * @details 从ros的imu消息中构造imu数据结构
     */
    // IMUData(const sensor_msgs::ImuConstPtr &imu_msg):time(imu_msg->header.stamp.toSec())
    // {
    //     linear_acceleration.x = imu_msg->linear_acceleration.x;
    //     linear_acceleration.y = imu_msg->linear_acceleration.y;
    //     linear_acceleration.z = imu_msg->linear_acceleration.z;

    //     angular_velocity.x = imu_msg->angular_velocity.x;
    //     angular_velocity.y = imu_msg->angular_velocity.y;
    //     angular_velocity.z = imu_msg->angular_velocity.z;

    //     orientation.x = imu_msg->orientation.x;
    //     orientation.y = imu_msg->orientation.y;
    //     orientation.z = imu_msg->orientation.z;
    //     orientation.w = imu_msg->orientation.w;
    // }

    /**
     * @brief Construct a new IMUData object from sensor_msgs::ImuConstPtr
     * 
     * @param imu_msg 
     * @details 从ros的imu消息中构造imu数据结构
     */
    IMUData(const sensor_msgs::ImuConstPtr &imu_msg)
    {
        time = imu_msg->header.stamp.toSec();
        gyro_ << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
        acce_ << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
        qd_ = Quatd(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
    }

    ~IMUData()
    {
    }

    // struct LinearAcceleration 
    // {
    //   double x = 0.0;
    //   double y = 0.0;
    //   double z = 0.0;
    // };

    // struct AngularVelocity 
    // {
    //   double x = 0.0;
    //   double y = 0.0;
    //   double z = 0.0;
    // };

    // class Orientation 
    // {
    //   public:
    //     double x = 0.0;
    //     double y = 0.0;
    //     double z = 0.0;
    //     double w = 0.0;
    //   public:
    //     void Normlize() {
    //       double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
    //       x /= norm;
    //       y /= norm;
    //       z /= norm;
    //       w /= norm;
    //     }
    // };

    double time = 0.0;

    // LinearAcceleration  linear_acceleration;//线加速度
    // AngularVelocity     angular_velocity;   //角速度
    // Orientation         orientation;        //姿态

    Vec3d gyro_ = Vec3d::Zero();   
    Vec3d acce_ = Vec3d::Zero();
    Quatd qd_ = Quatd::Identity();

    //根据四元数计算旋转矩阵
    Mat3d GetRotationMatrix();

    //根据四元数计算欧拉角
    Vec3d GetEulerAngle();

    //同步imu数据
    static bool SyncData(std::deque<IMUData> &UnsyncedData, 
                        std::deque<IMUData> &SyncedData, 
                        double &sync_time);
};


/***
 * @brief imu数据订阅类
 * @details imu数据订阅类，订阅imu数据，将imu数据存入队列
 * 
*/
class IMUSubscriber
{
    private:

        ros::NodeHandle nh_;
        ros::Subscriber imu_sub_;

        std::deque<IMUData> new_imu_data_;
        std::string imu_topic_;

        int imu_data_buff_size_ = 2000;
        double imu_time_ = 0.0;
        bool first_imu_ = true;

        std::mutex buff_mutex_;

    public:
        IMUSubscriber() = default;

        IMUSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size):
        nh_(nh), imu_topic_(topic_name), imu_data_buff_size_(buff_size)
        {
            imu_sub_ = nh_.subscribe(imu_topic_, imu_data_buff_size_, &IMUSubscriber::Callback, this);
        }

        
        void ParseData(std::deque<IMUData> &imu_data_deque);
        void Callback(const sensor_msgs::ImuConstPtr &imu_msg);

        std::string GetTopicName()
        {
            return imu_topic_;
        }

        // void ClearBuf();
        // double GetTime();
        // bool HasData();
        // IMUData GetData();
        // IMUData GetDataFront();
        // IMUData GetDataBack();
        // ~IMUSubscriber();

};

}//end namespace lio

#endif