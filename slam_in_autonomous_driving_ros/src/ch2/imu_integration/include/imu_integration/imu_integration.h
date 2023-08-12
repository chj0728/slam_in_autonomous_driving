/**
 * @file imu_integration.html
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef IMU_INTEGRATION_H_
#define IMU_INTEGRATION_H_

#include <imu_integration/eigen_types.h>

#include <string>
#include <iomanip>
#include <fstream>
#include <iosfwd>
#include <functional>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
    // #include <tf2/LinearMath/Quaternion.h>

/**
 * @brief Struct IMU 数据
 * @param timestamp_    double-时间辍  
 * @param gyro_         Vec3d-角速度向量  
 * @param acce_         Vec3d-加速度向量
 */
struct IMU {

    IMU() = default;

    IMU(double t, const Vec3d& gyro, const Vec3d& acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

    IMU(const sensor_msgs::Imu::ConstPtr &imu_msg){

        timestamp_ = imu_msg->header.stamp.toSec();

        gyro_[0] =  imu_msg->angular_velocity.x;
        gyro_[1] =  imu_msg->angular_velocity.y;
        gyro_[2] =  imu_msg->angular_velocity.z;
    
        acce_[0] =  imu_msg->linear_acceleration.x;
        acce_[1] =  imu_msg->linear_acceleration.y;
        acce_[2] =  imu_msg->linear_acceleration.z;

    }

    double timestamp_ = 0.0;
    Vec3d gyro_ = Vec3d::Zero();
    Vec3d acce_ = Vec3d::Zero();
};


class IMUIntegration{
    public:
        /**
         * @brief Construct a new IMUIntegration object
         * 
         * @param gravity
         * @param init_bg
         * @param init_ba
         * @param file_path 保存imu_state文件路径
         */
        IMUIntegration(const Vec3d& gravity, const Vec3d& init_bg, const Vec3d& init_ba, const std::string &file_path);

        virtual ~IMUIntegration(){

        };
    
    private:

        /**
         * @brief 回调函数
         * 
         * @param imu_msg sensor_msgs::Imu
         */
        void IMUCallBack(const sensor_msgs::Imu::ConstPtr &imu_msg);

        
        /**
         * @brief 发布tf
         * 
         * @param R 
         * @param v 
         * @param p 
         */
        void Publish_Tf(const sensor_msgs::Imu::ConstPtr &imu_msg, const Vec3d &v,  const Vec3d &p);

        /**
         * @brief 发布里程计
         * 
         * @param R 
         * @param v 
         * @param p 
         */
        void Publish_Odom(const sensor_msgs::Imu::ConstPtr &imu_msg, const Vec3d &v, const Vec3d &p);
        
        bool publish_tf_;
        bool publish_odom_;
        bool flag_;

        ros::Subscriber imu_sub_;
        ros::Publisher  imu_pub_;

        std::string imu_sub_topic_;
        std::string imu_pub_topic_;

        // 累计量
        SO3 R_ ;
        Vec3d v_ = Vec3d::Zero();
        Vec3d p_ = Vec3d::Zero();

        double timestamp_ = 0.0;

        // 零偏，由外部设定
        Vec3d bg_ = Vec3d::Zero();
        Vec3d ba_ = Vec3d::Zero();
        Vec3d gravity_ = Vec3d(0, 0, -9.8);  // 重力

        std::function<void(std::ofstream& fout, double timestamp, const Sophus::SO3d& R, const Vec3d& v,const Vec3d& p)> save;
        std::ofstream fout;
};
#endif