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
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


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
        void Publish_Tf(const sensor_msgs::Imu::ConstPtr &imu_msg, Vec3d &v,  Vec3d &p);

        /**
         * @brief 发布里程计
         * 
         * @param R 
         * @param v 
         * @param p 
         */
        void Publish_Odom(const sensor_msgs::Imu::ConstPtr &imu_msg, Vec3d &v,  Vec3d &p);
        
        bool publish_tf_;
        bool publish_odom_;

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

IMUIntegration::IMUIntegration(const Vec3d& gravity, const Vec3d& init_bg, const Vec3d& init_ba, const std::string &file_path):
gravity_(gravity),bg_(init_bg), ba_(init_ba),fout(file_path)
{
            
    ros::NodeHandle p_nh("~");
    ros::NodeHandle nh;

    auto save_result = [](std::ofstream& fout, double timestamp, const Sophus::SO3d& R, const Vec3d& v,
                    const Vec3d& p) 
    {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) 
        { 
            fout << v[0] << " " << v[1] << " " << v[2] << " "; 
        };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) 
        {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(fout, p);
        save_quat(fout, R.unit_quaternion());
        save_vec3(fout, v);
        fout << std::endl;
    };
    
    save = std::move(save_result);

    p_nh.param("imu_sub_topic", imu_sub_topic_, std::string("imu/data"));            
    p_nh.param("imu_pub_topic", imu_pub_topic_, std::string("imu/odom"));

    p_nh.param("publish_tf", publish_tf_, true);             
    p_nh.param("publish_odom", publish_odom_, true);             

    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(imu_sub_topic_, 1, &IMUIntegration::IMUCallBack, this);
    imu_pub_ = nh.advertise<nav_msgs::Odometry>(imu_pub_topic_,10);

}


void IMUIntegration::IMUCallBack(const sensor_msgs::Imu::ConstPtr &imu_msg){
    
    double dt = imu_msg->header.stamp.toSec() - timestamp_;
    // 假设IMU时间间隔在0至0.1以内
    if (dt > 0 && dt < 0.1) {
        
        /**
        * @brief 将imu_msg 转成 struct IMU
        * 
        * @return struct IMU 
        */
        IMU imu(imu_msg);

        p_ = p_ + v_ * dt + 0.5 * gravity_ * dt * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt;
        v_ = v_ + R_ * (imu.acce_ - ba_) * dt + gravity_ * dt;
        R_ = R_ * Sophus::SO3d::exp((imu.gyro_ - bg_) * dt);

        // std::cout<< "P:" << p_ <<std::endl;
        // std::cout<< "V:" << v_ <<std::endl;
        // std::cout<< R_ <<std::endl;
        save(fout, timestamp_, R_, v_, p_);


        if(publish_tf_){
            Publish_Tf(imu_msg, v_, p_);
        }


        if(publish_odom_){
            Publish_Odom(imu_msg, v_, p_);
        }

    }
        // 更新时间
    timestamp_ = imu_msg->header.stamp.toSec();

}


void IMUIntegration::Publish_Tf(const sensor_msgs::Imu::ConstPtr &imu_msg, Vec3d &v,  Vec3d &p){
    static tf2_ros::TransformBroadcaster tb_;

    geometry_msgs::TransformStamped tfs_;

    tfs_.header.frame_id = "odom";
    tfs_.header.stamp = ros::Time::now();
    tfs_.child_frame_id = "imu_link";

    tfs_.transform.translation.x = p[0];
    tfs_.transform.translation.y = p[1];
    tfs_.transform.translation.z = p[2];

    tfs_.transform.rotation.x = imu_msg->orientation.x;
    tfs_.transform.rotation.y = imu_msg->orientation.y;
    tfs_.transform.rotation.z = imu_msg->orientation.z;
    tfs_.transform.rotation.w = imu_msg->orientation.w;

    tb_.sendTransform(tfs_);
}

void IMUIntegration::Publish_Odom(const sensor_msgs::Imu::ConstPtr &imu_msg, Vec3d &v,  Vec3d &p){
    
    nav_msgs::Odometry odom_;

    odom_.header.stamp = ros::Time::now();
    odom_.header.frame_id = "odom";
    odom_.child_frame_id  = "imu_link";

    odom_.pose.pose.position.x = p[0];
    odom_.pose.pose.position.y = p[1];
    odom_.pose.pose.position.z = p[2];

    odom_.pose.pose.orientation.x = imu_msg->orientation.x;
    odom_.pose.pose.orientation.y = imu_msg->orientation.y;
    odom_.pose.pose.orientation.z = imu_msg->orientation.z;
    odom_.pose.pose.orientation.w = imu_msg->orientation.w;

    odom_.twist.twist.linear.x = v[0];
    odom_.twist.twist.linear.y = v[1];
    odom_.twist.twist.linear.z = v[2];

    odom_.twist.twist.angular.x = imu_msg->angular_velocity.x;
    odom_.twist.twist.angular.y = imu_msg->angular_velocity.y;
    odom_.twist.twist.angular.z = imu_msg->angular_velocity.z;

    imu_pub_.publish(odom_);
}


#endif