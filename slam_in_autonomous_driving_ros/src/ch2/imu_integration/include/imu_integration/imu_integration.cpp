/**
 * @file imu_integration.cpp 
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <imu_integration/imu_integration.h>

#include <imu_integration/eigen_types.h>

class IMUIntegration;

IMUIntegration::IMUIntegration(const Vec3d& gravity, const Vec3d& init_bg, const Vec3d& init_ba, const std::string &file_path):
gravity_(gravity),bg_(init_bg), ba_(init_ba),fout(file_path),flag_(true)
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

void IMUIntegration::IMUCallBack(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    double w = imu_msg->orientation.w;
    double x = imu_msg->orientation.x;
    double y = imu_msg->orientation.y;
    double z = imu_msg->orientation.z;

    Quatd q(w,x,y,z);
    // q.normalize();
    R_.setQuaternion(q);

    if (flag_)
    {
        timestamp_  = imu_msg->header.stamp.toSec();
        flag_ = false;
    }
   

    else 
    {
        double dt = imu_msg->header.stamp.toSec() - timestamp_;
        /**
        * @brief 将imu_msg 转成 struct IMU
        * 
        * @return struct IMU 
        */
        IMU imu(imu_msg);

        p_ = p_ + v_ * dt + 0.5 * gravity_ * dt * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt;
        v_ = v_ + R_ * (imu.acce_ - ba_) * dt + gravity_ * dt;
        // R_ = R_ * Sophus::SO3d::exp((imu.gyro_ - bg_) * dt);

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


void IMUIntegration::Publish_Tf(const sensor_msgs::Imu::ConstPtr &imu_msg, const Vec3d &v,  const Vec3d &p){
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

void IMUIntegration::Publish_Odom(const sensor_msgs::Imu::ConstPtr &imu_msg, const Vec3d &v, const Vec3d &p){
    
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