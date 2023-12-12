/**
 * @file imu_preintegraion.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <lio/imu_preintegration.h>

#include <iostream>

namespace lio
{
IMUPreintegration::IMUPreintegration(Options options)
{
    bg_ = options.init_bg_;
    ba_ = options.init_ba_;
    const float ng2 = options.noise_gyro_ * options.noise_gyro_;
    const float na2 = options.noise_acce_ * options.noise_acce_;
    noise_gyro_acce_.diagonal() << ng2, ng2, ng2, na2, na2, na2;

    ros::NodeHandle p_nh("~");
    ros::NodeHandle nh;

    p_nh.param("imu_sub_topic", imu_sub_topic_, std::string("imu/data"));            
    p_nh.param("imu_pub_topic", imu_pub_topic_, std::string("imu/odom"));

    // p_nh.param("publish_tf", publish_tf_, true);             
    // p_nh.param("publish_odom", publish_odom_, true);             

    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(imu_sub_topic_, 1, &IMUPreintegration::IMUCallBack, this);

    ROS_INFO_STREAM("\033[1;33m---->IMUPreintegration initialized.\033[0m");
}

void IMUPreintegration::IMUCallBack(const sensor_msgs::Imu::ConstPtr &imu_msg){
    if  (first_imu_){
        first_imu_ = false;
        last_imu_time_ = imu_msg->header.stamp.toSec();
        return;
    }

    double dt = imu_msg->header.stamp.toSec() - last_imu_time_;

    // 去掉零偏的测量
    Vec3d gyr(imu_msg->angular_velocity.x,
                imu_msg->angular_velocity.y,
                imu_msg->angular_velocity.z);
    gyr -= bg_;  // 陀螺

    Vec3d acc (imu_msg->linear_acceleration.x,
                imu_msg->linear_acceleration.y,
                imu_msg->linear_acceleration.z);
    acc -= ba_;  // 加计

    // dR先不更新，因为A, B阵还需要现在的dR

    // 运动方程雅可比矩阵系数，A,B阵，见(4.29)
    // 另外两项在后面
    Eigen::Matrix<double, 9, 9> A;
    A.setIdentity();
    Eigen::Matrix<double, 9, 6> B;
    B.setZero();

    Mat3d acc_hat = SO3::hat(acc);
    double dt2 = dt * dt;

// NOTE A, B左上角块与公式稍有不同
    A.block<3, 3>(3, 0) = -dR_.matrix() * dt * acc_hat;
    A.block<3, 3>(6, 0) = -0.5f * dR_.matrix() * acc_hat * dt2;
    A.block<3, 3>(6, 3) = dt * Mat3d::Identity();

    B.block<3, 3>(3, 3) = dR_.matrix() * dt;
    B.block<3, 3>(6, 3) = 0.5f * dR_.matrix() * dt2;

    // 更新各雅可比，见式(4.39)
    dP_dba_ = dP_dba_ + dV_dba_ * dt - 0.5f * dR_.matrix() * dt2;                      // (4.39d)
    dP_dbg_ = dP_dbg_ + dV_dbg_ * dt - 0.5f * dR_.matrix() * dt2 * acc_hat * dR_dbg_;  // (4.39e)
    dV_dba_ = dV_dba_ - dR_.matrix() * dt;                                             // (4.39b)
    dV_dbg_ = dV_dbg_ - dR_.matrix() * dt * acc_hat * dR_dbg_;                         // (4.39c)

    // 旋转部分
    Vec3d omega = gyr * dt;         // 转动量
    Mat3d rightJ = SO3::jr(omega);  // 右雅可比
    // Mat3d rightJ = SO3::rightJ(omega);  // 右雅可比
    SO3 deltaR = SO3::exp(omega);   // exp后
    dR_ = dR_ * deltaR;             // (4.9)

    A.block<3, 3>(0, 0) = deltaR.matrix().transpose();
    B.block<3, 3>(0, 0) = rightJ * dt;

    // 更新噪声项(4.31)
    cov_ = A * cov_ * A.transpose() + B * noise_gyro_acce_ * B.transpose();

    // 更新dR_dbg
    dR_dbg_ = deltaR.matrix().transpose() * dR_dbg_ - rightJ * dt;  // (4.39a)

    // 增量积分时间
    dt_ += dt;

    last_imu_time_ = imu_msg->header.stamp.toSec();

    std::cout<<"DeltaVelocity:\n"<<GetDeltaVelocity(bg_, ba_).transpose()<<std::endl;
    std::cout<<"DeltaPosition:\n"<<GetDeltaPosition(bg_, ba_).transpose()<<std::endl;
    std::cout<<"DeltaRotation:\n"<<GetDeltaRotation(bg_).matrix()<<std::endl;
    std::cout<<"增量积分时间dt_:"<<dt_<<std::endl;
}

// (4.32)
SO3 IMUPreintegration::GetDeltaRotation(const Vec3d &bg) { 
    return dR_ * SO3::exp(dR_dbg_ * (bg - bg_)); 
}

Vec3d IMUPreintegration::GetDeltaVelocity(const Vec3d &bg, const Vec3d &ba) {
    return dv_ + dV_dbg_ * (bg - bg_) + dV_dba_ * (ba - ba_);
}

Vec3d IMUPreintegration::GetDeltaPosition(const Vec3d &bg, const Vec3d &ba) {
    return dp_ + dP_dbg_ * (bg - bg_) + dP_dba_ * (ba - ba_);
}
} // namespace lio

using namespace lio;
int main (int argc, char** argv){
    ros::init(argc, argv, "imu_preintegration_node");

    IMUPreintegration::Options options;
    IMUPreintegration imu_preintegration(options);

    ros::spin();
    return (0);
}    







