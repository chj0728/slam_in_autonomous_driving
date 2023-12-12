/**
 * @file imu_Pre.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <imu_pre.h>

// #include <utility_lio.h>

// #include <static_imu_init.h>

// #include <math_utils.h>


IMU_Pre::IMU_Pre(/* args */)
{
    // IMU static init params
    static_imu_init_options_.init_time_seconds_ =               nh.param<double>("init_time_seconds", 10.0);
    static_imu_init_options_.init_imu_queue_max_size_ =         nh.param<int>("init_imu_queue_max_size", 200);
    static_imu_init_options_.static_odom_pulse_ =               nh.param<int>("static_odom_pulse", 10);
    static_imu_init_options_.use_speed_for_static_checking_ =   nh.param<bool>("use_speed_for_static_checking", false);
    static_imu_init_options_.max_static_gyro_var =              nh.param<double>("max_static_gyro_var", 0.5);
    static_imu_init_options_.max_static_acce_var =              nh.param<double>("max_static_acce_var", 0.05);
    static_imu_init_options_.gravity_norm_ =                    nh.param<double>("gravity_norm", 9.81);

    static_imu_init_ = StaticIMUInit(static_imu_init_options_);

    // IMU pre params
    bg_ = imu_pre_options_.init_bg_;
    ba_ = imu_pre_options_.init_ba_;
    const float ng2 = imu_pre_options_.noise_gyro_ * imu_pre_options_.noise_gyro_;
    const float na2 = imu_pre_options_.noise_acce_ * imu_pre_options_.noise_acce_;
    noise_gyro_acce_.diagonal() << ng2, ng2, ng2, na2, na2, na2;

    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(imu_topic_, 1, &IMU_Pre::IMUCallBack, this,ros::TransportHints().tcpNoDelay());
}

IMU_Pre::~IMU_Pre()
{
    ROS_INFO_STREAM("\033[1;33m---->IMU_Pre destroyed.\033[0m");
}


void IMU_Pre::IMUCallBack(const sensor_msgs::Imu::ConstPtr &imu_msg)
{

    IMU imu_(imu_msg);

    // imu init   
    if (use_imu_static_init_ && !static_imu_init_.InitSuccess()) 
    {
        ROS_INFO_STREAM_THROTTLE(static_imu_init_options_.init_time_seconds_+1,"\033[1;33m---->IMU is initializing.\033[0m");
        ROS_INFO_STREAM_THROTTLE(static_imu_init_options_.init_time_seconds_+1,"\033[1;33m---->Please wait for a while.\033[0m");
        static_imu_init_.AddIMU(imu_);

        // 如果初始化成功
        // 更新预积分器的bg, ba
        if (use_imu_static_init_ && static_imu_init_.InitSuccess()) 
        {
            bg_ = static_imu_init_.GetInitBg();
            ba_ = static_imu_init_.GetInitBa();
        }  
        return;
    }

    // imu first
    if  (first_imu_){
        first_imu_ = false;
        last_imu_time_ = imu_msg->header.stamp.toSec();
        return;
    }

    double dt = imu_msg->header.stamp.toSec() - last_imu_time_;

    // 去掉零偏的测量
    Vec3d gyr = imu_.gyro_ - bg_;  // 陀螺
    Vec3d acc = imu_.acce_ - ba_;  // 加计

    // 更新dv, dp, 见(4.13), (4.16)
    dp_ = dp_ + dv_ * dt + 0.5f * dR_.matrix() * acc * dt * dt;
    dv_ = dv_ + dR_ * acc * dt;

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
    SO3 deltaR = SO3::exp(omega);   // exp后
    dR_ = dR_ * deltaR;             // (4.9)

    A.block<3, 3>(0, 0) = deltaR.matrix().transpose();
    B.block<3, 3>(0, 0) = rightJ * dt;

    // 更新噪声项
    cov_ = A * cov_ * A.transpose() + B * noise_gyro_acce_ * B.transpose();

    // 更新dR_dbg
    dR_dbg_ = deltaR.matrix().transpose() * dR_dbg_ - rightJ * dt;  // (4.39a)(4.35)

    // 增量积分时间
    dt_ += dt;

    last_imu_time_ = imu_msg->header.stamp.toSec();

    // std::cout<<"增量积分时间dt_:"<<dt_<<std::endl;  
}

// 零偏更新(4.32)
SO3 IMU_Pre::GetDeltaRotation(const Vec3d &bg) { return dR_ * SO3::exp(dR_dbg_ * (bg - bg_)); }

Vec3d IMU_Pre::GetDeltaVelocity(const Vec3d &bg, const Vec3d &ba) {
    return dv_ + dV_dbg_ * (bg - bg_) + dV_dba_ * (ba - ba_);
}

Vec3d IMU_Pre::GetDeltaPosition(const Vec3d &bg, const Vec3d &ba) {
    return dp_ + dP_dbg_ * (bg - bg_) + dP_dba_ * (ba - ba_);
}

NavStated IMU_Pre::Predict(const NavStated &start, const Vec3d &grav) const {
    SO3 Rj = start.R_ * dR_;
    Vec3d vj = start.R_ * dv_ + start.v_ + grav * dt_;
    Vec3d pj = start.R_ * dp_ + start.p_ + start.v_ * dt_ + 0.5f * grav * dt_ * dt_;

    auto state = NavStated(start.timestamp_ + dt_, Rj, pj, vj);
    state.bg_ = bg_;
    state.ba_ = ba_;
    return state;
}