/**
 * @file imu.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef MAPPING_IMU_H
#define MAPPING_IMU_H

#include "eigen_types.h"

// ROS
#include <sensor_msgs/Imu.h>

// std
#include <memory>

// namespace sad {

/// IMU 读数
struct IMU {

    IMU() = default;

    IMU(double t, const Vec3d& gyro, const Vec3d& acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

    /**
     * @brief Construct a new IMU object
     * 
     * @param sensor_msgs::ImuConstPtr imu_msg 
     */
    IMU(const sensor_msgs::ImuConstPtr &imu_msg) : timestamp_(imu_msg->header.stamp.toSec()) 
    {
        gyro_ << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
        acce_ << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
    }

    double timestamp_ = 0.0;
    Vec3d gyro_ = Vec3d::Zero();
    Vec3d acce_ = Vec3d::Zero();
};
// }  // namespace sad

using IMUPtr = std::shared_ptr<IMU>;

#endif  // MAPPING_IMU_H
