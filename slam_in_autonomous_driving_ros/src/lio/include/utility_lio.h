/**
 * @file utility.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_UTILITY_H
#define LIO_UTILITY_H

#include <ros/ros.h>

#include "eigen_types.h"
#include "imu.h"
#include "nav_state.h"

// ros msg
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

// tf   
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// #include <tf_conversions/tf_eigen.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include  <pcl/filters/voxel_grid.h>

// std
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <functional>

using namespace std;

class ParamServer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ros::NodeHandle nh;

    string imu_topic_;
    string odom_topic_;    

    // imu_pre_params;
    // vector<double> init_bg_vec_;
    // vector<double> init_ba_vec_;
    // Vec3d init_bg_ = Vec3d::Zero();     // 初始零偏
    // Vec3d init_ba_ = Vec3d::Zero();     // 初始零偏
    // double noise_gyro_ = 1e-2;          // 陀螺噪声，标准差
    // double noise_acce_ = 1e-1;          // 加计噪声，标准差    

    bool use_imu_static_init_;           // 是否使用imu初始化


    ParamServer()
    {
        // ros::NodeHandle nh("~");
        // topics
        nh.param<string>("imu_topic", imu_topic_, "imu/data");
        nh.param<string>("odom_topic", odom_topic_, "odom");

        // imu
        // nh.param<vector<double>>("init_bg", init_bg_vec_, vector<double>());
        // nh.param<vector<double>>("init_ba", init_ba_vec_, vector<double>());
        // if (init_bg_vec_.size() == 3)
        // {
        //     // init_bg_ << init_bg_vec_[0], init_bg_vec_[1], init_bg_vec_[2];
        //     init_bg_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(init_bg_vec_.data(), 3, 1);
        // }
        // if (init_ba_vec_.size() == 3)
        // {
        //     init_ba_ << init_ba_vec_[0], init_ba_vec_[1], init_ba_vec_[2];
        // }
        // nh.param<double>("noise_gyro", noise_gyro_, 1e-2);
        // nh.param<double>("noise_acce", noise_acce_, 1e-1);
        nh.param<bool>("use_imu_static_init", use_imu_static_init_, true);

    }
};

#endif