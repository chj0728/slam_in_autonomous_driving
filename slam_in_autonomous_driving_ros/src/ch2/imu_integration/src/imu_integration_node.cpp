/**
 * @file imu_integration_node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <imu_integration/imu_integration.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "imu_integration_node");

    Vec3d gravity(0, 0, -9.841);  // 重力方向
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);

    std::string file_path="/home/caohaojie/SLAM/BOOKS/slam_in_autonomous_driving_ros/src/data/state.txt";

    IMUIntegration imu_integration(gravity, init_bg, init_ba, file_path);

    ros::spin();

    return (0);

}



