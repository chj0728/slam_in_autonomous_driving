/**
 * @file icp_node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <icp_2d.h>

int main(int argc, char **argv) 
{       
    ros::init(argc, argv, "icp_node");
    // ros::NodeHandle nh;

    Icp2d icp;
    ros::spin();

    return 0;
}