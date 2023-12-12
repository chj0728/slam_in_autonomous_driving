/**
 * @file occup_node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

# include "occupancy_map.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "occup_node");
    ros::NodeHandle nh;

    OccupancyMap occ_map;

    ros::spin();

    return 0;
}