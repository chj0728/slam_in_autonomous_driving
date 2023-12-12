/**
 * @file occupancy_map.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "occupancy_map.h"

const int UNKNOW = -1;
#define FREE    0;
#define OCCUPY  100;

OccupancyMap::OccupancyMap()
{
    BuildModel();

    /********************************************/
    occupancy_grid_msg_.header.frame_id = "base_link";
    occupancy_grid_msg_.info.resolution = resolution_;

    // 地图图片像素的大小, width为地图的宽度是多少个像素
    occupancy_grid_msg_.info.width = 2 * model_size_ ;//+ 1;
    occupancy_grid_msg_.info.height = 2 * model_size_ ;//+ 1;

    // 地图左下角的点对应的物理坐标
    occupancy_grid_msg_.info.origin.position.x = -model_size_ * resolution_;
    occupancy_grid_msg_.info.origin.position.y = -model_size_ * resolution_;

    // 对数组进行初始化, 数组的大小为实际像素的个数
    occupancy_grid_msg_.data.resize(occupancy_grid_msg_.info.width * occupancy_grid_msg_.info.height);
    occupancy_grid_msg_.data.assign(occupancy_grid_msg_.data.size(), UNKNOW);
    /********************************************/


    // 读取参数
    laser_scan_subscriber_  = node_handle_.subscribe("scan_multi", 1,&OccupancyMap::ScanCallback, this);
    map_publisher_          = node_handle_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    map_publisher_metadata_ = node_handle_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> Occupacny Map started.\033[0m"); 

}

void OccupancyMap::BuildModel()
{   
    for (int x = -model_size_; x <= model_size_; x++) {
        for (int y = -model_size_; y <= model_size_; y++) {
            Model2DPoint pt;
            pt.dx_ = x;
            pt.dy_ = y;
            pt.range_ = sqrt(x * x + y * y) * resolution_;
            pt.angle_ = std::atan2(y, x);
            pt.angle_ = pt.angle_ > M_PI ? pt.angle_ - 2 * M_PI : pt.angle_;  // limit in 2pi
            model_.push_back(pt);
        }
    }
}

void OccupancyMap::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // // 1. 将激光雷达数据转换到世界坐标系下
    // std::vector<Vec2f> world_msg;
    // // LidarUtils::LidarToWorld(msg_msg, world_msg, pose_);
    // LidarUtils::LidarToWorld(msg_msg, world_msg, pose_);

    // // 2. 将激光雷达数据转换到栅格坐标系下
    // std::vector<Vec2i> grid_msg;
    // LidarUtils::WorldToGrid(world_msg, grid_msg, resolution_, model_size_);

    // // 3. 将激光雷达数据转换到栅格坐标系下
    // for (const auto& pt : grid_msg) {
    //     SetPoint(pt, true);
    // }

    // // 4. 发布占据栅格地图
    // PublishOccupancyGrid();

    // occupancy_grid_msg_.data.clear();
    for (size_t i = 0; i < msg->ranges.size(); ++i) 
    {
        if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max) {
            continue;
        }

        double real_angle = msg->angle_min + i * msg->angle_increment;
        double x = msg->ranges[i] * std::cos(real_angle);
        double y = msg->ranges[i] * std::sin(real_angle);
        x = ceil(x - occupancy_grid_msg_.info.origin.position.x)/resolution_;
        y = ceil(y - occupancy_grid_msg_.info.origin.position.y)/resolution_;

        occupancy_grid_msg_.data[MAP_IDX(occupancy_grid_msg_.info.width, x, y)] = OCCUPY;

        // endpoints.emplace(World2Image(frame->pose_ * Vec2d(x, y)));
    }

    map_publisher_.publish(occupancy_grid_msg_);
    map_publisher_metadata_.publish(occupancy_grid_msg_.info);
    ROS_INFO_STREAM("\033[1;32m---->publishing Map .\033[0m"); 
}

// void OccupancyMap::AddLidarFrame(std::shared_ptr<Frame> frame, GridMethod method) {
//     auto& scan = frame->scan_;
    
//     // 此处不能直接使用frame->pose_submap_，因为frame可能来自上一个地图
//     // 此时frame->pose_submap_还未更新，依旧是frame在上一个地图中的pose

        //frame->pose_可以理解为激光雷达在世界坐标系下的位姿,可以由odom和lidar的tf变换得到
        //pose_可以理解为子图在世界坐标系下的位姿
        //pose_in_submap可以理解为激光雷达在子图坐标系下的位姿
        // pose_ * pose_in_submap = frame->pose_

//     SE2 pose_in_submap = pose_.inverse() * frame->pose_;
//     float theta = pose_in_submap.so2().log();
//     has_outside_pts_ = false;

//     // 先计算末端点所在的网格
//     std::set<Vec2i, less_vec<2>> endpoints;

//     for (size_t i = 0; i < scan->ranges.size(); ++i) {
//         if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
//             continue;
//         }

//         double real_angle = scan->angle_min + i * scan->angle_increment;
//         double x = scan->ranges[i] * std::cos(real_angle);
//         double y = scan->ranges[i] * std::sin(real_angle);

//         endpoints.emplace(World2Image(frame->pose_ * Vec2d(x, y)));
//     }

//     if (method == GridMethod::MODEL_POINTS) {
//         // 遍历模板，生成白色点
//         std::for_each(std::execution::par_unseq, model_.begin(), model_.end(), [&](const Model2DPoint& pt) {
//             Vec2i pos_in_image = World2Image(frame->pose_.translation());
//             Vec2i pw = pos_in_image + Vec2i(pt.dx_, pt.dy_);  // submap下

//             if (pt.range_ < closest_th_) {
//                 // 小距离内认为无物体
//                 SetPoint(pw, false);
//                 return;
//             }

//             double angle = pt.angle_ - theta;  // 激光系下角度
//             double range = FindRangeInAngle(angle, scan);

//             if (range < scan->range_min || range > scan->range_max) {
//                 /// 某方向无测量值时，认为无效
//                 /// 但离机器比较近时，涂白
//                 if (pt.range_ < endpoint_close_th_) {
//                     SetPoint(pw, false);
//                 }
//                 return;
//             }

//             if (range > pt.range_ && endpoints.find(pw) == endpoints.end()) {
//                 /// 末端点与车体连线上的点，涂白
//                 SetPoint(pw, false);
//             }
//         });
//     } else {
//         Vec2i start = World2Image(frame->pose_.translation());
//         std::for_each(std::execution::par_unseq, endpoints.begin(), endpoints.end(),
//                       [this, &start](const auto& pt) { BresenhamFilling(start, pt); });
//     }

//     /// 末端点涂黑
//     std::for_each(endpoints.begin(), endpoints.end(), [this](const auto& pt) { SetPoint(pt, true); });
// }