/**
 * @file occupancy_map.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_OCCUPANCY_MAP_H
#define LIO_OCCUPANCY_MAP_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "eigen_types.h"
// #include "lidar_utils.h"

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class OccupancyMap {
   public:
    /// 栅格模板，预先计算
    struct Model2DPoint {
        int dx_ = 0;
        int dy_ = 0;
        double angle_ = 0;  // in rad
        float range_ = 0;   // in meters
    };

    OccupancyMap();

    /// 激光雷达回调函数
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /// 往这个占据栅格地图中增加一个frame
    // void AddLidarFrame(std::shared_ptr<Frame> frame, GridMethod method = GridMethod::BRESENHAM);
    // void AddLidarFrame(std::shared_ptr<Frame> frame, GridMethod method = GridMethod::BRESENHAM);

    /// 获取原始的占据栅格地图
    // cv::Mat GetOccupancyGrid() const { return occupancy_grid_; }

    /// 获取黑白灰形式的占据栅格，作可视化使用
    // cv::Mat GetOccupancyGridBlackWhite() const;

    /// 设置中心点
    void SetPose(const SE2& pose) { pose_ = pose; }

    bool HasOutsidePoints() const { return has_outside_pts_; }

    /// 获取分辨率
    double Resolution() const { return resolution_; }

    /// 在某个点填入占据或者非占据信息
    // void SetPoint(const Vec2i& pt, bool occupy);

    /// 生成填充模板
    void BuildModel();

   private:

    /// 从世界坐标系转到图像坐标系
    // template <class T>
    // inline Vec2i World2Image(const Eigen::Matrix<T, 2, 1>& pt) {
    //     Vec2d pt_map = (pose_.inverse() * pt) * resolution_ + center_image_;
    //     int x = int(pt_map[0]);
    //     int y = int(pt_map[1]);
    //     return Vec2i(x, y);
    // }

    /// 查找某个角度下的range值
    // double FindRangeInAngle(double angle, Scan2d::Ptr scan);

    /**
     * Bresenham直线填充，给定起始点和终止点，将中间的区域填充为白色
     * @param p1
     * @param p2
     */
    // void BresenhamFilling(const Vec2i& p1, const Vec2i& p2);

    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::Publisher map_publisher_;          // 声明一个Publisher
    ros::Publisher map_publisher_metadata_; // 声明一个Publisher

    // cv::Mat occupancy_grid_;  // 8bit 占据栅格图像
    nav_msgs::OccupancyGrid occupancy_grid_msg_;

    SE2 pose_;  // T_W_S
    SE2 submap_pose_in_world_;  // T_W_S
    // Vec2d center_image_ = Vec2d(image_size_ / 2, image_size_ / 2);
    Vec2d center_map_ = Vec2d(occupancy_grid_msg_size_ / 2, occupancy_grid_msg_size_ / 2);

    bool has_outside_pts_ = false;  // 标注栅格化过程中是否有落在外部的点

    // 模板
    std::vector<Model2DPoint> model_;  // 用于填充占据栅格的模板，都是世界系下的点

    // 参数
    inline static constexpr double closest_th_ = 0.2;         // 近距离阈值
    inline static constexpr double endpoint_close_th_ = 0.1;  // 末端点障碍物近距离阈值

    // inline static constexpr double resolution_ = 20.0;        // 1m 多少像素
    // inline static constexpr float inv_resolution_ = 0.05;     // 1个像素多少米（栅格分辨率）
    inline static constexpr double resolution_ = 0.05;        // 栅格分辨率分辨率,1个像素多少米
    inline static constexpr float  resolution_inver_ = 20;     // 1m 多少像素

    // inline static constexpr int image_size_ = 1000;           // 图像大小
    inline static constexpr int occupancy_grid_msg_size_ = 1000;// 地图大小
    inline static constexpr int model_size_ = 400;            // 模板像素大小
};

#endif