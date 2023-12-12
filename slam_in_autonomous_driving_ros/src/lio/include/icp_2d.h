/**
 * @file icp_2d.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef LIO_ICP_2D_H
#define LIO_ICP_2D_H

#include "eigen_types.h"
#include "point_types.h"
// #include "lidar_utils.h"


//ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

//tf2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>           // tf2::Transform
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>//转换函数头文件:tf2::fromMsg, tf2::toMsg


#include <pcl/search/kdtree.h>


/**
 * 第六章谈到的各种类型的ICP代码实现
 * 用法：先SetTarget, 此时构建target点云的KD树；再SetSource，然后调用Align*方法
 */
class Icp2d {
   public:
    using Point2d = pcl::PointXY;
    using Point3d = pcl::PointXYZI;
    using Cloud2d = pcl::PointCloud<Point2d>;
    using Cloud3d = pcl::PointCloud<Point3d>;
    Icp2d();
    /// 使用高斯牛顿法进行配准
    // bool AlignGaussNewton(SE2& init_pose);

    /// 使用高斯牛顿法进行配准, Point-to-Plane
    // bool AlignGaussNewtonPoint2Plane(SE2& init_pose);

    void LaserSacnCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    void CreateTfFromXYTheta(double &x, double &y, double &&theta, tf2::Transform &t);

    void PublishTFAndOdometry();

    void BuildTargetKdTree();// 建立目标点云的Kdtree

    //缓存激光角度的cos和sin值
    void CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

   private:
    
    
    pcl::search::KdTree<Point2d> kdtree_;
    Cloud2d::Ptr target_cloud_;  // PCL 形式的target cloud
    // Cloud3d::Ptr cloud_;  // PCL 形式的source cloud

    //ROS
    ros::NodeHandle nh_;
    // ros::NodeHandle private_nh_;

    ros::Subscriber laser_scan_sub_;
    ros::Publisher odom_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    tf2::Transform laser_transform_; // 激光雷达的位姿

    sensor_msgs::LaserScan::ConstPtr last_scan_ = nullptr;
    sensor_msgs::LaserScan::ConstPtr current_scan_ = nullptr;

    bool first_scan_ = true; // 是否是第一帧
    std::vector<double> angle_; //保存下来雷达各个角度
    std::vector<double> a_cos_; // 保存下来雷达各个角度的cos值
    std::vector<double> a_sin_; // 保存下来雷达各个角度的sin值
    
    double iterations_ = 10;  // 迭代次数
    double max_dis2_ = 0.01;    // 最近邻时的最远距离（平方）
    double max_use_range_;  // 激光雷达最大使用距离
    int min_effect_pts_ = 20;  // 最小有效点数

    std::string scan_topic_ = "scan";
    std::string odom_topic_ = "odom";
    std::string base_frame_ = "base_link";  // 机器人的base_link
    std::string laser_frame_ = "laser"; // 激光雷达的frame_id
    std::string map_frame_ = "map"; // map frame_id
    std::string odom_frame_ = "odom"; // odom frame_id
    
    ros::Time current_time_ = ros::Time(0);
};



#endif  // SLAM_IN_AUTO_DRIVING_ICP_2D_H
