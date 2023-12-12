//
// Created by xiang on 2022/3/15.
//

#include "icp_2d.h"
#include "math_utils.h"

#include <glog/logging.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/impl/kdtree.hpp>

Icp2d::Icp2d():tf_listener_(tf_buffer_)
{ 
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m---->ICP2d odometry started.\033[0m");

    laser_transform_.setIdentity();

    // 读取参数
    nh_.param<std::string>("scan_topic", scan_topic_, "scan_multi");
    nh_.param<std::string>("odom_topic", odom_topic_, "odom");

    nh_.param<double>("max_dis2", max_dis2_, 0.01);
    nh_.param<double>("iteration", iterations_, 6);
    nh_.param<double>("max_use_range", max_use_range_, 15);
    nh_.param<int>("min_effect_pts", min_effect_pts_, 20);

    // 订阅激光话题
    laser_scan_sub_ = nh_.subscribe(scan_topic_, 1, &Icp2d::LaserSacnCallBack, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    
}

void Icp2d::BuildTargetKdTree() {
    if (last_scan_ == nullptr) {
        LOG(ERROR) << "target is not set";
        return;
    }
    target_cloud_.reset(new Cloud2d);
    for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
        
        // 排除错误的激光点
        if (last_scan_->ranges[i] < last_scan_->range_min || last_scan_->ranges[i] > last_scan_->range_max) {
            continue;
        }

        // 如果d>max_use_range_，则将d设置为max_use_range_
        double d = last_scan_->ranges[i];
        if (d > max_use_range_)
        {
             d = max_use_range_;
        }
           

        Point2d p;

        // double real_angle = last_scan_->angle_min + i * last_scan_->angle_increment;
        // p.x = last_scan_->ranges[i] * std::cos(real_angle);
        // p.y = last_scan_->ranges[i] * std::sin(real_angle);
        
        p.x = d * a_cos_[i];
        p.y = d * a_sin_[i];

        target_cloud_->points.push_back(p);
    }
    target_cloud_->width = target_cloud_->points.size();
    target_cloud_->is_dense = false;
    
    kdtree_.setInputCloud(target_cloud_);
    LOG(INFO) << "build kdtree finished";
}

void Icp2d::LaserSacnCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg) 
{
    double t1 = ros::Time::now().toSec();

    //scan_msg转换为pcl点云格式
    // source_cloud_.reset(new Cloud2d);
    // for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
    //     if (last_scan_->ranges[i] < last_scan_->range_min || last_scan_->ranges[i] > last_scan_->range_max) {
    //         continue;
    //     }

    //     double real_angle = last_scan_->angle_min + i * last_scan_->angle_increment;

    //     Point2d p;
    //     p.x = last_scan_->ranges[i] * std::cos(real_angle);
    //     p.y = last_scan_->ranges[i] * std::sin(real_angle);
    //     source_cloud_->points.push_back(p);
    // }
    // source_cloud_->width = source_cloud_->points.size();
    // source_cloud_->is_dense = false;
    current_time_ = scan_msg->header.stamp;

    if (first_scan_) {

        first_scan_ = false;

        // 将雷达各个角度的sin与cos值保存下来，以节约计算量
        CreateCache(scan_msg);

        last_scan_ = scan_msg;
        return;
    }

    double t2 = ros::Time::now().toSec();
    LOG(INFO) << "build kdtree start: " << t2;
    BuildTargetKdTree();
    double t3 = ros::Time::now().toSec();
    LOG(INFO) << "build kdtree end: " << t3;

    LOG(INFO) << "build kdtree cost: " << t3 - t2;

    // 进行配准
    double cost = 0, lastCost = 0;
    SE2 current_pose;

    for (int iter = 0; iter < iterations_; ++iter) 
    {
        Mat3d H = Mat3d::Zero();
        Vec3d b = Vec3d::Zero();
        cost = 0;

        int effective_num = 0;  // 有效点数

        // 遍历source
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) 
        {
            // 排除错误的激光点
            double r = scan_msg->ranges[i];
            if (r < scan_msg->range_min || r > scan_msg->range_max) 
            {
                continue;
            }
            // 如果d>max_use_range_，则将d设置为max_use_range_
            if (r > max_use_range_)
            {
                r = max_use_range_;
            }
                
            // float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            double theta = current_pose.so2().log();

            // Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
            Vec2d pw = current_pose * Vec2d(r * a_cos_[i], r * a_sin_[i]);
            Point2d pt;
            pt.x = pw.x();
            pt.y = pw.y();
        
            // 最近邻
            std::vector<int> nn_idx;
            std::vector<float> dis;
            kdtree_.nearestKSearch(pt, 1, nn_idx, dis);

            if (nn_idx.size() > 0 && dis[0] < max_dis2_) {
                effective_num++;
                Mat32d J;
                J << 1, 0, 0, 1, -r * std::sin(angle_[i] + theta), r * std::cos(angle_[i] + theta);
                //高斯牛顿法
                //Xk+1 = Xk - (J*J^T)^-1 * J * e
                // J.transpose()= Jacobian
                H += J * J.transpose();

                Vec2d e(pt.x - target_cloud_->points[nn_idx[0]].x, pt.y - target_cloud_->points[nn_idx[0]].y);
                b += -J * e;

                cost += e.dot(e);
            }
        }

        if (effective_num < min_effect_pts_) 
        {
            break;
        }

        // solve for dx
        Vec3d dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {
            break;
        }

        cost /= effective_num;
        if (iter > 0 && cost >= lastCost) {
            break;
        }

        LOG(INFO) << "iter " << iter << " cost = " << cost << ", effect num: " << effective_num;

        // /迭代一次更新一次位姿
        current_pose.translation() += dx.head<2>();
        current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);
        lastCost = cost;
    }

    tf2::Transform change_tf;
    CreateTfFromXYTheta(current_pose.translation().x(), current_pose.translation().y(), current_pose.so2().log(), change_tf);
    laser_transform_ = laser_transform_ * change_tf;

    last_scan_ = scan_msg;
    LOG(INFO) << "estimated pose: " << current_pose.translation().transpose()
              << ", theta: " << current_pose.so2().log();

    PublishTFAndOdometry();

    

    double t4 = ros::Time::now().toSec();
    LOG(INFO) << "scan match time: " << t4 - t3;
    LOG(INFO) << "call back cost time: " << t4 - t1;
}

/**
 * 从x,y,theta创建tf
 */
void Icp2d::CreateTfFromXYTheta(double &x, double &y, double &&theta, tf2::Transform &t)
{
    t.setOrigin(tf2::Vector3(x, y, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    t.setRotation(q);
}

/**
 * 发布tf与odom话题
 */
void Icp2d::PublishTFAndOdometry()
{
    // 发布 odom 到 base_link 的 tf
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform = tf2::toMsg(laser_transform_);
    tf_broadcaster_.sendTransform(tf_msg);


    // 发布 odomemtry 话题
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = base_frame_;
    tf2::toMsg(laser_transform_, odom_msg.pose.pose);
    // odom_msg.twist.twist = latest_velocity_;
    odom_pub_.publish(odom_msg);

}

// 雷达数据间的角度是固定的，因此可以将对应角度的cos与sin值缓存下来，不用每次都计算
void Icp2d::CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    a_cos_.clear();
    a_sin_.clear();
    double angle;

    for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
    {
        angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        angle_.push_back(angle);
        a_cos_.push_back(std::cos(angle));
        a_sin_.push_back(std::sin(angle));
    }
}