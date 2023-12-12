//
// Created by xiang on 2022/3/15.
//

#include "icp_2d.h"
#include "math_utils.h"

#include <glog/logging.h>

Icp2d::Icp2d():tf_listener_(tf_buffer_)
{ 
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m---->ICP2d odometry started.\033[0m");

    laser_transform_.setIdentity();

    // 读取参数
    nh_.param<std::string>("scan_topic", scan_topic_, "scan_multi");
    nh_.param<std::string>("odom_topic", odom_topic_, "odom");

    nh_.param<double>("max_dis2", max_dis2_, 0.01);
    nh_.param<int>("min_effect_pts", min_effect_pts_, 20);
    nh_.param<double>("iteration", iterations_, 10);

    // 订阅激光话题
    laser_scan_sub_ = nh_.subscribe(scan_topic_, 100, &Icp2d::LaserSacnCallBack, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    
}

void Icp2d::BuildTargetKdTree() {
    if (last_scan_ == nullptr) {
        LOG(ERROR) << "target is not set";
        return;
    }

    target_cloud_.reset(new Cloud3d);
    for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
        if (last_scan_->ranges[i] < last_scan_->range_min || last_scan_->ranges[i] > last_scan_->range_max) {
            continue;
        }

        double real_angle = last_scan_->angle_min + i * last_scan_->angle_increment;

        Point3d p;
        p.x = last_scan_->ranges[i] * std::cos(real_angle);
        p.y = last_scan_->ranges[i] * std::sin(real_angle);
        p.z = 0;
        target_cloud_->points.emplace_back(p);
    }
    target_cloud_->width = target_cloud_->points.size();
    target_cloud_->is_dense = false;
    
    kdtree_.setInputCloud(target_cloud_);
}

void Icp2d::LaserSacnCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg) 
{
    double t1 = ros::Time::now().toSec();

    current_time_ = scan_msg->header.stamp;

    if (first_scan_) {
        first_scan_ = false;
        last_scan_ = scan_msg;
        return;
    }

    double t2 = ros::Time::now().toSec();
    BuildTargetKdTree();
    double t3 = ros::Time::now().toSec();

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
        // Cloud3d::Ptr cloud_(new Cloud3d);
        cloud_.reset(new Cloud3d);
        std::vector<float>  angle_index(scan_msg->ranges.size());
        std::vector<float>  r_index(scan_msg->ranges.size());
        // std::vector<int>    cloud_index;
        float theta = current_pose.so2().log();
        for (size_t i = 0,n = 0; i < scan_msg->ranges.size(); ++i) 
        {
            float r = scan_msg->ranges[i];
            if (r < scan_msg->range_min || r > scan_msg->range_max) 
            {
                continue;
            }
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            // float theta = current_pose.so2().log();
            Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
            Point3d pt;
            pt.x = pw.x();
            pt.y = pw.y();
            pt.z = 0;
            pt.intensity = 0;
            cloud_->points.emplace_back(pt);

            angle_index[n] = angle;
            r_index[n] = r;
            // cloud_index[n] = n;
            ++n;
        }
        cloud_->width = cloud_->points.size();
        cloud_->is_dense = false;
        std::vector<int> cloud_index(cloud_->points.size());

        for (int i = 0; i < cloud_->points.size(); i++) {
            cloud_index[i] = i;
        }
        LOG(INFO) << "cloud_index.size()" << cloud_index.size();

        // 最近邻
        std::vector<std::vector<int>> result_index(cloud_->points.size());
        std::vector<std::vector<float>> result_distance(cloud_->points.size());
        kdtree_.nearestKSearch(*cloud_, cloud_index, 1, result_index, result_distance);
        for(size_t i = 0; i < cloud_index.size(); ++i)
        {
            LOG(INFO) << "result_index[i].size()" << result_index[i].size();
            LOG(INFO) << "result_distance[i][0]" << result_distance[i][0];
            if (result_index[i].size() > 0 && result_distance[i][0] < max_dis2_) {
                effective_num++;
                Mat32d J;
                J << 1, 0, 0, 1, - r_index[i] * std::sin(angle_index[i] + theta), r_index[i]* std::cos(angle_index[i] + theta);
                H += J * J.transpose();

                Vec2d e(cloud_->points[i].x - target_cloud_->points[result_index[i][0]].x, cloud_->points[i].y - target_cloud_->points[result_index[i][0]].y);
                b += -J * e;

                cost += e.dot(e);
            }
        }
        LOG(INFO) << "effective_num " << effective_num ;
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
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform = tf2::toMsg(laser_transform_);

    // 发布 odom 到 base_link 的 tf
    tf_broadcaster_.sendTransform(tf_msg);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    tf2::toMsg(laser_transform_, odom_msg.pose.pose);
    // odom_msg.twist.twist = latest_velocity_;

    // 发布 odomemtry 话题
    odom_pub_.publish(odom_msg);
}
