//
// Created by xiang on 2022/3/15.
//

#include "icp_2d.h"
#include "math_utils.h"

#include <cmath>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/impl/kdtree.hpp>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

//构建代价函数
//公式(6.4)
//r_    :   当前激光点到雷达坐标系的距离
//pi_:   当前激光点到雷达坐标系的角度
//target_x_:KDtree里目标点到的x坐标
//target_y_:KDtree里目标点到的y坐标

//待优化的是  x,  y,  theta
struct CostFunctor 
{
    CostFunctor(double r, double pi, double target_x, double target_y):
    r_(r), pi_(pi), target_x_(target_x), target_y_(target_y)
    {}
  
    template <typename T>
    bool operator()(const T* const x, const T* const y, const T* const theta,T* residual) const
    {
        residual[0] = x[0]+ r_*cos(pi_ + theta[0])    -target_x_;//残存1
        residual[1] = y[0]+ r_*sin(pi_ + theta[0])    -target_y_;//残存2
        return true;
    }
    private:
    double target_x_ ;
    double target_y_ ;
    double r_ ;
    double pi_ ;
};

Icp2d::Icp2d():tf_listener_(tf_buffer_)
{ 
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m---->ICP2d odometry started.\033[0m");

    laser_transform_.setIdentity();

    // 读取参数
    nh_.param<std::string>("scan_topic", scan_topic_, "scan_multi");
    nh_.param<std::string>("odom_topic", odom_topic_, "odom");

    nh_.param<double>("max_dis2", max_dis2_, 0.01);
    nh_.param<double>("iteration", iterations_, 5);
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
    SE2 current_pose;    

    for (int iter = 0; iter < iterations_; ++iter) 
    {
        int effective_num = 0;  // 有效点数

        double x1 = 0;
        double y1 = 0;
        double theta1 = 0;

        // Build the problem.
        Problem problem;
        Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        Solver::Summary summary;

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
                
            //一次优化后,根据优化后的x1,y1,theta1,更新当前激光点的r,angle
            Vec2d pw = current_pose * Vec2d(r * a_cos_[i], r * a_sin_[i]);
            double r_update;
            double angle_update;
            r_update = sqrt(pow(pw.x(),2)+pow(pw.y(),2));
            angle_update = atan2(pw.y(),pw.x());
                
            //并且重新寻最近邻点
            Point2d pt;
            pt.x = pw.x();
            pt.y = pw.y();
            std::vector<int> nn_idx;
            std::vector<float> dis;
            kdtree_.nearestKSearch(pt, 1, nn_idx, dis);

            if (nn_idx.size() > 0 && dis[0] < max_dis2_) 
            {
                effective_num++;
                //
                problem.AddResidualBlock(new AutoDiffCostFunction<CostFunctor, 2, 1, 1, 1>
                (new CostFunctor(r_update, angle_update, target_cloud_->points[nn_idx[0]].x, target_cloud_->points[nn_idx[0]].y)),
                nullptr,&x1,&y1,&theta1
                );
            }
        }//end for add residual block

        //Run the solver!
        Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << "\n";
        std::cout << "x1 -> " << x1 << "\n";
        std::cout << "y1 -> " << y1 << "\n";
        std::cout << "theta1-> " << theta1 << "\n";

        if (effective_num < min_effect_pts_) 
        {
            break;
        }


        LOG(INFO) << "iter: " << iter << ", effect num: " << effective_num;

        // /迭代一次更新一次位姿
        current_pose.translation().x() += x1;
        current_pose.translation().y() += y1;
        current_pose.so2() = current_pose.so2() * SO2::exp(theta1);
    }//end iterations

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