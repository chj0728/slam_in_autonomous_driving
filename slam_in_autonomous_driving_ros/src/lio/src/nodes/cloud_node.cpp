/**
 * @file cloud_node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>

#include <lio/mapping/front_end/front_end.hpp>//前端

#include <lio/subcriber/cloud_subscriber.hpp>//点云订阅者
#include <lio/publisher/cloud_publisher.hpp>//点云发布者
#include <lio/publisher/odometry_publisher.hpp>//里程计发布者
#include <lio/tf2/tf2_broadcaster.hpp>//tf发布
// #include <lio/models/cloud_filter/voxel_filter.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace lio
{
    class FrontEndFlow
    {   
        public:
            FrontEndFlow(ros::NodeHandle& nh, const YAML::Node& config_node,
                        std::string cloud_topic, 
                        std::string odom_topic, 
                        std::string map_frame_id, 
                        std::string lidar_frame_id)
            {
                cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 1000);//订阅者

                cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "lio_points", "rslidar", 1000); //当前帧发布者
                local_map_pub_ptr = std::make_shared<CloudPublisher>(nh, "local_map", "map", 1000); //局部地图发布者
                

                laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, odom_topic, map_frame_id, lidar_frame_id, 100);//里程计发布者
                
                map_lidar_tf_ptr_   = std::make_shared<TF2BroadCaster>(map_frame_id, lidar_frame_id);

                front_end_ptr_ = std::make_shared<FrontEnd>(config_node);//前端指针

            }
            bool Run()
            {   
                double time = ros::Time::now().toSec();

                cloud_sub_ptr_->ParseData(cloud_data_buff_);

                while(cloud_data_buff_.size() > 0)
                {
                    LOG(INFO) << "cloud_data_buff_.size():"<<cloud_data_buff_.size();

                    current_cloud_data_ = cloud_data_buff_.front();

                    cloud_data_buff_.pop_front();

                    double time1 = ros::Time::now().toSec();
                    if(UpdateLaserOdometry())
                    {
                        PublishData();
                    }
                    double time_end1 = ros::Time::now().toSec();
                    LOG(INFO) << "UpdateLaserOdometry Time: " << time_end1 - time1;
                }

                double time_end = ros::Time::now().toSec();
                LOG(INFO) << "Run() Time: " << time_end - time;
                return true;
            }

            bool UpdateLaserOdometry()
            {   
                static bool odometry_inited = false;

                if(!odometry_inited)
                {
                    odometry_inited = true;
                    front_end_ptr_->SetInitPose(Mat4f::Identity());
                    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
                }

                return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);

            }
            //发布里程计和点云
            bool PublishData()
            {
                laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);//前端里程计

                cloud_pub_ptr->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);//当前帧
                local_map_pub_ptr->Publish(front_end_ptr_->GetLocalMapPtr(), current_cloud_data_.time);//局部地图

                map_lidar_tf_ptr_->SendTF(laser_odometry_, current_cloud_data_.time);//map->lidar tf
                return true;
            }

        private:
            std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;//点云话题订阅

            std::shared_ptr<CloudPublisher> cloud_pub_ptr ; //点云话题发布
            std::shared_ptr<CloudPublisher> local_map_pub_ptr ; //局部地图点云发布

            std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;//前端里程计发布

            std::shared_ptr<TF2BroadCaster> map_lidar_tf_ptr_;//map—>lidar tf发布

            std::shared_ptr<FrontEnd> front_end_ptr_;//前端指针


            std::deque<CloudData> cloud_data_buff_;//CloudData 队列
                
            CloudData current_cloud_data_;

            Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

    };

}// namespace lio


using namespace lio;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::deque<CloudData> cloud_data_buff;
    std::string cloud_topic;

    // float filter_size;
    double filter_size;

    // 读取配置文件
    std::string config_file_path = "/home/caohaojie/SLAM/BOOKS/slam_in_autonomous_driving_ros/src/lio/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    cloud_topic = config_node["lio_mapping"]["lidar_topic"].as<std::string>();
    filter_size = config_node["lio_mapping"]["voxel_size"].as<double>();
    LOG(INFO)<<config_file_path; 
    LOG(INFO)<< cloud_topic;
    LOG(INFO)<< filter_size;

    // nh_private.param<std::string>("cloud_topic", cloud_topic, "/rslidar_points");

    // std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100);
    // std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "lio_points", "lio_link", 100); 
    // std::shared_ptr<CloudPublisher> cloud_filter_pub_ptr = std::make_shared<CloudPublisher>(nh, "filter_points", "lio_link", 100); 
    // std::shared_ptr<VoxelFilter> voxel_filter_ptr = std::make_shared<VoxelFilter>(filter_size);

    std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh, config_node, cloud_topic, "lio_odom", "map", "rslidar"); 
    
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        // cloud_sub_ptr->ParseData(cloud_data_buff);

        // if(cloud_data_buff.size() > 0)
        // {

        //     CloudData cloud_data = cloud_data_buff.front();

        //     CloudPtr filter_cloud_ptr(new PointCloudType);
        //     voxel_filter_ptr->Filter(cloud_data.cloud_ptr, filter_cloud_ptr);

        //     cloud_pub_ptr->Publish(cloud_data.cloud_ptr);
        //     cloud_filter_pub_ptr->Publish(filter_cloud_ptr);

        //     cloud_data_buff.pop_front();
        // }

        double time = ros::Time::now().toSec();
        front_end_flow_ptr->Run();
        double time_end = ros::Time::now().toSec();

        // LOG(INFO) << "Front End Flow Time: " << time_end - time;

        rate.sleep();
    }
    return 0;

}