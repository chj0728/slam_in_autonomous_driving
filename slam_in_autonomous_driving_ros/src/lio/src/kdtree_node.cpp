/**
 * @file kdtree_node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "kdtree.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <chrono>

// #include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
// #include <pcl/kdtree/kdtree_flann.h>



DEFINE_string(first_scan_path, "./src/data/first.pcd", "第一个点云路径");


/**
 * 统计代码运行时间
 * @tparam FuncT
 * @param func  被调用函数
 * @param func_name 函数名
 * @param times 调用次数
 */
template <typename FuncT>
void evaluate_and_call(FuncT&& func, const std::string& func_name = "", int times = 10) {
    double total_time = 0;
    for (int i = 0; i < times; ++i) {
        auto t1 = std::chrono::high_resolution_clock::now();
        func();
        auto t2 = std::chrono::high_resolution_clock::now();
        total_time += std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
    }

    LOG(INFO) << "方法 " << func_name << " 平均调用时间/次数: " << total_time / times << "/" << times << " 毫秒.";
}

int main(int argc, char **argv) 
{       
    CloudPtr first(new PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);

    CloudPtr cloud(new PointCloudType);
    PointType p1, p2, p3, p4;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;

    p2.x = 1;
    p2.y = 0;
    p2.z = 0;

    p3.x = 0;
    p3.y = 1;
    p3.z = 0;

    p4.x = 1;
    p4.y = 1;
    p4.z = 0;

    cloud->points.push_back(p1);
    cloud->points.push_back(p2);
    cloud->points.push_back(p3);
    cloud->points.push_back(p4);

    KdTree kdtree;
    pcl::search::KdTree<PointType> kdtree_pcl;
    pcl::KdTreeFLANN<PointType> kdtree_pcl_flann;
    // kdtree.BuildTree(first);
    // kdtree.PrintAll();
    evaluate_and_call([&kdtree, &first](){kdtree.BuildTree(first);}, "BuildTree", 2);

    // kdtree.BuildTree_for(first);
    // kdtree.PrintAll();
    evaluate_and_call([&kdtree, &first](){kdtree.BuildTree_for(first);}, "BuildTree_for", 2);

    evaluate_and_call([&first, &kdtree_pcl]() {kdtree_pcl.setInputCloud(first); }, "Kd Tree build", 100);

    evaluate_and_call([&first, &kdtree_pcl_flann]() {kdtree_pcl_flann.setInputCloud(first); }, "Kd Tree build flann", 100);

    int k = 5;
    std::vector<int> indices(k);
    std::vector<float> distances(k);
    std::cout << "K nearest neighbor search at (" << p1.x << " " << p1.y << " " << p1.z << ") with K=" << k << std::endl;


    if(kdtree_pcl.nearestKSearch(p1, k, indices, distances) > 0)
    {
        for(size_t i = 0; i < indices.size(); ++i)
            std::cout << "    " << first->points[indices[i]].x
                    << " " << first->points[indices[i]].y
                    << " " << first->points[indices[i]].z
                    << " (squared distance: " << distances[i] << ")" << std::endl;
    }


    if (kdtree_pcl_flann.nearestKSearchT(p1, k, indices, distances) > 0)
    {
        for(size_t i = 0; i < indices.size(); ++i)
            std::cout << "    " << first->points[indices[i]].x
                    << " " << first->points[indices[i]].y
                    << " " << first->points[indices[i]].z
                    << " (squared distance: " << distances[i] << ")" << std::endl;
    }
    


    //输入点云与目标点云的KNN搜索
    pcl::search::KdTree<PointType>  kdtree_pcl_1;
    kdtree_pcl_1.setInputCloud(first);
    //输入点云的索引
    std::vector<int> cloud_index(cloud->size());
    for (int i = 0; i < cloud->points.size(); i++) {
        cloud_index[i] = i;
    }

    std::vector<std::vector<int>> result_index;
    std::vector<std::vector<float>> result_distance;
    LOG(INFO) << "cloud_->points.size()" << cloud_index.size();
    LOG(INFO) << "cloud->points.size()" << cloud->points.size();
    kdtree_pcl_1.nearestKSearch(*cloud, cloud_index, 1, result_index, result_distance);
    for(size_t i = 0; i < cloud_index.size(); ++i)
    {
        for (int j = 0; j < result_index[i].size(); ++j)
        {
            std::cout << "    " << first->points[result_index[i][j]].x
                    << " " << first->points[result_index[i][j]].y
                    << " " << first->points[result_index[i][j]].z
                    << " (squared distance: " << result_distance[i][j] << ")" << std::endl;
        }
    }
    
    return 0;
}