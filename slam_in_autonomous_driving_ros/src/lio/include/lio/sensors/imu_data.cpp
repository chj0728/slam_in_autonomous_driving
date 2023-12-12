/**
 * @file imu_data.cpp
 * @author your name (you@domain.com)
 * @brief imu_data.hpp的实现文件
 * @version 0.1
 * @date 2023-11-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <lio/sensors/imu_data.hpp>

namespace lio{


//IMUData
//*********************************************************************************
Mat3d IMUData::GetRotationMatrix() 
{
    // Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    // Eigen::Matrix3d matrix = q.matrix().cast<double>();

    Mat3d rotation_matrix = qd_.matrix().cast<double>();
    return rotation_matrix;
}

Vec3d IMUData::GetEulerAngle() 
{
    // Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    // Eigen::Vector3d euler = q.matrix().eulerAngles(2, 1, 0);

    Vec3d euler = qd_.matrix().eulerAngles(2, 1, 0);
    return euler;   
}

bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double &sync_time) 
{
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time) 
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    IMUData front_data = UnsyncedData.at(0);
    IMUData back_data = UnsyncedData.at(1);
    IMUData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);

    synced_data.time = sync_time;

    synced_data.gyro_ = front_data.gyro_ * front_scale + back_data.gyro_ * back_scale;
    synced_data.acce_ = front_data.acce_ * front_scale + back_data.acce_ * back_scale;
    synced_data.qd_   = front_data.qd_.slerp(front_scale, back_data.qd_);
    synced_data.qd_.normalize();
    SyncedData.emplace_back(synced_data);

    // synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    // synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    // synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    // synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    // synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    // synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
    // // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
    // // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
    // synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    // synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    // synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    // synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
    // // 线性插值之后要归一化
    // synced_data.orientation.Normlize();
    // SyncedData.push_back(synced_data);

    return true;
}

//IMUSubscriber
//*********************************************************************************

void IMUSubscriber::Callback(const sensor_msgs::ImuConstPtr &imu_msg) 
{
    buff_mutex_.lock();

    IMUData imu_data(imu_msg);

    new_imu_data_.emplace_back(imu_data);

    buff_mutex_.unlock();
}
void IMUSubscriber::ParseData(std::deque<IMUData>& imu_data_deque) {
    buff_mutex_.lock();
    if (new_imu_data_.size() > 0) {
        imu_data_deque.insert(imu_data_deque.end(), new_imu_data_.begin(), new_imu_data_.end());
        new_imu_data_.clear();
    }
    buff_mutex_.unlock();
}

} // namespace lio