/**
 * @file velocity_data.cpp
 * @brief velocity数据的实现文件
 * @author GWH
 * @version 0.1
 * @date 2020-03-05
 */
#include "lidar_localization/sensor_data/velocity_data.hpp"
#include "glog/logging.h"

namespace lidar_localization{
bool VelocityData::SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time){
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time) {
            LOG(INFO) << "First velocity data of unsyncedData deque is later than sync_time";
            return false;
        }
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            LOG(INFO) << "There are at least two frames of VelocityData before sync_time";
            continue;  // 到达这里可以保证sync_time前之有一帧IMUData
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            LOG(WARNING) << "VelocityData maybe wrong, throw it";
            UnsyncedData.pop_front();
            return false;
        }

        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            LOG(WARNING) << "VelocityData maybe wrong, throw it";
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2) return false;

    VelocityData front_data = UnsyncedData.at(0);
    VelocityData back_data = UnsyncedData.at(1);
    VelocityData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.linear_velocity.x = front_data.linear_velocity.x * front_scale + back_data.linear_velocity.x * back_scale;
    synced_data.linear_velocity.y = front_data.linear_velocity.y * front_scale + back_data.linear_velocity.y * back_scale;
    synced_data.linear_velocity.z = front_data.linear_velocity.z * front_scale + back_data.linear_velocity.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;

    SyncedData.push_back(synced_data);

    return true;
}

void VelocityData::TransformCoordinate(Eigen::Matrix4f transform_matrix) {
    Eigen::Matrix4d matrix = transform_matrix.inverse().cast<double>(); // lidar_to_imu的逆
    Eigen::Matrix3d t_R = matrix.block<3,3>(0,0);
    Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);
    w = t_R * w;   
    v = t_R * v;

    Eigen::Vector3d r(matrix(0,3), matrix(1,3), matrix(2,3));  // lidar_to_imu的平移部分
    Eigen::Vector3d delta_v;
    delta_v(0) = w(1) * r(2) - w(2) * r(1);                    // w叉乘r
    delta_v(1) = w(2) * r(0) - w(0) * r(2);
    delta_v(2) = w(1) * r(1) - w(1) * r(0);
    v = v + delta_v;

    angular_velocity.x = w(0);
    angular_velocity.y = w(1);
    angular_velocity.z = w(2);
    linear_velocity.x = v(0);
    linear_velocity.y = v(1);
    linear_velocity.z = v(2);
}

}
