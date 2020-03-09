/**
 * @file pose_data.cpp
 * @brief pose数据结构的实现文件
 * @author GWH
 * @version 0.1
 * @date 2020-03-09
 */
#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization {
Eigen::Quaternionf PoseData::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3, 3>(0, 0);

    return q;
}
}
