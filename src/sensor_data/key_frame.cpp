/**
 * @file key_frame.cpp
 * @brief 关键帧数据结构的实现文件
 * @author GWH
 * @version 0.1
 * @date 2020-03-09
 */
#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
Eigen::Quaternionf KeyFrame::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3, 3>(0, 0);

    return q;
}
}
