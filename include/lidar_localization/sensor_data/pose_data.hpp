/**
 * @file pose_data.hpp
 * @brief 存放处理后的IMU姿态以及GNSS位置
 * @author GWH
 * @version 0.1
 * @date 2020-03-08
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class PoseData {
  public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double time = 0.0;

  public:
    Eigen::Quaternionf GetQuaternion();
};
}

#endif
