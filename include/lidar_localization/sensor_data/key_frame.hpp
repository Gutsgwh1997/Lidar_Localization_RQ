/**
 * @file key_frame.hpp
 * @brief 关键帧(只保存位姿)，在各个模块之间传递数据
 * @author GWH
 * @version 0.1
 * @date 2020-03-09
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class KeyFrame {
  public:
    double time = 0.0;
    unsigned int index = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  public:
    Eigen::Quaternionf GetQuaternion();
};
}

#endif
