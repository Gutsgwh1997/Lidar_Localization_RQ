/**
 * @file distortion_adjust.hpp
 * @brief 点云畸变补偿
 * @author GWH
 * @version 0.1
 * @date 2020-03-07
 */

#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
class DistortionAdjust {
   public:
    DistortionAdjust() = default;
    bool SetMotionInfo(float scan_period_sec,const VelocityData& velocity_data);
    bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

   private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

   private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
};
}

#endif
