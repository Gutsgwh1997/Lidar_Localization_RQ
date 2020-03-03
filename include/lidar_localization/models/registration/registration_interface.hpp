/**
 * @file registration_interface.hpp
 * @brief 点云匹配的接口类
 * @author GWH
 * @version 0.1
 * @date 2020-03-03
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class RegistrationInterface {
   public:
    virtual ~RegistrationInterface() = default;

    virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target_ptr) = 0;
    virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source_ptr,
                           const Eigen::Matrix4f& predict_pose,
                           CloudData::CLOUD_PTR& result_cloud_ptr,
                           Eigen::Matrix4f& result_pose) = 0;
};
}

#endif
