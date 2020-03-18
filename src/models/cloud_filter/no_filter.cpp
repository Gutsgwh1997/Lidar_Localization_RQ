/**
 * @file no_filter.cpp
 * @brief 不进行滤波
 * @author Ren Qian
 * @version 0.1
 * @date 2020-03-17 18:28:22
 */
#include "lidar_localization/models/cloud_filter/no_filter.hpp"
#include "glog/logging.h"

namespace lidar_localization {
NoFilter::NoFilter() {
}

bool NoFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    filtered_cloud_ptr.reset(new CloudData::CLOUD(*input_cloud_ptr));
    return true;
}
} 
