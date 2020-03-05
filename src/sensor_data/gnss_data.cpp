/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-06 20:42:23
 */
#include "lidar_localization/sensor_data/gnss_data.hpp"

#include "glog/logging.h"

// 静态成员变量必须在类外初始化
bool lidar_localization::GNSSData::origin_position_inited = false;
// 将GPS的经纬度(WGS84)转化为东北天坐标的类
GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_converter;

namespace lidar_localization {

void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

bool GNSSData::SyncData (std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time){
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time) {
            LOG(INFO) << "First GNSS data of unsyncedData deque is later than sync_time";
            return false;
        }
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            LOG(INFO) << "There are at least two frames of GNSSData before sync_time";
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            LOG(WARNING) << "GNSSData maybe wrong, throw it";
            return false;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            LOG(WARNING) << "GNSSData maybe wrong, throw it";
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2) {
        return false;
    }

    GNSSData front_data = UnsyncedData.at(0);
    GNSSData back_data  = UnsyncedData.at(1);
    GNSSData synced_data;

    double front_scale    = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale     = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time      = sync_time;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude  = front_data.latitude * front_scale + back_data.latitude * back_scale;
    synced_data.altitude  = front_data.altitude * front_scale + back_data.altitude * back_scale;

    SyncedData.push_back(synced_data);

    return true;
}

}
