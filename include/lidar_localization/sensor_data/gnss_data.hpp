/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:25:13
 * reference: https://blog.csdn.net/He3he3he/article/details/82502540
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <vector>
#include <string>

#include "Geocentric/LocalCartesian.hpp"

using std::vector;
using std::string;

namespace lidar_localization {
class GNSSData {
  public:
    double time = 0.0;
    double longitude = 0.0;  // 经度
    double latitude = 0.0;   // 维度
    double altitude = 0.0;   // 高程
    double local_E = 0.0;    // 转换后的在东北天下的坐标
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;          // 定位的状态，无法准确定位/未能精确定位/基于卫星的增强
    int service = 0;         // Bits defining which Global Navigation Satellite System signals were used by the receiver.

  private:
    static GeographicLib::LocalCartesian geo_converter;   // 地理坐标转换的类
    static bool origin_position_inited;

  public: 
    void InitOriginPosition();
    void UpdateXYZ();
};
}
#endif
