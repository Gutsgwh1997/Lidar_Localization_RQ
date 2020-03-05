/**
 * @file velocity_data.hpp
 * @brief velocity 数据
 * @author GWH
 * @version 0.1
 * @date 2020-03-05
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_HPP_
#include <Eigen/Dense>
#include <deque>

namespace lidar_localization {
class VelocityData {
   public:
    struct LinearVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;

   public:
    static bool SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncdData, double sync_time);
};
}
#endif
