/**
 * @file odometry_subscriber.hpp
 * @brief 订阅odometry数据
 * @author Ren Qian
 * @version 0.1
 * @date 2020-03-10 13:30:30
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization {
class OdometrySubscriber {
  public:
    OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    OdometrySubscriber() = default;
    void ParseData(std::deque<PoseData>& deque_pose_data);

  private:
    void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<PoseData> new_pose_data_; 
};
}
#endif
