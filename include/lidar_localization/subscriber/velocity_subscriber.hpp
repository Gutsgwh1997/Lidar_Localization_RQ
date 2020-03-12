/**
 * @file velocity_subscriber.hpp
 * @brief velocity subscriber
 * @author GWH
 * @version 0.1
 * @date 2020-03-05
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization{
class VelocitySubscriber{
    public:
        VelocitySubscriber(ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size);
        VelocitySubscriber()=default;
        void ParseData(std::deque<VelocityData>& velocity_data_buff);
    private:
        void msg_callback(const geometry_msgs::TwistStampedConstPtr& velocity_msg);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<VelocityData> new_velocity_data_;
};
}

#endif
