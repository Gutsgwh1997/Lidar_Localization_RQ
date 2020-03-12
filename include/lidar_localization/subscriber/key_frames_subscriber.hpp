/**
 * @file key_frames_subscriber.hpp
 * @brief keyframes接收类
 * @author GWH
 * @version 0.1
 * @date 2020-03-11 17:04:19
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <deque>
#include <string>

#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
class KeyFramesSubscriber {
   public:
    KeyFramesSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    void ParseData(std::deque<KeyFrame>& deque_key_frames_data);

   private:
    void msg_callback(const nav_msgs::Path::ConstPtr& key_frames_data_ptr);

   private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<KeyFrame> new_key_frames_data_;
};
}
#endif
