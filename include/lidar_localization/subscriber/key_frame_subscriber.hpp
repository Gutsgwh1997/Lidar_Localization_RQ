/**
 * @file key_frame_subscriber.hpp
 * @brief 关键帧消息接受类型，viewer中使用
 * @author GWH
 * @version 0.1
 * @date 2020-03-11 16:46:20
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <deque>
#include <string>
#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
class KeyFrameSubscriber {
   public:
    KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    void ParseData(std::deque<KeyFrame>& deque_key_frame_data);

   private:
    void msg_callback(const geometry_msgs::PoseStampedConstPtr& key_frame_data_ptr);

   private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<KeyFrame> new_key_frame_data_;
};
}
#endif
