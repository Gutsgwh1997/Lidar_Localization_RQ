/**
 * @file key_frame_subscriber.cpp
 * @brief key_frame_subscriber的实现文件
 * @author Ren Qian
 * @version 0.1
 * @date 2020-03-11 17:19:27
 */
#include "lidar_localization/subscriber/key_frame_subscriber.hpp"
#include <Eigen/Dense>
namespace lidar_localization {
KeyFrameSubscriber::KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &KeyFrameSubscriber::msg_callback, this);
}

void KeyFrameSubscriber::msg_callback(const geometry_msgs::PoseStampedConstPtr& key_frame_data_ptr) {
    KeyFrame key_frame;
    key_frame.time  = key_frame_data_ptr->header.stamp.toSec();
    key_frame.index = key_frame_data_ptr->header.seq;

    key_frame.pose(0, 3) = key_frame_data_ptr->pose.position.x;
    key_frame.pose(1, 3) = key_frame_data_ptr->pose.position.y;
    key_frame.pose(2, 3) = key_frame_data_ptr->pose.position.z;

    Eigen::Quaternionf q;
    q.x() = key_frame_data_ptr->pose.orientation.x;
    q.y() = key_frame_data_ptr->pose.orientation.y;
    q.z() = key_frame_data_ptr->pose.orientation.z;
    q.w() = key_frame_data_ptr->pose.orientation.w;

    key_frame.pose.block<3, 3>(0, 0) = q.matrix();

    new_key_frame_data_.push_back(key_frame);
}

void KeyFrameSubscriber::ParseData(std::deque<KeyFrame>& deque_key_frame_data) {
    if (!new_key_frame_data_.empty()) {
        deque_key_frame_data.insert(deque_key_frame_data.end(), new_key_frame_data_.begin(),
                                    new_key_frame_data_.end());
        new_key_frame_data_.clear();
    }
}
}
