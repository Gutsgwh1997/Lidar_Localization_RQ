/**
 * @file velocity_subscriber.cpp
 * @brief velocity subscriber的实现代码
 * @author GWH
 * @version 0.1
 * @date 2020-03-05
 */
#include "lidar_localization/subscriber/velocity_subscriber.hpp"

namespace lidar_localization {
VelocitySubscriber::VelocitySubscriber(ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size)
    : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &VelocitySubscriber::msg_callback, this);
}

void VelocitySubscriber::ParseData(std::deque<VelocityData>& velocity_data_buff) {
    if (!new_velocity_data_.empty()){
        velocity_data_buff.insert(velocity_data_buff.end(), new_velocity_data_.begin(), new_velocity_data_.end());
        new_velocity_data_.clear();
    }
}

void VelocitySubscriber::msg_callback(const geometry_msgs::TwistStampedConstPtr& velocity_msg_ptr) {
    VelocityData velocity_data;
    velocity_data.time               = velocity_msg_ptr->header.stamp.toSec();

    velocity_data.linear_velocity.x  = velocity_msg_ptr->twist.linear.x;
    velocity_data.linear_velocity.y  = velocity_msg_ptr->twist.linear.y;
    velocity_data.linear_velocity.z  = velocity_msg_ptr->twist.linear.z;

    velocity_data.angular_velocity.x = velocity_msg_ptr->twist.angular.x;
    velocity_data.angular_velocity.y = velocity_msg_ptr->twist.angular.y;
    velocity_data.angular_velocity.z = velocity_msg_ptr->twist.angular.z;

    new_velocity_data_.push_back(velocity_data);
}

}
