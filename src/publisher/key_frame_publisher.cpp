/**
 * @file key_frames_publisher.cpp
 * @brief 单个key frame信息发布
 * @author Ren Qian
 * @version 0.1
 * @date 2020-03-10 13:16:56
 */
#include "lidar_localization/publisher/key_frame_publisher.hpp"

#include <Eigen/Dense>

namespace lidar_localization {
KeyFramePublisher::KeyFramePublisher(ros::NodeHandle& nh, 
                                     std::string topic_name, 
                                     std::string frame_id,
                                     int buff_size)
    :nh_(nh), frame_id_(frame_id) {

    publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name, buff_size);
}

void KeyFramePublisher::Publish(KeyFrame& key_frame) {
    geometry_msgs::PoseStamped pose_stamped;

    ros::Time ros_time((float)key_frame.time);
    pose_stamped.header.stamp = ros_time;
    pose_stamped.header.frame_id = frame_id_;

    pose_stamped.header.seq = key_frame.index;

    pose_stamped.pose.position.x = key_frame.pose(0,3);
    pose_stamped.pose.position.y = key_frame.pose(1,3);
    pose_stamped.pose.position.z = key_frame.pose(2,3);

    Eigen::Quaternionf q = key_frame.GetQuaternion();
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    publisher_.publish(pose_stamped);
}

bool KeyFramePublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}
