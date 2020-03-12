/**
 * @file cloud_publisher.cpp
 * @brief 发布点云
 * @author GWH
 * @version 0.1
 * @date 2020-03-09 10:55:08
 */
#include "lidar_localization/publisher/cloud_publisher.hpp"

namespace lidar_localization {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, size_t buff_size)
    : nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish( CloudData::CLOUD_PTR& cloud_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr_input, time);
}

void CloudPublisher::Publish( CloudData::CLOUD_PTR& cloud_ptr_input, double time) {
    ros::Time ros_time((float)time);
    PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::PublishData( CloudData::CLOUD_PTR& cloud_ptr_input, ros::Time time) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
    cloud_ptr_output->header.stamp    = time;
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}

bool CloudPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}  // namespace data_output
