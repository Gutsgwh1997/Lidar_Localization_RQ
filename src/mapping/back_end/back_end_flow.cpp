/**
 * @file back_end_flow.cpp
 * @brief back end任务管理，放在类里边代码更清晰
 * @author Ren Qian
 * @version 0.1
 * @date 2020-03-10 13:20:14
 */
#include "lidar_localization/mapping/back_end/back_end_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
BackEndFlow::BackEndFlow(ros::NodeHandle& nh) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);         // 原始点云数据(与gnss做过时间对齐)
    gnss_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);   // gnss的位姿
    laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/laser_odom", 100000);   // 雷达里程计的位姿

    transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/transformed_odom", "map", "lidar", 1000);
    key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_frame", "map", 1000);
    key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(nh, "/optimized_key_frames", "map", 1000);

    back_end_ptr_ = std::make_shared<BackEnd>();
}

bool BackEndFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        UpdateBackEnd();

        PublishData();
    }

    return true;
}

/**
 * @brief 当bag文件播放完之后，关键帧里的后面几帧可能并没有得到优化,
 *        因为上面三个条件计数还没达到新一次优化所需的数量要求
 *        获得全部优化的关键帧并且发布出去
 * @return 
 */
bool BackEndFlow::ForceOptimize() {
    back_end_ptr_->ForceOptimize();
    if (back_end_ptr_->HasNewOptimized()) {
        std::deque<KeyFrame> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);
    }
    return true;
}

bool BackEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);
    laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);

    return true;
}

bool BackEndFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (gnss_pose_data_buff_.size() == 0)
        return false;
    if (laser_odom_data_buff_.size() == 0)
        return false;

    return true;
}

bool BackEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();         // 这里接收的是时间同步之后的点云与GNSS数据，laser_odom经过front_end处理后
    current_gnss_pose_data_ = gnss_pose_data_buff_.front(); // 不丢帧的情况下laser_odom的数据也是时间对齐的
    current_laser_odom_data_ = laser_odom_data_buff_.front();

    double diff_gnss_time = current_cloud_data_.time - current_gnss_pose_data_.time;
    double diff_laser_time = current_cloud_data_.time - current_laser_odom_data_.time;

    if (diff_gnss_time < -0.05 || diff_laser_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_pose_data_buff_.pop_front();
        return false;
    }

    if (diff_laser_time > 0.05) {
        laser_odom_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_pose_data_buff_.pop_front();
    laser_odom_data_buff_.pop_front();

    return true;
}

bool BackEndFlow::UpdateBackEnd() {
    static bool odometry_inited = false;
    static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

    if (!odometry_inited) {
        odometry_inited = true;
        odom_init_pose = current_gnss_pose_data_.pose * current_laser_odom_data_.pose.inverse();
    }
    // 将lidar的数据与gnss的首帧对应
    // 因为lidar的参考帧是Identity()，所以这样子转换
    current_laser_odom_data_.pose = odom_init_pose * current_laser_odom_data_.pose;

    return back_end_ptr_->Update(current_cloud_data_, current_laser_odom_data_, current_gnss_pose_data_);
}

/**
 * @brief 发布与GNSS统一坐标后的lidar_odom的位姿
 *        发布最近的关键帧KeyFrame
 *        发布优化后的keyframes_deque
 * @return 
 */
bool BackEndFlow::PublishData() {
    transformed_odom_pub_ptr_->Publish(current_laser_odom_data_.pose, current_laser_odom_data_.time);

    if (back_end_ptr_->HasNewKeyFrame()) {
        KeyFrame key_frame;
        back_end_ptr_->GetLatestKeyFrame(key_frame);
        key_frame_pub_ptr_->Publish(key_frame);
    }

    if (back_end_ptr_->HasNewOptimized()) {
        std::deque<KeyFrame> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);
    }

    return true;
}
}
