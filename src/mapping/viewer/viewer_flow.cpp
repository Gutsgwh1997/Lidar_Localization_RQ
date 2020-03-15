/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/mapping/viewer/viewer_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
ViewerFlow::ViewerFlow(ros::NodeHandle& nh) {
    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 100000);
    transformed_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/transformed_odom", 100000);
    optimized_key_frames_sub_ptr_ = std::make_shared<KeyFramesSubscriber>(nh, "/optimized_key_frames", 100000);
    // publisher
    optimized_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/optimized_odom", "map", "lidar", 100);
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "map", 100);
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "map", 100);
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "map", 100);
    // viewer
    viewer_ptr_ = std::make_shared<Viewer>();
}

bool ViewerFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;
        viewer_ptr_->UpdateWithNewKeyFrame(key_frame_buff_,current_transformed_odom_,current_cloud_data_);
        PublishLocalData();
    }
    if(optimized_key_frames_.size()>0){
        viewer_ptr_->UpdateWithOptimizedKeyFrames(optimized_key_frames_);
        PublishGlobalData();
    }

    return true;
}

bool ViewerFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    transformed_odom_sub_ptr_->ParseData(transformed_odom_buff_);
    key_frame_sub_ptr_->ParseData(key_frame_buff_);
    optimized_key_frames_sub_ptr_->ParseData(optimized_key_frames_);

    return true;
}

/**
 * @brief 点云与点云的姿态，不知道为何不判定key_frame_buff_
 *
 * @return 
 */
bool ViewerFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (transformed_odom_buff_.size() == 0)
        return false;

    return true;
}

bool ViewerFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_transformed_odom_ = transformed_odom_buff_.front();

    double diff_odom_time = current_cloud_data_.time - current_transformed_odom_.time;

    if (diff_odom_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_odom_time > 0.05) {
        transformed_odom_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    transformed_odom_buff_.pop_front();

    return true;
}

/**
 * @brief 接收当前帧的点云和位姿，按照优化后的位姿(非lidar_odom前端)投影
 *
 * @param new_key_frames 从后端输出的与GNSS对齐的lidar_odom关键帧位姿
 * @param optimized_key_frames 从后端接收的优化后的位姿
 * @param transformed_data 当前帧的位姿
 * @param cloud_data       当前帧的点云
 *
 * @return 
 */
// bool ViewerFlow::UpdateViewer() {
//     return viewer_ptr_->Update(key_frame_buff_, 
//                                optimized_key_frames_, 
//                                current_transformed_odom_, 
//                                current_cloud_data_);
// }

/**
 * @brief 发布当前帧的位姿（投影到已经优化的轨迹上去）
 *        若有接收者，发布局部地图和全局地图
 *
 * @return 
 */
// bool ViewerFlow::PublishData() {
//     optimized_odom_pub_ptr_->Publish(viewer_ptr_->GetCurrentPose());
//     current_scan_pub_ptr_->Publish(viewer_ptr_->GetCurrentScan());

//     if (viewer_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers()) {
//         CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
//         viewer_ptr_->GetLocalMap(cloud_ptr);
//         local_map_pub_ptr_->Publish(cloud_ptr);
//     }

//     if (viewer_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
//         CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
//         viewer_ptr_->GetGlobalMap(cloud_ptr);
//         global_map_pub_ptr_->Publish(cloud_ptr);
//     }

//     return true;
// }

/**
 * @brief 发布当前帧的位姿（投影到已经优化的轨迹上去）
 *        若有接收者，发布局部地图
 *
 * @return 
 */
bool ViewerFlow::PublishLocalData() {
    optimized_odom_pub_ptr_->Publish(viewer_ptr_->GetCurrentPose());
    current_scan_pub_ptr_->Publish(viewer_ptr_->GetCurrentScan());
    if (viewer_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        viewer_ptr_->GetLocalMap(cloud_ptr);
        local_map_pub_ptr_->Publish(cloud_ptr);
    }

    return true;
}

/**
 * @brief 发布全局地图（每次优化完成之后，使用优化之后的位姿生成的）
 *
 * @return 
 */
bool ViewerFlow::PublishGlobalData() {
    if (viewer_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        viewer_ptr_->GetGlobalMap(cloud_ptr);
        global_map_pub_ptr_->Publish(cloud_ptr);
    }

    return true;
}

bool ViewerFlow::SaveMap() {
    return viewer_ptr_->SaveMap();
}
}
