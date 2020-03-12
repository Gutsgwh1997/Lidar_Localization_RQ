/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-29 03:49:12
 */
#include "lidar_localization/mapping/viewer/viewer.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
Viewer::Viewer() {
    InitWithConfig();
}

bool Viewer::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/viewer/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    InitParam(config_node);
    InitDataPath(config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("global_map", global_map_filter_ptr_, config_node);

    return true;
}

bool Viewer::InitParam(const YAML::Node& config_node) {
    local_frame_num_ = config_node["local_frame_num"].as<int>();
    return true;
}

bool Viewer::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if (data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }

    key_frames_path_ = data_path + "/slam_data/key_frames";
    map_path_ = data_path + "/slam_data/map";

    if (!filemanager::InitDirectory(map_path_, "点云地图文件"))
        return false;

    return true;
}

bool Viewer::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << "viewer_" + filter_user << "选择的滤波方法为：" << filter_mothod;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        return false;
    }

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
bool Viewer::Update(std::deque<KeyFrame>& new_key_frames, std::deque<KeyFrame>& optimized_key_frames,
                    PoseData transformed_data, CloudData cloud_data) {
    ResetParam();

    if (optimized_key_frames.size() > 0) {
        optimized_key_frames_ = optimized_key_frames;
        optimized_key_frames.clear();
        OptimizeKeyFrames();
        has_new_global_map_ = true;
    }

    if (new_key_frames.size()) {
        all_key_frames_.insert(all_key_frames_.end(), new_key_frames.begin(), new_key_frames.end());
        new_key_frames.clear();
        has_new_local_map_ = true;
    }

    optimized_odom_ = transformed_data;
    optimized_odom_.pose = pose_to_optimize_ * optimized_odom_.pose;

    optimized_cloud_ = cloud_data;
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *optimized_cloud_.cloud_ptr, optimized_odom_.pose);

    return true;
}

void Viewer::ResetParam() {
    has_new_local_map_ = false;
    has_new_global_map_ = false;
}

/**
 * @brief 根据优化后的位姿修正全局位姿态
 * @detail 索引一致的关键帧，修改全局位姿就是优化后的位姿
 *         其他的在全局位姿中的关键帧按照优化后的最后一帧投影过来
 * @return 
 */
bool Viewer::OptimizeKeyFrames() {
    size_t optimized_index = 0;
    size_t all_index = 0;
    while (optimized_index < optimized_key_frames_.size() && all_index < all_key_frames_.size()) {
        if (optimized_key_frames_.at(optimized_index).index < all_key_frames_.at(all_index).index) {
            optimized_index ++;
        } else if (optimized_key_frames_.at(optimized_index).index > all_key_frames_.at(all_index).index) { 
            all_index ++;    // 这里按照C++语法是无意义的，实际上我觉得不会出现这种状况，但是这样改动出现了轨迹的不连续
        } else {
            pose_to_optimize_ = optimized_key_frames_.at(optimized_index).pose * all_key_frames_.at(all_index).pose.inverse();
            all_key_frames_.at(all_index) = optimized_key_frames_.at(optimized_index);
            optimized_index ++;
            all_index ++;
        }
    }

    // 已经优化的关键帧位姿
    // 全部关键帧的位姿
    // 全部关键帧的位姿按照最后一个已经优化的位姿拽过来。
    while (all_index < all_key_frames_.size()) {
        all_key_frames_.at(all_index).pose = pose_to_optimize_ * all_key_frames_.at(all_index).pose;
        all_index ++;
    }

    return true;
}

/**
 * @brief 对已经优化完成的关键帧拼接点云地图
 *
 * @param global_map_ptr 点云地图的ptr返回值
 *
 * @return 
 */
bool Viewer::JointGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    JointCloudMap(optimized_key_frames_, global_map_ptr);
    return true;
}

/**
 * @brief 局部地图（从当前all_key_frames向前20帧生成
 *
 * @param local_map_ptr 局部地图返回ptr
 *
 * @return 
 */
bool Viewer::JointLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    size_t begin_index = 0;
    if (all_key_frames_.size() > (size_t)local_frame_num_)
        begin_index = all_key_frames_.size() - (size_t)local_frame_num_;

    std::deque<KeyFrame> local_key_frames;
    for (size_t i = begin_index; i < all_key_frames_.size(); ++i)
        local_key_frames.push_back(all_key_frames_.at(i));

    JointCloudMap(local_key_frames, local_map_ptr);
    return true;
}

/**
 * @brief 从硬盘中读取关键帧点云，根据key_frames变换到世界坐标系统
 *
 * @param key_frames 主要存储关键帧的索引和位姿信息
 * @param map_cloud_ptr 转换后拼接在一块的点云
 *
 * @return  是否成功
 */
bool Viewer::JointCloudMap(const std::deque<KeyFrame>& key_frames, CloudData::CLOUD_PTR& map_cloud_ptr) {
    map_cloud_ptr.reset(new CloudData::CLOUD());

    CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
    std::string file_path = "";

    for (size_t i = 0; i < key_frames.size(); ++i) {
        file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames.at(i).index) + ".pcd";
        pcl::io::loadPCDFile(file_path, *cloud_ptr);
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, key_frames.at(i).pose);
        *map_cloud_ptr += *cloud_ptr;
    }
    return true;
}

/**
 * @brief 保存optimized_key_frames的地图到硬盘
 *
 * @return 
 */
bool Viewer::SaveMap() {
    if (optimized_key_frames_.size() == 0)
        return false;

    CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
    JointGlobalMap(global_map_ptr);

    std::string map_file_path = map_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr);

    LOG(INFO) << "地图保存完成，地址是：" << std::endl << map_file_path << std::endl << std::endl;

    return true;
}

/**
 * @brief 当前帧点云按照优化后的位姿投影之后的位姿
 *
 * @return 
 */
Eigen::Matrix4f& Viewer::GetCurrentPose() {
    return optimized_odom_.pose;
}

/**
 * @brief 获取当前帧按照优化后的位姿投影后的点云
 *
 * @return 
 */
CloudData::CLOUD_PTR& Viewer::GetCurrentScan() {
    frame_filter_ptr_->Filter(optimized_cloud_.cloud_ptr, optimized_cloud_.cloud_ptr);
    return optimized_cloud_.cloud_ptr;
}

/**
 * @brief 获取局部点云地图
 *
 * @param local_map_ptr 返回的指向局部点云地图的指针
 *
 * @return 
 */
bool Viewer::GetLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    JointLocalMap(local_map_ptr);
    local_map_filter_ptr_->Filter(local_map_ptr, local_map_ptr);
    return true;
}

/**
 * @brief 获取全局地图
 *
 * @param global_map_ptr 返回的指向全局地图的指针
 *
 * @return 
 */
bool Viewer::GetGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    JointGlobalMap(global_map_ptr);
    global_map_filter_ptr_->Filter(global_map_ptr, global_map_ptr);
    return true;
}

bool Viewer::HasNewLocalMap() {
    return has_new_local_map_;
}

bool Viewer::HasNewGlobalMap() {
    return has_new_global_map_;
}
}
