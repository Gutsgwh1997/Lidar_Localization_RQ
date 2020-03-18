/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "lidar_localization/matching/matching.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/models/registration/ndt_registration.hpp"
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"

namespace lidar_localization {
Matching::Matching()
    :local_map_ptr_(new CloudData::CLOUD()),
     global_map_ptr_(new CloudData::CLOUD()),
     current_scan_ptr_(new CloudData::CLOUD()) {
    
    InitWithConfig();

    InitGlobalMap();

    ResetLocalMap(0.0, 0.0, 0.0);
}

bool Matching::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/matching/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------地图定位初始化-------------------" << std::endl;
    InitDataPath(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("global_map", global_map_filter_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);
    InitBoxFilter(config_node);

    return true;
}

// 从哪里读取全局地图
bool Matching::InitDataPath(const YAML::Node& config_node) {
    map_path_ = config_node["map_path"].as<std::string>();
    return true;
}

// 初始化点云匹配方法
bool Matching::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "地图匹配选择的点云匹配方式为：" << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
        return false;
    }

    return true;
}

// global_map;local_map;frame选择的滤波方法
bool Matching::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "地图匹配" << filter_user << "选择的滤波方法为：" << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        return false;
    }

    return true;
}

// 初始化box的filter（之后SetOrigin才用意义）
bool Matching::InitBoxFilter(const YAML::Node& config_node) {
    box_filter_ptr_ = std::make_shared<BoxFilter>(config_node);
    return true;
}

//读取保存好的点云地图并且进行滤波
bool Matching::InitGlobalMap() {
    pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
    LOG(INFO) << "load global map size:" << global_map_ptr_->points.size();

    // 这里使用局部滤波器，这样不用每次分割之后在局部滤波
    local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
    LOG(INFO) << "filtered global map size:" << global_map_ptr_->points.size();

    has_new_global_map_ = true;

    return true;
}

// 根据新的Box中心位置过滤点云地图
bool Matching::ResetLocalMap(float x, float y, float z) {
    std::vector<float> origin = {x, y, z};
    box_filter_ptr_->SetOrigin(origin);
    box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

    registration_ptr_->SetInputTarget(local_map_ptr_);

    has_new_local_map_ = true;

    std::vector<float> edge = box_filter_ptr_->GetEdge();
    LOG(INFO) << "new local map:" << edge.at(0) << ","
                                  << edge.at(1) << ","
                                  << edge.at(2) << ","
                                  << edge.at(3) << ","
                                  << edge.at(4) << ","
                                  << edge.at(5) << std::endl << std::endl;

    return true;
}

// 在调用此函数之前需要先SetGNSsPose();
// 得到的位姿是相对于设定的初始帧的也就是第一帧GNSS的位姿
bool Matching::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, indices);

    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;

    if (!has_inited_) {
        predict_pose = current_gnss_pose_;
    }

    // 与地图匹配
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, cloud_pose);
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *current_scan_ptr_, cloud_pose);

    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * cloud_pose;
    predict_pose = cloud_pose * step_pose;
    last_pose = cloud_pose;

    // 匹配之后判断是否需要更新局部地图
    std::vector<float> edge = box_filter_ptr_->GetEdge();
    for (int i = 0; i < 3; i++) {
        if (fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
            fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0)
            continue;  // 不需要更新局部点云
        ResetLocalMap(cloud_pose(0,3), cloud_pose(1,3), cloud_pose(2,3));
        break;
    }

    return true;
}

// 设定GNSS的初始位姿态(确定最开始去哪里分割点云)
bool Matching::SetGNSSPose(const Eigen::Matrix4f& gnss_pose) {
    current_gnss_pose_ = gnss_pose;

    static int gnss_cnt = 0;
    if (gnss_cnt < 3) {         // 多初始化几次，应该更加鲁棒吧
        SetInitPose(gnss_pose);
    } else if (gnss_cnt > 3) {
        has_inited_ = true;
    }
    gnss_cnt ++;
    return true;
}

bool Matching::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    ResetLocalMap(init_pose(0,3), init_pose(1,3), init_pose(2,3));

    return true;
}

// 使用global_map_filter_ptr进行二次滤波，显示的时候不消耗资源
void Matching::GetGlobalMap(CloudData::CLOUD_PTR& global_map) {
    global_map_filter_ptr_->Filter(global_map_ptr_, global_map);
    has_new_global_map_ = false;
}

// 使用Box分割得到的局部点云，和global_map_同一个分辨率（使用local_map_filter过滤的)
CloudData::CLOUD_PTR& Matching::GetLocalMap() {
    return local_map_ptr_;
}

// 这个current_scan是没有经过任何滤波的
CloudData::CLOUD_PTR& Matching::GetCurrentScan() {
    return current_scan_ptr_;
}

bool Matching::HasInited() {
    return has_inited_;
}

bool Matching::HasNewGlobalMap() {
    return has_new_global_map_;
}

bool Matching::HasNewLocalMap() {
    return has_new_local_map_;
}
}