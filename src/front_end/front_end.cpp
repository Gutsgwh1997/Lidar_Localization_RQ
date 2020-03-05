/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "lidar_localization/front_end/front_end.hpp"

#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <boost/filesystem.hpp>
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

FrontEnd::FrontEnd()
    : local_map_ptr_(new CloudData::CLOUD())
    , global_map_ptr_(new CloudData::CLOUD())
    , result_cloud_ptr_(new CloudData::CLOUD()) {
    InitWithConfig();
}

/**
 * @brief 激光里程计（当前帧与局部地图NDT）
 *
 * @param cloud_data 当前帧点云
 * @param cloud_pose 估计的当前帧位姿
 *
 * @return 是否成功
 */
bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;  // 存储移除后的点原始的索引值
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);
    
    // 这里的静态变量很细节,静态变量的初始化只有一次
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;
        UpdateNewFrame(current_frame_);
        cloud_pose = current_frame_.pose;
        return true;
    }

    // 不是第一帧，就正常匹配
    registration_ptr_->ScanMatch(filtered_cloud_ptr,predict_pose,result_cloud_ptr_,current_frame_.pose);
    cloud_pose = current_frame_.pose;

    // 更新相邻两帧的相对运动,使用静态变量确实很不错
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
        UpdateNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }

    return true;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    if (has_new_local_map_) {
        display_filter_ptr_->Filter(local_map_ptr_,local_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    if (has_new_global_map_) {
        has_new_global_map_ = false;
        display_filter_ptr_->Filter(global_map_ptr_, global_map_ptr);
        global_map_ptr_.reset(new CloudData::CLOUD());
        return true;
    }
    return false;
}

bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr) {
    display_filter_ptr_->Filter(result_cloud_ptr_, current_scan_ptr);
    return true;
}

/**
 * @brief 将保存的关键帧地图拼凑城整个点云
 *
 * @return 是否成功
 */
bool FrontEnd::SaveMap(){
    global_map_ptr_.reset(new CloudData::CLOUD());
    CloudData::CLOUD_PTR key_frame_cloud_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

    for (size_t i = 0; i < global_map_frames_.size(); ++i) {
        std::string key_frame_path = key_frame_path_ + "/" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(key_frame_path, *key_frame_cloud_ptr);
        display_filter_ptr_->Filter(key_frame_cloud_ptr, key_frame_cloud_ptr);
        pcl::transformPointCloud(*key_frame_cloud_ptr, *transformed_cloud_ptr, global_map_frames_.at(i).pose);
        *global_map_ptr_ += *transformed_cloud_ptr;
    }

    std::string global_map_path = data_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(global_map_path, *global_map_ptr_);
    has_new_global_map_ = true;
    return true;
}

/**
 * @brief 更新局部地图，保存关键帧到硬盘
 *
 * @param new_key_frame 关键帧
 *
 * @return 是否成功
 */
bool FrontEnd::UpdateNewFrame(const Frame& new_key_frame) {
    // 把关键帧点云存储到硬盘中
    std::string key_frame_path = key_frame_path_ + "/" + std::to_string(global_map_frames_.size()) + ".pcd";
    pcl::io::savePCDFileBinary(key_frame_path, *new_key_frame.cloud_data.cloud_ptr);
    Frame key_frame = new_key_frame;

    // 这一步的目的是为了把关键帧的点云保存下来（深度拷贝）
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云(因为以current_frame_为参数,每次只改变*current_frame_.cloudptr)
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    
    // 更新局部地图
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_local_map_ = true;

    // 更新ndt匹配的目标点云
    if (local_map_frames_.size() < 10) {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_,filtered_local_map_ptr);
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    // 更新全局地图
    // 存储之前，点云先要释放
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD());
    global_map_frames_.push_back(key_frame);
    return true;
}

/**
 * @brief 总体初始部分，会读取配置文件，路径在这！
 *
 * @return 是否成功
 */
bool FrontEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node       = YAML::LoadFile(config_file_path);

    InitParam(config_node);
    InitDataPath(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("display", display_filter_ptr_, config_node);

    return true;
}

/**
 * @brief 设定基本的参数（关键帧选取距离，局部地图帧数量）
 *
 * @param config_node 工程配置文件
 *
 * @return 是否成功
 */
bool FrontEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_    = config_node["local_frame_num"].as<int>();

    return true;
}

/**
 * @brief 设定地图数据存放的文件夹
 *
 * @param config_node 工程的配置文件
 *
 * @return 是否成功
 */
bool FrontEnd::InitDataPath(const YAML::Node& config_node) {
    data_path_ = config_node["data_path"].as<std::string>();
    if (data_path_ == "./") {
        data_path_ = WORK_SPACE_PATH;
    }
    data_path_ += "/slam_data";

    if (boost::filesystem::is_directory(data_path_)) {
        boost::filesystem::remove_all(data_path_);
    }

    boost::filesystem::create_directory(data_path_);
    if (!boost::filesystem::is_directory(data_path_)) {
        LOG(WARNING) << "文件夹 " << data_path_ << " 未创建成功!";
        return false;
    } else {
        LOG(INFO) << "地图点云存放地址：" << data_path_;
    }

    key_frame_path_ = data_path_ + "/key_frames";
    boost::filesystem::create_directory(key_frame_path_);
    if (!boost::filesystem::is_directory(key_frame_path_)) {
        LOG(WARNING) << "文件夹 " << key_frame_path_ << " 未创建成功!";
        return false;
    } else {
        LOG(INFO) << "关键帧点云存放地址：" << key_frame_path_ << std::endl << std::endl;
    }

    return true;
}

/**
 * @brief 根据不同用途(filter_user)和配置文件(config_node)构造滤波器
 *
 * @param filter_user 不同用途，例如fram，display，local_map
 * @param filter_ptr  构造出来的滤波器指针
 * @param config_node 工程的总体配置文件
 *
 * @return 是否成功
 */
bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << filter_user << "选择的滤波方法为：" << filter_method << std::endl;

    if (filter_method == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_method << " 相对应的滤波方法！";
        return false;
    }
    return true;
}

/**
 * @brief 根据工程总体配置文件，构造点云匹配方法
 *
 * @param registration_ptr 获取的点云匹配方法的指针
 * @param config_node 工程的配置文件
 *
 * @return 是否成功
 */
bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node){
    std::string registration_method = config_node["registration_method"].as<std::string>();
    LOG(INFO)<<"选择的点云匹配方法为： "<<registration_method;

    if (registration_method == "NDT"){
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    }else{
        LOG(ERROR)<<"点云匹配方法 "<<registration_method<<"未找到！";
        return false;
    }
    return true;
}


}
