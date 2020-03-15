/**
 * @file back_end.cpp
 * @brief back end 具体实现
 * @author Ren Qian
 * @version 0.1
 * @date 2020-03-10 09:21:13
 */
#include "lidar_localization/mapping/back_end/back_end.hpp"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tools/file_manager.hpp"

namespace lidar_localization {
BackEnd::BackEnd() {
    InitWithConfig();
}

/**
 * @brief 此类的总体Init部分，根据配置文件配置对应参数
 *
 * @return 
 */
bool BackEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/back_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    InitParam(config_node);
    InitDataPath(config_node);
    InitGraphOptimizer(config_node);

    return true;
}

bool BackEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    return true;
}

/**
 * @brief 根据配置文件设定g2o优化相关的参数
 *
 * @param config_node 配置文件back_end/config.yaml
 *
 * @return 
 */
bool BackEnd::InitGraphOptimizer(const YAML::Node& config_node){
    std::string graph_optimizer_type = config_node["graph_optimizer_type"].as<std::string>();
    if(graph_optimizer_type == "g2o"){
        graph_optimizer_ptr_ = std::make_shared<G2oGraphOptimizer>("lm_var");
    }else{
        LOG(ERROR)<<"没有找到与"<<graph_optimizer_type<<"对应的图优化模式，检查配置文件";
        return false;
    }
    LOG(INFO)<<"后端选择的优化器为: "<<graph_optimizer_type<<std::endl<<std::endl;

    graph_optimizer_config_.use_gnss = config_node["use_gnss"].as<bool>();
    graph_optimizer_config_.use_loop_close = config_node["use_loop_close"].as<bool>();

    graph_optimizer_config_.optimize_step_with_key_frame= config_node["optimize_step_with_key_frame"].as<int>();
    graph_optimizer_config_.optimize_step_with_gnss = config_node["optimize_step_with_gnss"].as<int>();
    graph_optimizer_config_.optimize_step_with_loop = config_node["optimize_step_with_loop"].as<int>();

    for (int i = 0; i < 6; ++i) {
        graph_optimizer_config_.odom_edge_noise(i) =
            config_node[graph_optimizer_type + "_param"]["odom_edge_noise"][i].as<double>();
        graph_optimizer_config_.close_loop_noise(i) =
            config_node[graph_optimizer_type + "_param"]["close_loop_noise"][i].as<double>();
    }
    for (int i = 0; i < 3; i++) {
        graph_optimizer_config_.gnss_noise(i) =
            config_node[graph_optimizer_type + "_param"]["gnss_noise"][i].as<double>();
    }

    return true;
}

/**
 * @brief 根据配置文件中的设定，创建关键帧点云和轨迹的保存文件
 *
 * @param config_node back_end/config.yaml
 *
 * @return 
 */
bool BackEnd::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if (data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }

    if (!filemanager::InitDirectory(data_path + "/slam_data", "slam_data存储路径"))
        return false;

    key_frames_path_ = data_path + "/slam_data/key_frames";
    trajectory_path_ = data_path + "/slam_data/trajectory";

    if (!filemanager::InitDirectory(key_frames_path_, "关键帧点云"))
        return false;
    if (!filemanager::InitDirectory(trajectory_path_, "轨迹文件"))
        return false;

    if (!filemanager::CreateFile(ground_truth_ofs_, trajectory_path_ + "/ground_truth.txt"))
        return false;
    if (!filemanager::CreateFile(laser_odom_ofs_, trajectory_path_ + "/laser_odom.txt"))
        return false;

    return true;
}

bool BackEnd::Update(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_pose) {
    ResetParam();

    SaveTrajectory(laser_odom, gnss_pose);

    if (MaybeNewKeyFrame(cloud_data, laser_odom)) {
        AddNodeAndEdge(gnss_pose);
        MaybeOptimized();
    }

    return true;
}

/**
 * @brief 当bag文件播放完之后，关键帧里的后面几帧可能并没有得到优化,
 *        因为上面三个条件计数还没达到新一次优化所需的数量要求
 * @return 
 */
bool BackEnd::ForceOptimize(){
    if(graph_optimizer_ptr_->Optimize())
        has_new_optimized_ = true;
    return has_new_optimized_;
}

void BackEnd::ResetParam() {
    has_new_key_frame_ = false;
    has_new_optimized_ = false;
}

/**
 * @brief 将laser和gnss的轨迹保存到硬盘中
 *
 * @param laser_odom
 * @param gnss_pose
 *
 * @return 
 */
bool BackEnd::SaveTrajectory(const PoseData& laser_odom, const PoseData& gnss_pose) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {

            ground_truth_ofs_ << gnss_pose.pose(i, j);
            laser_odom_ofs_ << laser_odom.pose(i, j);
            
            if (i == 2 && j == 3) {
                ground_truth_ofs_ << std::endl;
                laser_odom_ofs_ << std::endl;
            } else {
                ground_truth_ofs_ << " ";
                laser_odom_ofs_ << " ";
            }
        }
    }

    return true;
}

/**
 * @brief 为顶点添加约束
 *
 * @param gnss_data 当前帧对应的/synced_gnss
 *
 * @return 
 */
bool BackEnd::AddNodeAndEdge(const PoseData& gnss_data) {
    Eigen::Isometry3d isometry;
    // 添加关键帧顶点
    // 下面这个函数matrix() return a writable expression of the transformation matrix
    isometry.matrix() = current_key_frame_.pose.cast<double>();
    graph_optimizer_ptr_->AddSe3Node(isometry, false);
    ++new_key_frame_cnt_;

    // 添加激光里程计对应的边
    static KeyFrame last_key_frame = current_key_frame_;
    int node_num = graph_optimizer_ptr_->GetNodeNum();
    if (node_num > 1) {
        Eigen::Matrix4f relative_pose = last_key_frame.pose.inverse() * current_key_frame_.pose;
        isometry.matrix() = relative_pose.cast<double>();
        graph_optimizer_ptr_->AddSe3Edge(node_num - 2, node_num - 1, isometry, graph_optimizer_config_.odom_edge_noise);
    }
    last_key_frame = current_key_frame_;

    // 添加Gnss位置（xyz）对应的先验边
    if (graph_optimizer_config_.use_gnss){
        Eigen::Vector3d xyz(static_cast<double>(gnss_data.pose(0,3)),
                            static_cast<double>(gnss_data.pose(1,3)),
                            static_cast<double>(gnss_data.pose(2,3)));
        graph_optimizer_ptr_->AddSe3PriorXYZEdge(node_num-1, xyz, graph_optimizer_config_.gnss_noise);
        ++new_gnss_cnt_;
    }

    return true;
}

/**
 * @brief 目前完成的功能是根据距离识别关键帧，并将关键帧存到硬盘中
 *
 * @param cloud_data 当前帧的点云
 * @param laser_odom 当前帧的激光里程计位姿
 *
 * @return 
 */
bool BackEnd::MaybeNewKeyFrame(const CloudData& cloud_data, const PoseData& laser_odom) {
    // static bool first_key_frame = true;   // 这个写法后来不会保存第一帧
    // if (first_key_frame){
    //    last_key_pose_ = laser_odom.pose; 
    //    first_key_frame = false;
    // }

    if (key_frames_deque_.size() == 0) {     // 这一块的目的是保存第一帧
        has_new_key_frame_ = true;
        last_key_pose_ = laser_odom.pose;
    }

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(laser_odom.pose(0,3) - last_key_pose_(0,3)) + 
        fabs(laser_odom.pose(1,3) - last_key_pose_(1,3)) +
        fabs(laser_odom.pose(2,3) - last_key_pose_(2,3)) > key_frame_distance_) {

        has_new_key_frame_ = true;
        last_key_pose_ = laser_odom.pose;
    }
    if (has_new_key_frame_) {
        // 把关键帧点云存储到硬盘里
        std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames_deque_.size()) + ".pcd";
        pcl::io::savePCDFileBinary(file_path, *cloud_data.cloud_ptr);

        KeyFrame key_frame;
        key_frame.time = laser_odom.time;
        key_frame.index = (unsigned int)key_frames_deque_.size();
        key_frame.pose = laser_odom.pose;
        key_frames_deque_.push_back(key_frame);

        current_key_frame_ = key_frame;

    }

    return has_new_key_frame_;
}

/**
 * @brief 当Gnss,key_frames满足要求时候开始优化
 *
 * @return 返回是否成功
 */
bool BackEnd::MaybeOptimized() {
    bool need_optimize = false;

    if (new_gnss_cnt_ >= graph_optimizer_config_.optimize_step_with_gnss) 
        need_optimize = true;
    if (new_key_frame_cnt_ >= graph_optimizer_config_.optimize_step_with_key_frame) 
        need_optimize = true;
    if (new_loop_cnt_ >= graph_optimizer_config_.optimize_step_with_loop) 
        need_optimize = true;

    if (!need_optimize) 
        return false;

    new_gnss_cnt_ = 0;
    new_loop_cnt_ = 0;
    new_key_frame_cnt_ = 0;

    if (graph_optimizer_ptr_->Optimize()) 
        has_new_optimized_ = true;

    return true;
}

/**
 * @brief 获取存储关键帧位姿的deque
 *
 * @param key_frames_deque 返回获取到的deque
 */
void BackEnd::GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque) {
    key_frames_deque.clear();
    if(graph_optimizer_ptr_->GetNodeNum() > 0){
        std::deque<Eigen::Matrix4f> optimized_pose;
        graph_optimizer_ptr_->GetOptimizedPose(optimized_pose);
        KeyFrame key_frame;
        for (size_t i = 0; i<optimized_pose.size(); ++i){
            key_frame.pose = optimized_pose.at(i);
            key_frame.index = (unsigned int)i;
            key_frames_deque.push_back(key_frame);
        }
    }
}

bool BackEnd::HasNewKeyFrame() {
    return has_new_key_frame_;
}

bool BackEnd::HasNewOptimized() {
    return has_new_optimized_;
}

/**
 * @brief 获取最新的关键帧
 *
 * @param key_frame 返回的最新的关键帧
 */
void BackEnd::GetLatestKeyFrame(KeyFrame& key_frame) {
    key_frame = current_key_frame_;
}
}
