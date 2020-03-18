/**
 * @file read_trajectory.cpp
 * @brief 将保存好的关键帧和轨迹读取出来，拼接点云
 * @author GWH
 * @version 0.1
 * @date 2020-03-18 18:35:41
 */
#include <string>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/tools/tic_toc.hpp"

using namespace lidar_localization;

int main(){
    TicToc time_clock;
    std::cout<<"开始读取关键帧和轨迹位姿拼接点云"<<std::endl;

    std::string config_file_path = WORK_SPACE_PATH + "/config/loop_closing/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::string data_path = config_node["data_path"].as<std::string>();
    std::string key_frames_path = data_path + "/slam_data/key_frames";
    std::string optimized_pose_path = data_path + "/slam_data/trajectory/optimized.txt";
    std::string map_save_path = data_path + "/slam_data/map";

    std::vector<Eigen::Matrix4f> optimized_poses;
    filemanager::ReadTrajectory(optimized_pose_path, optimized_poses);  // 读取到了轨迹信息

    CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR map_cloud_ptr(new CloudData::CLOUD());
    std::string file_path = "";

    for (size_t i = 0; i < optimized_poses.size(); ++i) {
        file_path = key_frames_path + "/key_frame_" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(file_path, *cloud_ptr);
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, optimized_poses.at(i));
        *map_cloud_ptr += *cloud_ptr;
        std::cout<<"拼接完成第 "<<i<<" 帧关键帧"<<std::endl;
    }

    std::cout<<"开始体素滤波"<<std::endl;
    float leaf_size[3] = {0.4,0.4,0.2};
    auto filter_ptr = std::make_shared<VoxelFilter>(leaf_size[0],leaf_size[1],leaf_size[2]);
    filter_ptr->Filter(map_cloud_ptr, map_cloud_ptr);
    std::cout<<"滤波完成"<<std::endl;

    pcl::io::savePCDFileBinary(map_save_path + "/synthetic_map.pcd", *map_cloud_ptr);
    std::cout<<"点云拼接完成，一共耗时 "<<time_clock.toc()<<" 秒"<<std::endl;

    return 0;

}
