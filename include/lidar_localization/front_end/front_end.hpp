/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_

#include <deque>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
    /**
     * @brief 构造后先设定InitPose()，之后调用Update便可以获得位姿态
     */
class FrontEnd {
  public:
    class Frame {  
      public:  
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

  public:
    FrontEnd();

    /**
     * @brief 计算当前帧点云的位姿Twl
     *
     * @param cloud_data 当前帧点云
     *
     * @return 相对于第一帧点云的位姿,首帧点云的位姿通过SetInitPose()设定
     */
    Eigen::Matrix4f Update(const CloudData& cloud_data);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool SetPredictPose(const Eigen::Matrix4f& predict_pose);

    bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);   // 这几个函数中改变了display_filter_变量，故不能是const成员函数
    bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
    bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);
  
  private:
    /**
     * @brief 更新局部地图与全局地图(全局地图的更新需要优化)
     *
     * @param new_key_frame 插入的新的帧
     */
    void UpdateNewFrame(const Frame& new_key_frame);

  private:
    pcl::VoxelGrid<CloudData::POINT> cloud_filter_;               // 对新帧点云滤波
    pcl::VoxelGrid<CloudData::POINT> local_map_filter_;           // 对于局部地图滤波
    pcl::VoxelGrid<CloudData::POINT> display_filter_;             // GetNew...函数中，给显示用滤波
    pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;  // ndt匹配方法的对象

    std::deque<Frame> local_map_frames_;
    std::deque<Frame> global_map_frames_;

    bool has_new_local_map_ = false;
    bool has_new_global_map_ = false;
    CloudData::CLOUD_PTR local_map_ptr_;     // 他们实际上都是智能指针，不必担心内存泄漏问题
    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR result_cloud_ptr_;  // 每一帧点云匹配变换后的结果
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();  // Tag4.0中没有使用到
};
}

#endif
