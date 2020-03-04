/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_

#include <deque>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Dense>

#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/registration/ndt_registration.hpp"
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
    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);
   
    bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);   // 这几个函数中改变了display_filter_变量，故不能是const成员函数
    bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
    bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);
    bool SaveMap();
  
  private:
    bool UpdateNewFrame(const Frame& new_key_frame);
    bool InitWithConfig();
    bool InitParam(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML:Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML:Node& config_node);

  private:
   std::string data_path_;
   std::string key_frame_path_;

   std::shared_ptr<RegistrationInterface> registration_ptr_;
   std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;      // 对新帧点云滤波
   std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;  // 对于局部地图滤波;
   std::shared_ptr<CloudFilterInterface> display_filter_ptr_;    // GetNew...函数中，给显示用滤波;

   Frame current_frame_;
   std::deque<Frame> local_map_frames_;
   std::deque<Frame> global_map_frames_;
   CloudData::CLOUD_PTR local_map_ptr_;  // 他们实际上都是智能指针，不必担心内存泄漏问题
   CloudData::CLOUD_PTR global_map_ptr_;
   CloudData::CLOUD_PTR result_cloud_ptr_;  // 每一帧点云匹配变换后的结果

   bool has_new_local_map_   = false;
   bool has_new_global_map_  = false;
   float key_frame_distance_ = 2.0;
   int local_frame_num_      = 20;
  
   Eigen::Matrix4f init_pose_    = Eigen::Matrix4f::Identity();
   Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();  // 暂时没有用到
};
}

#endif
