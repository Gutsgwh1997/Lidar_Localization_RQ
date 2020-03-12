/**
 * @file data_preprocessing.cpp
 * @brief 数据预处理的实现文件
 * @author GWH
 * @version 0.1
 * @date 2020-03-09
 */
#include "lidar_localization/data_preprocessing/data_preprocessing_flow.hpp"
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

namespace lidar_localization {
DataPreprocessFlow::DataPreprocessFlow(ros::NodeHandle& nh) {
    cloud_sub_ptr_      = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 10000);
    imu_sub_ptr_        = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 10000);
    gnss_sub_ptr_       = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 10000);         // gnss的经纬度相关信息
    velocity_sub_ptr_   = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 10000);      // oxts模块的线速度、角速度信息。
    lidar_to_imu_ptr_   = std::make_shared<TFListener>(nh, "imu_link", "velo_link");

    cloud_pub_ptr_      = std::make_shared<CloudPublisher>(nh, "/synced_cloud","map", 1000);
    gnss_pub_ptr_       = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "map", "lidar", 1000);

    distortion_adjust_ptr_ =  std::make_shared<DistortionAdjust>();
}

bool DataPreprocessFlow::Run(){
    if (!ReadData()) {
        LOG(INFO) << "ReadData process is not satisfied.";
        return false;
    }
    if (!InitCalibration()){
        LOG(INFO)<<"Transform matrix of Lidat_to_Imu received false";
        return false;
    }
    if (!InitGNSS()){
        LOG(INFO)<<"InitGNSS false";
        return false;
    }
    while (HasData()) {
        if (!ValidData())
            continue;

        TransformData();
        LOG(INFO)<<"--------------------------当前帧去畸变完成---------------------------";
        PublishData();
        LOG(INFO)<<"--------------------发布经过时间同步以及去掉畸变的点云-------------------";
    }

    return true;
}

bool DataPreprocessFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<GNSSData> unsynced_gnss_;
    static std::deque<VelocityData> unsynced_velocity_;

    imu_sub_ptr_->ParseData(unsynced_imu_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);
    velocity_sub_ptr_->ParseData(unsynced_velocity_);

    if (cloud_data_buff_.size() == 0) {
        LOG(INFO) << "cloud_data_buff中没有缓存的数据";
        return false;
    }

    double cloud_time   = cloud_data_buff_.front().time;
    bool valid_imu      = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    bool valid_gnss     = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);
    bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);

    static bool init_synced = false;
    if (!init_synced) {
        if (!valid_imu || !valid_gnss || !valid_velocity) {
            cloud_data_buff_.pop_front();
            LOG(INFO) << "匹配不到对应时间戳的IMU、GNSS or Velocity，丢掉cloud, cloud_data_buff中还有 "
                      << cloud_data_buff_.size() << " 帧缓存";
            init_synced = true;
            return false;
        }
    }

    return true;
}

bool DataPreprocessFlow::InitCalibration(){
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
            LOG(INFO) << "Lidat to IMU transform is reveived :" << std::endl << lidar_to_imu_;
        }
    }
    return calibration_received;
}

bool DataPreprocessFlow::InitGNSS(){
    static bool gnss_inited =false;
    if (!gnss_inited && gnss_data_buff_.size()>0){
        GNSSData first_gnss_data = gnss_data_buff_.front();
        first_gnss_data.InitOriginPosition();
        LOG(INFO)<<"GNSS origin is inited.";
        gnss_inited =true;
    }

    return gnss_inited;
}

bool DataPreprocessFlow::HasData() {
    if (cloud_data_buff_.size() == 0) {
        LOG(INFO) << "There are no data in cloud_data_buff_";
        return false;
    }
    if (imu_data_buff_.size() == 0) {
        LOG(INFO) << "There are no data in imu_data_buff_";
        return false;
    }
    if (gnss_data_buff_.size() == 0) {
        LOG(INFO) << "There are no data in gnss_data_buff_";
        return false;
    }
    if (velocity_data_buff_.size() == 0) {
        LOG(INFO) << "There are no data in velocity_data_buff_";
        return false;
    }
    LOG(INFO)<<"There are data in all buff";
    return true;
}

// 做了时间同步之后就不需要检验这一步了，同步后时间戳是对齐的
bool DataPreprocessFlow::ValidData() {
    current_cloud_data_    = cloud_data_buff_.front();
    current_imu_data_      = imu_data_buff_.front();
    current_gnss_data_     = gnss_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();

    double diff_imu_time      = current_cloud_data_.time - current_imu_data_.time;
    double diff_gnss_time     = current_cloud_data_.time - current_gnss_data_.time;
    double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;

    if ( diff_imu_time < -0.05 || diff_gnss_time < -0.05 || diff_velocity_time < -0.05){
        LOG(INFO)<<"GNSSData、IMUData or VelocityData is too much later.";
        cloud_data_buff_.pop_front();
        return false;
    }
    if (diff_imu_time > 0.05){
        LOG(INFO)<<"CloudData is too much later than IMUData.";
        imu_data_buff_.pop_front();
        return false;
    }
    if (diff_gnss_time > 0.05){
        LOG(INFO)<<"CloudData is too much later than GNSSData.";
        gnss_data_buff_.pop_front();
        return false;
    }
    if (diff_velocity_time > 0.05){
        LOG(INFO)<<"CloudData is too much later than VelocityData.";
        velocity_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    velocity_data_buff_.pop_front();

    LOG(INFO)<<"Data is valid!";
    return true;
}

bool DataPreprocessFlow::TransformData(){
    gnss_pose_ = Eigen::Matrix4f::Identity();

    // GNSS的真实位姿态变换为Lidar的
    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();

    gnss_pose_ *= lidar_to_imu_; // 转换成雷达的位姿
  
    // 点云去掉畸变，只需要点云时刻的Lidar的线速度、角速度
    // static int count = 0;
    // std::string key_frame_path_before = "/home/gwh/Desktop/key_frame/before_distortion";
    // std::string key_frame_path_after = "/home/gwh/Desktop/key_frame/after_distortion";
    // pcl::io::savePCDFileBinary(key_frame_path_before+"/"+ std::to_string(count) + ".pcd", *current_cloud_data_.cloud_ptr);

    current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    // distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, distortioned_cloud_data_.cloud_ptr);

    // pcl::io::savePCDFileBinary(key_frame_path_after+"/"+ std::to_string(count) +".pcd", *distortioned_cloud_data_.cloud_ptr);
    // count++;
    return true;
}

bool DataPreprocessFlow::PublishData() {
    cloud_pub_ptr_->Publish(distortioned_cloud_data_.cloud_ptr, current_cloud_data_.time);
    // cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time);
    return true;
}
} //namespace lidar_localization
