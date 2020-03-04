/**
 * @file front_end_flow.cpp
 * @brief 任务管理类的实现
 * @author GWH
 * @version 0.1
 * @date 2020-03-04
 */
#include "lidar_localization/front_end/front_end_flow.hpp"
#include "glog/logging.h"

namespace lidar_localization {

FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    cloud_sub_ptr_      = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 10000);
    imu_sub_ptr_        = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 10000);
    gnss_sub_ptr_       = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 10000);
    lidar_to_imu_ptr_   = std::make_shared<TFListener>(nh, "velo_link", "imu_link");

    cloud_pub_ptr_      = std::make_shared<CloudPublisher>(nh, "current_scan", 10000, "/map");
    local_map_pub_ptr_  = std::make_shared<CloudPublisher>(nh, "local_map", 10000, "/map");
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", 10000, "/map");
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100000); // topic,base_frame,child_frame
    gnss_pub_ptr_       = std::make_shared<OdometryPublisher>(nh, "gnss", "map", "lidar", 100000);

    front_end_ptr_ = std::make_shared<FrontEnd>();
    local_map_ptr_.reset(new CloudData::CLOUD());
    global_map_ptr_.reset(new CloudData::CLOUD());
    current_scan_ptr_.reset(new CloudData::CLOUD());
}

bool FrontEndFlow::Run(){
    // ReadData();
    // if (!InitCalibration())
    //     return false;
    // while(HasData()){
    //     if (!ValidData())
    //         continue;
    //     InitGNSS();
    //     UpdateGNSSOdometry();
    //     if (UpdateLaserOdometry())
    //         PublishData();
    // }

    // return true;
    ReadData();

    if (!InitCalibration()) return false;

    if (!InitGNSS()) return false;  // GNSS的初始化不必等到所有的数据都满足条件
                                    // 只要某一帧的激光轨迹与GNSS的对其就行，不必非要第一帧开始对其。
    while (HasData()) {
        if (!ValidData()) continue;
        UpdateGNSSOdometry();
        if (UpdateLaserOdometry()) PublishData();
    }

    return true;
}

bool FrontEndFlow::SaveMap(){
    return front_end_ptr_->SaveMap();
}

bool FrontEndFlow::PublishGlobalMap() {
    if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_)) {
        global_map_pub_ptr_->Publish(global_map_ptr_);
        global_map_ptr_.reset(new CloudData::CLOUD());
        return true;
    }
    return false;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(imu_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);

    return true;
}

bool FrontEndFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
            LOG(INFO)<<"Lidat to IMU transform is reveived :"<<std::endl<<lidar_to_imu_;
        }
    }
    return calibration_received;
}

bool FrontEndFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited && gnss_data_buff_.size() > 0) {
        GNSSData first_gnss_data = gnss_data_buff_.front();
        // GNSSData first_gnss_data = first_valid_gnss_data_;  // 注意初始化时机
        first_gnss_data.InitOriginPosition();
        LOG(INFO)<<"GNSS origin is inited.";
        gnss_inited = true;
    }
    return gnss_inited;
}

bool FrontEndFlow::HasData() {
    if ( cloud_data_buff_.size()>0 && imu_data_buff_.size()>0 && gnss_data_buff_.size()>0)
        return true;
    else
        LOG(ERROR)<<"There is no data in buff!";
        return false;
}

bool FrontEndFlow::ValidData() {
    static bool is_first_gnss_data = true;
    current_imu_data_   = imu_data_buff_.front();
    current_gnss_data_  = gnss_data_buff_.front();
    current_cloud_data_ = cloud_data_buff_.front();

    double d_time = current_cloud_data_.time - current_imu_data_.time;
    if (d_time < -0.05) {
        cloud_data_buff_.pop_front();
        LOG(INFO) << "IMU and GNSS data is late." << std::endl;
        return false;
    } else if (d_time > 0.05) {
        imu_data_buff_.pop_front();
        gnss_data_buff_.pop_front();
        LOG(INFO) << "Cloud points data is late." << std::endl;
        return false;
    } else {
        if(is_first_gnss_data){
            first_valid_gnss_data_ = gnss_data_buff_.front();
            is_first_gnss_data = false;
        }
        cloud_data_buff_.pop_front();
        imu_data_buff_.pop_front();
        gnss_data_buff_.pop_front();
    }
    return true;
}

bool FrontEndFlow::UpdateGNSSOdometry() {
    gnss_odometry_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_odometry_(0, 3) = current_gnss_data_.local_E;
    gnss_odometry_(1, 3) = current_gnss_data_.local_N;
    gnss_odometry_(2, 3) = current_gnss_data_.local_U;
    gnss_odometry_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();
    gnss_odometry_ *= lidar_to_imu_;  // lidar的轨迹

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {    // 这里调用FrontEnd的类的方法
    static bool front_end_pose_inited = false;
    if (!front_end_pose_inited) {
        front_end_pose_inited = true;
        front_end_ptr_->SetInitPose(gnss_odometry_);
        laser_odometry_ = gnss_odometry_;
        return true;
    }

    laser_odometry_ = Eigen::Matrix4f::Identity();
    if (front_end_ptr_->Update(current_cloud_data_, laser_odometry_)) {
        LOG(INFO) << "Lidar odometry update!";
        return true;
    } else
        return false;
}

bool FrontEndFlow::PublishData() {
    gnss_pub_ptr_->Publish(gnss_odometry_);
    laser_odom_pub_ptr_->Publish(laser_odometry_);

    if (front_end_ptr_->GetCurrentScan(current_scan_ptr_)) cloud_pub_ptr_->Publish(current_scan_ptr_);
    if (front_end_ptr_->GetNewLocalMap(local_map_ptr_)) local_map_pub_ptr_->Publish(local_map_ptr_);
    return true;
}
}
