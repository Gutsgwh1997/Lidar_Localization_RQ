/**
 * @file front_end_flow.cpp
 * @brief 任务管理类的实现
 * @author GWH
 * @version 0.1
 * @date 2020-03-04
 */
#include "lidar_localization/mapping/front_end/front_end_flow.hpp"
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tools/file_manager.hpp"
#include <boost/filesystem.hpp>
#include "glog/logging.h"

namespace lidar_localization {

FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    cloud_sub_ptr_      = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_odom", "map", "lidar", 1000); // topic,base_frame,child_frame

    front_end_ptr_ = std::make_shared<FrontEnd>();
}

bool FrontEndFlow::Run(){
    if (!ReadData()) {
        LOG(WARNING) << "Lidar Odometry 未接收到数据!";
        return false;
    }

    while (HasData()) {
        if (!ValidData()) continue;
        if (UpdateLaserOdometry()) {
            PublishData();
        }
    }

    return true;
}

// bool FrontEndFlow::SaveMap(){
//     return front_end_ptr_->SaveMap();
// }

// bool FrontEndFlow::SaveTrajectory(){
//     std::string data_directory = WORK_SPACE_PATH + "/slam_data/trajectory";
//     static bool is_file_created = false;
//     static std::ofstream ground_truth, laser_odom;

//     if(!is_file_created){
//         if(!filemanager::CreateDirectory(data_directory))
//             return false;
//         if(!filemanager::CreateFile(ground_truth,data_directory+"/ground_truth.txt"))
//             return false;
//         if(!filemanager::CreateFile(laser_odom,data_directory+"/laser_odom.txt"))
//             return false;
//         is_file_created = true;
//     }

//     for (int i =0; i<3; ++i){
//         for (int j=0; j<4; ++j){
//             ground_truth<<gnss_odometry_(i,j);
//             laser_odom<<laser_odometry_(i,j);
//             if (i==2&&j==3){
//                 ground_truth<<std::endl;
//                 laser_odom<<std::endl;
//             }else{
//                 ground_truth<<" ";
//                 laser_odom<<" ";
//             }
//         }
//     }
//     return true;
// }

// bool FrontEndFlow::PublishGlobalMap() {
//     if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_)) {
//         global_map_pub_ptr_->Publish(global_map_ptr_);
//         global_map_ptr_.reset(new CloudData::CLOUD());
//         return true;
//     }
//     return false;
// }

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    return true;
}

// bool FrontEndFlow::InitCalibration() {
//     static bool calibration_received = false;
//     if (!calibration_received) {
//         if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
//             calibration_received = true;
//             LOG(INFO)<<"Lidat to IMU transform is reveived :"<<std::endl<<lidar_to_imu_;
//         }
//     }
//     return calibration_received;
// }

// bool FrontEndFlow::InitGNSS() {
//     static bool gnss_inited = false;
//     if (!gnss_inited && gnss_data_buff_.size() > 0) {
//         GNSSData first_gnss_data = gnss_data_buff_.front();
//         // GNSSData first_gnss_data = first_valid_gnss_data_;  // 注意初始化时机
//         first_gnss_data.InitOriginPosition();
//         LOG(INFO)<<"GNSS origin is inited.";
//         gnss_inited = true;
//     }
//     return gnss_inited;
// }

bool FrontEndFlow::HasData() { return cloud_data_buff_.size() > 0; }

bool FrontEndFlow::ValidData() {
    // static bool is_first_gnss_data = true;
    // current_imu_data_   = imu_data_buff_.front();
    // current_gnss_data_  = gnss_data_buff_.front();
    // current_cloud_data_ = cloud_data_buff_.front();
    // current_velocity_data_ = velocity_data_buff_.front();

    // double d_time = current_cloud_data_.time - current_imu_data_.time;
    // if (d_time < -0.05) {
    //     cloud_data_buff_.pop_front();
    //     LOG(INFO) << "IMU and GNSS data is late." << std::endl;
    //     return false;
    // } else if (d_time > 0.05) {
    //     imu_data_buff_.pop_front();
    //     gnss_data_buff_.pop_front();
    //     velocity_data_buff_.pop_front();
    //     LOG(INFO) << "Cloud points data is late." << std::endl;
    //     return false;
    // } else {
    //     if(is_first_gnss_data){
    //         first_valid_gnss_data_ = gnss_data_buff_.front();
    //         is_first_gnss_data = false;
    //     }
    //     cloud_data_buff_.pop_front();
    //     imu_data_buff_.pop_front();
    //     gnss_data_buff_.pop_front();
    //     velocity_data_buff_.pop_front();
    // }
    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();

    return true;
}

// bool FrontEndFlow::UpdateGNSSOdometry() {
//     gnss_odometry_ = Eigen::Matrix4f::Identity();

//     current_gnss_data_.UpdateXYZ();
//     gnss_odometry_(0, 3) = current_gnss_data_.local_E;
//     gnss_odometry_(1, 3) = current_gnss_data_.local_N;
//     gnss_odometry_(2, 3) = current_gnss_data_.local_U;
//     gnss_odometry_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();
//     gnss_odometry_ *= lidar_to_imu_;  // lidar的轨迹

//     return true;
// }

bool FrontEndFlow::UpdateLaserOdometry() {    // 这里调用FrontEnd的类的方法
    static bool front_end_pose_inited = false;
    if (!front_end_pose_inited) {
        front_end_pose_inited = true;
        front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }

    if (front_end_ptr_->Update(current_cloud_data_, laser_odometry_)) {
        LOG(INFO) << "Lidar odometry updating...";
        return true;
    } else
        return false;
}

bool FrontEndFlow::PublishData() {
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);

    return true;
}
}
