/**
 * @file data_preprocessing_node.cpp
 * @brief 预处理的node文件
 * @author GWH
 * @version 0.1
 * @date 2020-03-09 13:18:12
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/data_preprocessing/data_preprocessing_flow.hpp"
#include "lidar_localization/global_defination/global_defination.h"

using namespace lidar_localization;
int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "data_preprocessing_node");
    ros::NodeHandle nh;

    std::shared_ptr<DataPreprocessFlow> data_preprocessing_ptr = std::make_shared<DataPreprocessFlow>(nh);
    
    ros::Rate rate(50);
    while (ros::ok()){
        ros::spinOnce();

        data_preprocessing_ptr->Run();

        rate.sleep();
    }

    return 0;
}
