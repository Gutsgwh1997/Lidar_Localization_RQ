/**
 * @file back_end_node.cpp
 * @brief 后端优化的node文件
 * @author GWH
 * @version 0.1
 * @date 2020-03-10 19:10:25
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/mapping/back_end/back_end_flow.hpp"

using namespace lidar_localization;

int main(int argc, char* argv[]){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;

    std::shared_ptr<BackEndFlow> back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);

    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();

        back_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}
