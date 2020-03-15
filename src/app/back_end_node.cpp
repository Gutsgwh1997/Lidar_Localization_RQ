/**
 * @file back_end_node.cpp
 * @brief 后端优化的node文件
 * @author GWH
 * @version 0.1
 * @date 2020-03-10 19:10:25
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_localization/optimizeMap.h>
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/mapping/back_end/back_end_flow.hpp"

using namespace lidar_localization;

std::shared_ptr<BackEndFlow> _back_end_flow_ptr;
bool _need_optimize_map = false;

bool optimize_map_callback(optimizeMap::Request &request, optimizeMap::Response &response) {
    _need_optimize_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char* argv[]){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;

    _back_end_flow_ptr =  std::make_shared<BackEndFlow>(nh);
    ros::ServiceServer service = nh.advertiseService("optimize_map", optimize_map_callback);

    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();

        _back_end_flow_ptr->Run();
        if(_need_optimize_map){
            _back_end_flow_ptr->ForceOptimize();
            _need_optimize_map = false;
        }

        rate.sleep();
    }

    return 0;
}
