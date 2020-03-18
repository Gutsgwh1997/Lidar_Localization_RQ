/*
 * @Description: 前端里程计的node文件
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

// #include <lidar_localization/saveMap.h>
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/mapping/front_end/front_end_flow.hpp"

using namespace lidar_localization;

std::shared_ptr<FrontEndFlow> _front_end_flow_ptr;

// bool save_map_callback(saveMap::Request &request, saveMap::Response &response) {
//     LOG(INFO)<<"Start saveMap";
//     // response.succeed = _front_end_flow_ptr->SaveMap();  // 一定要先保存地图
//     LOG(INFO)<<"Save map finished! ";
//     // _front_end_flow_ptr->PublishGlobalMap();            // 才能发布地图
//     return response.succeed;
// }

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 0;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    // ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);
    _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();

        _front_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}
