/**
 * @file file_manager.cpp
 * @brief 一些文件读写的方法
 * @author GWH
 * @version 0.1
 * @date 2020-03-06
 */

#include "lidar_localization/tools/file_manager.hpp"
#include "glog/logging.h"

namespace lidar_localization {
namespace filemanager {

bool CreateFile(std::ofstream& ofs, const std::string& file_path) {
    // ofs.open(file_path.c_str(), std::ios::app);  // 析构函数(destructor)将会自动调用关闭函数close
    // if (!ofs) {
    //     LOG(WARNING) << "无法生成文件！" << file_path;
    //     return false;
    // }
    // return true;
    ofs.close();
    boost::filesystem::remove(file_path.c_str());

    ofs.open(file_path.c_str(), std::ios::out);
    if (!ofs) {
        LOG(WARNING) << "无法生成文件: " << std::endl << file_path << std::endl << std::endl;
        return false;
    }

    return true;
}

bool CreateDirectory(const std::string& directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
        LOG(INFO) << "创建文件夹: " << directory_path;
    }
    if (!boost::filesystem::is_directory(directory_path)) {
        return false;
    }
    return true;
}

bool InitDirectory(std::string directory_path, std::string use_for) {
    if (boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::remove_all(directory_path); // 会删除父目录,所以加上一个"/tail"
        // CreateDirectory(directory_path, use_for);
        // LOG(INFO) << use_for << "存放地址：" << std::endl << directory_path << std::endl << std::endl;
        // return true;
    }

    return CreateDirectory(directory_path, use_for);
}

bool CreateDirectory(std::string directory_path, std::string use_for) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }

    if (!boost::filesystem::is_directory(directory_path)) {
        LOG(WARNING) << "无法创建文件夹: " << std::endl << directory_path << std::endl << std::endl;
        return false;
    }

    LOG(INFO) << use_for << "存放地址：" << std::endl << directory_path << std::endl << std::endl;
    return true;
}

} // namespace filemanager
} // namespace lidar_localization
