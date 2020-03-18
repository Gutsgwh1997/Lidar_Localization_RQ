/**
 * @file file_manager.cpp
 * @brief 一些文件读写的方法
 * @author GWH
 * @version 0.1
 * @date 2020-03-06
 */

#include "lidar_localization/tools/file_manager.hpp"
#include "glog/logging.h"
#include <string>
#include <iostream>

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

bool ReadTrajectory(std::string data_path, std::vector<Eigen::Matrix4f>& trajecroty) {
    trajecroty.clear();
    std::ifstream fin(data_path.c_str());
    std::string line_info;
    float temp_matrix_array[12] = {};
    if (fin) {                             // 有该文件
        while (getline(fin, line_info)) {  // line_info中不包括每行的换行符
            std::stringstream input(line_info);
            float input_result;
            int cout = 0;
            while(input >> input_result) {
                temp_matrix_array[cout] = input_result;
                ++cout;
            }
            Eigen::Matrix<float, 3, 4, Eigen::RowMajor> temp_matrix(temp_matrix_array);
            Eigen::Matrix4f trans_pose = Eigen::Matrix4f::Identity();
            trans_pose.block<3,3>(0,0) = temp_matrix.block<3,3>(0,0);
            trans_pose.block<3,1>(0,3) = temp_matrix.block<3,1>(0,3);
            trajecroty.push_back(trans_pose);
        }
        fin.close();
    } else{  // 没有该文件
        std::cout << "No such file to read trajectory." << std::endl;
        return false;
    }
    return true;
}

} // namespace filemanager
} // namespace lidar_localization
