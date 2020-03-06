/**
 * @file file_manager.hpp
 * @brief 文件读写的函数声明
 * @author GWH
 * @version 0.1
 * @date 2020-03-06
 */
#ifndef LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_
#define LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_

#include <boost/filesystem.hpp>
#include <fstream>

namespace lidar_localization {
namespace filemanager {

bool CreateFile(std::ofstream& ofs, const std::string& file_path);

bool CreateDirectory(const std::string& directory_path);

} // namespace filemanager
} // namespace lidar_localization

#endif
