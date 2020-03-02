/*
 * @Description: tf监听模块
 * @Author: Ren Qian
 * @Date: 2020-02-06 16:01:21
 */
#ifndef LIDAR_LOCALIZATION_TF_LISTENER_HPP_
#define LIDAR_LOCALIZATION_TF_LISTENER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace lidar_localization {
    /**
     * @brief base_frame到child_frame的变换矩阵
     */
class TFListener {
  public:
    TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
    TFListener() = default;

    /**
     * @brief 获取base_frame到child_frame的变换矩阵
     *
     * @param transform_matrix  得到的结果
     *
     * @return 是否成功
     */
    bool LookupData(Eigen::Matrix4f& transform_matrix);
  
  private:
    /**
     * @brief 将从tf监听得到的带有坐标转化关系的tf消息转换为Eigen::Matrix4f
     *
     * @param transform 监听得到的tf消息
     * @param transform_matrix 转换后的结果
     *
     * @return 是否成功
     */
    bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);

  private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;
};
}

#endif
