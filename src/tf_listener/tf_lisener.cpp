/*
 * @Description: tf监听模块
 * @Author: Ren Qian
 * @Date: 2020-02-06 16:10:31
 * @reference:https://blog.csdn.net/start_from_scratch/article/details/50762293
 */
#include "lidar_localization/tf_listener/tf_listener.hpp"

#include <Eigen/Geometry>

namespace lidar_localization {
TFListener::TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id) 
    :nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {
}

bool TFListener::LookupData(Eigen::Matrix4f& transform_matrix) {
    try {
        tf::StampedTransform transform;  // Tf tree中描述一小段父子变换关系的
        listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform); // 监听Tf tree，获取从base_frame到child_frame的变换
        TransformToMatrix(transform, transform_matrix);
        return true;
    } catch (tf::TransformException &ex) {
        return false;
    }
}

bool TFListener::TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix) {
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

    // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

    // 等价的实现
    // tf::Quaternion tf_quatenion = transform.getRotation();
    // Eigen::Quaternionf eigen_quatenion(tf_quatenion.getX(),tf_quatenion.getY(),tf_quatenion.getZ(),tf_quatenion.getW());
    // Eigen::Vector3f trans_eigen(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    // Eigen::Matrix4f transform_matrix_2 = Eigen::Matrix4f::Identity();
    // transform_matrix_2.block<3,1>(0,3) = trans_eigen;
    // transform_matrix_2.block<3,3>(0,0) = eigen_quatenion.toRotationMatrix();

    // std::cout<<"自己写的矩阵运算结果是：\n"<<transform_matrix.inverse()*transform_matrix_2<<std::endl;
    return true;
}
}
