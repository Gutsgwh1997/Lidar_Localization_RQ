/**
 * @file ndt_registration.cpp
 * @brief NDTRegistratio的实现
 * @author GWH
 * @version 0.1
 * @date 2020-03-03
 */
#include "lidar_localization/models/registration/ndt_registration.hpp"
#include "glog/logging.h"

namespace lidar_localization {

NDTRegistration::NDTRegistration(const YAML::Node& node)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
    float res       = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter    = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source_ptr,
                                const Eigen::Matrix4f& predict_pose,
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    ndt_ptr_->setInputSource(input_source_ptr);
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);        // 参数是变换后的点云，初始位姿估计值(这个估计值是从第一帧推来的)
    result_pose = ndt_ptr_->getFinalTransformation();        // 原始点云到目标点云的刚体变换(由于初始估计值的设定方式，得到的是相对于第一帧的位姿)

    return true;
}

bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target_ptr) {
    ndt_ptr_->setInputTarget(input_target_ptr);

    return true;
}

bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps,
                                           int max_iter) {
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);

    LOG(INFO) << "NDT 的匹配参数为：" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter << std::endl
              << std::endl;

    return true;
}

float NDTRegistration::GetFitnessScore() {
    return ndt_ptr_->getFitnessScore();
}

}  // namespace
