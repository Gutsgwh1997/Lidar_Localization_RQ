/**
 * @file ndt_registration.hpp
 * @brief ndt方法的点云匹配类
 * @author GWH
 * @version 0.1
 * @date 2020-03-03
 */
#ifndef LIDAR_LOCALIZATION_MODELS_NDT_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_NDT_REGISTRATION_HPP_

#include <pcl/registration/ndt.h>
#include "lidar_localization/models/registration/registration_interface.hpp"
namespace lidar_localization {
class NDTRegistration : public RegistrationInterface {
   public:
    NDTRegistration(const YAML::Node& node);
    NDTRegistration(float res, float step_size, float trans_eps, int max_iter);
    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target_ptr) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source_ptr, const Eigen::Matrix4f& predict_pose,
                   CloudData::CLOUD_PTR& result_cloud_ptr, Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;
   private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

   private:
    pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
};
}
#endif
