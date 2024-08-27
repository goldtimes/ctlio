#pragma once
#include "factor/poseParameterization.hh"

namespace ctlio {

// 自定义解析式的factor
// 1 残差维度，3待优化的变量维度，4待优化的变量维度
class LidarPlaneNormFactor : public ceres::SizedCostFunction<1, 3, 4> {
   public:
    LidarPlaneNormFactor(const Eigen::Vector3d &point_lidar, const Eigen::Vector3d &norm_vect, const double norm_offset,
                         double weight = 1.0);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d point_lidar_;
    Eigen::Vector3d norm_vect_;

    double weight_;
    double norm_offset_;

    static Eigen::Vector3d t_il;
    static Eigen::Quaterniond q_il;
    static double sqrt_info;
};
}  // namespace ctlio