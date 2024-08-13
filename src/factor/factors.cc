#include "factor/factors.hh"

namespace ctlio {

double LidarPlaneNormalFactor::sqrt_info;
Eigen::Vector3d LidarPlaneNormalFactor::t_IL;
Eigen::Quaterniond LidarPlaneNormalFactor::q_IL;

LidarPlaneNormalFactor::LidarPlaneNormalFactor(const Eigen::Vector3d &point_lidar, const Eigen::Vector3d norm_vect,
                                               const double norm_offset, double weight)
    : point_lidar_(point_lidar), norm_vect_(norm_vect), norm_offset_(norm_offset), weight_(weight) {
}

bool LidarPlaneNormalFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    // 点到面的距离
    Eigen::Vector3d transaltion(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond rotation(parameters[1][0], parameters[1][1], parameters[1][2], parameters[1][3]);

    Eigen::Vector3d point_world = rotation * point_lidar_ + transaltion;

    double distance = norm_vect_.dot(point_world) + norm_offset_;

    if (jacobians) {
        // 待优化的变量为两个，所以jacobians也有两个
        if (jacobians[0]) {
            // distance 对 trans的jacobian 1x3
            Eigen::Map<Eigen::Matrix<double, 1, 3>, Eigen::RowMajor> jacobian_trans(jacobians[0]);
            jacobian_trans.setZero();
            jacobian_trans.block<1, 3>(0, 0) = sqrt_info * norm_vect_.transpose() * weight_;
        }
        if (jacobians[1]) {
            // distance 对 rotation的jacobian 1x4
            Eigen::Map<Eigen::Matrix<double, 1, 4>, Eigen::RowMajor> jacobian_rotation(jacobians[0]);
            jacobian_rotation.setZero();
            jacobian_rotation.block<1, 3>(0, 0) =
                -sqrt_info * norm_vect_.transpose() * rotation.toRotationMatrix() * Math::skew(point_lidar_) * weight_;
        }
    }
    return true;
}
}  // namespace ctlio