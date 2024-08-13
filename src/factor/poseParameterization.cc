#include "factor/poseParameterization.hh"
#include "lio/lio_utils.hh"

namespace ctlio {
//这里的pose用四元素表示，所以需要告诉ceres 四元素的加法
bool PoseParameterization::Plus(const double* x, const double* delta, double* x_pluse_delta) const {
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);
    Eigen::Map<const Eigen::Vector3d> dp(delta);

    // 怎么表示四元素的增量？
    Eigen::Quaterniond dq = Math::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> updated_p(x_pluse_delta);
    Eigen::Map<Eigen::Quaterniond> updated_q(x_pluse_delta + 3);

    updated_p = _p + dp;
    updated_q = (_q * dq).normalized();

    return true;
}

bool PoseParameterization::ComputeJacobian(const double* x, double* jacobian) const {
    // pose 对 xyz, roll,pitch,yaw的jacobian
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();
    return true;
}

bool RotationParameterization::Plus(const double* x, const double* delta, double* x_pluse_delta) const {
    Eigen::Map<const Eigen::Quaterniond> _q(x);
    Eigen::Quaterniond dq = Math::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta));
    Eigen::Map<Eigen::Quaterniond> updated_q(x_pluse_delta);
    updated_q = (_q * dq).normalized();
    return true;
}

bool RotationParameterization::ComputeJacobian(const double* x, double* jacobian) const {
    // pose 对 xyz, roll,pitch,yaw的jacobian
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
    j.topRows<3>().setIdentity();
    j.bottomRows<1>().setZero();
    return true;
}

}  // namespace ctlio