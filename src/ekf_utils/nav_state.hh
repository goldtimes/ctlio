#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace ctlio {
/**
 * @brief 我们用eskf来递推，当然是想获取递推后的imu姿态信息
 *  于是我们创建一个NavState来保存状态
 */
class NavState {
   public:
    using SO3 = Sophus::SO3d;
    using SE3 = Sophus::SE3d;

    NavState() = default;
    NavState(double time, const SO3& R, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& bg,
             const Eigen::Vector3d& ba)
        : timestamp_(time), R_(R), p_(p), v_(v), bg_(bg), ba_(ba) {
    }
    NavState(double time, const SE3& pose, const Eigen::Vector3d& vel = Eigen::Vector3d::Zero())
        : timestamp_(time), R_(pose.so3()), p_(pose.translation()), v_(vel) {
    }

    Sophus::SE3d GetSE3() const {
        return SE3(R_, p_);
    }

    // 重载<<
    friend std::ostream& operator<<(std::ostream& os, const NavState& s) {
        os << "p: " << s.p_.transpose() << ", v: " << s.v_.transpose()
           << ", q: " << s.R_.unit_quaternion().coeffs().transpose() << ", bg: " << s.bg_.transpose()
           << ", ba: " << s.ba_.transpose();
        return os;
    }

   public:
    double timestamp_ = 0.0;
    SO3 R_;
    Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba_ = Eigen::Vector3d::Zero();
};
}  // namespace ctlio