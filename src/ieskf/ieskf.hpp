/**
 * @brief 松耦合的Ieskf
 */

#pragma once
#include "ekf_utils/nav_state.hh"
#include "glog/logging.h"
#include "sensors/imu.hh"
#include "sensors/odom.hh"
#include "tools/math_utils.hh"

namespace ctlio {
/**
 * @brief 变量顺序p v q bg ba gravity 18维变量
 */
class IESKF {
   public:
    using SO3 = Sophus::SO3d;
    using SE3 = Sophus::SE3d;

    struct Options {
        double imu_dt_ = 0.01;
        double odom_dt_ = 0.1;

        // 噪声
        double gyro_var_ = 1e-6;
        double acc_var_ = 1e-2;
        double bias_gyro_var_ = 1e-6;
        double bias_acc_var_ = 1e-4;

        // odom
        double odom_var_ = 0.5;
        double wheel_radius_ = 0.155;
        double circle_pulse_ = 1024.0;
    };
    IESKF(Options options = Options()) : options_(options) {
        BuildNoise(options_);
    }

    void SetInitialConditions(Options options, const Eigen::Vector3d& init_bg, const Eigen::Vector3d& init_ba,
                              const Eigen::Vector3d& gravity = Eigen::Vector3d(0, 0, -9.81)) {
        BuildNoise(options);
        options_ = options;
        ba_ = init_ba;
        bg_ = init_bg;
        gravity_ = gravity;
        cov_ = Eigen::Matrix<double, 18, 18>::Identity() * 1e-4;
    }

    // lidar松耦合的位姿观测
    bool ObserveSE3(const SE3& pose, double trans_noise = 0.1, double ang_noise = 1.0 * kDEG2RAD);
    bool ObserveSE3(const SE3& pose, const Eigen::Matrix<double, 6, 1>& noise);
    // 速度观测
    bool ObserveWheelSpeed(const Odom& odom);

    bool Predict(const IMU& imu);

    void UpdateAndReset() {
        p_ += dx_.segment<3>(0);
        v_ += dx_.segment<3>(3);
        R_ = R_ * Sophus::SO3d::exp(dx_.segment<3>(6));
        bg_ += dx_.segment<3>(9);
        ba_ += dx_.segment<3>(12);
        gravity_ += dx_.segment<3>(15);
        // project cov;
        dx_.setZero();
    }

    // 获取状态
    NavState GetNominalState() const {
        return NavState(current_time_, R_, p_, v_, bg_, ba_);
    }

    SE3 GetNominalSE3() const {
        return SE3(R_, p_);
    }

    Eigen::Vector3d GetNomialVel() const {
        return v_;
    }

    Eigen::Vector3d GetGravity() const {
        return gravity_;
    }

    // set状态
    void SetX(const NavState& state, const Eigen::Vector3d& gravity) {
        p_ = state.p_;
        v_ = state.v_;
        R_ = state.R_;
        bg_ = state.bg_;
        ba_ = state.ba_;
        gravity_ = gravity;
    }

    void SetCov(const Eigen::Matrix<double, 18, 18>& cov) {
        cov_ = cov;
    }

   private:
    void BuildNoise(Options options) {
        double ev = options.acc_var_;
        double et = options.gyro_var_;
        double eg = options.bias_gyro_var_;
        double ea = options.bias_acc_var_;

        double ev2 = ev;  // * ev;
        double et2 = et;  // * et;
        double eg2 = eg;  // * eg;
        double ea2 = ea;  // * ea;

        // 设置过程噪声
        Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;
        double o2 = options_.odom_var_ * options_.odom_var_;
        odom_noise_.diagonal() << o2, o2, o2;
    }

    void ProjectCov() {
        Eigen::Matrix<double, 18, 18> J = Eigen::Matrix<double, 18, 18>::Identity();
        J.block<3, 3>(6, 6) = Eigen::Matrix<double, 3, 3>::Identity() - SO3::hat(dx_.segment<3>(6));
        cov_ = J * cov_ * J.transpose();
    }

   private:
    Options options_;
    // 名义状态量
    Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_ = Eigen::Vector3d::Zero();
    SO3 R_;
    Eigen::Vector3d bg_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_{0, 0, -9.81};
    // 误差状态量(Ieskf)用到的变量
    Eigen::Matrix<double, 18, 1> dx_;
    // 协方差矩阵
    Eigen::Matrix<double, 18, 18> cov_ = Eigen::Matrix<double, 18, 18>::Identity();
    // 系统噪声矩阵
    Eigen::Matrix<double, 18, 18> Q_ = Eigen::Matrix<double, 18, 18>::Identity();
    Eigen::Matrix<double, 3, 3> odom_noise_;
    double current_time_;
    NavState navi_state;
};

bool IESKF::Predict(const IMU& imu) {
    assert(imu.timestamp_ >= current_time_);
    double dt = imu.timestamp_ - current_time_;
    if (dt > 5 * options_.imu_dt_ || dt < 0) {
        LOG(INFO) << "skip this imu because dt_ = " << dt;
        current_time_ = imu.timestamp_;
        return false;
    }
    // 名义状态的递推
    Eigen::Vector3d new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acc_ - ba_)) * dt * dt + 0.5 * gravity_ * dt * dt;
    Eigen::Vector3d new_v = v_ + (R_ * (imu.acc_ - ba_)) * dt + gravity_ * dt;
    SO3 new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);

    p_ = new_p;
    R_ = new_R;
    v_ = new_v;
    // error状态的递推 dp, dv, dR
    Eigen::Matrix<double, 18, 18> F = Eigen::Matrix<double, 18, 18>::Identity();
    // p / v
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    // v / theta
    F.block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(imu.acc_ - ba_) * dt;
    // v / ba
    F.block<3, 3>(3, 12) = -R_.matrix() * dt;
    // v / g
    F.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
    // theta / theta
    F.block<3, 3>(6, 6) = SO3::exp(-(imu.gyro_ - bg_) * dt).matrix();
    // theta / bg
    F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;
    // dx_ = F * dx_;
    cov_ = F * cov_.eval() * F.transpose() + Q_;
    current_time_ = imu.timestamp_;
    return true;
}

bool IESKF::ObserveSE3(const SE3& pose, double trans_noise = 0.1, double ang_noise = 1.0 * kDEG2RAD) {
    Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
    // p 对 p
    H.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    // v 对 R
    H.block<3, 3>(3, 6) = Eigen::Matrix<double, 3, 3>::Identity();
    // K
    Eigen::Matrix<double, 6, 1> noise;
    noise << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise, ang_noise;
    Eigen::Matrix<double, 6, 6> noise_matrix = noise.asDiagonal();
    Eigen::Matrix<double, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + noise_matrix).inverse();

    Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
    error.segment<3>(0) = pose.translation() - p_;
    error.segment<3>(3) = (R_.inverse() * pose.so3()).log();
    dx_ = K * error;
    cov_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * cov_;

    UpdateAndReset();
    return true;
}
bool IESKF::ObserveSE3(const SE3& pose, const Eigen::Matrix<double, 6, 1>& noise) {
    Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
    // p 对 p
    H.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    // v 对 R
    H.block<3, 3>(3, 6) = Eigen::Matrix<double, 3, 3>::Identity();
    // K
    // Eigen::Matrix<double, 6, 1> noise;
    // noise << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise, ang_noise;
    Eigen::Matrix<double, 6, 6> noise_matrix = noise.asDiagonal();
    Eigen::Matrix<double, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + noise_matrix).inverse();

    Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
    error.segment<3>(0) = pose.translation() - p_;
    error.segment<3>(3) = (R_.inverse() * pose.so3()).log();
    dx_ = K * error;
    cov_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * cov_;

    UpdateAndReset();
    return true;
}
bool IESKF::ObserveWheelSpeed(const Odom& odom) {
    assert(odom.timestamp_ >= current_time_);
    // 速度对18维的观测
    Eigen::Matrix<double, 3, 18> H = Eigen::Matrix<double, 3, 18>::Zero();
    // 对v的jacobian
    H.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    // 卡尔曼增益k
    Eigen::Matrix<double, 18, 3> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + odom_noise_).inverse();
    // dx = K(观测 - 预测)
    // 计算轮速
    double velo_l = options_.wheel_radius_ * odom.left_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_dt_;
    double velo_r = options_.wheel_radius_ * odom.right_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_dt_;
    double average_vel = 0.5 * (velo_l + velo_r);
    // 速度是车体系下的速度
    Eigen::Vector3d vel_odom(average_vel, 0, 0);
    Eigen::Vector3d vel_world = R_ * vel_odom;
    dx_ = K * (vel_world - v_);
    // update cov;
    cov_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * cov_;
    UpdateAndReset();
    return true;
}

}  // namespace ctlio