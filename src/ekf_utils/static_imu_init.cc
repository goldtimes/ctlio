#include "ekf_utils/static_imu_init.hh"
#include <glog/logging.h>
#include "tools/math_utils.hh"

namespace ctlio {
bool StaticImuInit::TryInit() {
    if (init_imu_queue_.size() < 20) {
        return false;
    }
    // 计算均值和方差
    // 循环的方式能写，但是代码异常冗余
    // Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    // Eigen::Vector3d mean_gyro = Eigen::Vector3d::Zero();
    // auto iter = init_imu_queue_.begin();
    // while (iter != init_imu_queue_.end()) {
    //     mean_acc.x() += iter->acc_.x();
    //     mean_acc.y() += iter->acc_.y();
    //     mean_acc.z() += iter->acc_.z();
    // }
    // mean_acc /= init_imu_queue_.size();
    // iter = init_imu_queue_.begin();
    // while (iter != init_imu_queue_.end()) {
    //     mean_gyro.x() += iter->gyro_.x();
    //     mean_gyro.y() += iter->gyro_.y();
    //     mean_gyro.z() += iter->gyro_.z();
    // }
    // mean_gyro /= init_imu_queue_.size();
    Eigen::Vector3d acc_mean, gyro_mean;
    ComputeMeanAndCovDiag(init_imu_queue_, acc_mean, cov_acce_, [](const IMU& imu) { return imu.acc_; });
    ComputeMeanAndCovDiag(init_imu_queue_, gyro_mean, cov_gyro_, [](const IMU& imu) { return imu.gyro_; });
    LOG(INFO) << "mean acc:" << acc_mean.transpose();
    // 以均值为方向，9.81长度为重力
    gravity_ = -acc_mean / acc_mean.norm() * options_.gravity;
    // 加上重力后 重新计算协方差
    ComputeMeanAndCovDiag(init_imu_queue_, acc_mean, cov_acce_, [this](const IMU& imu) { return imu.acc_ + gravity_; });
    // 检查噪声
    if (cov_gyro_.norm() > options_.max_static_gyro_var) {
        LOG(ERROR) << "陀螺仪测量噪声太大" << cov_gyro_.norm() << " > " << options_.max_static_gyro_var;
        return false;
    }
    if (cov_acce_.norm() > options_.max_static_acc_var) {
        LOG(ERROR) << "加计测量噪声太大" << cov_acce_.norm() << " > " << options_.max_static_acc_var;
        return false;
    }
    init_ba_ = acc_mean;
    init_bg_ = gyro_mean;

    LOG(INFO) << "IMU 初始化成功，初始化时间= " << curr_time - init_start_time << ", bg = " << init_bg_.transpose()
              << ", ba = " << init_ba_.transpose();
    LOG(INFO) << "gyro sq = " << cov_gyro_.transpose() << ", acce sq = " << cov_acce_.transpose()
              << ", grav = " << gravity_.transpose() << ", norm: " << gravity_.norm();
    LOG(INFO) << "mean gyro: " << gyro_mean.transpose() << " acce: " << acc_mean.transpose();
    init_success_ = true;
    return init_success_;
}
}  // namespace ctlio