#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include "sensors/imu.hh"
#include "sensors/odom.hh"

namespace ctlio {
/**
 * @brief imu水平静止初始化，静止一段时间，统计静止时间段内的imu数据
 */
class StaticImuInit {
   public:
    struct InitOptions {
        bool use_odom = false;           // 使用速度来判断静止
        double init_time_seconds = 1.0;  // 静止时间
        // 设置零偏和噪声的阈值，太大则认为初始化失败
        double max_static_gyro_var = 0.5;
        double max_static_acc_var = 0.5;
        // 轮速计的脉冲
        int static_odom_pulse = 5;
        double gravity = 9.81;
        // imu队列
        int max_imu_size = 2000;
    };

    explicit StaticImuInit(InitOptions options = InitOptions()) : options_(options) {
    }

    void AddImu(const IMU& imu) {
        if (init_success_) {
            return;
        }
        if (options_.use_odom && !is_static_) {
            init_imu_queue_.clear();
        }
        // 静止且第一帧开始初始化
        if (init_imu_queue_.empty()) {
            init_start_time = imu.timestamp_;
            return;
        }
        init_imu_queue_.push_back(imu);
        curr_time = imu.timestamp_;
        double init_time = curr_time - init_start_time;
        if (init_time > options_.init_time_seconds) {
            TryInit();
        }
        while (init_imu_queue_.size() > options_.max_imu_size) {
            init_imu_queue_.pop_front();
        }
    }
    void AddOdom(const Odom& odom) {
        if (init_success_) {
            return;
        }
        if (!options_.use_odom) {
            return;
        }
        if (odom.left_pulse_ < options_.static_odom_pulse && odom.right_pulse_ < options_.static_odom_pulse) {
            is_static_ = true;
        } else {
            is_static_ = false;
        }
        curr_time = odom.timestamp_;
    }

    bool TryInit();

   private:
    InitOptions options_;
    // 初始化成功的标志
    bool init_success_ = false;
    bool is_static_ = false;
    // 零偏
    Eigen::Vector3d init_bg_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d init_ba_ = Eigen::Vector3d::Zero();
    // noise
    Eigen::Vector3d cov_acce_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d cov_gyro_ = Eigen::Vector3d::Zero();
    // 重力
    Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();
    std::deque<IMU> init_imu_queue_;
    double init_start_time;
    double curr_time;
};
}  // namespace ctlio