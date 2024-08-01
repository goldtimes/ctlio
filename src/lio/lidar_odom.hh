#pragma once
#include <glog/logging.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <sophus/se3.hpp>
#include <string>
#include <vector>
#include "ekf_utils/static_imu_init.hh"
#include "eskf/eskf.hpp"
#include "lio/lio_utils.hh"
#include "sensors/imu.hh"
#include "sensors/point_types.hh"
#include "tools/lidar_utils.hh"
#include "tools/timer/timer.hh"

namespace ctlio {
// lio的配置
struct LioOptions {};

class LidarOdom {
   public:
    LidarOdom();
    ~LidarOdom() = default;
    bool init(const std::string& config_file);
    void pushImu(IMUPtr& imu_msg);
    void pushLidar(const std::vector<point3D>&, std::pair<double, double> data);
    // 死循环函数，一直处理传感器数据
    void run();

   private:
    void loadOptions(const std::string& config_file);
    std::vector<MeasureGroup> getMeasurments();
    void processMeasurements(const MeasureGroup& meas);
    void TryInitIMU();
    void Predict();

   private:
    LioOptions options_;

    int index_frame;
    IMUPtr last_imu;

    StaticImuInit static_imu_init;
    double lidar_imu_dt;
    // 从右往左看
    Sophus::SE3d T_IL;
    Eigen::Vector3d P_lidar_in_imu;
    Eigen::Matrix3d R_lidar_to_imu;

    double last_timestamp_imu_;
    double last_timestamp_lidar_;
    double processed_measure_time_;
    std::condition_variable cond;
    std::mutex mtx_data;
    // data
    std::deque<IMUPtr> imu_buff;
    std::deque<std::vector<point3D>> lidar_buffer;
    std::deque<std::pair<double, double>> time_buffer;

    MeasureGroup mearsure_;
    bool imu_need_init_ = true;
    ESKFD eskf_;
    std::vector<NavState> imu_states_;
};
}  // namespace ctlio