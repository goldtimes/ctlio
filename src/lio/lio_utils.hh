#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include <vector>
#include "lio/lio_utils.hh"
#include "sensors/imu.hh"
#include "sensors/point_types.hh"

namespace ctlio {
enum ICPMODEL { POINT_TO_PLANE = 0, CT_POINT_TO_PLANE = 1 };

struct NeightborHood {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    double a2D = 1.0;
};

struct MeasureGroup {
    double lidar_begin_time = 0.0;
    double lidar_end_time = 0.0;
    std::deque<IMUPtr> imu_datas;  // 一帧之间的imu数据
    std::vector<point3D> lidar_;   // 雷达数据
};

}  // namespace ctlio