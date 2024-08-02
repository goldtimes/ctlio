#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <vector>
#include "sensors/imu.hh"
#include "sensors/point_types.hh"
#include "tools/lidar_utils.hh"

namespace ctlio {

// 运动去畸变的模式
enum MotionCompensation { NONE = 0, CONSTANT_VELOCITY = 1, ITERATIVE = 2, CONTINUOUS = 3 };

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

class State {
   public:
    Eigen::Quaterniond rotation_begin;
    Eigen::Vector3d translation_begin;
    Eigen::Vector3d velocity_begin;
    Eigen::Vector3d ba_begin;
    Eigen::Vector3d bg_begin;

    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;

    State(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_, const Eigen::Vector3d &velocity_,
          const Eigen::Vector3d &ba_, const Eigen::Vector3d &bg_);

    State::State(const std::shared_ptr<State> state_temp, bool copy);
};

// 关键帧的定义
class CloudFrame {
   public:
    double time_frame_begin;
    double time_frame_end;
    int frame_id;
    std::shared_ptr<State> p_state;
    // points
    std::vector<point3D> points_world;
    std::vector<point3D> points_lidar;
    std::vector<point3D> keypoints;
    double dt_offset;
    bool success;

    CloudFrame(std::vector<point3D> &points_world_tmp, std::vector<point3D> &points_lidar_tmp,
               std::shared_ptr<State> p_state_);

    CloudFrame(std::shared_ptr<CloudFrame> p_cloud_frame);

    void release();
};

void transformPoint(MotionCompensation motion_compensation, point3D &point, const Eigen::Quaterniond &rotation_begin,
                    const Eigen::Vector3d &trans_begin, const Eigen::Quaterniond &rotation_end,
                    const Eigen::Vector3d &trans_end, const Eigen::Matrix3d &R_IL, const Eigen::Vector3d &t_IL);

}  // namespace ctlio