#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include <iostream>
#include <memory>
#include <sophus/se3.hpp>
#include <tr1/unordered_map>
#include <vector>
#include "sensors/imu.hh"
#include "sensors/point_types.hh"
#include "tools/lidar_utils.hh"
#include "voxel/voxel_map.hpp"

namespace ctlio {

// 运动去畸变的模式
enum MotionCompensation { NONE = 0, CONSTANT_VELOCITY = 1, ITERATIVE = 2, CONTINUOUS = 3 };

enum IcpModel { POINT_TO_PLANE = 0, CT_POINT_TO_PLANE = 1 };

struct Neighborhood {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
    double a2D = 1.0;  // Planarity coefficient
};

struct MeasureGroup {
    double lidar_begin_time_ = 0;  // 雷达包的起始时间
    double lidar_end_time_ = 0;    // 雷达的终止时间
    std::vector<point3D> lidar_;   // 雷达点云
    std::deque<IMUPtr> imu_;       // 上一时时刻到现在的IMU读数
};

class state {
   public:
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;

    Eigen::Quaterniond rotation_begin;
    Eigen::Vector3d translation_begin;
    Eigen::Vector3d velocity_begin;
    Eigen::Vector3d ba_begin;
    Eigen::Vector3d bg_begin;

    state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_, const Eigen::Vector3d &velocity_,
          const Eigen::Vector3d &ba_, const Eigen::Vector3d &bg_);

    state(const state *state_temp, bool copy = false);

    void release();
};

class cloudFrame {
   public:
    double time_frame_begin;  //  current frame front stamp
    double time_frame_end;    //    next frame front stamp

    int frame_id;

    state *p_state;

    std::vector<point3D> point_surf;  //  global frame
    std::vector<point3D> const_surf;  //  lidar frame

    std::vector<point3D> surf_keypoints;

    double dt_offset;

    bool success;

    cloudFrame(std::vector<point3D> &point_surf_, std::vector<point3D> &const_surf_, state *p_state_);

    cloudFrame(cloudFrame *p_cloud_frame);

    void release();
};

void transformPoint(MotionCompensation motion_compensation, point3D &point, const Eigen::Quaterniond &rotation_begin,
                    const Eigen::Quaterniond &rotation_end, const Eigen::Vector3d &trans_begin,
                    const Eigen::Vector3d &trans_end, const Eigen::Matrix3d &R_IL, const Eigen::Vector3d &t_IL);

void GridSampling(const std::vector<point3D> &cloud_in, std::vector<point3D> &keypoints, double size_voxel_subsampling);
void SubSampleFrame(std::vector<point3D> &cloud_in, double size_voxel);
double AngularDistance(const Eigen::Vector3d &qa, const Eigen::Vector3d &qb);
double AngularDistance(const Eigen::Quaterniond &qa, const Eigen::Quaterniond &qb);

class Math {
   public:
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta) {
        using Scalar_t = typename Derived::Scalar;
        // 将旋转矢量转换为四元素
        Eigen::Quaternion<Scalar_t> dq;

        // Eigen::Quaternion<Scalar_t> dq;
        // Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        // half_theta /= static_cast<Scalar_t>(2.0);
        // dq.w() = static_cast<Scalar_t>(1.0);
        // dq.x() = half_theta.x();
        // dq.y() = half_theta.y();
        // dq.z() = half_theta.z();
        // dq.normalize();
        // return dq;

        Scalar_t theta_norm = theta.norm();
        if (theta_norm > static_cast<Scalar_t>(0.0)) {
            Scalar_t half_theta = theta_norm / static_cast<Scalar_t>(2.0);
            Eigen::Matrix<Scalar_t, 3, 1> u = theta / theta_norm;
            dq.w() = std::cos(half_theta);
            dq.x() = std::sin(half_theta) * u.x();
            dq.y() = std::sin(half_theta) * u.y();
            dq.z() = std::sin(half_theta) * u.z();
        } else {
            dq.w() = static_cast<Scalar_t>(1.0);
            dq.x() = static_cast<Scalar_t>(0.0);
            dq.y() = static_cast<Scalar_t>(0.0);
            dq.z() = static_cast<Scalar_t>(0.0);
        }
        dq.normalize();
        return dq;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> &vect) {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> mat;
        // clang-format off
        mat << typename Derived::Scalar(0), -mat(2), mat(1),
                mat(2), typename Derived::Scalar(0), -mat(0),
                -mat(1), mat(0), typename Derived::Scalar(0);
        // clang-format on
        return mat;
    }
};

}  // namespace ctlio