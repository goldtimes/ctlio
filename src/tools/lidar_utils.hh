#pragma once
#include <Eigen/Core>

namespace ctlio {
// 定义voxelmap中需要用到的点
struct point3D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d raw_point;  // 原始点
    Eigen::Vector3d point;      // 世界坐标系下的点

    double intensity;
    double alpha_time = 0.0;     // 当前帧中相对最后一个点的时间偏移
    double relative_time = 0.0;  // 参考当前帧的时间
    double timespan = 0.0;
    double timestamp = 0.0;  // global timestamp
    int ring;
};
}  // namespace ctlio