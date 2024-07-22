#pragma once

#include <tsl/robin_map.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tools/lidar_utils.hh>
#include <vector>

namespace ctlio {
// 定义体素化
struct Voxel {
    Voxel() = default;
    Voxel(short x, short y, short z) : x(x), y(y), z(z) {
    }
    inline static Voxel Coordinate(const Eigen::Vector3d& point, double voxel_size) {
        // 体素化
        short x_coor = short(point.x() / voxel_size);
        short y_coor = short(point.y() / voxel_size);
        short z_coor = short(point.z() / voxel_size);
        return Voxel(x_coor, y_coor, z_coor);
    }
    // 比较
    inline bool operator<(const Voxel& other) const {
    }
    // ==
    bool operator==(const Voxel& other) const {
        return other.x == x && other.y == y && other.z == z;
    }

    short x;
    short y;
    short z;
};
// 定义体素空间中每个体素格子存放多少个点
struct VoxelBlock {
    explicit VoxelBlock(int capacity = 20) : capacity_(capacity) {
        points.reserve(capacity_);
    }

    void AddPoint(const Eigen::Vector3d& point) {
        points.push_back(point);
    }

    int GetNumPoints() const {
        return points.size();
    }

    int GetCapacity() const {
        return capacity_;
    }

    bool IsFull() const {
        return points.size() == capacity_;
    }

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;
    int capacity_;
};

using VoxelHashMap = tsl::robin_map<Voxel, VoxelBlock>;
}  // namespace ctlio

namespace std {
template <>
struct hash<ctlio::Voxel> {
    // 重载()
    std::size_t operator()(const ctlio::Voxel& voxel) const {
        const size_t kP1 = 73856093;
        const size_t kP2 = 19349669;
        const size_t kP3 = 83492791;
        return voxel.x * kP1 + voxel.y * kP2 + voxel.z * kP3;
    }
};
}  // namespace std