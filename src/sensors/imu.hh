#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
namespace ctlio {
class IMU {
   public:
    IMU() = default;
    IMU(double time, const Eigen::Vector3d acc, const Eigen::Vector3d gyro) : timestamp_(time), acc_(acc), gyro_(gyro) {
    }
    double timestamp_;
    Eigen::Vector3d acc_;
    Eigen::Vector3d gyro_;
};
}  // namespace ctlio

using IMUPtr = std::shared_ptr<ctlio::IMU>;