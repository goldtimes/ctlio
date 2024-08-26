#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <numeric>
#include <sophus/se3.hpp>
namespace ctlio {
constexpr double kDEG2RAD = M_PI / 180.0;
constexpr double kRAD2DEG = 180 / M_PI;

/**
 * @brief 计算均值和协方差的对角线值
 * @tC 容器类型
 * @tD 均值/协方差的类型
 * @tGetter 仿函数，获取容器内的值
 */
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& datas, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = datas.size();
    // 均值
    mean = std::accumulate(datas.begin(), datas.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) /
           len;
    // 协方差对角线 |值-均值| / (len-1)
    cov_diag = std::accumulate(datas.begin(), datas.end(), D::Zero().eval(),
                               [&getter, &mean](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) /
               (len - 1);
}

template <typename C, int dim, typename Getter>
void ComputeMeanAndCov(const C& datas, Eigen::Matrix<double, dim, 1>& mean, Eigen::Matrix<double, dim, dim>& cov,
                       Getter&& getter) {
    using D = Eigen::Matrix<double, dim, 1>;
    using E = Eigen::Matrix<double, dim, dim>;
    size_t len = datas.size();
    // 均值
    mean = std::accumulate(datas.begin(), datas.end(), Eigen::Matrix<double, dim, 1>::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) /
           len;
    // 协方差
    cov = std::accumulate(datas.begin(), datas.end(), Eigen::Matrix<double, dim, dim>::Zero().eval(),
                          [&getter, &mean](const E& sum, const auto& data) -> E {
                              D v = getter(data) - mean;
                              return sum += v * v.transpose();
                          }) /
          (len - 1);
}

/**
 * @brief pose插值算法
 * @param T 数据类型
 * @param C 容器类型
 * @param FT 获取时间函数
 * @param FP 获取body姿态的函数
 */
template <typename T, typename C, typename FT, typename FP>
bool PoseInterp(double query_time, C&& datas, FT&& take_time_func, FP&& take_pose_func, Sophus::SE3d& result,
                T& best_match, float time_th = 0.5) {
    if (datas.empty()) {
        return false;
    }
    double end_time = take_time_func(*datas.rbegin());
    if (query_time > end_time) {
        // 0.5s的阈值
        if (query_time < (end_time + 0.5)) {
            // 直接取最后一个pose
            result = take_pose_func(*datas.rbegin());
            best_match = *datas.rbegin();
            return true;
        }
        return false;
    }
    // 找到query_time的时间戳
    auto match_iter = datas.begin();
    for (auto iter = datas.begin(); iter != datas.end(); iter++) {
        auto next_iter = iter;
        next_iter++;
        if (query_time > take_time_func(*iter) && query_time <= take_time_func(*next_iter)) {
            match_iter = iter;
            break;
        }
    }
    auto match_iter_next = match_iter;
    match_iter_next++;
    double dt = take_time_func(*match_iter_next) - take_time_func(*match_iter);
    // 防止dt为0
    double alpha = (query_time - take_time_func(*match_iter)) / (dt + 0.0000000001);
    // dt = 0;
    if (std::fabs(dt) < 1e-6) {
        best_match = *match_iter;
        result = take_pose_func(*match_iter);
        return true;
    }
    // 插值
    Sophus::SE3d pose_first = take_pose_func(*match_iter);
    Sophus::SE3d pose_second = take_pose_func(*match_iter_next);
    result = {pose_first.unit_quaternion().slerp(alpha, pose_second.unit_quaternion()),
              pose_first.translation() * (1 - alpha) + pose_second.translation() * alpha};
    best_match = alpha < 0.5 ? *match_iter : *match_iter_next;
    return true;
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<S>& v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<S>& v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

}  // namespace ctlio