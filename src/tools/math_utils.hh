#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <numeric>
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
    mean = std::accumulate(datas.begin(), datas.end(), D::Zero()::eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) /
           len;
    // 协方差对角线 |值-均值| / (len-1)
    cov_diag = std::accumulate(datas.begin(), datas.end(), D::Zero()::eval(),
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
    mean = std::accumulate(datas.begin(), datas.end(), Eigen::Matrix<double, dim, 1>:Zero()::eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) /
           len;
    // 协方差
    cov = std::accumulate(datas.begin(), datas.end(), Eigen::Matrix<double, dim, dim>::Zero()::eval(),
                          [&getter, &mean](const E& sum, const auto& data) -> E {
                              D value = getter(data) - mean;
                              return sum += v * v.transpose();
                          }) /
          (len - 1);
}

}  // namespace ctlio