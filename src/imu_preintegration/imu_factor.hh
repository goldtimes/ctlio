#pragma once

#include <ceres/ceres.h>
#include "imu_preintegration/intergration_base.hh"
#include "imu_preintegration/utility.hh"
namespace ctlio {
/**
 * @brief 预积分的factor构建，ceres解析求导，需要定义残差，定义残差的对优化变量的雅克比矩阵
 * 15 imu factor的残差维度，7,9 是i时刻对pose, speedbias的变量维度 7,9是j时刻pose,speedbias的变量维度
 */
class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9> {
   public:
    IMUFactor() = delete;
    IMUFactor(IntergrationBase* _pre_intergration) : pre_integration(_pre_intergration) {
    }
    // 重载Evaluate函数告诉ceres去计算残差和jacobians
    virtual bool Evaluate(double const* const* paramters, double* residuals, double** jacobians) const {
        //构造待优化的变量
        Eigen::Vector3d Pi(paramters[0][0], paramters[0][1], paramters[0][2]);
        Eigen::Quaterniond Qi(paramters[0][6], paramters[0][3], paramters[0][4], paramters[0][5]);
        Eigen::Vector3d Vi(paramters[1][0], paramters[1][1], paramters[1][2]);
        Eigen::Vector3d Bai(paramters[1][3], paramters[1][4], paramters[1][5]);
        Eigen::Vector3d Bgi(paramters[1][6], paramters[1][7], paramters[1][8]);

        Eigen::Vector3d Pj(paramters[2][0], paramters[2][1], paramters[2][2]);
        Eigen::Quaterniond Qj(paramters[2][6], paramters[2][3], paramters[2][4], paramters[2][5]);
        Eigen::Vector3d Vj(paramters[3][0], paramters[3][1], paramters[3][2]);
        Eigen::Vector3d Baj(paramters[3][3], paramters[3][4], paramters[3][5]);
        Eigen::Vector3d Bgj(paramters[3][6], paramters[3][7], paramters[3][8]);

        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
        residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi, Pj, Qj, Vj, Baj, Bgj);
        // 信息矩阵
        Eigen::Matrix<double, 15, 15> sqrt_info =
            Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->cov_.inverse()).matrixL().transpose();
        // sqrt_info.setIdentity();
        residual = sqrt_info * residual;
        // jacobians
        if (jacobians) {
            // 保存一些预先的变量
            double dt = pre_integration->sum_dt;
            Eigen::Vector3d dp_dba = pre_integration->jacobian.block<3, 3>(O_P, O_BA);
            Eigen::Vector3d dp_dbg = pre_integration->jacobian.block<3, 3>(O_P, O_BG);
            Eigen::Vector3d dq_dbg = pre_integration->jacobian.block<3, 3>(O_R, O_BG);
            Eigen::Vector3d dv_dba = pre_integration->jacobian.block<3, 3>(O_V, O_BA);
            Eigen::Vector3d dv_dbg = pre_integration->jacobian.block<3, 3>(O_V, O_BG);
            if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8) {
                // ROS_WARN("numerical unstable in preintegration");
                // std::cout << pre_integration->jacobian << std::endl;
                ///                ROS_BREAK();
            }
            // 残差对Posei 15 x7
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 15, 7>, Eigen::RowMajor> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();
                // dres_p / dpi
                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                // dres_p / dqi
                jacobian_pose_i.block<3, 3>(O_P, O_R) =
                    Qi.inverse() * (Pj - Pi - Vi * dt + 0.5 * dt * dt * pre_integration->gravity);
                // dres_q / dqi
                Eigen::Quaterniond corrected_delta_q =
                    pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_i.block<3, 3>(O_R, O_R) =
                    -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
                // dres_v / dqi
                jacobian_pose_i.block<3, 3>(O_V, O_R) =
                    Utility::skewSymmetric(Qi.inverse() * (pre_integration->gravity * dt + Vj - Vi));
                if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8) {
                    // ROS_WARN("numerical unstable in preintegration");
                    // std::cout << pre_integration->jacobian << std::endl;
                    ///                ROS_BREAK();
                }
            }
            // 残差对SpeedBiasi 15 x 9
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 15, 9>, Eigen::RowMajor> jacobian_speedbias_i(jacobians[1]);
                jacobian_speedbias_i.setZero();
                jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * dt;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) =
                    -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;
                jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
                jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
                jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

                jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;
            }
            // 残差对Posej 15 x 7
            if (jacobians[2]) {
                Eigen::Map<Eigen::Matrix<double, 15, 7>, Eigen::RowMajor> jacobian_pose_j(jacobians[2]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

                Eigen::Quaterniond corrected_delta_q =
                    pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_j.block<3, 3>(O_R, O_R) =
                    Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();

                jacobian_pose_j = sqrt_info * jacobian_pose_j;
            }
            // 残差对SpeedBiasj 15 x 9
            if (jacobians[3]) {
                Eigen::Map<Eigen::Matrix<double, 15, 9>, Eigen::RowMajor> jacobian_speedbias_j(jacobians[3]);
                jacobian_speedbias_j.setZero();

                jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

                jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;
            }
        }
    }
    IntergrationBase* pre_integration;
};
}  // namespace ctlio