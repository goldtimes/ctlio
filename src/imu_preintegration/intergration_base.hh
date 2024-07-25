#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace ctlio {
enum StateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };

enum NoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };
/**
 * imu的预积分类
 */
class IntergrationBase {
   public:
    IntergrationBase() = delete;
    IntergrationBase() {
    }
    void PushData(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro) {
        dt_buf.push_back(dt);
        acc_buf.push_back(acc);
        gyro_buf.push_back(gyro);
        // 积分
        propagate(dt, acc, gyro);
    }
    void propagate(const double delta_time, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro) {
        // 中值积分
        dt = delta_time;
        acc_1 = acc;
        gyro_1 = gyro;
        Eigen::Vector3d result_dp;
        Eigen::Vector3d result_dv;
        Eigen::Quaterniond result_dq;
        Eigen::Vector3d result_linearized_ba;
        Eigen::Vector3d result_linearized_bg;
        mindPointIntergration(delta_time, acc_0, gyro_0, acc_1, gyro_1, delta_p, delta_q, delta_v, linearized_bg,
                              linearized_ba, result_dp, result_dq, result_dv, result_linearized_bg,
                              result_linearized_ba, 1);
        // 状态变更
        delta_q = result_dp;
        delta_p = result_dp;
        delta_v = result_dv;
        // 预积分阶段ba,bg认为不变
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_q.normalize();
        sum_dt += dt;
        acc_0 = acc_1;
        gyro_0 = gyro_1;
    }

    void mindPointIntergration(double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                               const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                               const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q,
                               const Eigen::Vector3d &delta_v, const Eigen::Vector3d &linearized_ba,
                               const Eigen::Vector3d &linearized_bg, Eigen::Vector3d &result_delta_p,
                               Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                               Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg,
                               bool update_jacobian) {
        // 计算世界坐标系下的acc,没噪声的acc
        // 这里的delta_p,delta_v更像是上一次积分得到的状态量
        Eigen::Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
        Eigen::Vector3d un_acc_1 = delta_q * (_acc_1 - linearized_ba);
        Eigen::Vector3d mean_gyro = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        Eigen::Vector3d mean_acc = 0.5 * (un_acc_0 + un_acc_1);
        result_delta_q =
            delta_q * Eigen::Quaterniond(1, mean_gyro(0) * _dt / 2, mean_gyro(1) * _dt / 2, mean_gyro(2) * _dt / 2);
        result_delta_p = delta_p + delta_v * _dt + 0.5 * mean_acc * dt * dt;
        result_delta_v = delta_v + mean_acc * dt;
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;
        // 求协方差的传播，求两个矩阵F,V
        if (update_jacobian) {
            Eigen::Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Eigen::Vector3d a_0_x = _acc_0 - linearized_ba;
            Eigen::Vector3d a_1_x = _acc_1 - linearized_ba;
            Eigen::Matrix3d R_w_x, R_a_0_x, R_a_1_x;

            R_w_x << 0, -w_x(2), w_x(1), w_x(2), 0, -w_x(0), -w_x(1), w_x(0), 0;
            R_a_0_x << 0, -a_0_x(2), a_0_x(1), a_0_x(2), 0, -a_0_x(0), -a_0_x(1), a_0_x(0), 0;
            R_a_1_x << 0, -a_1_x(2), a_1_x(1), a_1_x(2), 0, -a_1_x(0), -a_1_x(1), a_1_x(0), 0;

            Eigen::MatrixXd F = Eigen::MatrixXd::Zero(15, 15);
            F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                                  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x *
                                      (Eigen::Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = Eigen::MatrixXd::Identity(3, 3) * _dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12) = -1.0 * Eigen::MatrixXd::Identity(3, 3) * _dt;
            F.block<3, 3>(6, 3) =
                -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Eigen::Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();
            // cout<<"A"<<endl<<A<<endl;

            Eigen::MatrixXd V = Eigen::MatrixXd::Zero(15, 18);
            V.block<3, 3>(0, 0) = 0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) = 0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) = 0.5 * Eigen::MatrixXd::Identity(3, 3) * _dt;
            V.block<3, 3>(3, 9) = 0.5 * Eigen::MatrixXd::Identity(3, 3) * _dt;
            V.block<3, 3>(6, 0) = 0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) = 0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3, 3) * _dt;
            V.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * _dt;

            // step_jacobian = F;
            // step_V = V;
            jacobian = F * jacobian;
            cov_ = F * noise_ * F.transpose() + V * noise_ * V.transpose();
        }
    }
    /**
     * @brief 这个是用来构造残差部分，用在优化部分
     */
    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi,
                                          const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai,
                                          const Eigen::Vector3d &Bgi, const Eigen::Vector3d &Pj,
                                          const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj,
                                          const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj) {
        // 残差
        Eigen::Matrix<double, 15, 1> residuals;
        // 在预积分中已经算了jacobian
        // 于是将预计分量线性化之后 + 预积分量对ba,bg的jacobian来修正
        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);
        Eigen::Matrix3d dR_dbg = jacobian.block<3, 3>(O_R, O_BG);
        Eigen::Matrix3d dV_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dV_dbg = jacobian.block<3, 3>(O_V, O_BG);
        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;
        Eigen::Quaterniond corrected_delta_q;
        Eigen::Vector3d corrected_delta_v = delta_v + dV_dba + dV_dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba + dp_dbg;
        // gravity为正的
        residuals.block<3, 1>(O_P, 0) =
            Qi.inverse() * (Pj - Pi - Vi * sum_dt + 0.5 * sum_dt * sum_dt * gravity) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (Vj - Vi + gravity * sum_dt) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
        return residuals;
    }

    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg) {
        sum_dt = 0.0;
        acc_0 = linearized_acc;
        gyro_0 = linearized_gyro;
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
        linearized_ba = _linearized_ba;
        linearized_bg = _linearized_bg;
        jacobian.setIdentity();
        cov_.setZero();
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++) propagate(dt_buf[i], acc_buf[i], gyro_buf[i]);
    }

   public:
    // 积分的中间变量
    double dt;
    Eigen::Vector3d acc_0, acc_1;
    Eigen::Vector3d gyro_0, gyro_1;
    const Eigen::Vector3d linearized_acc, linearized_gyro;
    Eigen::Vector3d linearized_ba, linearized_bg;
    // 上一次积分得到的delta量
    Eigen::Vector3d delta_p;
    Eigen::Vector3d delta_v;
    Eigen::Quaterniond delta_q;
    double sum_dt;
    // acc,gyro,dt的数据存放
    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyro_buf;
    // jacobian
    Eigen::Matrix<double, 15, 15> jacobian;
    // cov
    Eigen::Matrix<double, 15, 15> cov_;
    Eigen::Matrix<double, 15, 18> noise_;
    Eigen::Vector3d gravity{0, 0, 9.81};
};
}  // namespace ctlio