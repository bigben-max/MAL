/*
 * @Date: 2019-09-25 20:42:30
 * @Author: jian.li
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-30 02:51:29
 */
#pragma once

#include <cmath>
#include <memory>
#include <queue>
#include <vector>
//
#include <base/common/macros.h>
#include <base/data_type/eigen_type.h>
//
#include "lidar_mapping/config_fusion.pb.h"
//
#include "lidar_mapping/algorithm/msf/state_helper.h"

namespace ldm::fusion {
class MeasurementModel {
 public:
  POINTER_TYPEDEFS(MeasurementModel);
  using time_ns_t = uint64_t;

  MeasurementModel(const Eigen::VectorXd &z_, const Eigen::MatrixXd &Q_,
                   const time_ns_t &t_)
      : z(z_), Q(Q_), t(t_) {}
  virtual ~MeasurementModel() {}

  virtual bool measurement_update(
      Eigen::VectorXd *state, Eigen::MatrixXd *Sigma,
      const Eigen::VectorXd &param = Eigen::VectorXd::Zero(1)) = 0;

  // Eq.(10-11)
  bool kalman_update(Eigen::VectorXd *state, Eigen::MatrixXd *Sigma,
                     const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
                     const Eigen::VectorXd &res) {
    // if (FusionConfig::fusion_use_chi_square_test()) {
    //   Eigen::MatrixXd mahanolabis_mat = H * (*Sigma) * H.transpose() + R;
    //   double chi_square_value =
    //       res.transpose() * mahanolabis_mat.inverse() * res;
    //   int res_size = res.cols();
    //   if (chi_square_value > chi2_inv_table[res_size]) return false;
    // }

    // kalman update, josseph form is used here
    Eigen::MatrixXd Tmp_Matrix = H * (*Sigma) * H.transpose() + R;
    Eigen::MatrixXd K = (*Sigma) * H.transpose() * Tmp_Matrix.inverse();

    int cov_row = (*Sigma).rows(), cov_col = (*Sigma).cols();
    Eigen::MatrixXd eye_mat = Eigen::MatrixXd::Identity(cov_row, cov_col);
    (*Sigma) = (eye_mat - K * H) * (*Sigma) * (eye_mat - K * H).transpose() +
               K * R * K.transpose();

    // calculate the state correction
    Eigen::VectorXd d_x = K * res;

    // correct p, v
    (*state).segment(0, 6) += d_x.segment(0, 6);

    // correct q
    base::Quatd q_GB;
    q_GB.w() = (*state)(6);
    q_GB.x() = (*state)(7);
    q_GB.y() = (*state)(8);
    q_GB.z() = (*state)(9);
    q_GB.normalize();

    base::Quatd d_q;
    base::V3d half_delta_angle(0.5f * d_x(6), 0.5f * d_x(7),
                                     0.5f * d_x(8));
    d_q.w() = sqrt(1 - half_delta_angle.transpose() * half_delta_angle);
    d_q.x() = 0.5f * d_x(6);
    d_q.y() = 0.5f * d_x(7);
    d_q.z() = 0.5f * d_x(8);

    base::Quatd curr_q_GB = q_GB * d_q;
    curr_q_GB.normalize();
    (*state).segment(6, 4) = Eigen::Vector4d(curr_q_GB.w(), curr_q_GB.x(),
                                             curr_q_GB.y(), curr_q_GB.z());
    // correct ba, bg
    (*state).segment(10, 6) += d_x.segment(9, 6);
    // norm restriction
    // for (int i = 10; i < 13; i++) {
    //   if (std::fabs((*state)(i)) > 0.5) (*state)(i) = (*state)(i) > 0 ? 0.2 :
    //   -0.2;
    // }
    // for (int i = 13; i < 16; i++) {
    //   if (std::fabs((*state)(i)) > 0.1) (*state)(i) = (*state)(i) > 0 ? 0.1 :
    //   -0.1;
    // }

    return true;
  }
  Eigen::VectorXd z;
  Eigen::MatrixXd F;
  Eigen::MatrixXd Q;
  time_ns_t t;
  bool is_delayed_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class LidarMeasurementModel : public MeasurementModel {
 public:
  LidarMeasurementModel(const Eigen::VectorXd &z_, const Eigen::MatrixXd &Q_,
                        const time_ns_t &t_)
      : MeasurementModel(z_, Q_, t_) {}
  virtual ~LidarMeasurementModel() {}

  void generate_3d_measurement(const int &cov_size, Eigen::VectorXd *state,
                               Eigen::VectorXd &res) {
    F = Eigen::MatrixXd::Zero(6, cov_size);
    F.block(0, 0, 3, 3) = base::M3d::Identity();
    F.block(3, 6, 3, 3) = base::M3d::Identity();

    const base::V3d p_GB = state->segment(X_P, X_P_Len);
    const base::V3d p_GB_lidar(z[0], z[1], z[2]);

    base::Quatd q_GB;
    q_GB.w() = (*state)(X_Aw);
    q_GB.x() = (*state)(X_Ax);
    q_GB.y() = (*state)(X_Ay);
    q_GB.z() = (*state)(X_Az);
    base::Quatd q_GB_lidar;
    q_GB_lidar.w() = z[3];
    q_GB_lidar.x() = z[4];
    q_GB_lidar.y() = z[5];
    q_GB_lidar.z() = z[6];

    base::Quatd res_q = q_GB.inverse() * q_GB_lidar;
    base::V3d res_q_vec = base::V3d::Zero();
    res_q_vec[0] = 2.0f * res_q.x() / res_q.w();
    res_q_vec[1] = 2.0f * res_q.y() / res_q.w();
    res_q_vec[2] = 2.0f * res_q.z() / res_q.w();

    res = Eigen::VectorXd::Zero(6);
    res.segment(0, 3) = p_GB_lidar - p_GB;
    res.segment(3, 3) = res_q_vec;
  }

  virtual bool measurement_update(
      Eigen::VectorXd *state, Eigen::MatrixXd *sigma,
      const Eigen::VectorXd &param = Eigen::VectorXd::Zero(1)) {
    // Eigen::Vector4d q_GB = (*state).segment(X_A, X_A_Len);
    // base::M3d R_GB = QuaternionToRotation(q_GB);
    // base::M3d Q_matrix = Q.block(0, 0, 3, 3);
    // Q.block(0, 0, 3, 3) = R_GB * Q_matrix * R_GB.transpose();
    const int cov_size = sigma->cols();
    Eigen::VectorXd res;
    generate_3d_measurement(cov_size, state, res);

    bool lidar_measurement_used = kalman_update(state, sigma, F, Q, res);
    return lidar_measurement_used;
  }
};

}  // namespace ldm::fusion
