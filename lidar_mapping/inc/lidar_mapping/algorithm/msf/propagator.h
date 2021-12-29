/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-29 02:14:09
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-30 02:39:46
 */
#pragma once
#include <base/common/macros.h>
#include <base/common/math_utils.h>
//
// #include "lidar_mapping/algorithm/msf/state.h"
#include "lidar_mapping/algorithm/msf/state_helper.h"
namespace ldm::fusion {
class Propagator {
 public:
  POINTER_TYPEDEFS(Propagator);
  using time_ns_t = uint64_t;
  Propagator(const Eigen::VectorXd &u, const Eigen::MatrixXd &R,
             const time_ns_t &t)
      : u_(u), Q_d(R), t_(t) {}
  ~Propagator() {}

  void process_update(Eigen::VectorXd *x, Eigen::MatrixXd *Sigma,
                      const base::V3d &prev_w_hat,
                      const base::V3d &prev_a_hat,
                      const base::V3d &g_G, const time_ns_t &last_t) {
    // time unit second
    const double dt = double(t_ - last_t) * 1e-9;

    base::V3d w_m = u_.head(3);
    base::V3d a_m = u_.segment(3, 3);

    const Eigen::VectorXd &state = (*x);
    const base::V3d p_GB = state.segment(X_P, X_P_Len);
    const base::V3d v_GB = state.segment(X_V, X_V_Len);
    const base::V4d q_GB = state.segment(X_A, X_A_Len).normalized();
    const base::V3d b_a = state.segment(X_Ba, X_Ba_Len);
    const base::V3d b_g = state.segment(X_Bg, X_Bg_Len);

    const base::V3d curr_w = w_m - b_g;
    const base::V3d curr_a = a_m - b_a;

    base::V4d delta_q;
    delta_q[0] = 1;
    delta_q.segment(1, 3) = 0.5f * curr_w * dt;

    base::Quatd prev_q;
    prev_q.w() = q_GB[0];
    prev_q.x() = q_GB[1];
    prev_q.y() = q_GB[2];
    prev_q.z() = q_GB[3];
    prev_q.normalize();

    const base::M3d R_GB = prev_q.toRotationMatrix();

    base::Quatd d_q;
    d_q.w() = delta_q[0];
    d_q.x() = delta_q[1];
    d_q.y() = delta_q[2];
    d_q.z() = delta_q[3];
    d_q.normalize();

    base::Quatd curr_q = prev_q * d_q;
    curr_q.normalize();

    base::V4d curr_q_GB;
    curr_q_GB << curr_q.w(), curr_q.x(), curr_q.y(), curr_q.z();

    base::V3d linear_acc = R_GB * curr_a + g_G;

    base::V3d curr_p_GB = p_GB + v_GB * dt + 0.5f * linear_acc * dt * dt;
    base::V3d curr_v_GB = v_GB + linear_acc * dt;

    (*x) << curr_p_GB, curr_v_GB, curr_q_GB, b_a, b_g;

    int cov_size = (*Sigma).cols();
    Eigen::MatrixXd F_x = Eigen::MatrixXd::Identity(cov_size, cov_size);
    F_x.block(0, 3, 3, 3) = base::M3d::Identity() * dt;
    F_x.block(3, 6, 3, 3) = -R_GB * base::SkewSymmetric(curr_a) * dt;
    F_x.block(3, 9, 3, 3) = -R_GB * dt;
    F_x.block(6, 6, 3, 3) =
        base::M3d::Identity() - base::SkewSymmetric(curr_w * dt);
    F_x.block(6, 12, 3, 3) = -base::M3d::Identity() * dt;
    F_x.block(9, 9, 3, 3) = base::M3d::Identity();
    F_x.block(12, 12, 3, 3) = base::M3d::Identity();

    // update the covariance
    (*Sigma) = F_x * (*Sigma) * F_x.transpose() + Q_d;
  }

 private:
  Eigen::VectorXd u_;
  Eigen::MatrixXd Q_d;
  time_ns_t t_;
};

}  // namespace ldm::fusion
