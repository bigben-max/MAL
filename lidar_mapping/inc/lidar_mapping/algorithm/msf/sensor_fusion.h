/*
 * @Date: 2019-09-27 10:57:18
 * @Author: jian.li
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-30 02:52:14
 */
#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
//
#include <base/common/log.h>
#include <base/data_type/sensor_type.h>
//
#include "lidar_mapping/config_fusion.pb.h"
//
#include "lidar_mapping/algorithm/msf/measurement_model.h"
#include "lidar_mapping/algorithm/msf/propagator.h"

#define STATE_LEN 16
#define COV_LEN 16

namespace ldm::fusion {

enum FilterState { UNINITIALIZED = 0, WORKING };
enum { Init_Bg = 0, Init_Att, Init_Vel, Init_Pos };

struct PoseMeasurement {
  POINTER_TYPEDEFS(PoseMeasurement);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PoseMeasurement() = default;
  PoseMeasurement(uint64_t _timestamp, const base::M4d &_pose)
      : timestamp(_timestamp), pose(_pose) {}
  uint64_t timestamp;
  base::Transformd pose;
};

class SensorFusion {
 public:
  POINTER_TYPEDEFS(SensorFusion);
  using time_ns_t = uint64_t;
  SensorFusion(const base::proto::ConfigFusion &config);
  ~SensorFusion() {}
  void Reset();

  void ImuUpdate(const base::ImuMeasurement &imu_data, Eigen::VectorXd *x,
                     Eigen::MatrixXd *sigma, const time_ns_t &last_process_t);

  void LidarUpdate(const PoseMeasurement &lidar_data, Eigen::VectorXd *state,
                   Eigen::MatrixXd *sigma);

  bool AddImuMessages(const std::vector<base::ImuMeasurement> &imu_datas,
                      base::V3d *t, base::Quatd *q);

  bool AddLidarMeasurement(const PoseMeasurement &lidar_pose, base::V3d *t,
                           base::Quatd *q);

  inline bool is_init() const { return filter_state_ == FilterState::WORKING; }

  void propogate_priori_covariance(const time_ns_t &t);
  void predict_state(const time_ns_t &baseline_time);

  bool initializeFilter();

  const Eigen::VectorXd getState() const { return state_; }
  const base::V3d getP() const { return state_.segment(X_P, X_P_Len); }
  const base::V3d getV() const { return state_.segment(X_V, X_V_Len); }
  const base::Quatd getQ() const {
    return base::Quatd(state_(X_Aw), state_(X_Ax), state_(X_Ay), state_(X_Az))
        .normalized();
  }
  const base::V3d getBa() const { return state_.segment(X_Ba, X_Ba_Len); }
  const base::V3d getBg() const { return state_.segment(X_Bg, X_Bg_Len); }

  const Eigen::MatrixXd &getSigma() { return sigma_; }
  time_ns_t get_last_process_t() const { return last_realtime_t_; }

 private:
  //
  base::proto::ConfigFusion config_;
  // state_ is the newest state
  Eigen::VectorXd state_;
  Eigen::MatrixXd sigma_;

  Eigen::MatrixXd Q_d_;
  Eigen::VectorXd g_G_;
  FilterState filter_state_ = FilterState::UNINITIALIZED;

  time_ns_t last_realtime_t_ = 0;

  // these state are used for outlier detection
  bool check_lidar_outlier_ = false;
  time_ns_t last_updated_lidar_time_ = 0;

  Eigen::VectorXd prev_w_hat_;
  Eigen::VectorXd prev_a_hat_;
  Eigen::Vector4d prev_q_GB_;
  std::vector<base::ImuMeasurement> imu_datas_;

  Eigen::Vector3d last_acc_;
  Eigen::Vector3d last_gyr_;

  int lidar_msg_arrived_num_ = 0;
  PoseMeasurement last_lidar_pose_;
  PoseMeasurement curr_lidar_pose_;

  // measurement noise
  double lidar_pos_x_noise_pow_2_;
  double lidar_pos_y_noise_pow_2_;
  double lidar_pos_z_noise_pow_2_;
  double lidar_pos_roll_noise_pow_2_;
  double lidar_pos_pitch_noise_pow_2_;
  double lidar_pos_yaw_noise_pow_2_;
};

}  // namespace ldm::fusion
