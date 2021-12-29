/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-30 01:37:20
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-30 02:52:34
 */
#include "lidar_mapping/algorithm/msf/sensor_fusion.h"

namespace ldm::fusion {

SensorFusion::SensorFusion(const base::proto::ConfigFusion &config)
    : config_(config) {
  lidar_pos_x_noise_pow_2_ = std::pow(config_.lidar_pos_x_noise_density(), 2);
  lidar_pos_y_noise_pow_2_ = std::pow(config_.lidar_pos_y_noise_density(), 2);
  lidar_pos_z_noise_pow_2_ = std::pow(config_.lidar_pos_z_noise_density(), 2);
  lidar_pos_roll_noise_pow_2_ =
      std::pow(base::Rad2Deg(config_.lidar_roll_noise_density()), 2);
  lidar_pos_pitch_noise_pow_2_ =
      std::pow(base::Rad2Deg(config_.lidar_pitch_noise_density()), 2);
  lidar_pos_yaw_noise_pow_2_ =
      std::pow(base::Rad2Deg(config_.lidar_yaw_noise_density()), 2);

  g_G_ << 0, 0, config_.gravirity_norm();

  Q_d_ = Eigen::MatrixXd::Zero(COV_LEN, COV_LEN);
  Eigen::MatrixXd Nc = Eigen::MatrixXd::Zero(11, 11);
  Nc.block(0, 0, 3, 3) = base::M3d::Identity() * config_.acc_noise_density() *
                         config_.acc_noise_density();
  Nc(3, 3) = config_.gyro_x_noise_density() * config_.gyro_x_noise_density();
  Nc(4, 4) = config_.gyro_y_noise_density() * config_.gyro_y_noise_density();
  Nc(5, 5) = config_.gyro_z_noise_density() * config_.gyro_z_noise_density();
  Nc.block(6, 6, 3, 3) = base::M3d::Identity() *
                         config_.acc_bias_noise_density() *
                         config_.acc_bias_noise_density();
  Nc.block(9, 9, 3, 3) = base::M3d::Identity() *
                         config_.gyro_bias_noise_density() *
                         config_.gyro_bias_noise_density();
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(16, 11);
  G.block(3, 0, 11, 11) = Eigen::MatrixXd::Identity(11, 11);
  Q_d_ = G * Nc * G.transpose();
}

void SensorFusion::ImuUpdate(const base::ImuMeasurement &imu_data,
                                  Eigen::VectorXd *x, Eigen::MatrixXd *sigma,
                                  const time_ns_t &last_process_t) {
  // if (imu_data.imu_gyro.array().isNaN() || imu_data.imu_acc.array().isNaN()) {
  //   AERROR << "error imu data input !";
  //   return;
  // }
  Eigen::VectorXd u = Eigen::VectorXd::Zero(6);
  u << imu_data.imu_gyro, imu_data.imu_acc;

  Propagator::Ptr process_model_ptr(
      new Propagator(u, Q_d_, imu_data.timestamp));
  prev_a_hat_ = base::V3d::Zero();
  prev_w_hat_ = base::V3d::Zero();
  process_model_ptr->process_update(x, sigma, prev_w_hat_, prev_a_hat_, g_G_,
                                    last_process_t);
  last_acc_ = imu_data.imu_acc - getBa();
  last_gyr_ = imu_data.imu_gyro - getBg();
  // std::cout << "ProcessUpdate finish!" << std::endl;
}

void SensorFusion::LidarUpdate(const PoseMeasurement &lidar_data,
                               Eigen::VectorXd *state, Eigen::MatrixXd *sigma) {
  // perform lidar measurement update
  Eigen::VectorXd z = Eigen::VectorXd::Zero(7);
  base::Quatd lidar_pose_q(lidar_data.pose.rotation());
  z << lidar_data.pose.translation(), lidar_pose_q.w(), lidar_pose_q.x(),
      lidar_pose_q.y(), lidar_pose_q.z();
  Eigen::MatrixXd lidar_Q = Eigen::MatrixXd::Zero(6, 6);
  lidar_Q(0, 0) = lidar_pos_x_noise_pow_2_;
  lidar_Q(1, 1) = lidar_pos_y_noise_pow_2_;
  lidar_Q(2, 2) = lidar_pos_z_noise_pow_2_;
  lidar_Q(3, 3) = lidar_pos_roll_noise_pow_2_;
  lidar_Q(4, 4) = lidar_pos_pitch_noise_pow_2_;
  lidar_Q(5, 5) = lidar_pos_yaw_noise_pow_2_;
  MeasurementModel::Ptr measure_ptr(
      new LidarMeasurementModel(z, lidar_Q, lidar_data.timestamp));
  measure_ptr->measurement_update(state, sigma);
}

bool SensorFusion::AddImuMessages(
    const std::vector<base::ImuMeasurement> &imu_datas, base::V3d *t,
    base::Quatd *q) {
  imu_datas_ = imu_datas;
  if (filter_state_ == FilterState::UNINITIALIZED) {
    return false;
  } else if (filter_state_ == FilterState::WORKING) {
    if (static_cast<int>(imu_datas_.size()) == 0) {
      return false;
    }
  }
  for (auto &imu_data : imu_datas) {
    ImuUpdate(imu_data, &state_, &sigma_, last_realtime_t_);
    last_realtime_t_ = imu_data.timestamp;
  }

  *t = getP();
  *q = getQ();
  return true;
}
bool SensorFusion::AddLidarMeasurement(const PoseMeasurement &lidar_pose,
                                       base::V3d *t, base::Quatd *q) {
  if (lidar_pose.timestamp < last_updated_lidar_time_) {
    filter_state_ = FilterState::UNINITIALIZED;
    imu_datas_.clear();
    last_updated_lidar_time_ = lidar_pose.timestamp;
    return false;
  }

  if (last_updated_lidar_time_ != 0 &&
      lidar_pose.timestamp - last_updated_lidar_time_ >
          config_.max_lost_lidar_num() * config_.lidar_msg_interval()) {
    filter_state_ = FilterState::UNINITIALIZED;
    last_updated_lidar_time_ = lidar_pose.timestamp;
    lidar_msg_arrived_num_ = 0;
    return false;
  }

  last_lidar_pose_ = curr_lidar_pose_;
  curr_lidar_pose_ = lidar_pose;

  lidar_msg_arrived_num_++;
  switch (filter_state_) {
    case FilterState::UNINITIALIZED:
      if (lidar_msg_arrived_num_ >= 2) {
        if (initializeFilter() == true) {
          filter_state_ = FilterState::WORKING;
          last_realtime_t_ = lidar_pose.timestamp;
        }
      }
      break;
    case FilterState::WORKING:

      LidarUpdate(lidar_pose, &state_, &sigma_);
      last_realtime_t_ = lidar_pose.timestamp;
      break;
  }

  last_updated_lidar_time_ = lidar_pose.timestamp;

  if (filter_state_ == FilterState::WORKING) {
    *t = getP();
    *q = getQ();
    return true;
  } else {
    return false;
  }
}

void SensorFusion::Reset() {
  filter_state_ = FilterState::UNINITIALIZED;
  imu_datas_.clear();
  last_updated_lidar_time_ = 0;
  lidar_msg_arrived_num_ = 0;
}

bool SensorFusion::initializeFilter() {
  // base::V3d last_lidar_t = last_lidar_pose_.pose.translation();
  base::V3d curr_lidar_t = curr_lidar_pose_.pose.translation();

  base::V3d init_V_GB = base::V3d::Zero();
  base::Quatd q_GB(curr_lidar_pose_.pose.rotation());

  // state
  state_ = Eigen::VectorXd::Zero(STATE_LEN);
  state_.segment(X_P, X_P_Len) = curr_lidar_t;
  state_.segment(X_V, X_V_Len) = init_V_GB;
  state_.segment(X_A, X_A_Len) =
      base::V4d(q_GB.w(), q_GB.x(), q_GB.y(), q_GB.z());

  // sigma
  sigma_ = Eigen::MatrixXd::Zero(COV_LEN, COV_LEN);
  sigma_.block(Cov_P, Cov_P, Cov_P_Len, Cov_P_Len) =
      base::M3d::Identity() * config_.init_p_sigma() * config_.init_p_sigma();
  sigma_.block(Cov_V, Cov_V, Cov_V_Len, Cov_V_Len) =
      base::M3d::Identity() * config_.init_v_sigma() * config_.init_v_sigma();
  sigma_.block(Cov_A, Cov_A, Cov_A_Len, Cov_A_Len) =
      base::M3d::Identity() * config_.init_a_sigma() * config_.init_a_sigma();

  sigma_(Cov_Bax, Cov_Bax) =
      config_.init_ba_x_sigma() * config_.init_ba_x_sigma();
  sigma_(Cov_Bay, Cov_Bay) =
      config_.init_ba_y_sigma() * config_.init_ba_y_sigma();
  sigma_(Cov_Baz, Cov_Baz) =
      config_.init_ba_z_sigma() * config_.init_ba_z_sigma();
  sigma_(Cov_Bgx, Cov_Bgx) =
      config_.init_bg_x_sigma() * config_.init_bg_x_sigma();
  sigma_(Cov_Bgy, Cov_Bgy) =
      config_.init_bg_y_sigma() * config_.init_bg_y_sigma();
  sigma_(Cov_Bgz, Cov_Bgz) =
      config_.init_bg_z_sigma() * config_.init_bg_z_sigma();
  AINFO << "Init state is : \n" << state_ << std::endl;
  AINFO << "Sensor Fusion init success!";
  return true;
}

}  // namespace ldm::fusion
