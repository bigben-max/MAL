/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-30 02:53:07
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-30 02:55:35
 */
#pragma once

#include <base/data_type/common_struct.h>
//
#include "lidar_mapping/algorithm/msf/sensor_fusion.h"

namespace ldm {
class Odometry {
 public:
  Odometry(/* args */);
  ~Odometry();

  void feedImuMeasurement(const base::ImuMeasurement &imu_msg);
  void feadLidarMeasurement(const base::LidarMeasurement &imu_msg);
  bool getOdometryPose();

 private:

};

}  // namespace ldm
