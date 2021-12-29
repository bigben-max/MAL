/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-21 00:34:27
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-30 02:27:40
 */
#pragma once
// cpp
#include <memory>
//
#include "base/common/macros.h"
#include "base/data_type/eigen_type.h"
#include "base/data_type/pcl_type.h"

namespace base {
// [accel, gyro] = [m/s^2, rad/s]
struct ImuMeasurement {
  POINTER_TYPEDEFS(ImuMeasurement);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuMeasurement() = default;
  ImuMeasurement(uint64_t _timestamp, const V3d& _imu_acc, const V3d& _imu_gyro)
      : timestamp(_timestamp), imu_acc(_imu_acc), imu_gyro(_imu_gyro) {}
  uint64_t timestamp;
  V3d imu_acc;
  V3d imu_gyro;
};

struct WheelMeasurement {
  POINTER_TYPEDEFS(WheelMeasurement);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  WheelMeasurement() = default;
  WheelMeasurement(uint64_t _timestamp, const V4d& _odo_velocity,
                   const int& _signal)
      : timestamp(_timestamp), odo_velocity(_odo_velocity), signal(_signal) {}
  uint64_t timestamp;
  V4d odo_velocity;
  int signal;
};

// struct ImageMeasurement {
//   POINTER_TYPEDEFS(ImageMeasurement);
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   uint64_t timestamp;
//   int camera_index;
//   cv::Mat image;

//   ImageMeasurement()
//       : timestamp(std::numeric_limits<uint64_t>::min()), camera_index(-1) {}
// };
struct LidarMeasurement {
  POINTER_TYPEDEFS(LidarMeasurement);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LidarMeasurement(){
    timestamp = 0;
    pointcloud.reset(new PointCloudI());
  }
  LidarMeasurement(uint64_t _timestamp, const PointCloudI::Ptr& _pointcloud)
      : timestamp(_timestamp), pointcloud(_pointcloud) {}
  uint64_t timestamp;
  PointCloudI::Ptr pointcloud;
};
}  // namespace base
