/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-21 00:00:42
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-21 00:42:39
 */
#pragma once

// thrid lib
#include <ros/ros.h>
//
#include "base/data_type/common_struct.h"

namespace base {

ImuMeasurement::Ptr convertRosImuToBaseImu(const RosImu::ConstPtr &imu_msg);

LidarMeasurement::Ptr convertRosLidarToBaseLidar(
    const RosPointCloud::ConstPtr &lidar_msg);

}  // namespace base
