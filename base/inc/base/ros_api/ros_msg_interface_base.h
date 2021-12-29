/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-20 23:48:01
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-21 00:42:51
 */
#pragma once
// cpp
#include <memory>

// thrid lib
#include <ros/ros.h>

#include "base/common/log.h"
#include "base/data_type/common_struct.h"
#include "base/ros_api/ros_msg_convert.h"
#include "base/ros_api/ros_publish.h"

namespace base {

inline void SigintHandler(int sig) {
  ros::shutdown();
  ACHECK(false);
}
class RosMsgInterfaceBase {
 public:
  POINTER_TYPEDEFS(RosMsgInterfaceBase);
  RosMsgInterfaceBase() = default;
  virtual ~RosMsgInterfaceBase() = default;

  // callback
  virtual void imuCallback(const RosImu::ConstPtr& imu_msg) {}
  virtual void lidarCallback(const RosPointCloud::ConstPtr& lidar_msg) {}

  // init ros subscriber
  virtual bool Init(ros::NodeHandle* ros_nh) = 0;

  // init ros publisher
  virtual void registerPublishers() {}

 private:
  DISALLOW_COPY_AND_ASSIGN(RosMsgInterfaceBase);
};

}  // namespace base
