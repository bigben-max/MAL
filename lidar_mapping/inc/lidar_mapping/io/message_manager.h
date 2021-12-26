/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-26 03:09:52
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-26 04:07:18
 */
#pragma once
#include <ros_interface/common/data_buffer.h>
#include <ros_interface/common/print_helper.h>
#include <ros_interface/data_type/common_struct.h>
//
#include "lidar_mapping/config_message_manager.pb.h"

namespace base::ldm {

using ImuMeasurement = base::ImuMeasurement;
using WheelMeasurement = base::WheelMeasurement;
using LidarMeasurement = base::LidarMeasurement;
template <typename T>
using DataBuffer = base::DataBuffer<T>;

using ImuMeasurementVec = std::vector<ImuMeasurement::Ptr>;
using WheelMeasurementVec = std::vector<WheelMeasurement::Ptr>;

struct MessageGroup {
  MessageGroup() {
    // lidar_beg_time = 0.0;
    this->lidar_msg.reset(new LidarMeasurement());
  }

  // double lidar_beg_time;
  // double lidar_end_time;
  ImuMeasurementVec imu_buffer;
  WheelMeasurementVec wheel_buffer;
  LidarMeasurement::Ptr lidar_msg;
};

class MessageManger {
 public:
  POINTER_TYPEDEFS(MessageManger);

  MessageManger(const base::proto::ConfigMessageManager& config)
      : config_(config) {}
  virtual ~MessageManger() {}

  virtual void addImuMsg(const ImuMeasurement::Ptr& imu_measurement) {
    AERROR << "Do nothing, please override this function!";
  }
  virtual void addLidarMsg(const LidarMeasurement::Ptr& lidar_measurement) {
    AERROR << "Do nothing, please override this function!";
  }

  virtual bool syncedMesaurement(uint64_t start_time_ns, uint64_t end_time_ns,
                                 MessageGroup* msg_group) {
    AERROR << "Do nothing, please override this function!";
    return false;
  }

 protected:
  base::proto::ConfigMessageManager config_;

  DISALLOW_COPY_AND_ASSIGN(MessageManger);
};

}  // namespace base::ldm