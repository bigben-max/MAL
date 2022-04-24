/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-20 02:16:28
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-27 02:34:01
 */

#pragma once
#include <condition_variable>
//
#include <ros/ros.h>
//
#include <base/interface/ros/ros_msg_convert.h>
//
#include "lidar_mapping/io/message_manager.h"

namespace base::ldm {

class MessageRos : public MessageManger {
 public:
  POINTER_TYPEDEFS(MessageRos);

  using MessageManger::config_;

  MessageRos(const base::proto::ConfigMessageManager& config)
      : MessageManger(config),
        imu_buffer_(
            new DataBuffer<ImuMeasurement::Ptr>(config.imu_buffer_size())),
        lidar_buffer_(
            new DataBuffer<LidarMeasurement::Ptr>(config.lidar_buffer_size())) {
  }
  virtual ~MessageRos() {}
  bool Init(ros::NodeHandle* ros_nh);

  void imuCallback(const RosImu::ConstPtr& imu_msg);
  void lidarCallback(const RosPointCloud::ConstPtr& lidar_msg);

  virtual void addImuMsg(const ImuMeasurement::Ptr& imu_measurement);
  virtual void addLidarMsg(const LidarMeasurement::Ptr& lidar_measurement);

  virtual bool syncedMesaurement(uint64_t start_time_ns, uint64_t end_time_ns,
                                 MessageGroup* msg_group);

  bool syncedMesaurementByLidar(MessageGroup* msg_group);

 private:
  DataBuffer<ImuMeasurement::Ptr>::Ptr imu_buffer_;
  DataBuffer<WheelMeasurement::Ptr>::Ptr wheel_buffer_;
  DataBuffer<LidarMeasurement::Ptr>::Ptr lidar_buffer_;
  // callback get lidar data
  bool lidar_data_update_ = false;
  uint64_t last_lidar_timestamp_ns_ = 0;
  std::mutex lidar_data_mtx_;

  std::unique_ptr<ros::NodeHandle> node_handle_ptr_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_lidar_;

  DISALLOW_COPY_AND_ASSIGN(MessageRos);
};

}  // namespace base::ldm
