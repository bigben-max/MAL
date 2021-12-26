/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-21 01:15:14
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-27 02:43:02
 */
#include "lidar_mapping/io/message_ros.h"

namespace base::ldm {
bool MessageRos::Init(ros::NodeHandle* ros_nh) {
  // Ros NodeHandle Init

  node_handle_ptr_ = std::make_unique<ros::NodeHandle>(*ros_nh);
  // Lidar subscriber.
  size_t kRosSubscriberQueueSizeLidar =
      config_.ros_topic().listen_topic().lidar().size();
  boost::function<void(const RosPointCloud::ConstPtr&)> lidar_callback =
      boost::bind(&MessageRos::lidarCallback, this, _1);
  sub_lidar_ = node_handle_ptr_->subscribe(
      config_.ros_topic().listen_topic().lidar().name(),
      kRosSubscriberQueueSizeLidar, lidar_callback);
  AINFO << "subscribe topic";
  printTopicInfo(config_.ros_topic().listen_topic().lidar());
  // IMU subscriber.
  size_t kRosSubscriberQueueSizeImu =
      config_.ros_topic().listen_topic().imu().size();
  boost::function<void(const RosImu::ConstPtr&)> imu_callback =
      boost::bind(&MessageRos::imuCallback, this, _1);
  sub_imu_ = node_handle_ptr_->subscribe(
      config_.ros_topic().listen_topic().imu().name(),
      kRosSubscriberQueueSizeImu, imu_callback);
  printTopicInfo(config_.ros_topic().listen_topic().imu());
  return true;
}

void MessageRos::imuCallback(const RosImu::ConstPtr& imu_msg) {
  ImuMeasurement::Ptr imu_measurement = base::convertRosImuToBaseImu(imu_msg);
  addImuMsg(imu_measurement);
}

void MessageRos::addImuMsg(const ImuMeasurement::Ptr& imu_measurement) {
  imu_buffer_->addMsgSafe(imu_measurement->timestamp, imu_measurement);
}

void MessageRos::lidarCallback(const RosPointCloud::ConstPtr& lidar_msg) {
  std::unique_lock<std::mutex> lock(lidar_data_mtx_);
  LidarMeasurement::Ptr lidar_measurement =
      base::convertRosLidarToBaseLidar(lidar_msg);
  addLidarMsg(lidar_measurement);
}

void MessageRos::addLidarMsg(const LidarMeasurement::Ptr& lidar_measurement) {
  lidar_buffer_->addMsgSafe(lidar_measurement->timestamp, lidar_measurement);
}

bool MessageRos::syncedMesaurement(uint64_t start_time_ns, uint64_t end_time_ns,
                                   MessageGroup* msg_group) {
  if (lidar_buffer_->emptySafe() || imu_buffer_->emptySafe() ||
      msg_group == nullptr) {
    return false;
  }
  auto msg_pair = lidar_buffer_->frontSafe();
  msg_group->lidar_msg = msg_pair->second;
  imu_buffer_->searchMsgByRangeTimeSafe(start_time_ns, end_time_ns,
                                        &msg_group->imu_buffer);
  lidar_buffer_->popFrontSafe();
  return msg_group->lidar_msg != nullptr;
}

bool MessageRos::syncedMesaurementByLidar(MessageGroup* msg_group) {
  if (!lidar_buffer_->emptySafe()) {
    uint64_t end_time_ns = lidar_buffer_->frontSafe()->first;
    if (last_lidar_timestamp_ns_ == 0) {
      last_lidar_timestamp_ns_ = end_time_ns;
      lidar_buffer_->popFrontSafe();
      return false;
    } else {
      bool sucess =
          syncedMesaurement(last_lidar_timestamp_ns_, end_time_ns, msg_group);
      last_lidar_timestamp_ns_ = end_time_ns;
      return sucess;
    }
  }
  return false;
}
}  // namespace base::ldm
