/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-09-13 00:59:22
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-09-14 01:27:57
 */
#include <glog/logging.h>
//
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
//
#include "base/common/log.h"
#include "base/ros_api/ros_msg_convert.h"
#include "base/ros_api/ros_utils.h"
//

namespace base {
ImuMeasurement::Ptr convertRosImuToBaseImu(const RosImu::ConstPtr &imu_msg) {
  ACHECK(imu_msg);
  ImuMeasurement::Ptr imu_measurement(new ImuMeasurement);
  imu_measurement->timestamp = rosTimeToNanoseconds(imu_msg->header.stamp);
  imu_measurement->imu_acc << imu_msg->linear_acceleration.x,
      imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
  imu_measurement->imu_gyro << imu_msg->angular_velocity.x,
      imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
  return imu_measurement;
}

LidarMeasurement::Ptr convertRosLidarToBaseLidar(
    const RosPointCloud::ConstPtr &lidar_msg) {
  ACHECK(lidar_msg);
  LidarMeasurement::Ptr lidar_measurement(new LidarMeasurement);
  lidar_measurement->timestamp = rosTimeToNanoseconds(lidar_msg->header.stamp);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*lidar_msg, pcl_pc2);
  lidar_measurement->pointcloud = boost::make_shared<PointCloudI>();
  pcl::fromPCLPointCloud2(pcl_pc2, *lidar_measurement->pointcloud);
  return lidar_measurement;
}

}  // namespace base
