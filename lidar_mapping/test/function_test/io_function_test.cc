/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-26 03:35:57
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-27 02:53:46
 */
#include <iostream>
//
#include <gtest/gtest.h>
//
#include <base/common/print_helper.h>
#include <base/common/time_utils.h>
#include <base/interface/ros/ros_publish.h>
#include <base/log/log_helper.h>
//
#include "lidar_mapping/io/message_ros.h"

namespace base::ldm {
TEST(IOTest, message_ros_test) {
  base::proto::ConfigMessageManager config;
  std::string config_path =
      "/home/max/catkin_ws/src/LidarMAL/lidar_mapping/config/"
      "config_lidar_mapping.cfg";
  ACHECK(GetProtoFromASCIIFile(config_path, &config));
  MessageRos msg_ros(config);
  ros::NodeHandle ros_nh;
  msg_ros.Init(&ros_nh);

  ros::Publisher pub_sync_cloud = ros_nh.advertise<RosPointCloud>(
      config.ros_topic().publish_topic().sync_lidar().name(), 1);
  MessageGroup msg_group;
  ros::Rate lidar_rate(100);
  while (ros::ok()) {
    if (msg_ros.syncedMesaurementByLidar(&msg_group)) {
      int imu_size = msg_group.imu_buffer.size();
      AINFO << "find imu_msgs size = " << imu_size;
      auto current_lidar_time_ns = msg_group.lidar_msg->timestamp;
      if (imu_size > 0) {
        auto imu_msg = msg_group.imu_buffer[imu_size - 1];
        AINFO << "Last Imu msg timestamp = "
              << base::to_seconds(imu_msg->timestamp) << "s";
        AINFO << "Last Imu acc = " << imu_msg->imu_acc.transpose();
        AINFO << "Last Imu gyro = " << imu_msg->imu_gyro.transpose();
        AINFO << "Lidar msg timestamp - Imu msg timestamp = "
              << base::to_milliseconds(current_lidar_time_ns) -
                     base::to_milliseconds(imu_msg->timestamp)
              << "ms";
      }

      PointCloudRGB::Ptr cloud_show = boost::make_shared<PointCloudRGB>();
      for (auto pt : msg_group.lidar_msg->pointcloud->points) {
        PointRGB pt_rgb;
        pt_rgb.x = pt.x;
        pt_rgb.y = pt.y;
        pt_rgb.z = pt.z;
        float intensity = pt.intensity / 50.0f;
        int rgb[3];
        floatToRgb(intensity, rgb);
        pt_rgb.r = static_cast<int8_t>(rgb[0]);
        pt_rgb.g = static_cast<int8_t>(rgb[1]);
        pt_rgb.b = static_cast<int8_t>(rgb[2]);
        cloud_show->push_back(pt_rgb);
      }
      publishPointCloud<PointRGB>(pub_sync_cloud, current_lidar_time_ns,
                                  cloud_show, "world", "world");
    }
    ros::spinOnce();
    lidar_rate.sleep();
  }
}
} // namespace base::ldm

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "io_test");

  testing::InitGoogleTest(&argc, argv);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  std::string log_path = ".";
  base::InitLogging(log_path, "lidar_mapping_test");

  AINFO << "Run Test ...";
  const int output = RUN_ALL_TESTS();
  return output;
}
