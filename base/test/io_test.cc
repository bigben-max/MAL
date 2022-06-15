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
#include "base/common/file.h"
#include "base/io/data_io.h"
#include "base/io/las_io.h"
#include <base/common/print_helper.h>
#include <base/common/time_utils.h>
#include <base/interface/ros/ros_publish.h>
#include <base/log/log_helper.h>

namespace base::ldm {
TEST(BASEIOTest, read_kitti_test) {
  ros::NodeHandle ros_nh;
  ros::Publisher pub_sync_cloud =
      ros_nh.advertise<RosPointCloud>("kitti_data", 1);
  ros::Rate lidar_rate(10);

  std::string kitti_datadir =
      "/mnt/d/Dataset/KITTI/data/odometry/dataset/sequences/00/velodyne/";
  auto file_list = ListSubPaths(kitti_datadir, DT_REG);
  for (int i = 0; i < file_list.size(); i++) {
    base::LidarMeasurement lidar_mes;
    std::string file_name = formatString("%s/%06d.bin", kitti_datadir, i);
    lidar_mes.timestamp = i;
    if (loadKittiCloud(file_name, &lidar_mes)) {
      publishPointCloud<PointI>(pub_sync_cloud, i, lidar_mes.pointcloud,
                                "world", "world");
    }
    ros::spinOnce();
    lidar_rate.sleep();
  }
}

TEST(BASEIOTest, write_las_test) {

  std::string kitti_datadir =
      "/mnt/d/Dataset/KITTI/data/odometry/dataset/sequences/00/velodyne/";
  std::string las_datadir =
      "/mnt/d/Dataset/KITTI/data/odometry/dataset/sequences/00/las/";
  base::EnsureDirectory(las_datadir);
  auto file_list = ListSubPaths(kitti_datadir, DT_REG);
  for (int i = 0; i < file_list.size(); i++) {
    base::LidarMeasurement lidar_mes;
    std::string file_name = formatString("%s/%06d.bin", kitti_datadir, i);
    lidar_mes.timestamp = i;
    if (loadKittiCloud(file_name, &lidar_mes)) {
      std::string las_file_name = formatString("%s/%06d.las", las_datadir, i);
      AINFO << "save " << las_file_name;
      if (savePointCloudToLas(lidar_mes, las_file_name))
        AINFO << "sucessful";
    }
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
  base::InitLogging(log_path, "base_io_test");

  AINFO << "Run Test ...";
  const int output = RUN_ALL_TESTS();
  return output;
}
