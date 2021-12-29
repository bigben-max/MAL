/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-21 00:39:47
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-21 00:45:16
 */
#pragma once
// ros message
#include <geometry_msgs/Twist.h>
#include <gps_common/GPSFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
// kf message
#include <kf_msgs/PoseFrame.h>

namespace base {

using RosImu = sensor_msgs::Imu;
using RosOdometry = nav_msgs::Odometry;
using RosPointCloud = sensor_msgs::PointCloud2;
using RosTwist = geometry_msgs::Twist;
using RosGps = gps_common::GPSFix;

using KfRosPose = kf_msgs::PoseFrame;

}  // namespace base
