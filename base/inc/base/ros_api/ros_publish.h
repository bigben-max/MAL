/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-21 00:00:49
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-27 02:46:37
 */
#pragma once
#include <ros/ros.h>
//
#include "base/data_type/common_struct.h"

namespace base {

void publishTF(const Transformd& T_fi_fj, const std::string& frame,
               const std::string& child_frame);

void publishTF(const Transformd& T_fi_fj, const std::string& frame,
               const std::string& child_frame, const ros::Time& ros_time);

void publishLatestOdometry(const RosOdometry& lidar_pose,
                           const ros::Publisher& lidar_pose_pub);
template <typename PointT>
void publishPointCloud(const ros::Publisher& publisher,
                       const uint64_t& sensor_time,
                       const typename pcl::PointCloud<PointT>::Ptr cloud,
                       const std::string& frame_id,
                       const std::string& child_frame_id);

}  // namespace base

#include "base/ros_api/ros_publish_impl.h"
