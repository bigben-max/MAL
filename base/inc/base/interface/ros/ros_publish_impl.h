/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-09-17 02:03:39
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-09-17 02:59:49
 */
#pragma once

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
//
#include <tf/transform_broadcaster.h>
#include "base/interface/ros/ros_msg_convert.h"

namespace base {

template <typename PointT>
void publishPointCloud(const ros::Publisher& publisher,
                       const uint64_t& sensor_time,
                       const typename pcl::PointCloud<PointT>::Ptr cloud,
                       const std::string& frame_id,
                       const std::string& child_frame_id) {
  std_msgs::Header ros_hdr;
  ros_hdr.frame_id = frame_id;
  ros_hdr.stamp.fromNSec(sensor_time);
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header = ros_hdr;
  // output.header.child_frame_id = child_frame_id;
  publisher.publish(output);
}

// void publishPose(const ros::Publisher& publisher, int64_t sensor_time,
//                  const Eigen::Affine3d& odo_pose,
//                  const Eigen::Matrix<double, 6, 6>& covarice) const {
//   std_msgs::Header ros_hdr;
//   ros_hdr.frame_id = "world";
//   ros_hdr.stamp.fromNSec(uint64_t(sensor_time * 1000ul));
//   Eigen::Quaterniond ndt_quatd(odo_pose.linear());
//   tf::Quaternion ros_q(ndt_quatd.x(), ndt_quatd.y(), ndt_quatd.z(),
//                        ndt_quatd.w());

//   geometry_msgs::Quaternion odom_quat;
//   tf::quaternionTFToMsg(ros_q, odom_quat);

//   geometry_msgs::TransformStamped odom_trans;
//   odom_trans.header = ros_hdr;
//   odom_trans.header.frame_id = "world";
//   odom_trans.child_frame_id = "base_link";
//   odom_trans.transform.translation.x = odo_pose.translation()[0];
//   odom_trans.transform.translation.y = odo_pose.translation()[1];
//   odom_trans.transform.translation.z = odo_pose.translation()[2];
//   odom_trans.transform.rotation = odom_quat;

//   nav_msgs::Odometry output = RosMsgCovert(ros_hdr, "base_link", odo_pose);
//   int k = 0;
//   for (int i = 0; i < 6; i++) {
//     for (int j = 0; j < 6; j++) {
//       output.pose.covariance[k] = covarice(i, j);
//     }
//   }

//   publisher.publish(output);
// }

void publishTF(const Transformd& T, const std::string& frame_id,
               const std::string& child_frame_id) {
  publishTF(T, frame_id, child_frame_id, ros::Time::now());
}

void publishTF(const Transformd& T, const std::string& frame_id,
               const std::string& child_frame_id, const ros::Time& ros_time) {
  ACHECK(!frame_id.empty());
  ACHECK(!child_frame_id.empty());
  const V3d& p = T.translation();

  tf::Transform tf_transform;
  tf_transform.setOrigin(tf::Vector3(p(0), p(1), p(2)));

  Quatd tf_q(T.rotation());
  tf::Quaternion tf_quaternion(tf_q.x(), tf_q.y(), tf_q.z(), tf_q.w());
  tf_transform.setRotation(tf_quaternion);

  static tf::TransformBroadcaster tf_br;
  tf_br.sendTransform(
      tf::StampedTransform(tf_transform, ros_time, frame_id, child_frame_id));
}

}  // namespace base
