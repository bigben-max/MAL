/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-21 00:35:29
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-30 02:39:15
 */
#pragma once
// eign
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace base {

using MXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using V3f = Eigen::Vector3f;
using V4f = Eigen::Vector4f;
// using V6f = MXf(6, 1);
// using V7f = MXf(7, 1);
using M3f = Eigen::Matrix3f;
using M4f = Eigen::Matrix4f;

using MXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using V3d = Eigen::Vector3d;
using V4d = Eigen::Vector4d;
// using V6f = MXd(6, 1);
// using V7f = MXd(7, 1);
using M3d = Eigen::Matrix3d;
using M4d = Eigen::Matrix4d;

using Quatf = Eigen::Quaternionf;
using Quatd = Eigen::Quaterniond;
using Transformd = Eigen::Isometry3d;

}  // namespace base
