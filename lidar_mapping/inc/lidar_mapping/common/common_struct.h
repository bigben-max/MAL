/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-04 01:36:00
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-26 03:34:18
 */
#pragma once
#include <Eigen/Core>  // for Eigen struct

namespace base::ldm {
// typedef Eigen struct
using MXf = Eigen::Matrix<float, Dynamic, Dynamic>;
using V3f = Eigen::Vector3f;
using V4f = Eigen::Vector4f;
using V6f = MXf(6, 1);
using V7f = MXf(7, 1);
using M3f = Eigen::Matrix3f;
using M4f = Eigen::Matrix4f;

using MXd = Eigen::Matrix<double, Dynamic, Dynamic>;
using V3d = Eigen::Vector3d;
using V4d = Eigen::Vector4d;
using V6f = MXd(6, 1);
using V7f = MXd(7, 1);
using M3d = Eigen::Matrix3d;
using M4d = Eigen::Matrix4d;

using Quatf = Eigen::Quaternionf;
using Quatd = Eigen::Quaterniond;
using Transformd = Eigen::Isometry3d;

}  // namespace base::ldm
