/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-21 00:38:11
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-21 00:38:15
 */
#pragma once
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace base {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using PointI = pcl::PointXYZI;
using PointCloudI = pcl::PointCloud<PointI>;
using PointRGB = pcl::PointXYZRGB;
using PointCloudRGB = pcl::PointCloud<PointRGB>;

}  // namespace base
