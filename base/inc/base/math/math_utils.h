/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-29 02:35:11
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-29 02:38:04
 */
#pragma once

#include "base/data_type/eigen_type.h"

namespace base {

template <typename T>
inline T Rad2Deg(T rad) {
  return rad * 180.0 / M_PI;
}

template <typename T>
inline T NormalizeRad(T rad) {
  rad = fmod(rad + M_PI, 2 * M_PI);
  if (rad < 0) {
    rad += 2 * M_PI;
  }
  return rad - M_PI;
}

template <typename T>
inline T Deg2Rad(T deg) {
  return deg / 180.0 * M_PI;
}

template <typename T>
inline T NormalizeDeg(T deg) {
  deg = fmod(deg + 180.0, 360.0);
  if (deg < 0) {
    deg += 360.0;
  }
  return deg - 180.0;
}

inline bool RadLt(double a, double b) { return NormalizeRad(a - b) < 0; }

inline bool RadGt(double a, double b) { return NormalizeRad(a - b) > 0; }

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(
    const Eigen::MatrixBase<Derived> &v3d) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
  m << typename Derived::Scalar(0), -v3d.z(), v3d.y(), v3d.z(),
      typename Derived::Scalar(0), -v3d.x(), -v3d.y(), v3d.x(),
      typename Derived::Scalar(0);
  return m;
}

}  // namespace base
