/*
 * @Date: 2019-08-28 17:50:38
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-28 02:09:01
 */
#pragma once

#include <Eigen/Core>

namespace base::map {

template <typename T>
class Rect2D {
 public:
  /**@brieft The constructor. */
  Rect2D();
  /**@brief The constructor. */
  Rect2D(T min_x, T min_y, T max_x, T max_y);
  /**@brief Copy constructor. */
  Rect2D(const Rect2D<T>& ref);
  /**@brief Overloading the operator =. */
  Rect2D<T>& operator=(const Rect2D<T>& ref);
  /**@brief Get the min x of the rectangle. */
  T GetMinX() const;
  /**@brief Get the min y of the rectangle. */
  T GetMinY() const;
  /**@brief Get the max x of the rectangle. */
  T GetMaxX() const;
  /**@brief Get the max y of the rectangle. */
  T GetMaxY() const;
  /**@brief Get the left top corner of the rectangle. */
  Eigen::Matrix<T, 2, 1> GetLeftTopCorner() const;
  /**@brief Get the right bottom corner of the rectangle. */
  Eigen::Matrix<T, 2, 1> GetRightBottomCorner() const;

 private:
  /**@brief min_x, min_y, max_x, max_y. */
  Eigen::Matrix<T, 4, 1> _data;
};

template <typename T>
Rect2D<T>::Rect2D() {}

template <typename T>
Rect2D<T>::Rect2D(T min_x, T min_y, T max_x, T max_y)
    : _data(min_x, min_y, max_x, max_y) {}

template <typename T>
Rect2D<T>::Rect2D(const Rect2D<T>& ref) {
  _data = ref._data;
}

template <typename T>
Rect2D<T>& Rect2D<T>::operator=(const Rect2D<T>& ref) {
  _data = ref._data;
  return *this;
}

template <typename T>
T Rect2D<T>::GetMinX() const {
  return _data(0);
}

template <typename T>
T Rect2D<T>::GetMinY() const {
  return _data(1);
}

template <typename T>
T Rect2D<T>::GetMaxX() const {
  return _data(2);
}

template <typename T>
T Rect2D<T>::GetMaxY() const {
  return _data(3);
}

template <typename T>
Eigen::Matrix<T, 2, 1> Rect2D<T>::GetLeftTopCorner() const {
  Eigen::Matrix<T, 2, 1> corner;
  corner(0) = _data(0);
  corner(1) = _data(1);
  return corner;
}

template <typename T>
Eigen::Matrix<T, 2, 1> Rect2D<T>::GetRightBottomCorner() const {
  Eigen::Matrix<T, 2, 1> corner;
  corner(0) = _data(2);
  corner(1) = _data(3);
  return corner;
}

}  // namespace base::map
