/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-27 01:09:32
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-21 00:43:05
 */
#pragma once

#include "base/common/time_utils.h"
#include "base/data_type/common_struct.h"
namespace base {

template <typename T>
uint64_t getRosMsgTimeNs(T msg) {
  return msg->header.stamp.toNSec();
}

inline int64_t rosTimeToNanoseconds(const ros::Time& rostime) {
  return from_seconds(static_cast<int64_t>(rostime.sec)) +
         static_cast<int64_t>(rostime.nsec);
}

}  // namespace base
