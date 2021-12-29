/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-09-14 00:57:59
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-09-16 01:37:10
 */

#pragma once
#include <boost/format.hpp>
#include <memory>  // for unqiue_ptr
#include <string>  // string

#include "base/common/log.h"
#include "base/config_rosinterface.pb.h"

namespace base {

template <typename... Args>
std::string formatString(std::string const& formate, const Args... args) {
  return boost::str((boost::format(formate) % ... % args));
}

inline std::string stringSystemTime() {
  auto tt =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  struct tm ptm = *localtime(&tt);
  return formatString("%04d-%02d-%02d_%02d-%02d-%02d", ptm.tm_year + 1900,
                      ptm.tm_mon + 1, ptm.tm_mday, ptm.tm_hour, ptm.tm_min,
                      ptm.tm_sec);
}

// inline std::string stringSystemTime() {
//   time_t it = std::time(0);
//   char local_time_char[64];
//   std::tm time_tm;
//   localtime_r(&it, &time_tm);
//   std::strftime(local_time_char, sizeof(local_time_char),
//   "%Y-%m-%d_%H-%M-%S",
//                 &time_tm);
//   std::string local_time_str = local_time_char;
//   return local_time_str;
// }

inline void printTopicInfo(const proto::Topic& topic) {
  AINFO << "topic name: " << topic.name();
  AINFO << "topic freq: " << topic.freq() << "ms";
  AINFO << "topic size: " << topic.size();
}

void floatToRgb(const float& value, int *rgb);
}  // namespace base
