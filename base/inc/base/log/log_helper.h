/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-09-14 02:30:54
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-09-15 02:47:59
 */

#pragma once

#include "base/common/file.h"
#include "base/log/log.h"

namespace base {
inline void InitLogging(const std::string &log_path,
                        const std::string &module_name) {
  auto log_folder = log_path + "/" + module_name;
  AINFO << "log_folder = " << log_folder;
  ACHECK(EnsureDirectory(log_folder));
  std::string error_log_filename = log_folder + "/ERROR_" + module_name + "_";
  std::string info_log_filename = log_folder + "/INFO_" + module_name + "_";
  google::SetLogDestination(google::ERROR, error_log_filename.c_str());
  google::SetLogDestination(google::WARNING, error_log_filename.c_str());
  google::SetLogDestination(google::FATAL, error_log_filename.c_str());
  google::SetLogDestination(google::INFO, info_log_filename.c_str());
  google::SetStderrLogging(google::INFO);
}
}  // namespace base
