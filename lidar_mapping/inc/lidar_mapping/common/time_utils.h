/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-04 01:50:12
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-27 02:51:55
 */
#pragma once

#include <chrono> // for std::chrono
#include <string> // for std::string

namespace ldm::common {
class TimeReporter {
 public:
  TimeReporter() : begin_(clock_::now()) {}
  void reset() { begin_ = clock_::now(); }

  double elapsed() const {
    return std::chrono::duration_cast<std::chrono::microseconds>(clock_::now() -
                                                                 begin_)
        .count();
  }

 private:
  typedef std::chrono::high_resolution_clock clock_;
  typedef std::chrono::duration<double, std::ratio<1> > second_;
  std::chrono::time_point<clock_> begin_;
};

double time_ms();

std::string fill_string(std::string base, int times);

double time_begin(std::string name = "", std::string parent = "root");

double time_end(std::string name = "");

std::string time_report();

}  // namespace base::ldm
