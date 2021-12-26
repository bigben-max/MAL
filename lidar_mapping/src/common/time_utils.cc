/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-04 01:57:57
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-27 02:54:54
 */

#include <sys/time.h>
#include <unistd.h>
#include <map>
#include <mutex>
#include <vector>
//
#include <ros_interface/common/print_helper.h>
#include "lidar_mapping/common/time_utils.h"
// #include "lidar_mapping/common/printout_utils.h"

namespace ldm::common {

std::string fill_string(std::string base, int times) {
  std::string ans;
  for (int i = 0; i < times; i++) ans += base;
  return ans;
}

double time_ms() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}

class Time_Profile {
 public:
  class TimeNode {
   public:
    std::string name_;
    double time_begin_;
    double sum_time_;
    double max_time_;
    double cur_time_;
    int sum_cnt;

    TimeNode() {
      time_begin_ = -1;
      name_ = "";
      sum_time_ = 0;
      sum_cnt = 0;
      max_time_ = 0;
      cur_time_ = 0;
    }

    TimeNode(std::string name_) {
      time_begin_ = -1;
      name_ = name_;
      sum_time_ = 0;
      sum_cnt = 0;
      max_time_ = 0;
      cur_time_ = 0;
    }

    double total_time() { return sum_time_; }

    double avg_time() {
      if (sum_cnt > 0) return sum_time_ / sum_cnt;
      return 0;
    }
    double curr_time() {
      if (sum_cnt > 0) return cur_time_;
      return 0;
    }
    double max_time() { return max_time_; }
  };

  Time_Profile() { hirearchy["root"] = std::vector<std::string>(); }

  static Time_Profile *instance() {
    if (p == NULL) p = new Time_Profile();
    return p;
  }

  double time_start(std::string name = "", std::string father = "root") {
    std::lock_guard<std::mutex> guard(lock);
    if (hirearchy.find(name) == hirearchy.end()) {
      if (hirearchy.find(father) != hirearchy.end()) {
        hirearchy[father].push_back(name);
        hirearchy[name] = std::vector<std::string>();
      }
    }
    if (name.length() == 0) {
      last_time = time_ms();
      return last_time;
    } else {
      if (time_map.find(name) == time_map.end()) {
        time_map[name] = TimeNode(name);
      }
      time_map[name].time_begin_ = time_ms();
      return time_map[name].time_begin_;
    }
  }

  double time_end(std::string name = "") {
    std::lock_guard<std::mutex> guard(lock);
    if (name.length() == 0) {
      if (last_time < 0) {
        printf("time count error! not init for Anonymous\n");
        return -1;
      }
      double ret = time_ms() - last_time;
      last_time = -1;
      return ret;
    } else if (time_map.find(name) == time_map.end()) {
      return -1;
    } else {
      auto &node = time_map[name];
      if (node.time_begin_ < 0) {
        printf("time count error! not init for %s\n", name.c_str());
        return -1;
      }
      double tm = time_ms() - node.time_begin_;
      // node.times.push_back(tm);
      node.time_begin_ = -1;
      node.sum_cnt += 1;
      node.sum_time_ += tm;
      node.cur_time_ = tm;
      if (tm > node.max_time_) node.max_time_ = tm;
      return tm;
    }
  }

  int max_name_length(std::string name, int tab_len) {
    if (hirearchy.find(name) == hirearchy.end()) return 0;
    int max_leng = tab_len + name.size();
    for (auto &son : hirearchy[name])
      max_leng = std::max(max_leng, max_name_length(son, tab_len + 4));
    return max_leng;
  }

  std::string item_report(std::string name, std::string tabs,
                          int max_name_length) {
    std::string ans;
    if (hirearchy.find(name) != hirearchy.end()) {
      for (auto &son : hirearchy[name]) {
        if (time_map.find(son) != time_map.end()) {
          std::string fills(" ", max_name_length - son.size() - tabs.size());
          std::fill(fills.begin(), fills.end(), ' ');
          ans += base::formatString(
              "| %s%s%s | %10.1f | %8d | %10.1f | %10.1f | %10.1f |\n",
              tabs.c_str(), son.c_str(), fills.c_str(),
              time_map[son].avg_time(), time_map[son].sum_cnt,
              time_map[son].total_time(), time_map[son].max_time(),
              time_map[son].curr_time());
          ans += item_report(son, tabs + "    ", max_name_length);
        }
      }
    }
    return ans;
  }

  std::string time_report() {
    std::lock_guard<std::mutex> guard(lock);
    int name_length = max_name_length("root", -4);
    std::string tabs;
    if (name_length < 5) {
      tabs = fill_string(" ", 5 - name_length);
      name_length = 5;
    }
    std::string line = fill_string("_", name_length + 64) + "\n";
    std::string ans;
    ans += line;
    ans +=
        "| " + fill_string(" ", name_length - 4) +
        "name |   avg time |      cnt | total time |   max time |  curr time "
        "|\n";
    ans += item_report("root", tabs, name_length);
    ans += line;
    return ans;
  }

 private:
  static Time_Profile *p;
  std::map<std::string, TimeNode> time_map;
  std::map<std::string, std::vector<std::string>> hirearchy;
  double last_time;
  std::mutex lock;
};

Time_Profile *Time_Profile::p = NULL;

double time_begin_(std::string name /* = "" */,
                   std::string parent /* = "root"*/) {
  return Time_Profile::instance()->time_start(name, parent);
}

double time_end(std::string name /* = "" */) {
  return Time_Profile::instance()->time_end(name);
}

std::string time_report() { return Time_Profile::instance()->time_report(); }

}  // namespace ldm::common
