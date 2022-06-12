/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-05 02:10:55
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-26 04:17:59
 */
#include <iostream>
//
#include <gtest/gtest.h>
//
#include <base/common/print_helper.h>
#include <base/log/log_helper.h>

TEST(UTILSTest, string_format_test) {
  int cnt = 999;
  std::string name = "hhhh";
  AINFO << base::formatString("count = %08d, name = %s", cnt, name)
            << std::endl;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  std::string log_path = ".";
  base::InitLogging(log_path, "lidar_mapping_test");

  AINFO << "Run Test ...";
  const int output = RUN_ALL_TESTS();
  return output;
}
