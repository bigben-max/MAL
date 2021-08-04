/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-05 02:10:55
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-08-05 02:22:11
 */
#include <iostream>
//
#include "common/printout_utils.h"

namespace cmc {
int string_format_test() {
  int cnt = 999;
  std::string name = "hhhh";
  std::cout << common::formatString("count = %d, name = %s", cnt, name)
            << std::endl;
  return 0;
}
}  // namespace cmc

int main(int argc, char **argv) {
  cmc::string_format_test();
  return 0;
}
