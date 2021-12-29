/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-09-16 01:36:29
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-09-16 01:37:35
 */

#include "base/common/print_helper.h"

namespace base {

void floatToRgb(const float &value, int *rgb) {
  ACHECK(rgb);

  int r, g, b;
  if (value >= 0 && value <= 0.11) {
    b = value / 0.11 * 112 + 143;
    g = 0;
    r = 0;
  } else if (value > 0.11 && value <= 0.125) {
    b = 255;
    g = 0;
    r = 0;
  } else if (value > 0.125 && value <= 0.36) {
    b = 255;
    g = (value - 0.125) / 0.235 * 255;
    r = 0;
  } else if (value > 0.36 && value <= 0.375) {
    b = 255;
    g = 255;
    r = 0;
  } else if (value > 0.375 && value <= 0.61) {
    b = 255 - (value - 0.375) / 0.235 * 255;
    g = 255;
    r = (value - 0.375) / 0.235 * 255;
  } else if (value > 0.61 && value <= 0.625) {
    b = 0;
    g = 255;
    r = 255;
  } else if (value > 0.625 && value <= 0.86) {
    b = 0;
    g = 255 - (value - 0.625) / 0.235 * 255;
    r = 255;
  } else if (value > 0.86 && value <= 0.875) {
    b = 0;
    g = 0;
    r = 255;
  } else if (value > 0.875 && value < 1) {
    b = 0;
    g = 0;
    r = 255 - (value - 0.875) / 0.125 * 127;
  } else {
    b = 0;
    g = 0;
    r = 0;
  }
  
  rgb[0] = r;
  rgb[1] = g;
  rgb[2] = b;
}
}  // namespace base
