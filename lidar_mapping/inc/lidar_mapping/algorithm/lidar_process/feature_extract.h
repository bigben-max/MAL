/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-11 02:00:05
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-08-12 01:55:05
 */

#include "common/common_struct.h"

namespace cmc::algo {
class FeatureExtraction {
 public:
  FeatureExtraction();
  ~FeatureExtraction();

  bool setInputCloud();

  bool getCornerPoints();
  bool getLinePoints();
  bool getSurfPoints();

 private:
  /* data */
};

}  // namespace cmc::algo
