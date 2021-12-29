/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-29 01:49:58
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-29 01:52:04
 */
#pragma once

namespace ldm::fusion {
static const int X_P_Len = 3;
static const int X_V_Len = 3;
static const int X_A_Len = 4;
static const int X_Ba_Len = 3;
static const int X_Bg_Len = 3;

static const int Cov_P_Len = 3;
static const int Cov_V_Len = 3;
static const int Cov_A_Len = 3;
static const int Cov_Ba_Len = 3;
static const int Cov_Bg_Len = 3;

enum { X_P = 0, X_V = 3, X_A = 6, X_Ba = 10, X_Bg = 13 };

enum {
  X_Px = 0,
  X_Py,
  X_Pz,
  X_Vx,
  X_Vy,
  X_Vz,
  X_Aw,
  X_Ax,
  X_Ay,
  X_Az,
  X_Bax,
  X_Bay,
  X_Baz,
  X_Bgx,
  X_Bgy,
  X_Bgz
};
enum { Cov_P = 0, Cov_V = 3, Cov_A = 6, Cov_Ba = 9, Cov_Bg = 12 };
enum {
  Cov_Px = 0,
  Cov_Py,
  Cov_Pz,
  Cov_Vx,
  Cov_Vy,
  Cov_Vz,
  Cov_Ax,
  Cov_Ay,
  Cov_Az,
  Cov_Bax,
  Cov_Bay,
  Cov_Baz,
  Cov_Bgx,
  Cov_Bgy,
  Cov_Bgz,
};

}  // namespace ldm::fusion
