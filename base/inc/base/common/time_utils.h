/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-09-13 01:18:10
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-09-14 01:12:01
 */

#pragma once

#include <chrono>
#include <cstdint>

namespace base {
namespace internal {
template <typename TimeUnit>
struct time_traits;
struct sec;
template <>
struct time_traits<sec> {
  static constexpr int64_t nanoseconds = 1e9;
};
struct milli;
template <>
struct time_traits<milli> {
  static constexpr int64_t nanoseconds = 1e6;
};
struct micro;
template <>
struct time_traits<micro> {
  static constexpr int64_t nanoseconds = 1e3;
};
struct nano;
template <>
struct time_traits<nano> {
  static constexpr int64_t nanoseconds = 1;
};
template <typename TimeUnit>
inline constexpr int64_t convertToNanosecondsCompileTime(int64_t value) {
  return value * time_traits<TimeUnit>::nanoseconds;
}
template <typename TimeUnit>
inline static int64_t convertToNanoseconds(double value) {
  return static_cast<int64_t>(value * time_traits<TimeUnit>::nanoseconds);
}
template <typename TimeUnit>
inline static double convertFromNanoseconds(int64_t value) {
  return static_cast<double>(value) /
         static_cast<double>(time_traits<TimeUnit>::nanoseconds);
}
}  // namespace internal

#define FUNC_ALIAS __attribute__((unused)) static auto
/// Convenience functions to convert the specified unit to the nanoseconds
/// format at compile time. Example: int64_t sampling_time =
/// base::microseconds(10);
constexpr auto seconds =
    internal::convertToNanosecondsCompileTime<internal::sec>;
constexpr auto milliseconds =
    internal::convertToNanosecondsCompileTime<internal::milli>;
constexpr auto microseconds =
    internal::convertToNanosecondsCompileTime<internal::micro>;
constexpr auto nanoseconds =
    internal::convertToNanosecondsCompileTime<internal::nano>;

/// Convert a timestamp (in nanoseconds) to other time units at runtime.
/// Example: double milliseconds = base::to_milliseconds(int64_t
/// timestamp_nanoseconds);
#define FUNC_ALIAS __attribute__((unused)) static auto
FUNC_ALIAS to_seconds = internal::convertFromNanoseconds<internal::sec>;
FUNC_ALIAS to_milliseconds = internal::convertFromNanoseconds<internal::milli>;
FUNC_ALIAS to_microseconds = internal::convertFromNanoseconds<internal::micro>;

/// Convert other time units to a timestamp (in nanoseconds) at runtime.
/// Example: int64_t timestamp_nanoseconds = base::from_seconds(double
/// seconds);
FUNC_ALIAS from_seconds = internal::convertToNanoseconds<internal::sec>;
FUNC_ALIAS from_milliseconds = internal::convertToNanoseconds<internal::milli>;
FUNC_ALIAS from_microseconds = internal::convertToNanoseconds<internal::micro>;

}  // namespace time