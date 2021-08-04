/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-04 01:52:03
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-08-05 02:12:56
 */
#pragma once
#include <string> // string
#include <memory> // for unqiue_ptr
#include <boost/format.hpp>

namespace cmc::common {
#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

template <typename... Args>
std::string formatString(std::string const& formate, const Args... args) {
  return boost::str((boost::format(formate) % ... % args));
}

}  // namespace cmc::common
