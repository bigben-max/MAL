cmake_minimum_required(VERSION 2.8.3)
project(base)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall -g")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# proto generation
set(PROTO_DEFNS proto/base/config_rosinterface.proto)
protobuf_catkin_generate_cpp2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

# find Eigen3
find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(inc)
file(GLOB_RECURSE ${PROJECT_NAME}_src src/*.cc)

find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(libLAS REQUIRED)


cs_add_library(${PROJECT_NAME} ${${PROJECT_NAME}_src} ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${libLAS_LIBRARIES} ${catkin_LIBRARIES} glog gflags)

add_subdirectory(test/)

set(${PROJECT_NAME}_LOCAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
cs_install()

cs_export(
  INCLUDE_DIRS
  inc
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  gps_common
  DEPENDS
  CFG_EXTRAS)
