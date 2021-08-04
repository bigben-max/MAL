cmake_minimum_required(VERSION 3.2.0)
project(cloudmapcreator)

set(PROJECT_NAME cloudmapcreator)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -O3 -Wall -g")

find_package(
  catkin REQUIRED
  COMPONENTS tf
             roscpp
             rospy
             cv_bridge
             image_transport
             std_msgs
             sensor_msgs
             geometry_msgs
             nav_msgs)

catkin_package(
  INCLUDE_DIRS inc 
CATKIN_DEPENDS  
DEPENDS 
)

include(${PROJECT_SOURCE_DIR}/cmake/compile.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/FindGlog.cmake)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/inc)

file(GLOB_RECURSE ${PROJECT_NAME}_SRCS src/*.cc)

set(${Name}_LIBS ${PCL_LIBRARIES})
cmake_add_library(${PROJECT_NAME})
add_subdirectory(test/function_test)