cmake_minimum_required(VERSION 3.5)

# Set the project name

# Add an executable

set(utils_test_SRCS utils_test.cc)
set(utils_test_LIBS ${PROJECT_NAME})

cmake_add_executable(utils_test)