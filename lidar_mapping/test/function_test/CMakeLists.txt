cmake_minimum_required(VERSION 3.5)

# Set the project name

# Add an executable

set(utils_test_SRCS utils_test.cc)

cs_add_executable(utils_test ${utils_test_SRCS})
target_link_libraries(utils_test ${PROJECT_NAME} gtest)

cs_add_executable(io_test io_function_test.cc)
target_link_libraries(io_test ${PROJECT_NAME} gtest)
