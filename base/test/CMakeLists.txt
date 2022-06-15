cmake_minimum_required(VERSION 3.5)

# Set the project name

# Add an executable

cs_add_executable(base_io_test io_test.cc)
target_link_libraries(base_io_test ${PROJECT_NAME} gtest)
