cmake_minimum_required(VERSION 3.0)

project(kalman_cpp)

set(CMAKE_CXX_STANDARD 11)

# Add Eigen library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add kalman filter header file
include_directories(include)
set(HEADER_FILES include/kalman.h)

# Add main executable
add_executable(kalman_cpp main.cpp ${HEADER_FILES})
