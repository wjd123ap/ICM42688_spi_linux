cmake_minimum_required(VERSION 3.0.2)
project(imu_test)
include_directories(include src)
add_library(spi_lib
    src/IMU_spi.c src/ICM42688.cpp
)
add_executable(imutest imu_test.cpp)
target_link_libraries(imutest spi_lib)
