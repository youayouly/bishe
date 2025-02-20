cmake_minimum_required(VERSION 3.10)
project(CameraCalibration)

find_package(OpenCV REQUIRED)

add_executable(calibration check_image.cpp)
target_link_libraries(calibration ${OpenCV_LIBS})
