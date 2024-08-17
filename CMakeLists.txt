cmake_minimum_required(VERSION 3.13)
project(rover_recorder CXX)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(rover_recorder src/main.cpp src/data_recorder.cpp src/stream_manager.cpp)

set(OrbbecSDK_DIR "/home/rock/camera_test/OrbbecSDK")
find_package(OrbbecSDK REQUIRED)
target_link_libraries(${PROJECT_NAME} OrbbecSDK::OrbbecSDK)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

include_directories("include")
