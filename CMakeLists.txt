cmake_minimum_required(VERSION 3.13)
project(rover_recorder CXX)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(rover_recorder src/main.cpp src/data_recorder.cpp src/stream_manager.cpp src/gpio_manager.cpp)
add_executable(os_wdt_toggle src/os_wdt_toggle.cpp)

set(OrbbecSDK_DIR "/home/rock/camera_test/OrbbecSDK")
find_package(OrbbecSDK REQUIRED)
target_link_libraries(${PROJECT_NAME} OrbbecSDK::OrbbecSDK)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)
include_directories(${GPIOD_INCLUDE_DIRS})
target_link_libraries(rover_recorder ${GPIOD_LIBRARIES})
target_link_libraries(os_wdt_toggle ${GPIOD_LIBRARIES})

include_directories("include")
