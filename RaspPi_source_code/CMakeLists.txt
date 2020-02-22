cmake_minimum_required(VERSION 3.13)  # CMake version check
project(Comp_Vision)               # Create project "simple_example"
set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_BUILD_TYPE Release)

find_package( OpenCV REQUIRED )
find_package(realsense2 REQUIRED)

add_executable( Comp_Vision Comp_Vision.cpp )
include_directories(include ${realsense_INCLUDE_DIR})
target_link_libraries( Comp_Vision ${OpenCV_LIBS} ${realsense2_LIBRARY} )

