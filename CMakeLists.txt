project(CarWheelVelocities)
cmake_minimum_required(VERSION 3.5)
add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})



set(sources src/main.cpp src/Wheel.cpp src/Wheel.h)

add_executable(CarWheelVelocities ${sources})
target_link_libraries( CarWheelVelocities ${OpenCV_LIBS} )
