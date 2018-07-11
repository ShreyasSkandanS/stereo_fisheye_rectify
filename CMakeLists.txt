cmake_minimum_required(VERSION 2.8.3)
project(stereo_fisheye_rectify)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11 -Wall")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge sensor_msgs image_transport)

## Declare a catkin package
catkin_package()

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
message(STATUS ${OpenCV_INCLUDE_DIRS})
message(STATUS ${OpenCV_LIBRARY_DIRS})
message(STATUS ${OpenCV_LIBS})

# Include CATKIN packages
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(stereo_fr src/rectify_main.cpp)
target_link_libraries(stereo_fr ${catkin_LIBRARIES} ${OpenCV_LIBS})
