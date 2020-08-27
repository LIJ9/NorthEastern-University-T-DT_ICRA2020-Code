cmake_minimum_required(VERSION 3.0)
project(roborts_tracking)
set(CMAKE_BUILD_TYPE Release)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  roborts_msgs
  cv_bridge
  image_transport
  )

if(NOT WIN32)
  ADD_DEFINITIONS("-std=c++11")
  set(CMAKE_CXX_STANDARD 14)
endif(NOT WIN32)
##################################################
include_directories(/home/tdt/roborts_ws/src/RoboRTS/roborts_detection/armor_detection/constraint_set)
include_directories(/home/tdt/roborts_ws/src/RoboRTS/roborts_common/include)
include_directories(/home/tdt/roborts_ws/src/RoboRTS/roborts_detection/util)
##################################################
catkin_package()

add_library(kcf_tracker STATIC
  KCFcpp/src/fhog.cpp
  KCFcpp/src/kcftracker.cpp)
target_link_libraries(kcf_tracker ${OpenCV_LIBRARIES})
target_include_directories(kcf_tracker PRIVATE KCFcpp/src)

add_executable(roborts_tracking_test
  tracking_test.cpp
  tracking_utility.cpp)
target_link_libraries(roborts_tracking_test
  kcf_tracker
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES})
target_include_directories(roborts_tracking_test
  PUBLIC
  ${catkin_INCLUDE_DIRS})
