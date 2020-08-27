cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)
SET(CMAKE_CXX_FLAGS "-std=c++0x")
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge image_transport)

find_package(OpenCV 3 REQUIRED)
## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES}
                             ${OpenCV_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES}
                               ${OpenCV_LIBRARIES})
                               
#target_link_libraries( DisplayImage ${OpenCV_LIBS} )
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
