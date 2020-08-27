cmake_minimum_required(VERSION 2.8.3)
project(serial)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  roslib
#package
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #/home/zww/catkin_ws/src/RoboRTS/roborts_decision
)
#添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})
set(BOOST_LIBS thread date_time system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
find_package(Threads REQUIRED)

set(SRCS src/main.cpp src/AsyncSerial.cpp src/StatusPublisher.cpp)
add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "")

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
add_executable(recv src/recv.cpp)
target_link_libraries(recv ${catkin_LIBRARIES})
add_executable(add_bullet src/add_bullet.cpp)
target_link_libraries(add_bullet  ${catkin_LIBRARIES} )
add_executable(bringup_pose_icra src/icra_tdt_pose.cpp)
target_link_libraries(bringup_pose_icra  ${catkin_LIBRARIES} )
add_executable(usart src/mains.cpp src/usart.cpp)
target_link_libraries(usart  ${catkin_LIBRARIES} ${OpenCV_LIBS} -lpthread)
add_executable(usart_ch src/mains_ch.cpp src/usart_ch.cpp)
target_link_libraries(usart_ch  ${catkin_LIBRARIES} ${OpenCV_LIBS} -lpthread)
add_executable(serial_ch src/main_ch.cpp src/AsyncSerial.cpp src/StatusPublisher_ch.cpp)
target_link_libraries(serial_ch  ${catkin_LIBRARIES})
add_executable(sendn src/send.cpp )
target_link_libraries(sendn  ${catkin_LIBRARIES})
add_executable(listen_tf src/listen_tf.cpp )
target_link_libraries(listen_tf  ${catkin_LIBRARIES})
add_executable(send_angle src/send_angle.cpp)
target_link_libraries(send_angle ${catkin_LIBRARIES})
