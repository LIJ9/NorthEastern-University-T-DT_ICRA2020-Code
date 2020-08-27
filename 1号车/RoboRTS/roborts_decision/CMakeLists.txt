project(roborts_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  roborts_costmap
  )

find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
  )

catkin_package()

add_executable(behavior_test_node
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  my_behavior/addhp_behavior.h
  my_behavior/shoot_behavior.h
  my_behavior/addbullet_behavior.h
  my_behavior/support_behavior.h
  my_behavior/counter_behavior.h
  my_behavior/myescape_behavior.h
  my_behavior/mychase_behavior.h
  my_behavior/mypatrol_behavior.h
  my_behavior/lidardetected_behavior.h
  my_behavior/back_boot_area_behavior.h
  my_behavior/mypatrol_behavior.h
  my_behavior/attack_behavior.h
  robot_information.h
  robot_communicate.h
  referee_system.h
  kill_pending.h
  my_file.h
  executor/chassis_executor.cpp
  behavior_test.cpp
  )

target_link_libraries(behavior_test_node
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(behavior_test_node
  roborts_msgs_generate_messages)
add_executable(decision_simulate src/decision_simulate.cpp)
target_link_libraries(decision_simulate  ${catkin_LIBRARIES})
add_executable(send_vel src/send_vel.cpp )
target_link_libraries(send_vel  ${catkin_LIBRARIES})
