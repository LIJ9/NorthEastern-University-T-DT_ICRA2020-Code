project(a_star_planner_red2)
set(CMAKE_BUILD_TYPE Release)
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto AStarPlannerConfigProtoSrc AStarPlannerConfigProtoHds ${ProtoFiles})

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  SHARED
  ${AStarPlannerConfigProtoSrc}
  ${AStarPlannerConfigProtoHds}
  a_star_planner.cpp
  )
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  roborts_costmap_red2
)
add_library(planning::global_planner::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
