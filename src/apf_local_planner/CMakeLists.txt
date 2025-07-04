cmake_minimum_required(VERSION 3.0.2)
project(apf_local_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  nav_core
  costmap_2d
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  angles
  visualization_msgs
  dwa_local_planner
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apf_local_planner
  CATKIN_DEPENDS
    roscpp pluginlib nav_core costmap_2d geometry_msgs tf2_ros tf2_geometry_msgs angles
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(apf_local_planner
  src/apf_local_planner.cpp
)
target_link_libraries(apf_local_planner ${catkin_LIBRARIES})
