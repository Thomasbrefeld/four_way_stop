cmake_minimum_required(VERSION 3.0.2)
project(four_way_stop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  piksi_multi_rtk_ros
  dbw_polaris_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/commander.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(four_way_stop_node src/intersection.cpp)
target_link_libraries(four_way_stop_node ${catkin_LIBRARIES})

add_dependencies(four_way_stop_node ${PROJECT_NAME}_gencfg)