cmake_minimum_required(VERSION 3.0.2)
project(rl_local_planner)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

add_message_files(
  DIRECTORY msg
  FILES NavCmd.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs nav_msgs sensor_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/rl_local_planner_node.py
  scripts/verify_lidar_sectors.py
  scripts/analyze_thrust_baseline.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
