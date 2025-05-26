cmake_minimum_required(VERSION 3.0.2)
project(hybrid_astar_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  visualization_msgs
  hybrid_astar_searcher
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs nav_msgs tf tf2 tf2_ros tf2_geometry_msgs grid_map_core grid_map_ros grid_map_msgs visualization_msgs hybrid_astar_searcher
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 原始节点可执行文件
add_executable(hybrid_astar_planner_node src/hybrid_astar_planner_node.cpp)
target_link_libraries(hybrid_astar_planner_node
  ${catkin_LIBRARIES}
  hybrid_astar_searcher
)

# 修复英文版本节点
add_executable(hybrid_astar_planner_node_fixed src/hybrid_astar_planner_node_fixed.cpp)
target_link_libraries(hybrid_astar_planner_node_fixed
  ${catkin_LIBRARIES}
  hybrid_astar_searcher
)

# 增强版本节点
add_executable(hybrid_astar_planner_enhanced src/hybrid_astar_planner_enhanced.cpp)
target_link_libraries(hybrid_astar_planner_enhanced
  ${catkin_LIBRARIES}
  hybrid_astar_searcher
)

# TF广播节点
add_executable(tf_broadcaster_node src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster_node
  ${catkin_LIBRARIES}
)

# 地图生成演示节点
add_executable(demo_map_creator src/simple_demo_node.cpp)
target_link_libraries(demo_map_creator
  ${catkin_LIBRARIES}
)

# 安装目标
install(TARGETS hybrid_astar_planner_node hybrid_astar_planner_node_fixed hybrid_astar_planner_enhanced tf_broadcaster_node demo_map_creator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# 安装launch和rviz文件
install(DIRECTORY launch/ rviz/ maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)