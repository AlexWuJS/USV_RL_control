cmake_minimum_required(VERSION 2.8.3)
project(gps_common CXX)

############
## Catkin ##
############

set(DEPS
  message_filters
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  rospy
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${DEPS}
)

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
    GPSStatus.msg
    GPSFix.msg
)

generate_messages(DEPENDENCIES 
  nav_msgs
  sensor_msgs
  std_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    ${DEPS}
)

###########
## Build ##
###########

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}/utm_odometry_node src/utm_odometry_node.cpp)
add_executable(${PROJECT_NAME}/utm_odometry_to_navsatfix_node src/utm_odometry_to_navsatfix_node.cpp)

add_executable(full_odom src/full_odom.cpp)

add_executable(odom_tf src/odom_tf.cpp)

add_executable(fuse src/fuse.cpp)

add_executable(pub_path src/pub_path_node.cpp)


target_link_libraries(pub_path
  ${catkin_LIBRARIES}
)

target_link_libraries(full_odom
  ${catkin_LIBRARIES}
)
target_link_libraries(odom_tf
  ${catkin_LIBRARIES}
)

target_link_libraries(fuse
  ${catkin_LIBRARIES}
)


set_target_properties(${PROJECT_NAME}/utm_odometry_node PROPERTIES OUTPUT_NAME "utm_odometry_node")
target_link_libraries(${PROJECT_NAME}/utm_odometry_node ${catkin_LIBRARIES})

set_target_properties(${PROJECT_NAME}/utm_odometry_to_navsatfix_node PROPERTIES OUTPUT_NAME "utm_odometry_to_navsatfix_node")
target_link_libraries(${PROJECT_NAME}/utm_odometry_to_navsatfix_node ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}/utm_odometry_node
  ${PROJECT_NAME}_generate_messages_cpp 
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}/utm_odometry_to_navsatfix_node
  ${PROJECT_NAME}_generate_messages_cpp 
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}/utm_odometry_node
  ${PROJECT_NAME}/utm_odometry_to_navsatfix_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/fix_translator scripts/bag_converter
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
