#!/bin/bash

current_path=$(pwd)

# 切换到workspace目录下
cd "$current_path"/gps_ws

source ./devel/setup.bash

# 启用roslaunch
roslaunch gps_common utm_odometry.launch

# rosrun gps_common fuse				# 发布odom_local坐标系
# rosrun gps_common odom_tf			# 发布了map->odom_local、odom_local->base_footprint的坐标变换
# rosrun gps_common pub_path_node		# 发布了地面站接受点集的路径

# 切换回原来的目录
cd "$current_path"
