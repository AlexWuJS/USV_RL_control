#!/bin/bash

current_path=$(pwd)

# 切换到workspace目录下
cd "$current_path"/rplidar_ws

source ./devel/setup.bash

# 启用roslaunch
roslaunch rplidar_ros rplidar_s2.launch

# 切换回原来的目录
cd "$current_path"
