#!/bin/bash

current_path=$(pwd)

# 切换到workspace目录下
cd "$current_path"/imu_ws

source ./devel/setup.bash

# 启用roslaunch
roslaunch sanchi_amov imu_100D4.launch

# 切换回原来的目录
cd "$current_path"
