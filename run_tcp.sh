#!/bin/bash

current_path=$(pwd)

# 切换到workspace目录下
cd "$current_path"/message_ws

source ./devel/setup.bash

# 启用roslaunch
roslaunch tcp tcp.launch

# 切换回原来的目录
cd "$current_path"