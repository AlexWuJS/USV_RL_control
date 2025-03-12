#!/bin/bash
# Description: Install ROS Noetic and other dependencies

# 显示说明文字（中文）：选择所需要安装的依赖
echo "选择所需要安装的依赖：[1] 所有依赖（默认） [2] 安装message_ws所需要的依赖"
read -p "请输入数字（默认1）：" choice

# 如果输入为空，则默认选择1
if [ -z "$choice" ]; then
    choice=1
fi  


sudo apt-get update

# message_ws 所需要的依赖
#------------------------------------------------------------
# =如果选择1或2，则安装message_ws所需要的依赖
if [ "$choice" -eq 1 ] || [ "$choice" -eq 2 ]; then
    # 安装 ros-noetic-catkin-virtualenv
    sudo apt-get install -y ros-noetic-catkin-virtualenv
    # 安装 gps_common
    sudo apt-get install -y ros-noetic-gps-common
    # 安装 GPSD 和 libgps
    sudo apt-get install -y gpsd libgps-dev
    # USV/USV_nav缺少功能包
    # 安装amcl, gmapping
    sudo apt-get install -y ros-noetic-navigation
    sudo apt-get install -y ros-noetic-slam-gmapping
    # 安装Eigen3
    sudo apt-get install -y libeigen3-dev
    sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
#------------------------------------------------------------
# 检查安装是否成功
if [ $? -eq 0 ]; then
    echo "All packages installed successfully."
else
    echo "Error occurred during installation."
    exit 1
fi