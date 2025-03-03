#!/bin/bash
# Description: Install ROS Noetic and other dependencies

sudo apt-get update

# message_ws 所需要的依赖
#------------------------------------------------------------
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