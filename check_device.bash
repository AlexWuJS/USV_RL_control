#!/bin/bash

RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color，用于恢复默认颜色

# 检查是否检测到设备：与 imu 相关的设备
if ls /dev/*imu* 1> /dev/null 2>&1; then
    echo "检测到设备： imu "
else
    echo "${RED}设备未接入： imu ${NC}"
fi

# 检查是否检测到设备：与 rtk 相关的设备
if ls /dev/*lidar* 1> /dev/null 2>&1; then
    echo "检测到设备： lidar "
else
    echo "${RED}设备未接入： lidar \n${NC}"
fi

# 检查是否检测到设备：与 rtk 相关的设备
if ls /dev/*rtk* 1> /dev/null 2>&1; then
    echo "检测到设备： rtk "
else
    echo "${RED}设备未接入： rtk \n${NC}"
fi

# 检查是否检测到设备：与 rtk 相关的设备
if ls /dev/*32port* 1> /dev/null 2>&1; then
    echo "检测到设备： stm32F4 "
else
    echo "${RED}设备未接入： stm32F4 \n${NC}"
fi

