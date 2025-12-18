#!/bin/bash

# 1. 设置 ROS 基础环境变量 (根据你的版本，如果是18.04改 noetic 为 melodic)
source /opt/ros/noetic/setup.bash

# 2. 设置你自己的工作空间环境变量
# 注意：这里写死绝对路径，防止在不同目录下运行脚本时找不到文件
source ~/haique/little-boat-motion-control/message_ws/devel/setup.bash

# 3. 等待网络硬件准备好 (可选)
# 有时候开机太快，网卡还没加载，ROS就启动了，会导致报错。
# 睡眠 5 秒

# sleep 5

# pactl set-sink-volume @DEFAULT_SINK@ 30%

# echo "设置系统音量为30%"
# 4. 打印提示
echo "========================================"
echo "正在启动系统状态监测节点 (Network Check)..."
echo "========================================"

source ~/haique/little-boat-motion-control/message_ws/devel/setup.bash
# 5. 启动 launch 文件
roslaunch tcp tcp.launch