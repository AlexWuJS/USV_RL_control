#!/bin/bash
# 多目标串规划离线冒烟测试：不需要实艇和传感器，用仓库自带先验地图跑通全链路。
# 用法: bash test_goal_sequence_offline.sh
# 结束时自动清理所有测试进程。输出最后一行 PASS / FAIL。
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../../../../.."   # message_ws
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"
MAP="$WS_DIR/src/guidance/planner/a_star_planner/maps/scans2.yaml"

cleanup() {
    pkill -x a_star_planner_node 2>/dev/null || true
    pkill -x map_server 2>/dev/null || true
    pkill -x rosmaster 2>/dev/null || true
    pkill -x rosout 2>/dev/null || true
    pkill -x roscore 2>/dev/null || true
}
trap cleanup EXIT

cleanup
sleep 1
roscore > /tmp/astar_test_roscore.log 2>&1 &
sleep 5
rosrun map_server map_server "$MAP" /map:=/prior_map > /tmp/astar_test_mapserver.log 2>&1 &
sleep 3
rosrun a_star_planner a_star_planner_node > /tmp/astar_test_node.log 2>&1 &
sleep 2
rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: -40.0, y: -85.0}}}}" > /dev/null
sleep 1

# 4 航点序列（scans2 地图内的自由空间坐标）
python3 "$SCRIPT_DIR/verify_goal_sequence.py" -40 -80 -20 -60 20 -20 30 10
