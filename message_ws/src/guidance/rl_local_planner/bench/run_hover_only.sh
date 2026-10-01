#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source /home/jetson/haique/little-boat-motion-control/message_ws/devel/setup.bash
B=/home/jetson/haique/little-boat-motion-control/message_ws/src/guidance/rl_local_planner/bench
cd $B
SELF="$ $$"
for pid in $(pgrep -f "huamo_node"; pgrep -f "rosmaster"); do
    case " $SELF " in *" $pid "*) continue ;; esac
    kill -9 "$pid" 2>/dev/null
done
sleep 2
nohup roscore > roscore.log 2>&1 &
RC=$!
sleep 5
nohup rosrun huamo huamo_node _guidance_mode:=rl _speed_max:=1.0 > huamo_h.log 2>&1 &
HM=$!
sleep 3
nohup python3 $B/hover_only.py > hover_test.log 2>&1 &
sleep 26
kill -INT $HM 2>/dev/null; sleep 1; kill -9 $HM 2>/dev/null
kill -INT $RC 2>/dev/null; sleep 1; kill -9 $RC 2>/dev/null
echo done
