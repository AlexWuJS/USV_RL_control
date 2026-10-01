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
rosnode list >/dev/null 2>&1 || { echo "roscore 启动失败"; kill $RC; exit 1; }
echo "roscore OK"
nohup rosrun huamo huamo_node _guidance_mode:=rl _speed_max:=1.0 > huamo.log 2>&1 &
HM=$!
sleep 3
rosnode list 2>/dev/null | grep -q huamo_node && echo "huamo OK" || { echo "huamo 未注册"; kill $RC; exit 1; }
timeout 90 python3 $B/bench_phase5.py > result.txt 2>&1
BENCH_RC=$?
echo "test exit=$BENCH_RC"
cat result.txt
echo "=== huamo 关键日志 ==="
grep -E "PWM平滑|进入定点悬停|guidance_mode" huamo.log | head -4
kill -INT $HM 2>/dev/null; sleep 1; kill -9 $HM 2>/dev/null
kill -INT $RC 2>/dev/null; sleep 1; kill -9 $RC 2>/dev/null
echo "=== teardown 完成 ==="
exit $BENCH_RC
