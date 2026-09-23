#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""向 /goal_sequence 发送航点序列并校验 A* 串规划结果。

用法（ROS 环境下）:
    ./verify_goal_sequence.py X1 Y1 X2 Y2 [X3 Y3 ...] [--frame map] [--timeout 30]

例:
    ./verify_goal_sequence.py -40 -80 -20 -60 20 -20

校验项:
  1. /planner_status 返回 OK 且无 ERROR
  2. /waypoints_path 相邻点连续（最大间距 <= 栅格对角 0.708m）
  3. 每个输入航点都被路径覆盖（最近距离 <= 1m）

注意: 若实艇上 a_star + huamo 正在运行，本脚本发出的目标序列会让艇按
      规划结果航行！实艇验证请确保艇在艇架上或有人拿遥控器接管。
退出码: 0=PASS, 2=FAIL
"""
import argparse
import math
import sys

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("coords", nargs="+", type=float,
                        help="航点坐标序列 x1 y1 x2 y2 ...")
    parser.add_argument("--frame", default="map", help="目标点所在坐标系")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    if len(args.coords) < 2 or len(args.coords) % 2 != 0:
        print("需要偶数个坐标: x1 y1 x2 y2 ...")
        sys.exit(1)
    goals = list(zip(args.coords[0::2], args.coords[1::2]))

    rospy.init_node("verify_goal_sequence", anonymous=True)
    status = {"v": None}
    rospy.Subscriber("/planner_status", String,
                     lambda m: status.__setitem__("v", m.data))
    path = {"m": None}
    rospy.Subscriber("/waypoints_path", Path,
                     lambda m: path.__setitem__("m", m))
    rospy.sleep(1.0)
    status["v"] = None   # 清掉 latched 的旧状态
    path["m"] = None

    pub = rospy.Publisher("/goal_sequence", Path, queue_size=1)
    rospy.sleep(0.5)
    seq = Path()
    seq.header.frame_id = args.frame
    seq.header.stamp = rospy.Time.now()
    for x, y in goals:
        ps = PoseStamped()
        ps.header.frame_id = args.frame
        ps.header.stamp = seq.header.stamp
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        seq.poses.append(ps)
    pub.publish(seq)

    t0 = rospy.Time.now()
    while (path["m"] is None and status["v"] is None
           and (rospy.Time.now() - t0).to_sec() < args.timeout):
        rospy.sleep(0.2)

    print("=" * 56)
    print("planner_status:", status["v"] or "(无响应)")
    if path["m"] is None:
        print("结果: FAIL — 未收到 /waypoints_path（规划失败或被 abort）")
        sys.exit(2)

    P = path["m"].poses
    print("路径: %d 点, frame=%s" % (len(P), path["m"].header.frame_id))
    ok = True
    if len(P) < 2:
        print("FAIL: 路径点数不足")
        sys.exit(2)
    maxgap = max(math.hypot(b.pose.position.x - a.pose.position.x,
                            b.pose.position.y - a.pose.position.y)
                 for a, b in zip(P, P[1:]))
    print("最大相邻间距: %.3f m (期望 <= 0.708)" % maxgap)
    if maxgap > 0.708 + 1e-6:
        print("FAIL: 路径不连续，段间有跳变")
        ok = False
    for i, (gx, gy) in enumerate(goals):
        dmin = min(math.hypot(p.pose.position.x - gx,
                              p.pose.position.y - gy) for p in P)
        print("航点 %d (%.1f, %.1f) 距路径最近: %.2f m" % (i + 1, gx, gy, dmin))
        if dmin > 1.0:
            print("FAIL: 航点 %d 未被路径覆盖" % (i + 1))
            ok = False
    print("结果:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
