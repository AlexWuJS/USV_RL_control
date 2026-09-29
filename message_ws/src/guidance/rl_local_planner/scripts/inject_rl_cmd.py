#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL 指令注入器：按剧本向 /guidance/rl_cmd 发 NavCmd（Phase 5 台架验证 + 转向辨识工具）。

用途：
  ① Phase 5 台架验证：速度/航向阶跃，验证 huamo 执行链与换算方向；
  ② 转向辨识（替代遥控器）：通过闭环航向指令测艇的 (期望航向, 转率) 响应——
     指令全程可录（/guidance/rl_cmd），解决了遥控器 RF 直连录不进 bag 的问题。

用法（须先有 guidance_mode:=rl 的 huamo 在跑）：
  rosrun rl_local_planner inject_rl_cmd.py --speed 0.6
      # 恒速 0.6 m/s，航向保持启动时刻的艇体航迹向
  rosrun rl_local_planner inject_rl_cmd.py --speed 0.5 --course-steps "0:8,30:8,-30:8,0:8"
      # 恒速 0.5，航向相对启动时刻依次 0→+30→-30→0 度（各 8 秒）——转向辨识剧本
  rosrun rl_local_planner inject_rl_cmd.py --speed-steps "0.3:10,0.6:10,0.9:10"
      # 速度阶梯 0.3→0.6→0.9（航向保持）
  rosrun rl_local_planner inject_rl_cmd.py --stop
      # 急停（desired_speed=0 持续 3 秒）
"""
import argparse
import math
import time

import rospy
from nav_msgs.msg import Odometry
from rl_local_planner.msg import NavCmd


def quat_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def parse_steps(spec):
    out = []
    for part in spec.split(","):
        v, t = part.split(":")
        out.append((float(v), float(t)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=0.5, help="恒定期望速度 m/s")
    ap.add_argument("--course-deg", type=float, default=None,
                    help="绝对 ENU 航迹向（度）；不给则保持启动时刻航向")
    ap.add_argument("--speed-steps", default="", help='"速度:秒,..." 速度阶梯')
    ap.add_argument("--course-steps", default="", help='"航向增量:秒,..." 航向阶梯（相对启动时刻，度）')
    ap.add_argument("--stop", action="store_true", help="急停 3 秒后退出")
    ap.add_argument("--hz", type=float, default=10.0)
    args = ap.parse_args()

    rospy.init_node("inject_rl_cmd", anonymous=True)
    pub = rospy.Publisher("/guidance/rl_cmd", NavCmd, queue_size=1)

    # 启动时刻的艇体航迹向（作为航向阶梯的基准）：odom_yaw - 91.5°（标定安装角）
    base_course = [0.0]
    def odom_cb(m):
        q = m.pose.pose.orientation
        base_course[0] = quat_yaw(q.x, q.y, q.z, q.w) - math.radians(91.5)
    rospy.Subscriber("/Odometry", Odometry, odom_cb)
    t0 = time.time()
    while time.time() - t0 < 1.0:
        time.sleep(0.05)

    if args.stop:
        seq = [(0.0, base_course[0], 3.0)]
        tag = "急停"
    elif args.speed_steps:
        seq = [(sp, base_course[0], t) for sp, t in parse_steps(args.speed_steps)]
        tag = "速度阶梯"
    elif args.course_steps:
        seq = []
        c = base_course[0]
        if args.course_deg is not None:
            c = math.radians(args.course_deg)
        for d, t in parse_steps(args.course_steps):
            c += math.radians(d)
            seq.append((args.speed, c, t))
        tag = "航向阶梯"
    else:
        c = base_course[0] if args.course_deg is None else math.radians(args.course_deg)
        seq = [(args.speed, c, 1e9)]
        tag = "恒速直航"

    print("注入剧本(%s): %s  @%.0f Hz" % (tag, seq, args.hz))
    r = rospy.Rate(args.hz)
    for sp, c, dur in seq:
        t_end = time.time() + dur
        while time.time() < t_end and not rospy.is_shutdown():
            m = NavCmd()
            m.header.stamp = rospy.Time.now()
            m.desired_speed = sp
            m.desired_course = c
            m.source = 1
            m.supervisor_active = 0
            pub.publish(m)
            r.sleep()
        if rospy.is_shutdown():
            break
    print("剧本执行完毕")


if __name__ == "__main__":
    main()
