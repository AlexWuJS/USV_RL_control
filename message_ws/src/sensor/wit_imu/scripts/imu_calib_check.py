#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IMU 陀螺校准验收工具。

流程:
  阶段1  静置 30s 测陀螺 Z 轴零偏 → 预测漂移率（快速预判）
  阶段2  继续静置到 600s 测 yaw 实际漂移率（最终判定）

判定标准:
  漂移 <0.2°/min  = PASS（可用）
  <1°/min         = 勉强（每次任务前必须记初始偏置）
  >=1°/min        = FAIL（模块不可用，考虑换 sanchi_amov 100D4）

前提: 系统刚断电重启、人离开艇静置 2 分钟完成陀螺自校准；
      测试全程不得触碰艇体（走动/靠艇都会污染数据）。

用法:
  python3 imu_calib_check.py          # 完整验收（约 11 分钟）
  python3 imu_calib_check.py --fast   # 只测零偏（约 35 秒）

退出码: 0=PASS  2=FAIL  3=无数据
"""
import argparse
import math
import statistics
import sys
import time

import rospy
from sensor_msgs.msg import Imu

RAD2DEG = 180.0 / math.pi
DRIFT_PASS = 0.2   # °/min
DRIFT_MARGINAL = 1.0


def unwrap_delta(prev, cur):
    d = cur - prev
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fast', action='store_true', help='只测 30s 零偏')
    ap.add_argument('--drift-sec', type=float, default=600.0)
    args = ap.parse_args()

    rospy.init_node('imu_calib_check', anonymous=True)
    samples = []  # (t, yaw_deg, wz)

    def cb(m):
        q = m.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * RAD2DEG
        samples.append((time.time(), yaw, m.angular_velocity.z))

    rospy.Subscriber('/imu/data', Imu, cb)

    print('等待数据...（无数据请检查 /imu/data 是否在发布）')
    t0 = time.time()
    while not samples and time.time() - t0 < 10:
        time.sleep(0.2)
    if not samples:
        print('FAIL: 10 秒内无 /imu/data')
        return 3

    # ---- 阶段1: 30s 零偏 ----
    print('阶段1: 静置测陀螺零偏 30s（请勿触碰艇体）...')
    time.sleep(30.0)
    wz = [s[2] for s in samples]
    mean_wz = statistics.mean(wz)
    std_wz = statistics.pstdev(wz) if len(wz) > 1 else 0.0
    pred = mean_wz * RAD2DEG * 60.0
    print('  样本 %d, 零偏均值 %.6f rad/s (%.4f °/s), 噪声标准差 %.6f rad/s'
          % (len(wz), mean_wz, mean_wz * RAD2DEG, std_wz))
    print('  按此零偏预测漂移率: %.3f °/min' % pred)
    bias_ok = abs(pred) < DRIFT_PASS
    print('  零偏判定: %s' % ('PASS' if bias_ok
                              else 'FAIL（零偏偏大，先用维特上位机做陀螺校准）'))

    if args.fast:
        return 0 if bias_ok else 2

    # ---- 阶段2: yaw 漂移 ----
    remain = max(args.drift_sec - 30.0, 0.0)
    print('阶段2: 继续静置 %.0f s 测 yaw 漂移...' % remain)
    time.sleep(remain)

    ys = [s[1] for s in samples]
    ts = [s[0] for s in samples]
    total = 0.0
    for i in range(1, len(ys)):
        total += unwrap_delta(ys[i - 1], ys[i])
    dt = ts[-1] - ts[0]
    rate = total / dt * 60.0 if dt > 0 else 0.0
    print('  时长 %.0f s, yaw 首尾差 %.2f°, 实测漂移率 %.3f °/min'
          % (dt, total, rate))

    if abs(rate) < DRIFT_PASS:
        verdict = 'PASS — 陀螺校准成功，可用'
    elif abs(rate) < DRIFT_MARGINAL:
        verdict = '勉强 — 每次任务前须记录初始偏置，且任务时长受限'
    else:
        verdict = 'FAIL — 模块不可用：重新校准仍如此则换 sanchi_amov 100D4'
    print('结论: %s' % verdict)
    return 0 if abs(rate) < DRIFT_PASS else 2


if __name__ == '__main__':
    sys.exit(main())
