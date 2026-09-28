#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""36 扇区雷达观测离线核对（岸上/码头静态验证，不碰艇不改参数）。

目的：用与实艇 rl_local_planner_node.py **完全相同**的 rl_common.LidarSectorizer
     和 hull_to_sensor_yaw 旋转，离线重放 bag 里的 /cloud_registered_body，
     输出 36 维扇区最近距离向量，验证三件事：
       1. 已知目标出现在正确扇区（验证 hull_to_sensor_yaw 标定）；
       2. 实测最近距离 ≈ 目标真实距离（验证测距/量程链路）；
       3. 高度带 [-0.3, +1.0] m 过滤是否符合预期（验证高度带）。
     这一步通过 = NX 观测与训练机 raycast 观测同源可信（同一份 rl_common）。

岸上操作流程：
  1. 艇静止（岸上或码头），启动 bringup 使定位链路出 /cloud_registered_body；
  2. 在艇旁放挡板/纸箱：距离 3~8 m，高度在传感器原点上下 [-0.3,+1.0] m 内，
     记下「艇体坐标方位角」——0=艇首，+90=左舷，-90=右舷，180=艇尾（单位度）；
  3. 录 20~30 s：
       rosbag record --lz4 -O ~/bags/sector_check /cloud_registered_body /Odometry /tf
  4. 离线核对（无需 roscore）：
       rosrun rl_local_planner verify_lidar_sectors.py ~/bags/sector_check_0.bag \
           --expect-bearing 90 --expect-dist 5
  5. 换 2~3 个方位重复（建议至少测 0°艇首 与 90°左舷——若两者互换则说明
     hull_to_sensor_yaw 差了 90°，正是 -91.5° 标定要抓的错位）。

判据（给了 --expect-bearing 时）：
  PASS：argmin 扇区 = 期望方位扇区（±1 扇区），且实测距离 ≈ 期望距离（±--tol）；
  否则 FAIL，打印实际 argmin 与期望差异。
退出码: 0=PASS/未设期望, 2=FAIL, 1=数据问题。

参数默认值自动读 config/rl_planner.yaml（hull_to_sensor_yaw / 高度带 / 量程），
命令行显式指定时以命令行为准。
"""
import argparse
import math
import os
import sys
from collections import Counter

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))

from rl_common import LidarSectorizer  # noqa: E402  与实艇节点同一份实现

import rosbag  # noqa: E402
from sensor_msgs import point_cloud2 as pc2  # noqa: E402

_CFG_PATH = os.path.join(_HERE, "..", "config", "rl_planner.yaml")


def load_cfg(path=_CFG_PATH):
    """读节点参数文件，缺文件/缺键时返回空 dict（用内置默认值兜底）。"""
    try:
        import yaml
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except Exception:  # noqa: BLE001
        return {}


def bearing_name(deg):
    """扇区方位的人话标注。"""
    deg = deg % 360.0
    names = [(0, "艇首"), (90, "左舷"), (180, "艇尾"), (270, "右舷")]
    for d, n in names:
        if abs(((deg - d + 180) % 360) - 180) <= 15:
            return n
    return "左" if deg < 180 else "右"


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="bag 文件路径")
    ap.add_argument("--cloud", default="/cloud_registered_body")
    ap.add_argument("--odom", default="/Odometry")
    ap.add_argument("--t0", type=float, default=0.0, help="只分析 bag 开始后第 t0 秒起")
    ap.add_argument("--t1", type=float, default=0.0, help="0=到结尾")
    ap.add_argument("--hull-yaw", type=float, default=None,
                    help="传感器→艇体安装角(rad)，默认取 rl_planner.yaml")
    ap.add_argument("--h-min", type=float, default=None)
    ap.add_argument("--h-max", type=float, default=None)
    ap.add_argument("--max-range", type=float, default=None)
    ap.add_argument("--expect-bearing", type=float, default=None,
                    help="期望目标方位角(度, 艇体坐标: 0=艇首 +90=左舷)")
    ap.add_argument("--expect-dist", type=float, default=None, help="期望目标距离(m)")
    ap.add_argument("--tol", type=float, default=1.5, help="距离判据容差(m)")
    ap.add_argument("--sector-tol", type=int, default=1, help="扇区判据容差(个)")
    ap.add_argument("--csv", default="", help="可选: 输出每帧扇区向量的csv前缀")
    args = ap.parse_args()

    cfg = load_cfg()
    obs = cfg.get("obs", {}) or {}
    frames = cfg.get("frames", {}) or {}
    hull_yaw = args.hull_yaw if args.hull_yaw is not None else float(frames.get("hull_to_sensor_yaw", 0.0))
    h_min = args.h_min if args.h_min is not None else float(obs.get("h_min", -0.3))
    h_max = args.h_max if args.h_max is not None else float(obs.get("h_max", 1.0))
    max_range = args.max_range if args.max_range is not None else float(obs.get("lidar_max_range", 20.0))
    n_sectors = int(obs.get("n_sectors", 36))

    sectorizer = LidarSectorizer(n_sectors=n_sectors, max_range=max_range,
                                 h_min=h_min, h_max=h_max)

    bag = rosbag.Bag(args.bag, "r")
    t_start = bag.get_start_time()
    t_end = bag.get_end_time()
    t1 = args.t1 if args.t1 > 0 else (t_end - t_start)

    vecs, n_in_band_all, n_total_all, z_in_range = [], [], [], []
    odom_yaws, odom_t = [], []
    frames_used = 0
    for topic, msg, t_bag in bag.read_messages(topics=[args.cloud, args.odom]):
        t_rel = t_bag.to_sec() - t_start
        if t_rel < args.t0 or (args.t1 > 0 and t_rel > args.t1):
            continue
        if topic == args.odom:
            odom_yaws.append(quat_yaw(msg.pose.pose.orientation))
            odom_t.append(t_rel)
            continue
        pts = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"),
                                            skip_nans=True)), dtype=np.float64)
        if pts.size == 0:
            continue
        n_total_all.append(len(pts))
        # --- 与节点 cloud_cb 逐行相同的旋转：传感器体轴 → 艇体体轴 ---
        c, s = math.cos(hull_yaw), math.sin(hull_yaw)
        xy = pts[:, :2] @ np.array([[c, -s], [s, c]]).T
        body = np.column_stack([xy, pts[:, 2]])
        z_in_range.append(body[:, 2])
        band = (body[:, 2] >= h_min) & (body[:, 2] <= h_max)
        n_in_band_all.append(int(band.sum()))
        vecs.append(sectorizer.process(body))
        frames_used += 1
    bag.close()

    if frames_used == 0:
        print("FAIL: 窗口内没有 %s 数据" % args.cloud)
        sys.exit(1)

    V = np.vstack(vecs)                      # (frames, 36)
    med = np.median(V, axis=0)
    worst = np.min(V, axis=0)
    n_band = np.median(n_in_band_all)
    n_tot = np.median(n_total_all)
    zs = np.concatenate(z_in_range) if z_in_range else np.zeros(1)

    # 艇是否保持静止（转了的话扇区会被抹匀）
    static_txt = "未录 odom，无法核查艇是否静止"
    if odom_yaws:
        y = np.unwrap(np.array(odom_yaws))
        sway = (y.max() - y.min()) * 180.0 / math.pi
        static_txt = "艇首摆动 %.1f° %s" % (sway, "OK(静止)" if sway < 5.0 else "!! 艇在动，扇区被抹匀，结果不可信")

    print("=" * 72)
    print("bag: %s   窗口: %.0f~%.0fs   帧数: %d   hull_to_sensor_yaw=%.4f rad (%.1f°)"
          % (os.path.basename(args.bag), args.t0, args.t0 + t1, frames_used,
             hull_yaw, math.degrees(hull_yaw)))
    print("高度带 [%.2f, %.2f] m: 帧内中位 %d/%d 点入带 (%.0f%%)   全量 z 分位: P5=%.2f P50=%.2f P95=%.2f m"
          % (h_min, h_max, n_band, n_tot, 100.0 * n_band / max(n_tot, 1),
             np.percentile(zs, 5), np.percentile(zs, 50), np.percentile(zs, 95)))
    print("静止核查: %s" % static_txt)
    print("-" * 72)
    print("%4s %8s %10s %10s  %s" % ("扇区", "方位°", "中位距离", "最差距离", "标注"))
    for i in range(n_sectors):
        b = i * 360.0 / n_sectors
        m, w = med[i], worst[i]
        ms = "%10.2f" % m if m < max_range - 1e-6 else "%10s" % "-"
        ws = "%10.2f" % w if w < max_range - 1e-6 else "%10s" % "-"
        tag = ""
        if args.expect_bearing is not None:
            exp = int(round(args.expect_bearing * n_sectors / 360.0)) % n_sectors
            if i == exp:
                tag = "  <-- 期望目标扇区"
        print("%4d %8.0f %10s %10s  %s%s" % (i, b, ms, ws, bearing_name(b), tag))

    argmin_med = int(np.argmin(med))
    argmin_worst = int(np.argmin(worst))
    if med.min() >= max_range - 1e-6:
        print("注: 全部扇区中位距离 = 满量程（窗口内没有持续可见障碍）——"
              "岸上静态核对应在量程内放一个已知目标")
    cnt = Counter(int(np.argmin(v)) for v in vecs)
    modal, modal_n = cnt.most_common(1)[0]
    print("-" * 72)
    print("argmin(中位向量): 扇区 %d (%.0f°%s) 距离 %.2f m"
          % (argmin_med, argmin_med * 360.0 / n_sectors, bearing_name(argmin_med * 10), med[argmin_med]))
    print("逐帧 argmin 众数: 扇区 %d (%d/%d 帧)   argmin(最差): 扇区 %d"
          % (modal, modal_n, frames_used, argmin_worst))

    if args.csv:
        np.savetxt(args.csv + "_sectors.csv", V, delimiter=",",
                   header=",".join("s%02d" % i for i in range(n_sectors)), comments="")
        print("已写 %s_sectors.csv" % args.csv)

    # --- 期望核对 ---
    if args.expect_bearing is None:
        sys.exit(0)
    exp = int(round(args.expect_bearing * n_sectors / 360.0)) % n_sectors
    diff = abs(argmin_med - exp)
    diff = min(diff, n_sectors - diff)
    print("-" * 72)
    print("期望: 方位 %.0f° → 扇区 %d" % (args.expect_bearing, exp))
    ok_idx = diff <= args.sector_tol
    print("扇区核对: argmin=%d 期望=%d (相差%d扇区) %s"
          % (argmin_med, exp, diff, "PASS" if ok_idx else "FAIL"))
    ok_dist = True
    if args.expect_dist is not None:
        got = med[argmin_med]
        ok_dist = abs(got - args.expect_dist) <= args.tol and got < max_range - 1e-6
        print("距离核对: 实测 %.2f m 期望 %.2f m (±%.1f) %s"
              % (got, args.expect_dist, args.tol, "PASS" if ok_dist else "FAIL"))
    sys.exit(0 if (ok_idx and ok_dist) else 2)


if __name__ == "__main__":
    main()
