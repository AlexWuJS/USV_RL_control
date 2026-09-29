#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shadow 安全层触发率归因分析：CSV（RL 决策流）+ bag（环境重建）→ 触发原因分布。

回答三个问题：
  1. 速度封顶是哪条包络压的——P4 障碍包络（d_min 小）还是 P4b 终点包络（s_remaining 小）？
  2. 航向截断的原因——策略输出偏离路径切线多少度？集中在任务哪个阶段？
  3. 策略激进度全貌——course_ref 相对切线的偏差分布（不看截断，看原始意图）。

用法:
  python3 analyze_shadow_supervisor.py --csv shadow_XXXX.csv --bag water_XXXX.bag
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))

from rl_common import PathFeatures  # noqa: E402

import rosbag  # noqa: E402


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_cfg_hull_yaw():
    try:
        import yaml
        cfg = yaml.safe_load(open(os.path.join(_HERE, "..", "config", "rl_planner.yaml")))
        return float(cfg["frames"]["hull_to_sensor_yaw"])
    except Exception:
        return -1.5971


def load_bag(bag_path):
    b = rosbag.Bag(bag_path)
    o_t, o_x, o_y, o_yaw = [], [], [], []
    paths = []           # (bag_t, frame_id, pts Nx2)
    tf_mo = []           # (t, x, y, yaw) map->camera_init
    for topic, msg, tb in b.read_messages(topics=["/Odometry", "/waypoints_path", "/tf"]):
        if topic == "/Odometry":
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
            o_t.append(msg.header.stamp.to_sec())
            o_x.append(msg.pose.pose.position.x)
            o_y.append(msg.pose.pose.position.y)
            o_yaw.append(yaw)
        elif topic == "/waypoints_path":
            pts = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
            if len(pts) >= 2:
                paths.append((tb.to_sec(), msg.header.frame_id or "map", pts))
        else:
            for tr in msg.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "camera_init":
                    q = tr.transform.rotation
                    tf_mo.append((tb.to_sec(), tr.transform.translation.x,
                                  tr.transform.translation.y,
                                  math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                             1.0 - 2.0 * (q.y ** 2 + q.z ** 2))))
    b.close()
    return (np.array(o_t), np.array(o_x), np.array(o_y), np.array(o_yaw),
            paths, np.array(tf_mo) if tf_mo else None)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--hull-yaw", type=float, default=None)
    ap.add_argument("--max-gap", type=float, default=0.3,
                    help="CSV 时刻到最近 odom 的最大对齐间隔(s)")
    args = ap.parse_args()

    hull_yaw = args.hull_yaw if args.hull_yaw is not None else load_cfg_hull_yaw()
    o_t, o_x, o_y, o_yaw, paths, tf_mo = load_bag(args.bag)
    if len(o_t) < 10:
        print("bag 无 /Odometry")
        sys.exit(1)

    rows = list(csv.DictReader(open(args.csv)))
    print("CSV %d 行 / bag odom %d 帧 / 规划线 %d 条 / TF 样本 %d"
          % (len(rows), len(o_t), len(paths), len(tf_mo) if tf_mo is not None else 0))

    def tf_at(t):
        if tf_mo is None or not len(tf_mo):
            return 1.0, 0.0, 0.0, 0.0
        k = int(np.argmin(np.abs(tf_mo[:, 0] - t)))
        return (math.cos(tf_mo[k, 3]), math.sin(tf_mo[k, 3]),
                tf_mo[k, 1], tf_mo[k, 2])

    n_speed_cap = n_p4 = n_p4b = n_p4crit = 0
    n_course_cut = 0
    n_aligned = n_unaligned = 0
    dev_all = []            # 策略航向意图偏差（全部行）
    dev_cut = []            # 被截断时的偏差
    dmin_p4 = []
    srem_p4b = []
    phase = {"<5m": [0, 0], "5-15m": [0, 0], ">15m": [0, 0]}   # [总行, 触发行]

    for r in rows:
        try:
            t = float(r["t"])
            v_ref = float(r["v_ref"]); c_ref = float(r["course_ref"])
            c_cmd = float(r["cmd_course"]); s_cmd = float(r["cmd_speed"])
            d_min = float(r["d_min"])
        except (ValueError, KeyError):
            continue
        i = int(np.searchsorted(o_t, t))
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(o_t):
                d = abs(o_t[j] - t)
                if best is None or d < best[0]:
                    best = (d, j)
        if best is None or best[0] > args.max_gap:
            n_unaligned += 1
            continue
        j = best[1]
        psi = wrap(o_yaw[j] + hull_yaw)
        # 最近一条 t<=当前+2s 的规划线 → camera_init 系
        path = None
        for pt, fid, pts in paths:
            if pt <= t + 2.0:
                path = (fid, pts)
        if path is None:
            n_unaligned += 1
            continue
        fid, pts = path
        if fid == "camera_init":
            pts_w = pts
        else:
            co, so, tx, ty = tf_at(t)
            pts_w = np.column_stack([co * pts[:, 0] - so * pts[:, 1] + tx,
                                     so * pts[:, 0] + co * pts[:, 1] + ty])
        feat = PathFeatures(pts_w, la1_dist=6.0, la2_dist=15.0).update_full(
            [o_x[j], o_y[j]], psi)
        s_rem = feat.s_remaining
        tangent = feat.tangent_course
        n_aligned += 1

        dev = abs(wrap(c_ref - tangent))
        dev_all.append(dev)
        sup = str(r.get("supervisor", "0")) == "1"
        bkt = "<5m" if s_rem < 5 else ("5-15m" if s_rem < 15 else ">15m")
        if sup:
            phase[bkt][1] += 1
        phase[bkt][0] += 1

        speed_capped = s_cmd < v_ref - 1e-6
        course_cut = abs(wrap(c_cmd - c_ref)) > 1e-6
        if not (speed_capped or course_cut):
            continue
        # 归因
        attributed = False
        if speed_capped:
            v_env_d = sv_speed_envelope(d_min)
            v_env_g = sv_goal_envelope(s_rem)
            bound = min(v_env_d, v_env_g)
            if abs(s_cmd - bound) < 0.02:
                if d_min <= 1.5:
                    n_p4crit += 1
                elif v_env_d <= v_env_g:
                    n_p4 += 1
                    dmin_p4.append(d_min)
                else:
                    n_p4b += 1
                    srem_p4b.append(s_rem)
                n_speed_cap += 1
                attributed = True
        if course_cut:
            n_course_cut += 1
            dev_cut.append(dev)
            attributed = True
        if not attributed:
            print("  !! 未归因触发: t=%.1f v_ref=%.2f cmd=%.2f d_min=%.2f s_rem=%.1f"
                  % (t, v_ref, s_cmd, d_min, s_rem))

    def pct(x):
        return "  ".join("P%d=%.0f°" % (p, math.degrees(np.percentile(x, p)))
                         for p in (50, 90, 99)) if x else "-"

    print("=" * 76)
    print("对齐 %d 行（未对齐 %d）" % (n_aligned, n_unaligned))
    print("触发: 速度封顶 %d（P4 障碍 %d / P4b 终点 %d / critical %d）  航向截断 %d"
          % (n_speed_cap, n_p4, n_p4b, n_p4crit, n_course_cut))
    print("策略航向意图 |course_ref-切线| 全体分布: %s" % pct(dev_all))
    if dev_cut:
        print("被截断时的原始偏差: %s" % pct(dev_cut))
    if dmin_p4:
        print("P4 触发时 d_min 分布: %s" % pct(dmin_p4))
    if srem_p4b:
        print("P4b 触发时 s_remaining 分布: %s" % pct(srem_p4b))
    print("按阶段: " + "  ".join("%s: %d/%d(%.0f%%)" % (k, v[1], v[0],
          100.0 * v[1] / max(v[0], 1)) for k, v in phase.items()))


def sv_speed_envelope(d_min, slow=8.0, crawl=3.0, crit=1.5, v_max=2.0, crawl_v=0.4):
    if d_min >= slow:
        return v_max
    if d_min > crawl:
        f = (slow - d_min) / (slow - crawl)
        return v_max + f * (crawl_v - v_max)
    if d_min > crit:
        f = (crawl - d_min) / (crawl - crit)
        return crawl_v * (1.0 - f)
    return 0.0


def sv_goal_envelope(s_rem, slow=15.0, stop=5.0, v_max=2.0, gv=0.4):
    if s_rem is None or s_rem >= slow:
        return v_max
    if s_rem > stop:
        f = (slow - s_rem) / (slow - stop)
        return v_max + f * (gv - v_max)
    return gv


if __name__ == "__main__":
    main()
