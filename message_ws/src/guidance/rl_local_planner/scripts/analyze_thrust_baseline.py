#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基线试验离线分析：多档直航 / 满油门 / 回转 → 推进-动力学参数一览表。

输入：按手册 §2.0 录的水试 bag（含 /Odometry、/auto_thruster_pwm；
     键盘回转段需要 /controller —— 键盘模式下 huamo 不发 /auto_thruster_pwm；
     CTE 需要录 /waypoints_path）。

输出（直接打印 + 可选 csv）：
  1. 直航段表：油门档位 → 稳态速度（1s 位移窗真值）、PWM 抖振/摆率、速度波动
  2. 档位表 + PWM→速度线性拟合 v = k·Δpwm + b → 死区、外推 Δpwm=250（huamo
     满档）与 300（键盘满舵）→ v_max_phys 估计；有满档实测时以实测为准
  3. 每段直航相对 /waypoints_path 规划线的 CTE（左正）+ **全局横向偏置矢量
     （东/北分量）** —— 各航向都偏向同一世界方向 → 世界系固定偏置（如 LOS
     +1 m 东偏）；偏置方向随航向旋转 → 艇体侧偏（标定/双桨不对称）
  4. 回转段表：差速 Δ → 稳态转率 r → k_r、r_max 观测值、阶跃 t63 → tau_r 估计

用法（无需 roscore）：
  rosrun rl_local_planner analyze_thrust_baseline.py ~/bags/water_0928_1030_0.bag
  rosrun rl_local_planner analyze_thrust_baseline.py xxx.bag --csv-prefix /tmp/baseline

口径说明：
  - 时间一律用 /Odometry 的 header stamp（录制时刻混入排队延迟，踩过坑）；
  - PWM 无 header，用 bag 接收时刻就近对齐到 odom 时间轴（±0.25 s）；
  - 直航段 = 油门档位稳定且 |差速| < 阈值 ≥ min_leg 秒；回转段 = |差速| >
    阈值 ≥ turn_min 秒；差速 Δ = (左−右)/2，Δ>0 = 右转（r<0）。
"""
import argparse
import math
import sys

import numpy as np
import rosbag

PWM_NEUTRAL = 1500.0
TURN_LOOKBACK_S = 1.5   # 回转段起点前移秒数：给 t63 阶跃检测留基线（见 segment/analyze_turn）


# ---------------------------------------------------------------------------
def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def sliding_median(x, t, win):
    """按时间窗 win 秒的滑动中位数（n 小，直接实现）。"""
    out = np.empty_like(x)
    lo = np.searchsorted(t, t - win / 2.0)
    hi = np.searchsorted(t, t + win / 2.0)
    for i in range(len(x)):
        out[i] = np.median(x[lo[i]:hi[i]])
    return out


def window_rate(v, t, win, signed=False):
    """中心窗位移/转率：v 端点差 / 时长。

    v 为 (n,2) 位置 → 位移速度；v 为 1D yaw → 偏航角速度
    （signed=False 取绝对值）。窗口不足 win/2 时记 0。
    """
    out = np.zeros(len(t))
    lo = np.searchsorted(t, t - win / 2.0)
    hi = np.searchsorted(t, t + win / 2.0)
    for i in range(len(t)):
        a, b = lo[i], hi[i]
        if b - a < 2:
            continue
        dt = t[b - 1] - t[a]
        if dt < win * 0.5:
            continue
        d = v[b - 1] - v[a]
        if np.ndim(d) == 1:
            out[i] = math.hypot(d[0], d[1]) / dt
        else:
            out[i] = (d if signed else abs(d)) / dt
    return out


# ---------------------------------------------------------------------------
def sanity_filter(O, v_max=5.0, r_max=3.0):
    """剔除"瞬移/跳变"样本：恢复数据里错位消息能反序列化成功但位姿是垃圾。

    前向遍历，相对上一个【保留】样本：位移速度 > v_max 或转率 > r_max → 丢弃；
    时间间隙 >2 s 视为断段，重置参考。
    """
    t, x, y, yaw = O["t"], O["x"], O["y"], O["yaw"]
    n = len(t)
    keep = np.zeros(n, dtype=bool)
    last = -1
    for i in range(n):
        if last < 0:
            keep[i] = True
            last = i
            continue
        dt = t[i] - t[last]
        if dt > 2.0:                       # 断段：保留并重置
            keep[i] = True
            last = i
            continue
        if dt <= 1e-3:
            continue                       # 重复/过近时间戳：丢弃后来的
        v = math.hypot(x[i] - x[last], y[i] - y[last]) / dt
        dy = abs(yaw[i] - yaw[last])
        dy = min(dy, 2 * math.pi - dy)
        if v <= v_max and dy / dt <= r_max:
            keep[i] = True
            last = i
        # 否则丢弃该样本（last 不动，下一条继续和 last 比）
    if keep.all():
        return O
    print("位姿 sanity: 剔除 %d/%d 帧（瞬移/跳变/重复戳）" % ((~keep).sum(), n))
    return dict(t=t[keep], x=x[keep], y=y[keep], yaw=yaw[keep])


def load_bag(path, args):
    bag = rosbag.Bag(path, "r")
    have = set(bag.get_type_and_topic_info().topics.keys())

    pwm_topic = None
    for cand in (args.pwm_topic, "/controller"):
        if cand and cand in have:
            pwm_topic = cand
            break

    odom_t, odom_x, odom_y, odom_yaw = [], [], [], []
    odom_frame = [None]
    pwm_t, pwm_l, pwm_r = [], [], []
    paths = []                                   # (bag_t, frame_id, (N,2))
    tf_map_odom = []                             # map→odom(camera_init) 样本 (t, x, y, yaw)
    topics = [t for t in (args.odom_topic, pwm_topic, args.path_topic, args.tf_topic) if t]
    for topic, msg, t_bag in bag.read_messages(topics=topics):
        if topic == args.odom_topic:
            if odom_frame[0] is None:
                odom_frame[0] = msg.header.frame_id or "camera_init"
            odom_t.append(msg.header.stamp.to_sec())
            odom_x.append(msg.pose.pose.position.x)
            odom_y.append(msg.pose.pose.position.y)
            odom_yaw.append(quat_yaw(msg.pose.pose.orientation))
        elif topic == args.path_topic:
            pts = np.array([(p.pose.position.x, p.pose.position.y)
                            for p in msg.poses], dtype=np.float64)
            if len(pts) >= 2:
                paths.append((t_bag.to_sec(), msg.header.frame_id or "map", pts))
        elif topic == args.tf_topic:
            for tr in msg.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "camera_init":
                    tf_map_odom.append(
                        (t_bag.to_sec(), tr.transform.translation.x,
                         tr.transform.translation.y, quat_yaw(tr.transform.rotation)))
        else:
            L, R = float(msg.left_thruster_pwm), float(msg.right_thruster_pwm)
            if not (1100 <= L <= 1900 and 1100 <= R <= 1900):
                continue                       # 损坏数据/错位记录，丢弃
            pwm_t.append(t_bag.to_sec())
            pwm_l.append(L)
            pwm_r.append(R)
    bag.close()

    O = dict(t=np.array(odom_t), x=np.array(odom_x), y=np.array(odom_y),
             yaw=np.unwrap(np.asarray(odom_yaw)))
    O = sanity_filter(O)
    P = dict(t=np.array(pwm_t), l=np.array(pwm_l), r=np.array(pwm_r)) if pwm_t else None

    # 规划线换算到 odom 系（本栈: map→camera_init 近似恒定，取时间最近样本）
    world = odom_frame[0]
    tf_missing = False
    note_tf = ""
    tf_t = np.array([s[0] for s in tf_map_odom])
    tf_xy = np.array([(s[1], s[2]) for s in tf_map_odom])
    tf_yaw = np.array([s[3] for s in tf_map_odom])
    xformed = []
    for pt, fid, pts in paths:
        if fid == world:
            xformed.append((pt, pts))
            continue
        if len(tf_t) == 0:
            tf_missing = True
            continue
        j = int(np.argmin(np.abs(tf_t - pt)))
        c, s = math.cos(tf_yaw[j]), math.sin(tf_yaw[j])
        R = np.array([[c, -s], [s, c]])
        # TF(map→camera_init) 语义: p_map = R·p_cam + t → p_cam = Rᵀ·(p_map − t)
        xformed.append((pt, (pts - tf_xy[j]) @ R))
    if xformed and len(tf_t):
        k = int(np.argmin(np.abs(tf_t - xformed[0][0])))
        note_tf = ("规划线已换算到 %s 系（map→camera_init: t=(%.2f, %.2f) yaw=%.1f°）"
                   % (world, tf_xy[k, 0], tf_xy[k, 1], math.degrees(tf_yaw[k])))
    if tf_missing:
        note_tf = "!! 规划线在 map 系且 bag 无 map→camera_init 的 /tf，这些线被丢弃"
    return O, P, xformed, pwm_topic, note_tf


def align_pwm(O, P, max_gap=0.25):
    """PWM 就近对齐到 odom 时间轴；超 max_gap 记 NaN。"""
    n = len(P["t"])
    fwd = np.full(len(O["t"]), np.nan)
    diff = np.full(len(O["t"]), np.nan)
    for i, j in enumerate(np.searchsorted(P["t"], O["t"])):
        best = None
        for jj in (j - 1, j):
            if 0 <= jj < n:
                d = abs(O["t"][i] - P["t"][jj])
                if best is None or d < best[0]:
                    best = (d, jj)
        if best and best[0] <= max_gap:
            L, R = P["l"][best[1]], P["r"][best[1]]
            fwd[i] = (L + R) / 2.0 - PWM_NEUTRAL
            diff[i] = (L - R) / 2.0            # 正 = 左推大于右推 = 右转
    return fwd, diff


def signed_cte_to_path(path, x, y):
    """点到折线的带符号距离，路径行进方向左侧为正。

    返回 (cte, nx, ny)：n 为最近段单位左法向（用于把 CTE 折算成世界系矢量）。
    """
    a, b = path[:-1], path[1:]                  # (N-1,2)
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
    ap = np.column_stack([x, y])[:, None, :] - a[None, :, :]      # (M,N-1,2)
    s = np.clip((ap * ab[None]).sum(axis=2) / L2[None], 0.0, 1.0)
    proj = a[None] + s[:, :, None] * ab[None]                     # (M,N-1,2)
    d2 = ((np.column_stack([x, y])[:, None, :] - proj) ** 2).sum(axis=2)
    seg = np.argmin(d2, axis=1)
    tang = ab[seg] / np.sqrt(L2[seg])[:, None]
    nrm = np.column_stack([-tang[:, 1], tang[:, 0]])              # 左法向
    closest = proj[np.arange(len(seg)), seg]
    delta = np.column_stack([x, y]) - closest
    cte = (delta * nrm).sum(axis=1)
    return cte, nrm[:, 0], nrm[:, 1]


# ---------------------------------------------------------------------------
def segment(O, fwd_s, diff, valid, min_leg, turn_min, turn_thr, step_thr):
    """先按油门档位分段，再在段内按差速切成直航/回转子段。"""
    segs = []
    n = len(fwd_s)
    i = 0
    while i < n:
        if not valid[i]:
            i += 1
            continue
        j = i
        ref = fwd_s[i]
        while j < n and valid[j] and abs(fwd_s[j] - ref) <= step_thr:
            j += 1
        k = i
        while k < j:
            turning = abs(diff[k]) > turn_thr
            k2 = k
            while k2 < j and (abs(diff[k2]) > turn_thr) == turning:
                k2 += 1
            dur = O["t"][k2 - 1] - O["t"][k]
            if turning and dur >= turn_min:
                # 起点前移 TURN_LOOKBACK_S：阶跃瞬间通常落在回转段边界之前，
                # analyze_turn 的 t63 需要这段低差速基线才能量准上升时间
                k_ext = k
                while k_ext > 0 and O["t"][k] - O["t"][k_ext - 1] <= TURN_LOOKBACK_S:
                    k_ext -= 1
                segs.append(dict(idx=np.arange(k_ext, k2), turning=True))
            elif not turning and dur >= min_leg:
                segs.append(dict(idx=np.arange(k, k2), turning=False))
            k = k2
        i = j
    return segs


def analyze_straight(O, idx, fwd, diff, paths, win_speed, win_cte):
    t, x, y = O["t"][idx], O["x"][idx], O["y"][idx]
    dur = t[-1] - t[0]
    sp = window_rate(np.column_stack([x, y]), t, win_speed)
    core = slice(int(0.1 * len(t)), max(int(0.9 * len(t)), int(0.1 * len(t)) + 1))
    f = fwd[idx]
    slew = np.abs(np.diff(f)) / np.maximum(np.diff(t), 1e-3)
    hdg = math.atan2(y[-1] - y[0], x[-1] - x[0])   # 位移方向 = 行进方向

    out = dict(t0=t[0], dur=dur, level=float(np.median(f)),
               level_std=float(np.std(f)),
               slew95=float(np.percentile(slew, 95)) if len(slew) else 0.0,
               speed=float(np.median(sp[core])), speed_std=float(np.std(sp[core])),
               diff_med=float(np.median(np.abs(diff[idx]))),
               hdg=hdg, cte_mean=None, cte_std=None, bias=(0.0, 0.0), n=len(idx))

    path = None
    for pt, pts in paths:                          # 最近一条 t<=段起点+2s 的规划线
        if pt <= t[0] + 2.0:
            path = pts
    if path is None and paths:
        path = paths[0][1]
    if path is not None:
        cte, nx, ny = signed_cte_to_path(path, x, y)
        out["cte_mean"] = float(cte.mean())
        out["cte_std"] = float(cte.std())
        out["bias"] = (float((cte * nx).mean()), float((cte * ny).mean()))
    return out


def analyze_turn(O, idx, diff, fwd, win_rate):
    t, yaw = O["t"][idx], O["yaw"][idx]
    r = window_rate(yaw, t, win_rate, signed=True)
    core = slice(int(0.15 * len(t)), max(int(0.85 * len(t)), int(0.15 * len(t)) + 1))
    r_ss = float(np.median(r[core]))
    # 稳态差速/油门取段中段：段首含 TURN_LOOKBACK_S 基线，不能进中位数
    d_med = float(np.median(diff[idx][core]))
    f_med = float(np.median(fwd[idx][core]))
    sp = window_rate(np.column_stack([O["x"][idx], O["y"][idx]]), t, win_rate)
    # t63：阶跃响应时间。段内含段前基线（segment() 已前移），
    # 阶跃起点 = 差速首次升到 0.7·|Δ稳态| 之前、最后一个低于 0.3·|Δ稳态| 的采样；
    # 量 |r| 自该点首次达 0.63·|r_ss| 的耗时 —— 一阶系统 tau ≈ t63
    t63 = None
    d = np.abs(diff[idx])
    ra = np.abs(r)
    if abs(r_ss) >= 0.05 and abs(d_med) > 1e-9:   # 与"有效回转"(|r|>0.05) 同门限，避免垃圾段给出假 t63
        lo_thr, hi_thr = 0.3 * abs(d_med), 0.7 * abs(d_med)
        above_hi = np.where(d >= hi_thr)[0]
        if above_hi.size:
            pre = np.where(d[:above_hi[0]] < lo_thr)[0]
            if pre.size:
                on = int(pre[-1])
            else:                       # 段首已在上升段（无基线）：退化为首个过 30% 点
                ge_lo = np.where(d >= lo_thr)[0]
                on = int(ge_lo[0]) if ge_lo.size else None
            if on is not None:
                for i in range(on, len(ra)):
                    if ra[i] >= 0.63 * abs(r_ss):
                        t63 = float(t[i] - t[on])
                        break
    return dict(t0=t[0], dur=t[-1] - t[0], diff=d_med, fwd=f_med, r_ss=r_ss,
                speed=float(np.median(sp[core])), t63=t63)


# ---------------------------------------------------------------------------
def fmt(v, nd=2, dash="-"):
    return ("%.*f" % (nd, v)) if v is not None and np.isfinite(v) else dash


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--odom-topic", default="/Odometry")
    ap.add_argument("--pwm-topic", default="/auto_thruster_pwm",
                    help="自动模式 PWM 话题；缺失时自动退回 /controller")
    ap.add_argument("--path-topic", default="/waypoints_path", help="CTE 参考规划线")
    ap.add_argument("--tf-topic", default="/tf", help="map→camera_init 变换来源")
    ap.add_argument("--min-leg", type=float, default=8.0, help="直航段最短秒数")
    ap.add_argument("--turn-min", type=float, default=5.0, help="回转段最短秒数")
    ap.add_argument("--turn-thr", type=float, default=25.0, help="差速回转阈值(PWM)")
    ap.add_argument("--step-thr", type=float, default=20.0, help="档位跳变阈值(PWM)")
    ap.add_argument("--win-speed", type=float, default=1.0)
    ap.add_argument("--win-rate", type=float, default=0.5)
    ap.add_argument("--max-delta-huamo", type=float, default=250.0,
                    help="huamo 满档 PWM 增量（速度输出 100 × 2.5 PWM/单位）")
    ap.add_argument("--max-delta-kbd", type=float, default=300.0, help="键盘满舵 PWM 增量")
    ap.add_argument("--csv-prefix", default="", help="可选: 输出 legs/turns/levels csv")
    args = ap.parse_args()

    O, P, paths, pwm_topic, note_tf = load_bag(args.bag, args)
    if len(O["t"]) < 10:
        print("FAIL: bag 里没有足够的 %s 数据" % args.odom_topic)
        sys.exit(1)
    if P is None:
        print("FAIL: bag 里既没有 %s 也没有 /controller，无法做推进辨识" % args.pwm_topic)
        sys.exit(1)
    print("PWM 话题: %s   odom: %d 帧  %.1f~%.1fs   pwm: %d 帧   规划线: %d 条"
          % (pwm_topic, len(O["t"]), O["t"][0], O["t"][-1], len(P["t"]), len(paths)))
    if note_tf:
        print(note_tf)

    fwd, diff = align_pwm(O, P)
    ok = np.isfinite(fwd)
    print("PWM 对齐覆盖: %d/%d odom 帧 (%.0f%%) —— 覆盖低说明该时段在手动/停船/未规划"
          % (ok.sum(), len(ok), 100.0 * ok.mean()))

    # 档位分段用就近填充后的平滑值；统计用原始对齐值
    vi = np.where(ok)[0]
    fwd_fill = np.interp(np.arange(len(fwd)), vi, fwd[vi])
    fwd_s = sliding_median(fwd_fill, O["t"], 0.5)
    fwd_s[~ok] = np.nan
    diff_fill = np.interp(np.arange(len(diff)), vi, diff[vi])

    segs = segment(O, fwd_s, diff_fill, ok, args.min_leg, args.turn_min,
                   args.turn_thr, args.step_thr)
    straights = [analyze_straight(O, s["idx"], fwd, diff, paths,
                                  args.win_speed, 0.0)
                 for s in segs if not s["turning"]]
    turns = [analyze_turn(O, s["idx"], diff, fwd, args.win_rate)
             for s in segs if s["turning"]]
    if not straights and not turns:
        print("没有找到足够长的直航/回转段（min_leg=%.0fs turn_min=%.0fs）。"
              "若本包是普通任务包，可试 --min-leg 5 --turn-min 3"
              % (args.min_leg, args.turn_min))
        sys.exit(1)

    # --- 直航段表 ---
    print("=" * 100)
    print("直航段（|差速|<%.0f，≥%.0fs）: %d 段" % (args.turn_thr, args.min_leg, len(straights)))
    print("%4s %7s %6s %7s %7s %7s %7s %7s %7s  %s"
          % ("段", "时刻s", "时长", "Δpwm", "速度", "σ速度", "σpwm", "slew95", "|差速|", "相对规划线CTE(左+)"))
    for i, s in enumerate(straights):
        cte = ("%+.2f±%.2f m" % (s["cte_mean"], s["cte_std"])
               if s["cte_mean"] is not None else "无规划线")
        print("%4d %7.0f %5.0fs %+7.0f %7s %7s %7.1f %7.0f %7.1f  %s"
              % (i, s["t0"], s["dur"], s["level"], fmt(s["speed"]), fmt(s["speed_std"]),
                 s["level_std"], s["slew95"], s["diff_med"], cte))

    # --- 档位表 + 拟合 → v_max_phys ---
    levels = {}
    for s in straights:
        levels.setdefault(int(round(s["level"] / 25.0) * 25), []).append(s)
    print("-" * 100)
    print("油门档位 → 稳态速度（按时长加权）:")
    pts_lv = []
    for key in sorted(levels):
        ss = levels[key]
        w = np.array([x["dur"] for x in ss])
        v = float(np.average([x["speed"] for x in ss], weights=w))
        vp = [x["speed"] for x in ss]
        print("  Δpwm≈%+4d: %d 段, 合计 %4.0fs, 稳态速度 %.2f m/s (段间 %.2f~%.2f)"
              % (key, len(ss), w.sum(), v, min(vp), max(vp)))
        pts_lv.append((key, v, float(w.sum())))

    v_huamo = None
    if len(pts_lv) >= 2:
        dp = np.array([p[0] for p in pts_lv], dtype=float)
        vv = np.array([p[1] for p in pts_lv])
        ww = np.array([p[2] for p in pts_lv])
        A = np.column_stack([dp, np.ones_like(dp)])
        sol, *_ = np.linalg.lstsq(A * ww[:, None], vv * ww, rcond=None)
        k, b = float(sol[0]), float(sol[1])
        if k > 0:
            d0 = -b / k
            v_huamo = max(0.0, k * args.max_delta_huamo + b)
            v_kbd = max(0.0, k * args.max_delta_kbd + b)
            print("拟合: v = %.4f·Δpwm %+.4f   (死区 Δ≈%.0f)"
                  % (k, b, d0 if 0 <= d0 <= 150 else 0))
            print("外推 v_max_phys: Δ=huamo满档%.0f → %.2f m/s   Δ=键盘满舵%.0f → %.2f m/s"
                  % (args.max_delta_huamo, v_huamo, args.max_delta_kbd, v_kbd))
            full = [p for p in pts_lv if p[0] >= args.max_delta_huamo - 25]
            if full:
                print("!! 已有满档实测：Δ≈%+d → %.2f m/s —— 以实测为准，外推仅参考"
                      % (full[0][0], full[0][1]))
        else:
            print("拟合斜率异常(≤0)，检查各档位速度是否随油门单调")
    else:
        print("有效档位数 <2，无法拟合 PWM→速度曲线（本次只出表；"
              "下次水试按 speed_max 0.2/0.5/1.0/满档 各跑一条直线）")

    # --- CTE 按航向 + 全局横向偏置矢量 ---
    print("-" * 100)
    print("CTE 按航向（相对规划线，左正；规划线本身含 A* 阶梯风格）:")
    for i, s in enumerate(straights):
        if s["cte_mean"] is None:
            continue
        print("  段%d: 行进方向 %6.1f°  CTE %+6.2f±%.2f m  (Δpwm %+d, %.0fs)"
              % (i, math.degrees(s["hdg"]) % 360, s["cte_mean"], s["cte_std"],
                 int(round(s["level"])), s["dur"]))
    have_bias = [s for s in straights if s["cte_mean"] is not None]
    if have_bias:
        w = np.array([s["dur"] for s in have_bias])
        bias = np.average(np.array([s["bias"] for s in have_bias]), axis=0, weights=w)
        print("全局横向偏置矢量: 东 %+.2f m, 北 %+.2f m（|偏| %.2f m @ %.0f°）"
              % (bias[0], bias[1], float(np.hypot(*bias)),
                 math.degrees(math.atan2(bias[1], bias[0])) % 360))
        print("解读: 各航向都偏向同一世界方向 → 世界系固定偏置（查 LOS +1m 东偏类问题）；"
              "偏置方向随航向转 → 艇体侧偏（标定/双桨不对称）")
    else:
        print("bag 未录 /waypoints_path，无法算相对规划线的 CTE 偏置")

    # --- 回转段表 + k_r / tau_r / r_max ---
    print("-" * 100)
    r_max = None
    if turns:
        print("回转段（|差速|>%.0f，≥%.0fs）: %d 段   [差速Δ=(左−右)/2, Δ>0=右转]"
              % (args.turn_thr, args.turn_min, len(turns)))
        print("%4s %7s %6s %8s %8s %9s %7s %8s %7s" %
              ("段", "时刻s", "时长", "差速Δ", "Δpwm", "r稳态", "速度", "k_r", "t63s"))
        for i, s in enumerate(turns):
            kr = s["r_ss"] / s["diff"] if abs(s["diff"]) > 5 else None
            print("%4d %7.0f %5.0fs %+8.0f %+8.0f %+9.3f %7s %8s %7s"
                  % (i, s["t0"], s["dur"], s["diff"], s["fwd"], s["r_ss"],
                     fmt(s["speed"]), fmt(kr, 4), fmt(s["t63"])))
        good = [s for s in turns if abs(s["r_ss"]) > 0.05]   # r≈0 的段没有符号/辨识意义
        if good:
            consistent = sum(1 for s in good
                             if s["diff"] * s["r_ss"] < 0) / float(len(good))
            print("符号一致性(仅|r|>0.05 的 %d 段): Δ>0(右转)→r<0 占 %.0f%% %s"
                  % (len(good), 100 * consistent,
                     "(正常)" if consistent > 0.8 else "(!! 与约定相反，核对 PWM 接线/话题左右序)"))
        else:
            print("所有回转段稳态转率 |r|≤0.05 rad/s（低速差动力矩不足或段太短），符号无从核对")
        t63s = [s["t63"] for s in turns if s["t63"]]
        if t63s:
            print("tau_r 估计: 中位 t63 = %.2f s（一阶系统：tau_r ≈ t63）"
                  % float(np.median(t63s)))
        r_max = max(abs(s["r_ss"]) for s in good) if good else None
        if r_max is not None:
            print("r_max 观测: 稳态最大 %.2f rad/s（%.0f°/s）"
                  % (r_max, math.degrees(r_max)))
    else:
        print("无回转段：键盘差速回转请把 /controller 录进 bag，且每个差速档保持 ≥%.0fs"
              % args.turn_min)

    # --- 结论行 ---
    print("=" * 100)
    sugg = "→ 训练参数建议: v_max_phys=%s" % (
        "%.1f" % round(v_huamo, 1) if v_huamo else "待多档数据")
    if r_max is not None:
        sugg += ", r_max=%.1f(取观测, 训练随机化上限可到 %.1f)" % (round(r_max, 1), round(r_max * 1.2, 1))
    print(sugg)

    if args.csv_prefix:
        np.savetxt(args.csv_prefix + "_legs.csv",
                   np.array([[s["t0"], s["dur"], s["level"], s["speed"], s["speed_std"],
                              s["level_std"], s["slew95"],
                              s["cte_mean"] if s["cte_mean"] is not None else np.nan,
                              s["cte_std"] if s["cte_std"] is not None else np.nan,
                              math.degrees(s["hdg"])] for s in straights]),
                   delimiter=",", fmt="%.4f",
                   header="t0,dur,dpwm,speed,speed_std,pwm_std,slew95,cte_mean,cte_std,hdg_deg",
                   comments="")
        if turns:
            np.savetxt(args.csv_prefix + "_turns.csv",
                       np.array([[s["t0"], s["dur"], s["diff"], s["r_ss"],
                                  s["speed"], s["t63"] if s["t63"] else np.nan]
                                 for s in turns]),
                       delimiter=",", fmt="%.4f",
                       header="t0,dur,diff,r_ss,speed,t63", comments="")
        print("已写 %s_legs.csv%s" % (args.csv_prefix, " + _turns.csv" if turns else ""))
    sys.exit(0)


if __name__ == "__main__":
    main()
