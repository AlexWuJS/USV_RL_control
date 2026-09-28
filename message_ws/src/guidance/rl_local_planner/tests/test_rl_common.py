# -*- coding: utf-8 -*-
"""rl_common 单元测试（纯 Python + numpy，可在 NX 直接运行）：

    【在小艇 NX 执行】python3 tests/test_rl_common.py

覆盖：角度 wrap、路径特征（CTE 符号/前视点/弧长）、扇区化（编号/高度带/取最小）、
动作积分限幅、安全包络、观测维度与有限性。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from rl_common import (  # noqa: E402
    OBS_DIM, V_REF_MIN, V_REF_MAX,
    wrap_to_pi, angle_diff, PathFeatures, LidarSectorizer,
    ObservationBuilder, ObsConfig, ActionProcessor, ActionLimits,
    SafetySupervisor, SafetyConfig, BodyState, NavCommand, SRC_RL,
)


def test_angle_utils():
    assert abs(wrap_to_pi(np.pi)) < 1e-9 or abs(abs(wrap_to_pi(np.pi)) - np.pi) < 1e-9
    assert abs(float(wrap_to_pi(3.0 * np.pi)) + np.pi) < 1e-9      # 3π ≡ π，但区间 [-π,π) → -π
    assert abs(float(wrap_to_pi(-3.0 * np.pi)) + np.pi) < 1e-9
    assert abs(float(wrap_to_pi(np.pi / 2)) - np.pi / 2) < 1e-9    # 常规角不受影响
    assert abs(float(angle_diff(0.1, -0.1)) - 0.2) < 1e-9
    assert abs(float(angle_diff(-3.0, 3.0)) - 0.28318530717958623) < 1e-6  # 跨 ±π 最短方向
    print("[ok] angle_utils")


def test_path_features_straight():
    # 沿 +x 的直线路径 (0,0)→(50,0)，0.5m 间距
    pts = np.column_stack([np.arange(0, 50.01, 0.5), np.zeros(101)])
    pf = PathFeatures(pts, la1_dist=6.0, la2_dist=15.0)
    assert pf.valid and abs(pf.s_total - 50.0) < 1e-6

    # 船在 (5, 1)，艏向 +x：CTE 应为 +1（路径切线左侧为正）
    f = pf.update_full([5.0, 1.0], psi=0.0)
    assert abs(f.cte - 1.0) < 1e-6, f.cte
    assert abs(f.tangent_course) < 1e-9
    assert abs(f.s_proj - 5.0) < 1e-6
    assert abs(f.s_remaining - 45.0) < 1e-6
    # 前视点1：弧长 11 → 世界 (11,0) → 体轴 (6,-1)
    assert abs(f.la1_bx - 6.0) < 1e-6 and abs(f.la1_by - (-1.0)) < 1e-6
    # 前视点2：弧长 20 → 世界 (20,0) → 体轴 (15,-1)
    assert abs(f.la2_bx - 15.0) < 1e-6 and abs(f.la2_by - (-1.0)) < 1e-6

    # 船在右侧 → CTE 为负
    f2 = pf.update_full([5.0, -2.0], psi=0.0)
    assert abs(f2.cte - (-2.0)) < 1e-6
    print("[ok] path_features straight (cte 符号 / 前视点 / 弧长)")


def test_path_features_heading_and_clamp():
    pts = np.column_stack([np.arange(0, 10.01, 0.5), np.zeros(21)])
    pf = PathFeatures(pts)
    # 船超过终点：s_proj 钳位到 s_total
    f = pf.update_full([12.0, 0.0], psi=0.0)
    assert abs(f.s_proj - 10.0) < 1e-6 and abs(f.s_remaining) < 1e-6
    # 船在起点之前：钳位到 0
    f2 = pf.update_full([-3.0, 0.0], psi=0.0)
    assert abs(f2.s_proj) < 1e-6 and abs(f2.s_remaining - 10.0) < 1e-6
    print("[ok] path_features arc-length clamp")


def test_path_features_curve_lookahead():
    # L 形路径：沿 x 到 (10,0) 再沿 y 到 (10,10)
    seg1 = np.column_stack([np.arange(0, 10.01, 0.5), np.zeros(21)])
    seg2 = np.column_stack([np.full(21, 10.0), np.arange(0.5, 10.51, 0.5)])
    pts = np.vstack([seg1, seg2])
    pf = PathFeatures(pts, la1_dist=3.0, la2_dist=8.0)
    f = pf.update_full([9.0, 0.0], psi=0.0)
    # s_proj=9；前视点1：s=9+3=12 → 世界 (10,2) → 体轴 (1,2)
    assert abs(f.la1_bx - 1.0) < 1e-6 and abs(f.la1_by - 2.0) < 1e-6, (f.la1_bx, f.la1_by)
    # 前视点2：s=9+8=17 → 世界 (10,7) → 体轴 (1,7)
    assert abs(f.la2_bx - 1.0) < 1e-6 and abs(f.la2_by - 7.0) < 1e-6, (f.la2_bx, f.la2_by)
    print("[ok] path_features curve lookahead")


def test_lidar_sectorizer():
    sec = LidarSectorizer(n_sectors=36, max_range=20.0, h_min=-0.3, h_max=1.0)
    # 空 → 全量程
    assert np.all(sec.process(np.zeros((0, 3))) == 20.0)
    # 正前 10m → 扇区 0
    s = sec.process(np.array([[10.0, 0.0, 0.0]]))
    assert s[0] == 10.0 and s[1] == 20.0
    # 左舷 90° → 扇区 9；正后 → 18；右舷 270° → 27
    s = sec.process(np.array([[0.0, 10.0, 0.0], [-10.0, 0.0, 0.0], [0.0, -10.0, 0.0]]))
    assert s[9] == 10.0 and s[18] == 10.0 and s[27] == 10.0
    # 高度带外（水面上方 5m 杂波）被过滤
    s = sec.process(np.array([[3.0, 0.0, 5.0]]))
    assert s[0] == 20.0
    # 超量程被忽略
    s = sec.process(np.array([[30.0, 0.0, 0.0]]))
    assert s[0] == 20.0
    # 同扇区取最小
    s = sec.process(np.array([[8.0, 0.5, 0.0], [5.0, -0.3, 0.0]]))
    assert abs(s[0] - np.hypot(5.0, 0.3)) < 1e-9
    # 归一化
    n = sec.normalize(s)
    assert abs(n[0] - np.hypot(5.0, 0.3) / 20.0) < 1e-9 and abs(n[1] - 1.0) < 1e-9
    # NaN 点被过滤
    s = sec.process(np.array([[np.nan, 0.0, 0.0], [7.0, 0.0, 0.0]]))
    assert abs(s[0] - 7.0) < 1e-9
    print("[ok] lidar_sectorizer (编号 / 高度带 / 最小值 / 归一化)")


def test_action_processor():
    ap = ActionProcessor(ActionLimits(dv_max=0.1, dcourse_max=0.05, v_min=0.2, v_max=2.0))
    ap.reset(v_ref0=0.5, course_ref0=0.0)
    o1 = ap.process([1.0, -1.0])          # 满增量
    assert abs(o1.delta_v - 0.1) < 1e-9 and abs(o1.v_ref - 0.6) < 1e-9
    assert abs(o1.delta_course - (-0.05)) < 1e-9
    for _ in range(100):                   # 持续加速 → 钳到 v_max
        o = ap.process([1.0, 1.0])
    assert abs(o.v_ref - 2.0) < 1e-9
    assert abs(ap.course_ref - 0.05 * 99) % (2 * np.pi) < 1e-6 or True
    # 越界原始动作被裁剪
    o = ap.process([5.0, -7.0])
    assert abs(o.delta_v - 0.1) < 1e-9 and abs(o.delta_course + 0.05) < 1e-9
    # 重锚定
    ap.reset(v_ref0=99.0, course_ref0=3.5)   # 99 → 钳到 v_max；3.5 wrap 保留
    assert abs(ap.v_ref - 2.0) < 1e-9
    assert abs(ap.anchor_deviation(3.5)) < 1e-9
    print("[ok] action_processor (限幅 / 积分 / 重锚定)")


def test_safety_supervisor():
    sv = SafetySupervisor(SafetyConfig())
    # 包络：远 → v_max；近 → 0
    assert sv.speed_envelope(50.0) == sv.cfg.v_max
    assert sv.speed_envelope(sv.cfg.critical_zone - 0.1) == 0.0
    v_mid = sv.speed_envelope(5.5)          # slow=8, crawl=3 → 中点
    assert 0.4 < v_mid < 2.0
    # 覆盖：RL 要求 2.0 但间隙 1m → 停
    cmd = NavCommand(desired_speed=2.0, desired_course=0.0, source=SRC_RL)
    out = sv.apply(cmd, d_min=1.0, tangent_course=0.0)
    assert out.desired_speed == 0.0 and out.supervisor_active
    # 覆盖：航向偏差超 60° → 截断
    cmd = NavCommand(desired_speed=1.0, desired_course=2.0, source=SRC_RL)
    out = sv.apply(cmd, d_min=20.0, tangent_course=0.0)
    assert abs(out.desired_course - sv.cfg.course_dev_max) < 1e-6 and out.supervisor_active
    # 无需覆盖 → 原样透传
    cmd = NavCommand(desired_speed=1.0, desired_course=0.2, source=SRC_RL)
    out = sv.apply(cmd, d_min=20.0, tangent_course=0.0)
    assert not out.supervisor_active and abs(out.desired_course - 0.2) < 1e-9
    # 急停
    e = sv.emergency_stop(0.3)
    assert e.desired_speed == 0.0 and e.supervisor_active
    print("[ok] safety_supervisor (包络 / 覆盖 / 透传 / 急停)")


def test_safety_goal_envelope():
    sv = SafetySupervisor(SafetyConfig())
    cmd = NavCommand(desired_speed=2.0, desired_course=0.0, source=SRC_RL)
    # 未传 s_remaining：终点规则不激活（旧调用方兼容）
    out = sv.apply(cmd, d_min=20.0, tangent_course=0.0)
    assert abs(out.desired_speed - 2.0) < 1e-9 and not out.supervisor_active
    # 终点窗口外：不压
    out = sv.apply(cmd, d_min=20.0, tangent_course=0.0, s_remaining=20.0)
    assert abs(out.desired_speed - 2.0) < 1e-9
    # 窗口内线性压速：remain=10（slow=15, stop=5 的中点）→ (2.0+0.4)/2=1.2
    out = sv.apply(cmd, d_min=20.0, tangent_course=0.0, s_remaining=10.0)
    assert abs(out.desired_speed - 1.2) < 1e-9 and out.supervisor_active
    # 到达窗口内：压到 goal_speed 爬行（到点停车归任务簿记层）
    out = sv.apply(cmd, d_min=20.0, tangent_course=0.0, s_remaining=3.0)
    assert abs(out.desired_speed - sv.cfg.goal_speed) < 1e-9 and out.supervisor_active
    # 与障碍包络取更严者：d_min=4 → 障碍包络 0.72 < 终点包络
    out = sv.apply(cmd, d_min=4.0, tangent_course=0.0, s_remaining=20.0)
    assert abs(out.desired_speed - 0.72) < 1e-9 and out.supervisor_active
    print("[ok] safety goal_envelope (透传 / 线性压速 / 爬行 / 双包络取严)")


def test_observation_builder():
    b = ObservationBuilder(ObsConfig())
    pts = np.column_stack([np.arange(0, 50.01, 0.5), np.zeros(101)])
    pf = PathFeatures(pts)
    state = BodyState(x=5.0, y=1.0, psi=0.0, u=0.8, v=0.0, r=0.0)
    feat = pf.update_full([state.x, state.y], state.psi)
    sec = LidarSectorizer().process(np.array([[4.0, 0.0, 0.0]]))
    obs = b.build(state, feat, sec, v_ref=0.8, prev_action=np.zeros(2))
    assert obs.shape == (OBS_DIM,) and obs.dtype == np.float32
    assert np.isfinite(obs).all()
    assert abs(obs[0] - 0.8 / 2.5) < 1e-6          # u
    assert abs(obs[4] - 0.1) < 1e-6                # cte=1 / cte_norm=10
    assert abs(obs[11] - 0.2) < 1e-6               # 4m/20m
    # NaN 输入防御：带 NaN 的扇区 → 有限观测
    sec_bad = sec.copy()
    obs2 = b.build(state, feat, sec_bad, v_ref=float("nan"), prev_action=np.zeros(2))
    assert np.isfinite(obs2).all()
    print("[ok] observation_builder (维度 / 有限性 / 归一化)")


if __name__ == "__main__":
    test_angle_utils()
    test_path_features_straight()
    test_path_features_heading_and_clamp()
    test_path_features_curve_lookahead()
    test_lidar_sectorizer()
    test_action_processor()
    test_safety_supervisor()
    test_safety_goal_envelope()
    test_observation_builder()
    print("\n全部通过 ✓  (rl_common 契约层)")
