#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""避障场景测试：几何 + 环境布线。

【本机可跑】几何用例为纯 numpy，不依赖 gymnasium。
环境级用例优先使用真 gymnasium（训练机）；NX 等无 gymnasium 的机器自动启用
一个最小替身（只覆盖本工程用到的 Box/Env/np_random 接口）来验证环境布线——
正式冒烟仍以训练机 `python3 env/smoke_test.py`（真 gymnasium）为准。

用法：python3 env/scene_test.py
"""
import math
import os
import sys
import types

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))   # 项目根


def _install_gymnasium_stub():
    """最小 gymnasium 替身：只覆盖本工程用到的接口（NX 无 gymnasium 时验证布线用）。"""
    m = types.ModuleType("gymnasium")

    class _Env:
        metadata = {}

        def __init__(self):
            self.np_random = np.random.default_rng()

        def reset(self, seed=None, options=None):
            if seed is not None:
                self.np_random = np.random.default_rng(seed)
            return None

        @property
        def unwrapped(self):
            return self

    class _Box:
        def __init__(self, low, high, shape=None, dtype=np.float32):
            self.low, self.high, self.dtype = low, high, dtype
            self.shape = tuple(shape) if shape is not None else (int(np.size(low)),)

        def sample(self):
            lo = np.asarray(self.low, dtype=np.float64)
            hi = np.asarray(self.high, dtype=np.float64)
            return (lo + (hi - lo) * np.random.random(self.shape)).astype(self.dtype)

    sp = types.ModuleType("gymnasium.spaces")
    sp.Box = _Box
    m.spaces = sp
    m.Env = _Env
    sys.modules["gymnasium"] = m
    sys.modules["gymnasium.spaces"] = sp


try:
    import gymnasium  # noqa: F401
    _HAS_REAL_GYM = True
except ImportError:
    _HAS_REAL_GYM = False
    _install_gymnasium_stub()
    print("[warn] 本机无 gymnasium → 启用最小替身验证环境布线"
          "（正式冒烟请在训练机跑 python3 env/smoke_test.py）")

from env.obstacles import (gen_shore_corridor, spawn_gates, spawn_moving,
                           spawn_path_local, spawn_static)  # noqa: E402
from env.path_gen import gen_path  # noqa: E402
from rl_common import LidarSectorizer  # noqa: E402

BASE_CFG = {
    "dt": 0.1, "max_episode_steps": 300, "dynamics": "first_order",
    "difficulty": "medium", "n_obstacles": 0,
    "dynamics_params": {"tau_u": 0.79, "tau_r": 0.53, "k_r": 0.8, "r_max": 0.6},
    "reward": {"w_progress": 4.0, "w_track": 1.0, "w_heading": 0.15,
               "w_smooth": 0.3, "w_safe_speed": 0.5,
               "obstacle": {}, "terminal": {}},
}


def _min_path_dist(path, x, y):
    return float(np.min(np.hypot(path[:, 0] - x, path[:, 1] - y)))


# ==================== 几何用例（纯 numpy） ====================
def test_legacy_off_path():
    """旧'避让式'摆放：障碍面距路径 ≥ margin（Stage2 行为必须保持）。"""
    for seed in range(6):
        rng = np.random.default_rng(seed)
        path = gen_path(rng, "medium")
        obs = spawn_static(rng, path, 5)
        assert len(obs) == 5
        for o in obs:
            assert _min_path_dist(path, o.x, o.y) - o.radius >= 3.0 - 1e-9
    print("[ok] 旧避让式摆放：障碍面距路径 ≥3.0m（Stage2 行为保持）")


def test_on_path_blocking():
    """压路径障碍：中心距路径 ≤1.5m（面直接压在路径上），彼此不合成墙。"""
    for seed in range(8):
        rng = np.random.default_rng(seed)
        path = gen_path(rng, "medium")
        obs = spawn_path_local(rng, path, 3, radius_range=(1.0, 3.0),
                               lat_range=(0.0, 1.5))
        assert len(obs) == 3, "期望 3 个，得到 %d" % len(obs)
        for o in obs:
            d = _min_path_dist(path, o.x, o.y)
            assert d <= 1.5 + 1e-6, "中心离路径 %.2fm 超限" % d
            assert d - o.radius <= 0.5 + 1e-6, "未压住路径（间隙 %.2fm）" % (d - o.radius)
        for i in range(len(obs)):
            for j in range(i + 1, len(obs)):
                dd = math.hypot(obs[i].x - obs[j].x, obs[i].y - obs[j].y)
                assert dd >= obs[i].radius + obs[j].radius + 3.0 - 1e-6, \
                    "两压路径障碍过近（间隙 <3m 会合成墙）"
    print("[ok] on_path 阻塞障碍：中心距路径 ≤1.5m、面压路径、间距 ≥3m 不合成墙")


def test_corridor():
    """岸线走廊：净宽在范围内、跟踪路径离两岸 ≥~1.5m、点云规模合理。"""
    for seed in range(8):
        rng = np.random.default_rng(seed)
        c = gen_shore_corridor(rng, "medium", width_range=(8.0, 25.0),
                               path_bias=0.35)
        assert c.widths.min() >= 8.0 - 1.7, "局部净宽过窄 %.2f" % c.widths.min()
        assert c.widths.max() <= 25.0 + 1.7, "局部净宽过宽 %.2f" % c.widths.max()
        # 路径离岸（点对点近似，岸点 1m 间隔 → 低估 ≤0.5m）
        sub = c.path[::4]
        dd = np.hypot(sub[:, None, 0] - c.shore_pts[None, :, 0],
                      sub[:, None, 1] - c.shore_pts[None, :, 1]).min(axis=1)
        assert dd.min() >= 1.1, "路径离岸过近 %.2fm" % dd.min()
        assert len(c.shore_pts) > 100 and len(c.path) > 100
        xmin, xmax, ymin, ymax = c.bbox
        assert xmax > xmin and ymax > ymin
    print("[ok] 岸线走廊：净宽∈[8,25]±1.7m、路径离岸 ≥1.1m（实值约≥1.5m）")


def test_shores_sensed():
    """岸线进入 36 扇区观测（与 env 同管线：世界→体轴→LidarSectorizer）。"""
    rng = np.random.default_rng(3)
    c = gen_shore_corridor(rng, "medium", width_range=(8.0, 20.0))
    sec = LidarSectorizer()
    t = c.path[2] - c.path[0]
    psi = math.atan2(t[1], t[0])
    dx = c.shore_pts[:, 0] - c.path[0, 0]
    dy = c.shore_pts[:, 1] - c.path[0, 1]
    cc, ss = math.cos(psi), math.sin(psi)
    body = np.column_stack([cc * dx + ss * dy, -ss * dx + cc * dy,
                            np.zeros(len(dx))])
    sectors = sec.process(body)
    assert sectors.min() < 20.0, "岸线不在量程内？"
    assert int((sectors < 20.0).sum()) >= 2, "只有 %d 个扇区看到岸" % int((sectors < 20.0).sum())
    print("[ok] 岸线进入扇区观测：最近岸距 %.1fm，%d 个扇区有回波"
          % (sectors.min(), int((sectors < 20.0).sum())))


def test_gate_gap():
    """窄通道净宽参数化：每门净宽落在 gap_range 内、左右对称。"""
    rng = np.random.default_rng(5)
    path = gen_path(rng, "medium")
    obs = spawn_gates(rng, path, 4, gap_range=(3.5, 8.0))
    assert len(obs) == 8
    for i in range(0, 8, 2):
        a, b = obs[i], obs[i + 1]
        gap = math.hypot(a.x - b.x, a.y - b.y) - 2.0 * a.radius
        assert 3.5 - 1e-6 <= gap <= 8.0 + 1e-6, "净宽 %.2f 越界" % gap
    print("[ok] 窄通道净宽参数化 ∈[3.5,8.0]m")


def test_radius_classes():
    """尺寸类：每个障碍半径落在所给类别内（浮标/常规/码头混合）。

    注：压路径障碍受"中心间距 ≥ r1+r2+3m"约束，约 100m 路径上限 ~10-20 个，
    故这里按 12 个请求、少于 12 个也允许（放置容量有限）。
    """
    ranges = [[0.3, 0.8], [1.0, 3.0], [3.0, 6.0]]
    for seed in range(4):
        rng = np.random.default_rng(seed)
        path = gen_path(rng, "medium")
        obs = spawn_path_local(rng, path, 12, radius_ranges=ranges,
                               lat_range=(0.0, 1.5))
        assert 8 <= len(obs) <= 12
        for o in obs:
            assert any(lo - 1e-9 <= o.radius <= hi + 1e-9 for lo, hi in ranges), \
                "半径 %.2f 不在类别内" % o.radius
        kinds = {i for o in obs for i, (lo, hi) in enumerate(ranges)
                 if lo - 1e-9 <= o.radius <= hi + 1e-9}
        assert len(kinds) >= 2, "类别混合未生效（只出现 %s）" % kinds
    print("[ok] 障碍尺寸类混合采样（半径均落在类别内）")


def test_corridor_moving():
    """走廊内动态障碍：初始位置在走廊净宽内、反弹边界生效。"""
    rng = np.random.default_rng(9)
    c = gen_shore_corridor(rng, "medium", width_range=(10.0, 25.0))
    xmin, xmax, ymin, ymax = c.bbox
    obs = spawn_moving(rng, c.path, 3, path_local=True, widths=c.widths,
                       bx=(xmin, xmax), by=(ymin, ymax))
    assert len(obs) == 3
    for o in obs:
        d = _min_path_dist(c.path, o.x, o.y)
        assert d <= 3.0 + 1e-6, "动态障碍初始离路径 %.2fm" % d
        for _ in range(5000):
            o.step(0.1)
        assert xmin - 1e-6 <= o.x <= xmax + 1e-6 and ymin - 1e-6 <= o.y <= ymax + 1e-6, \
            "反弹边界失效 (%s, %s)" % (o.x, o.y)
    print("[ok] 走廊动态障碍：贴路初始、反弹边界 = 走廊 bbox")


# ==================== 环境级用例（需 gymnasium 或替身） ====================
def _run_episode(env, seed, steps=200):
    obs, info = env.reset(seed=seed)
    assert obs.shape == (49,) and np.isfinite(obs).all(), "观测异常"
    rng = np.random.default_rng(seed)
    dmin_ep = env._observe()[3]
    for _ in range(steps):
        obs, r, term, trunc, info = env.step(rng.uniform(-1, 1, 2))
        assert np.isfinite(obs).all() and np.isfinite(r), "step 输出含 NaN"
        dmin_ep = min(dmin_ep, info["d_min"])
        if term or trunc:
            break
    assert info["end_reason"] in ("goal", "collision", "stuck", "timeout",
                                  "cte_abort", "running")
    return info, dmin_ep


def test_env_scenes():
    real = _HAS_REAL_GYM
    from env import USVLocalEnv

    # 1) 开阔场 + 全压路径障碍
    cfg = dict(BASE_CFG)
    cfg.update(n_obstacles=3, obstacle_on_path_frac=1.0)
    env = USVLocalEnv(cfg)
    info, dmin_ep = _run_episode(env, seed=7)
    assert info["scene"] == "open_field"
    print("[ok] env: open_field + 压路径障碍（episode 结束=%s，最近间隙 %.1fm）"
          % (info["end_reason"], dmin_ep))

    # 2) 岸线走廊（含窄门）
    cfg2 = dict(BASE_CFG)
    cfg2.update(scene="corridor", n_obstacles=2, narrow_gates=1,
                corridor_width_range=[8.0, 20.0])
    env2 = USVLocalEnv(cfg2)
    obs, ri = env2.reset(seed=7)
    assert ri["scene"] == "corridor"
    dmin0 = env2._observe()[3]
    assert dmin0 <= 15.0, "reset 时岸线未进入量程（d_min=%.1f）" % dmin0
    info2, dmin_ep2 = _run_episode(env2, seed=7)
    assert dmin_ep2 < 12.0, "整条 episode 未感知到岸线约束"
    print("[ok] env: corridor（reset 岸距 %.1fm，episode 最近 %.1fm，结束=%s）"
          % (dmin0, dmin_ep2, info2["end_reason"]))

    # 3) mixed：两种场景都会出现
    cfg3 = dict(BASE_CFG)
    cfg3.update(scene="mixed", corridor_mix_p=0.5, n_obstacles=1)
    env3 = USVLocalEnv(cfg3)
    seen = set()
    for s in range(8):
        _, ri = env3.reset(seed=s)
        seen.add(ri["scene"])
    assert seen == {"open_field", "corridor"}, "mixed 未覆盖两种场景: %s" % seen
    print("[ok] env: mixed 场景切换（%s）" % "/".join(sorted(seen)))

    print("[%s] 环境级用例通过" % ("真 gymnasium" if real else "最小替身"))


if __name__ == "__main__":
    test_legacy_off_path()
    test_on_path_blocking()
    test_corridor()
    test_shores_sensed()
    test_gate_gap()
    test_radius_classes()
    test_corridor_moving()
    test_env_scenes()
    print("\n场景测试全部通过 ✓")
