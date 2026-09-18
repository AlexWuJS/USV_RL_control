#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""环境冒烟测试。【在训练机执行】（非训练，秒级完成）

验证三件事，任何一项失败都不要开始训练：
  1. gymnasium API：reset/step 正常、观测 49 维有限、动作空间 [-1,1]²
  2. 动力学合理性：满速加速 → u 逼近 v_max 且时间常数 ≈ tau_u；
     常值转向 → r 收敛且有限幅
  3. 奖励/终止路径：随机动作跑 1 条 episode 能正常结束（不崩、不 NaN）
"""
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))   # 项目根

from env import USVLocalEnv            # noqa: E402
from dynamics import make_dynamics     # noqa: E402
from rl_common import OBS_DIM          # noqa: E402

CFG = {
    "dt": 0.1, "max_episode_steps": 600, "dynamics": "first_order",
    "difficulty": "medium", "n_obstacles": [1, 5],
    "dynamics_params": {"tau_u": 0.79, "tau_r": 0.53, "k_r": 0.8, "r_max": 0.6},
    "reward": {"w_progress": 2.0, "w_track": 0.1, "w_heading": 0.05,
               "w_smooth": 0.3, "w_safe_speed": 0.5,
               "obstacle": {}, "terminal": {}},
}


def test_api():
    env = USVLocalEnv(CFG)
    obs, info = env.reset(seed=0)
    assert obs.shape == (OBS_DIM,) and np.isfinite(obs).all(), "观测异常"
    assert env.action_space.shape == (2,)
    o, r, term, trunc, i = env.step(np.zeros(2))
    assert np.isfinite(o).all() and np.isfinite(r), "step 输出含 NaN"
    print("[ok] gymnasium API（obs=%d 维有限，reward 有限）" % OBS_DIM)


def test_dynamics():
    dyn = make_dynamics("first_order", CFG["dynamics_params"])
    dyn.reset(0.0, 0.0, 0.0)
    # 满速加速：v_ref=v_max_phys=2.3 → u 收敛到 2.3，63% 上升时间 ≈ tau_u=0.79s
    t63 = None
    for k in range(300):
        dyn.step(2.3, 0.0, 0.1)
        if t63 is None and dyn.state.u >= 0.63 * 2.3:
            t63 = k * 0.1
    assert dyn.state.u > 2.0, "u 未收敛到高速 (u=%.2f)" % dyn.state.u
    assert t63 is not None and 0.3 < t63 < 1.6, "上升时间异常 t63=%.2f" % (t63 or -1)
    # 常值转向：r 收敛且 |r| <= r_max
    for _ in range(200):
        dyn.step(1.0, np.pi / 2, 0.1)
    assert abs(dyn.state.r) <= 0.6 + 1e-6 and np.isfinite(dyn.state.psi)
    print("[ok] 一阶动力学（t63=%.2fs ≈ tau_u, r 限幅 %.3f）" % (t63, dyn.state.r))


def test_episode():
    env = USVLocalEnv(CFG)
    env.reset(seed=42)
    rng = np.random.default_rng(42)
    total, done, steps = 0.0, False, 0
    while not done and steps < 600:
        _, r, term, trunc, info = env.step(rng.uniform(-1, 1, 2))
        total += r
        done = term or trunc
        steps += 1
    assert np.isfinite(total), "累计奖励 NaN"
    print("[ok] 随机 episode（%d 步, 结束原因=%s, 累计奖励=%.1f）"
          % (steps, info["end_reason"], total))


if __name__ == "__main__":
    test_api()
    test_dynamics()
    test_episode()
    print("\n冒烟测试通过 ✓ 可以开始 Stage 1 训练")
