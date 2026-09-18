# -*- coding: utf-8 -*-
"""一阶惯性动力学（默认，RL_INTERFACE_DESIGN.md §9）。

    u_dot = (v_ref − u) / tau_u
    desired_r = clip(k_r · wrap(course_ref − course), ±r_max)
    r_dot = (desired_r − r) / tau_r
    v_dot = −v / tau_v
    ẋ = u·cosψ − v·sinψ ; ẏ = u·sinψ + v·cosψ ; ψ̇ = r

参数来源（实艇证据）：
    tau_u = 0.79 s  ← huamo.cpp:7  A=0.793023（稳态 u = B/A·thrust%，B·100 ≈ 2.36 m/s）
    tau_r = 0.53 s  ← sys_ident.cpp Nomoto T ≈ 0.5308（Z 形试验）
    k_r / r_max     ← 与 huamo 转弯量级一致的保守初值，shadow 数据回归修正

course = ψ + atan2(v, max(u, 0.05))（低速近似；漂角小艇量级下足够）。
"""
import numpy as np

from rl_common import BodyState, angle_diff
from .base import BaseDynamics


class FirstOrderDynamics(BaseDynamics):
    def __init__(self, params: dict):
        self.tau_u = float(params.get("tau_u", 0.79))
        self.tau_r = float(params.get("tau_r", 0.53))
        self.tau_v = float(params.get("tau_v", 0.5))
        self.k_r = float(params.get("k_r", 0.8))
        self.r_max = float(params.get("r_max", 0.6))
        self.v_max_phys = float(params.get("v_max_phys", 2.3))
        # 基准参数：域随机化每回合以此为基准重采样（不可累计相乘，否则随机游走出 0/inf）
        self._base = dict(tau_u=self.tau_u, tau_r=self.tau_r, tau_v=self.tau_v,
                          k_r=self.k_r, r_max=self.r_max,
                          v_max_phys=self.v_max_phys)
        self._state = BodyState()

    # ------------------------------------------------------------------
    def reset(self, x, y, psi, u=0.0, v=0.0, r=0.0):
        self._state = BodyState(x=x, y=y, psi=psi, u=u, v=v, r=r)

    def step(self, v_ref, course_ref, dt):
        s = self._state
        # 纵荡：跟踪速度参考（参考由 ActionProcessor 钳位在 [0.2, 2.0]，
        # 但 supervisor 可能给 0 → 停船，动力学允许 u 趋向 0）
        target_u = float(np.clip(v_ref, 0.0, self.v_max_phys))
        u = s.u + (target_u - s.u) / self.tau_u * dt

        # 偏航：course 反馈 → 期望转率 → 一阶跟踪
        course = s.psi + float(np.arctan2(s.v, max(s.u, 0.05)))
        desired_r = float(np.clip(self.k_r * angle_diff(course_ref, course),
                                  -self.r_max, self.r_max))
        r = s.r + (desired_r - s.r) / self.tau_r * dt

        # 横荡：被动衰减
        v = s.v - s.v / self.tau_v * dt

        # 运动学
        psi = s.psi + r * dt
        x = s.x + (u * np.cos(psi) - v * np.sin(psi)) * dt
        y = s.y + (u * np.sin(psi) + v * np.cos(psi)) * dt

        self._state = BodyState(x=x, y=y, psi=float(psi % (2 * np.pi)),
                                u=float(u), v=float(v), r=float(r))
        return self._state

    def randomize(self, rng, dr_ranges):
        # 每回合先恢复基准参数，再单次缩放（独立重采样，非累计）
        for k, v in self._base.items():
            setattr(self, k, v)
        if not dr_ranges:
            return
        f = rng.uniform(*dr_ranges.get("tau_scale", [1.0, 1.0]))
        self.tau_u *= f
        self.tau_r *= f
        self.r_max *= rng.uniform(*dr_ranges.get("r_max_scale", [1.0, 1.0]))
        self.k_r *= rng.uniform(*dr_ranges.get("k_r_scale", [1.0, 1.0]))

    @property
    def state(self):
        return self._state
