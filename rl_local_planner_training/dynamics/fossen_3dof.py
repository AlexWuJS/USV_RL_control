# -*- coding: utf-8 -*-
"""Fossen 3-DOF（surge-sway-yaw）动力学（对照/升级用，默认不启用）。

    M·ν̇ + C(ν)·ν + D(ν)·ν = τ(η, ν, ν_ref)

ν = [u, v, r]，η 位置/艏向。低层闭环（参考 → 推力）用简单 P 控制压缩：
    X_thrust = Kp_v · (v_ref − u)，左右差动 → τ_N = Kp_r · wrap(course_ref − course)
阻尼系数取自项目内已有数据：
    Xu = 51.3, Xu|u| = 72.4   ← pid_controller.cpp:170 的阻尼模型
    质量/回转惯量为估计值，TODO: system_identification 加减速/回转数据标定。
本实现为论文对照实验准备，与 first_order 共享同一 (v_ref, course_ref) 接口。
"""
import numpy as np

from rl_common import BodyState, angle_diff
from .base import BaseDynamics


class Fossen3DOF(BaseDynamics):
    def __init__(self, params: dict):
        # 惯性（估计值，TODO 标定）
        self.mass = float(params.get("mass", 80.0))
        self.Iz = float(params.get("Iz", 40.0))
        self.m_x_dot = float(params.get("m_x_dot", 15.0))   # 附加质量
        self.m_y_dot = float(params.get("m_y_dot", 40.0))
        self.n_r_dot = float(params.get("n_r_dot", 20.0))
        # 阻尼（surge 来自 pid_controller.cpp:170）
        self.X_u = float(params.get("X_u", 51.3))
        self.X_uu = float(params.get("X_uu", 72.4))
        self.Y_v = float(params.get("Y_v", 40.0))
        self.Y_vv = float(params.get("Y_vv", 80.0))
        self.N_r = float(params.get("N_r", 60.0))
        self.N_rrr = float(params.get("N_rrr", 50.0))
        # 低层闭环增益（近似 huamo 滑模的闭环刚度，TODO shadow 数据回归）
        self.kp_v = float(params.get("kp_v", 250.0))
        self.kp_r = float(params.get("kp_r", 250.0))
        self.thrust_max = float(params.get("thrust_max", 550.0))  # N/桨 @100%
        self.arm = 1.02                                          # m，pid_controller.cpp:180

        self._state = BodyState()
        self._M = np.diag([self.mass + self.m_x_dot,
                           self.mass + self.m_y_dot,
                           self.Iz + self.n_r_dot])

    # ------------------------------------------------------------------
    def reset(self, x, y, psi, u=0.0, v=0.0, r=0.0):
        self._state = BodyState(x=x, y=y, psi=psi, u=u, v=v, r=r)

    def _coriolis(self, u, v, r):
        m = self.mass
        return np.array([
            [0.0, 0.0, -m * (self.m_y_dot / (m + 1e-9)) * v - m * v],
            [0.0, 0.0, m * u],
            [m * v, -m * u, 0.0],
        ])

    def step(self, v_ref, course_ref, dt):
        s = self._state
        u, v, r = s.u, s.v, s.r

        # 低层闭环：速度/航向 P → 合力与力矩（近似，替代真实滑模）
        F_surge = np.clip(self.kp_v * (v_ref - u), 0.0, 2.0 * self.thrust_max)
        course = s.psi + float(np.arctan2(v, max(u, 0.05)))
        tau_n = np.clip(self.kp_r * angle_diff(course_ref, course),
                        -self.thrust_max * self.arm, self.thrust_max * self.arm)
        tau = np.array([F_surge, 0.0, tau_n])

        nu = np.array([u, v, r])
        D_nu = np.array([
            self.X_u * u + self.X_uu * abs(u) * u,
            self.Y_v * v + self.Y_vv * abs(v) * v,
            self.N_r * r + self.N_rrr * r ** 3,
        ])
        nu_dot = np.linalg.solve(self._M, tau - self._coriolis(u, v, r) @ nu - D_nu)

        u2, v2, r2 = nu + nu_dot * dt
        psi = s.psi + r2 * dt
        c, sn = np.cos(psi), np.sin(psi)
        x = s.x + (u2 * c - v2 * sn) * dt
        y = s.y + (u2 * sn + v2 * c) * dt

        self._state = BodyState(x=x, y=y, psi=float(psi % (2 * np.pi)),
                                u=float(u2), v=float(v2), r=float(r2))
        return self._state

    def randomize(self, rng, dr_ranges):
        if not dr_ranges:
            return
        f = rng.uniform(*dr_ranges.get("tau_scale", [1.0, 1.0]))
        self.X_u *= f
        self.X_uu *= f
        self.N_r *= f
        self.kp_v *= rng.uniform(*dr_ranges.get("k_r_scale", [1.0, 1.0]))
        self.kp_r *= rng.uniform(*dr_ranges.get("r_max_scale", [1.0, 1.0]))

    @property
    def state(self):
        return self._state
