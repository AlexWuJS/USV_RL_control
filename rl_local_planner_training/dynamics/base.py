# -*- coding: utf-8 -*-
"""动力学基类。

接口约定：输入 (v_ref, course_ref)（guidance 层参考，与实艇 huamo 执行段的
输入一致），内部隐含"参考 → 滑模/PID → 推进器 → 船体"的闭环压缩为
一阶/二阶响应；输出更新后的 BodyState。状态定义见 rl_common/contracts.py。
"""
from abc import ABC, abstractmethod


class BaseDynamics(ABC):
    """所有动力学实现的公共接口。"""

    @abstractmethod
    def reset(self, x: float, y: float, psi: float, u: float = 0.0,
              v: float = 0.0, r: float = 0.0):
        """复位到初始位姿/速度。"""

    @abstractmethod
    def step(self, v_ref: float, course_ref: float, dt: float):
        """前进一步，返回更新后的状态。"""

    @abstractmethod
    def randomize(self, rng, dr_ranges: dict):
        """域随机化（每 episode 一次）。无随机化的实现可空操作。"""

    @property
    @abstractmethod
    def state(self):
        """当前状态（BodyState）。"""
