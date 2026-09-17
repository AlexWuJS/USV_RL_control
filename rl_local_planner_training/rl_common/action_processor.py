# -*- coding: utf-8 -*-
"""动作处理：SAC 原始输出 [-1,1]² → 增量限幅 → 速度/航迹向参考积分。

对应 RL_INTERFACE_DESIGN.md §6：
    v_ref(t+1)      = clip(v_ref(t) + Δv_ref, V_MIN, V_MAX)
    course_ref(t+1) = wrap(course_ref(t) + Δcourse_ref)
重锚定（新路径 / cte 超限）由调用方调用 reset() 完成，规则见 §6。
"""
import numpy as np

from .contracts import ActionLimits, ActionOutput, V_REF_MIN, V_REF_MAX, DV_MAX, DCOURSE_MAX
from .angle_utils import wrap_scalar_to_pi


class ActionProcessor:
    def __init__(self, limits: ActionLimits = None):
        self.limits = limits or ActionLimits()
        self.v_ref = V_REF_MIN
        self.course_ref = 0.0

    # ------------------------------------------------------------------
    def reset(self, v_ref0: float, course_ref0: float):
        """新路径到达 / 参考失锚时调用：软起步（v_ref0 常取当前 u）。"""
        self.v_ref = float(np.clip(v_ref0, self.limits.v_min, self.limits.v_max))
        self.course_ref = wrap_scalar_to_pi(course_ref0)

    def force_v_ref(self, v: float):
        """Safety Supervisor 之外，仅用于测试/标定。"""
        self.v_ref = float(np.clip(v, self.limits.v_min, self.limits.v_max))

    # ------------------------------------------------------------------
    def process(self, raw_action) -> ActionOutput:
        """raw_action: [-1,1]²（SAC 输出，训练采样 / 部署取确定性均值）。"""
        a = np.clip(np.asarray(raw_action, dtype=np.float64).reshape(-1), -1.0, 1.0)
        out = ActionOutput()
        out.delta_v = float(a[0] * self.limits.dv_max)
        out.delta_course = float(a[1] * self.limits.dcourse_max)
        self.v_ref = float(np.clip(self.v_ref + out.delta_v,
                                   self.limits.v_min, self.limits.v_max))
        self.course_ref = wrap_scalar_to_pi(self.course_ref + out.delta_course)
        out.v_ref = self.v_ref
        out.course_ref = self.course_ref
        return out

    # ------------------------------------------------------------------
    def anchor_deviation(self, tangent_course: float) -> float:
        """当前 course_ref 相对路径切线的偏差（rad, wrap 后），供 Supervisor/重锚定判断。"""
        return wrap_scalar_to_pi(self.course_ref - tangent_course)
