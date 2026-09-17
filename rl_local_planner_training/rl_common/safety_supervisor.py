# -*- coding: utf-8 -*-
"""Safety Supervisor：独立于 RL 的规则式安全层（RL_INTERFACE_DESIGN.md §7）。

位置：RL 输出之后、/guidance/rl_cmd 发布之前。任何一条规则触发即
source=SUPERVISOR_OVERRIDE，并置 supervisor_active 供诊断。

优先级（高→低）：
    P1 看门狗（节点层实现：观测 NaN / 推理超时 / 模型缺失 → 冻结或急停）
    P2 绝对钳位（ActionProcessor 已做；本层再兜底一次）
    P3 参考界：course_ref 偏离路径切线超过 course_dev_max → 截断
    P4 间隙-速度包络：d_min 越小速度上限越低，critical_zone 内直接停
    P5 外部急停：is_auto_control=false → thruster_manager 现有逻辑（不在本层）

注意：P4 允许输出 0.0（停船），低于 ActionProcessor 的 v_min —— 这是刻意的：
v_min 约束 RL 参考的常态下限，Supervisor 的职责是兜底到停。
"""
from dataclasses import dataclass

import numpy as np

from .contracts import NavCommand, SRC_OVERRIDE, SRC_RL, V_REF_MIN, V_REF_MAX
from .angle_utils import wrap_scalar_to_pi, wrap_to_pi


@dataclass
class SafetyConfig:
    slow_zone: float = 8.0         # m，低于此开始线性降速
    crawl_zone: float = 3.0        # m，低于此降到 crawl_speed
    critical_zone: float = 1.5     # m，低于此立即停
    crawl_speed: float = 0.4       # m/s
    course_dev_max: float = 1.047  # rad ≈ 60°，course_ref 相对路径切线最大偏差
    v_min: float = V_REF_MIN
    v_max: float = V_REF_MAX


class SafetySupervisor:
    def __init__(self, cfg: SafetyConfig = None):
        self.cfg = cfg or SafetyConfig()

    # ------------------------------------------------------------------
    def speed_envelope(self, d_min: float) -> float:
        """最小间隙 → 允许的最大速度。分段线性，critical_zone 内为 0。"""
        c = self.cfg
        if d_min >= c.slow_zone:
            return c.v_max
        if d_min > c.crawl_zone:
            f = (c.slow_zone - d_min) / (c.slow_zone - c.crawl_zone)
            return c.v_max + f * (c.crawl_speed - c.v_max)
        if d_min > c.critical_zone:
            f = (c.crawl_zone - d_min) / (c.crawl_zone - c.critical_zone)
            return c.crawl_speed * (1.0 - f)
        return 0.0

    # ------------------------------------------------------------------
    def apply(self, cmd: NavCommand, d_min: float, tangent_course: float) -> NavCommand:
        """对 RL 指令施加 P2+P3+P4，返回（可能被覆盖的）新指令。"""
        c = self.cfg
        overridden = False

        # P4 间隙-速度包络
        v_allow = self.speed_envelope(d_min)
        speed = float(cmd.desired_speed)
        if speed > v_allow:
            speed = v_allow
            overridden = True

        # P3 参考界：航迹向参考不得偏离路径切线超过 course_dev_max
        course = float(cmd.desired_course)
        dev = float(wrap_to_pi(course - tangent_course))
        if abs(dev) > c.course_dev_max:
            course = wrap_scalar_to_pi(tangent_course + np.sign(dev) * c.course_dev_max)
            overridden = True

        # P2 绝对钳位兜底
        if speed < 0.0:
            speed = 0.0
            overridden = True

        return NavCommand(
            desired_speed=speed,
            desired_course=course,
            source=SRC_OVERRIDE if overridden else cmd.source,
            supervisor_active=overridden,
        )

    # ------------------------------------------------------------------
    def emergency_stop(self, tangent_course: float = 0.0) -> NavCommand:
        """P1 看门狗终极动作：停船，航向保持路径切线。"""
        return NavCommand(desired_speed=0.0, desired_course=wrap_scalar_to_pi(tangent_course),
                          source=SRC_OVERRIDE, supervisor_active=True)
