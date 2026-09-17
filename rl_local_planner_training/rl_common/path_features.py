# -*- coding: utf-8 -*-
"""全局路径局部特征：CTE / 切线方向 / 弧长前视点 / 剩余弧长。

实现依据（RL_INTERFACE_DESIGN.md §5）：
- 路径切线一律由相邻 pose **位置差分** 计算，不信任 pose.orientation 四元数
  （栅格 A* 发布单位四元数，getYaw 恒 0；Hybrid A* 才有真朝向）。
- CTE 计算方法同 set_of_los.cpp:64-70（路径切线系分解），符号约定：路径切线
  左侧为正。
- 输入路径为 camera_init 系 ENU 坐标点 (N,2)；ROS 节点负责 nav_msgs/Path
  （含 TF map→camera_init 变换）到此格式，Gymnasium 环境直接给 numpy 数组。
"""
import numpy as np
from dataclasses import dataclass

from .angle_utils import wrap_scalar_to_pi
from .contracts import PathFeatureOut

_MIN_SEG_LEN = 1e-6


class PathFeatures:
    """对一条固定路径做增量查询。路径更新时重新构造本对象（每条新路径一次）。"""

    def __init__(self, points, la1_dist: float = 6.0, la2_dist: float = 15.0):
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        if len(pts) >= 2:
            keep = np.ones(len(pts), dtype=bool)
            keep[1:] = np.hypot(*(pts[1:] - pts[:-1]).T) > _MIN_SEG_LEN
            pts = pts[keep]
        self.pts = pts
        self.la1_dist = float(la1_dist)
        self.la2_dist = float(la2_dist)

        if len(pts) < 2:
            self.valid = False
            self.s_total = 0.0
            return
        self.valid = True

        self.seg = pts[1:] - pts[:-1]                       # (N-1, 2)
        seg_len = np.hypot(self.seg[:, 0], self.seg[:, 1])  # (N-1,)
        self.seg_len = np.maximum(seg_len, _MIN_SEG_LEN)
        self.s_cum = np.concatenate(([0.0], np.cumsum(seg_len)))  # (N,)
        self.s_total = float(self.s_cum[-1])
        # 每个顶点的切线 = 所在线段方向；末点沿用最后一段
        self.tangent = self.seg / self.seg_len[:, None]     # (N-1, 2)
        self.tangent = np.vstack([self.tangent, self.tangent[-1]])

    # ------------------------------------------------------------------
    def update(self, pos) -> PathFeatureOut:
        """查询船当前位置的路径局部特征。pos: (2,) world。"""
        out = PathFeatureOut()
        if not self.valid:
            return out
        pos = np.asarray(pos, dtype=np.float64)
        i = int(np.argmin(((self.pts - pos) ** 2).sum(axis=1)))
        t = self.tangent[i]
        rel = pos - self.pts[i]

        # CTE：切线 × (船 - 最近点)，左侧为正
        out.cte = float(t[0] * rel[1] - t[1] * rel[0])
        # 投影弧长
        out.s_proj = float(np.clip(self.s_cum[i] + float(rel @ t), 0.0, self.s_total))
        out.closest_idx = i
        out.tangent_course = float(np.arctan2(t[1], t[0]))
        out.s_remaining = self.s_total - out.s_proj
        return out

    # ------------------------------------------------------------------
    def with_body_lookahead(self, feat: PathFeatureOut, pos, psi: float) -> PathFeatureOut:
        """把前视点转到体轴系并写回 feat（update() 与体轴变换合一步的便捷版）。"""
        pos = np.asarray(pos, dtype=np.float64)
        la1_w = self.point_at(feat.s_proj + self.la1_dist)
        la2_w = self.point_at(feat.s_proj + self.la2_dist)
        feat.la1_bx, feat.la1_by = self._to_body(la1_w - pos, psi)
        feat.la2_bx, feat.la2_by = self._to_body(la2_w - pos, psi)
        return feat

    def update_full(self, pos, psi: float) -> PathFeatureOut:
        """update + 体轴前视点，一步到位（推荐入口）。"""
        return self.with_body_lookahead(self.update(pos), pos, psi)

    # ------------------------------------------------------------------
    def point_at(self, s: float) -> np.ndarray:
        """弧长 s 处的路径点（线性插值，越界钳位到端点）。"""
        s = float(np.clip(s, 0.0, self.s_total))
        j = int(np.searchsorted(self.s_cum, s, side="right") - 1)
        j = min(max(j, 0), len(self.seg) - 1)
        f = (s - self.s_cum[j]) / self.seg_len[j]
        return self.pts[j] + f * self.seg[j]

    def tangent_at(self, s: float) -> float:
        """弧长 s 处切线方向 (ENU, rad)。"""
        s = float(np.clip(s, 0.0, self.s_total))
        j = min(max(int(np.searchsorted(self.s_cum, s, side="right") - 1), 0), len(self.seg) - 1)
        return float(np.arctan2(self.seg[j, 1], self.seg[j, 0]))

    def tangent_at_start(self) -> float:
        """路径起点切线方向，用于动作参考重锚定。"""
        return self.tangent_at(0.0) if self.valid else 0.0

    # ------------------------------------------------------------------
    @staticmethod
    def _to_body(vec_world, psi: float):
        """世界系向量 → 体轴系（艏向 +x，左舷 +y）。"""
        c, s = np.cos(psi), np.sin(psi)
        return float(c * vec_world[0] + s * vec_world[1]), float(-s * vec_world[0] + c * vec_world[1])
