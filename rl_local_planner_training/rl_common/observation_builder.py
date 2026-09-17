# -*- coding: utf-8 -*-
"""Observation 构建：把状态/路径特征/扇区距离组装成 49 维网络输入。

这是训练与部署共用的唯一观测实现。任何字段增删、顺序调整、归一化改动
必须同时体现在 contracts.py 常量与本文件，并同步两端。
"""
import numpy as np
from dataclasses import dataclass

from .contracts import (
    OBS_DIM, OBS_SLICE_SELF, OBS_SLICE_PATH, OBS_SLICE_LIDAR, OBS_SLICE_PREV_ACTION,
    N_SECTORS, LIDAR_MAX_RANGE, U_NORM, R_NORM, CTE_NORM, LA_NORM, S_REMAIN_NORM,
    BETA_NORM, V_REF_MIN, V_REF_MAX, normalize_v_ref, BodyState, PathFeatureOut,
)
from .angle_utils import wrap_scalar_to_pi


@dataclass
class ObsConfig:
    la1_dist: float = 6.0
    la2_dist: float = 15.0
    n_sectors: int = N_SECTORS
    lidar_max_range: float = LIDAR_MAX_RANGE
    cte_norm: float = CTE_NORM
    s_remain_norm: float = S_REMAIN_NORM
    v_min: float = V_REF_MIN
    v_max: float = V_REF_MAX


class ObservationBuilder:
    """无状态组装器（扇区化由调用方完成：实艇 LidarSectorizer / 仿真 raycast）。"""

    def __init__(self, cfg: ObsConfig = None):
        self.cfg = cfg or ObsConfig()

    # ------------------------------------------------------------------
    def build(self,
              state: BodyState,
              path_feat: PathFeatureOut,
              sectors_m: np.ndarray,
              v_ref: float,
              prev_action: np.ndarray) -> np.ndarray:
        """返回 (49,) float32 观测。保证有限值（NaN/Inf 置 0）。"""
        obs = np.zeros(OBS_DIM, dtype=np.float64)

        # A. 自身状态
        obs[OBS_SLICE_SELF] = [
            state.u / U_NORM,
            state.r / R_NORM,
            state.beta / BETA_NORM,
            normalize_v_ref(v_ref, self.cfg.v_min, self.cfg.v_max),
        ]

        # B. 路径特征（e_heading = 路径切线 − 航迹向，正 = 路径在航迹向左侧）
        e_heading = wrap_scalar_to_pi(path_feat.tangent_course - state.course)
        obs[OBS_SLICE_PATH] = [
            np.clip(path_feat.cte / self.cfg.cte_norm, -2.0, 2.0),
            e_heading / np.pi,
            path_feat.la1_bx / LA_NORM,
            path_feat.la1_by / LA_NORM,
            path_feat.la2_bx / LA_NORM,
            path_feat.la2_by / LA_NORM,
            np.clip(path_feat.s_remaining / self.cfg.s_remain_norm, 0.0, 2.0),
        ]

        # C. 激光扇区（调用方已完成米制扇区化，这里只归一化）
        sectors = np.asarray(sectors_m, dtype=np.float64)
        obs[OBS_SLICE_LIDAR] = np.clip(sectors / self.cfg.lidar_max_range, 0.0, 1.0)

        # D. 上一拍原始动作（[-1,1]）
        pa = np.asarray(prev_action, dtype=np.float64).reshape(-1)
        obs[OBS_SLICE_PREV_ACTION] = np.clip(pa, -1.0, 1.0)

        # NaN/Inf 防御：任何非有限值置 0（由看门狗另行报警）
        if not np.isfinite(obs).all():
            obs = np.nan_to_num(obs, nan=0.0, posinf=1.0, neginf=-1.0)
        return obs.astype(np.float32)
