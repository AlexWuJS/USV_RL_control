# -*- coding: utf-8 -*-
"""rl_common：训练与部署共用的观测/动作/安全契约层（纯 Python + numpy，零 ROS 依赖）。

同步纪律（RL_INTERFACE_DESIGN.md §14）：本目录由 NX 开发维护，
经 rl_local_planner_training/sync_rl_common.sh 复制到训练工程，
两端禁止各写一套观测/动作/安全实现。
"""
from .contracts import (
    OBS_DIM, ACTION_DIM,
    OBS_SLICE_SELF, OBS_SLICE_PATH, OBS_SLICE_LIDAR, OBS_SLICE_PREV_ACTION,
    N_SECTORS, LIDAR_MAX_RANGE,
    U_NORM, R_NORM, CTE_NORM, LA_NORM, S_REMAIN_NORM, BETA_NORM, COURSE_SPEED_MIN,
    V_REF_MIN, V_REF_MAX, DV_MAX, DCOURSE_MAX,
    SRC_LOS, SRC_RL, SRC_OVERRIDE,
    BodyState, PathFeatureOut, ActionLimits, ActionOutput, NavCommand,
    normalize_v_ref, denormalize_v_ref,
)
from .angle_utils import wrap_to_pi, wrap_to_2pi, angle_diff, wrap_scalar_to_pi
from .path_features import PathFeatures
from .lidar_sectorizer import LidarSectorizer
from .observation_builder import ObsConfig, ObservationBuilder
from .action_processor import ActionProcessor
from .safety_supervisor import SafetyConfig, SafetySupervisor

__all__ = [
    "OBS_DIM", "ACTION_DIM",
    "OBS_SLICE_SELF", "OBS_SLICE_PATH", "OBS_SLICE_LIDAR", "OBS_SLICE_PREV_ACTION",
    "N_SECTORS", "LIDAR_MAX_RANGE",
    "U_NORM", "R_NORM", "CTE_NORM", "LA_NORM", "S_REMAIN_NORM", "BETA_NORM",
    "COURSE_SPEED_MIN",
    "V_REF_MIN", "V_REF_MAX", "DV_MAX", "DCOURSE_MAX",
    "SRC_LOS", "SRC_RL", "SRC_OVERRIDE",
    "BodyState", "PathFeatureOut", "ActionLimits", "ActionOutput", "NavCommand",
    "normalize_v_ref", "denormalize_v_ref",
    "wrap_to_pi", "wrap_to_2pi", "angle_diff", "wrap_scalar_to_pi",
    "PathFeatures", "LidarSectorizer",
    "ObsConfig", "ObservationBuilder", "ActionProcessor",
    "SafetyConfig", "SafetySupervisor",
]
