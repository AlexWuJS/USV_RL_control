# -*- coding: utf-8 -*-
"""rl_common 契约层：观测/动作/指令的全部维度、单位、坐标系、归一化定义。

双端共用约定（训练机 Gymnasium 环境 与 小艇 NX ROS 节点 必须一致，禁止各写一套）：

坐标系
    世界系 = camera_init（FAST-LIO /Odometry 所在系），标准 ENU（x 东/y 北，yaw 逆时针为正）。
    体轴系 = 艏向为 +x、左舷为 +y 的右手系。
    注意：传感器体轴相对艇体首向可能存在固定安装旋转，由 ROS 节点的
    hull_to_sensor_yaw 参数标定，rl_common 不感知该差异（输入已是统一体轴系）。

单位
    距离 m，速度 m/s，角度 rad（内部一律 rad，仅在最外层与 yaml 交互时可用度），
    时间 s，控制频率 10 Hz（dt = 0.1）。

航向约定
    全部使用标准 ENU 角。huamo 内部的 "+91.5° / atan2 取反" 约定只在
    huamo_node 执行段做一次换算（desired_heading = course + pi），rl_common 不涉及。
"""
import numpy as np
from dataclasses import dataclass, field
from typing import Tuple

# ---------------------------------------------------------------------------
# 观测空间布局（总计 49 维，顺序即网络输入顺序，两端不得改动）
# ---------------------------------------------------------------------------
OBS_DIM = 49
ACTION_DIM = 2

OBS_SLICE_SELF = slice(0, 4)        # [u, r, beta, v_ref]
OBS_SLICE_PATH = slice(4, 11)       # [cte, e_heading, la1_x, la1_y, la2_x, la2_y, s_remaining]
OBS_SLICE_LIDAR = slice(11, 47)     # 36 扇区归一化距离
OBS_SLICE_PREV_ACTION = slice(47, 49)  # 上一拍原始动作（[-1,1]）

N_SECTORS = 36
LIDAR_MAX_RANGE = 20.0              # m，扇区无回波时填满量程

# 归一化尺度（与 RL_INTERFACE_DESIGN.md §5 表一致）
U_NORM = 2.5                        # m/s，实艇稳态极速 ≈ B*100 ≈ 2.36 (huamo.cpp A/B)
R_NORM = 1.0                        # rad/s
CTE_NORM = 10.0                     # m
LA_NORM = 20.0                      # m（前视点体轴坐标）
S_REMAIN_NORM = 100.0               # m
BETA_NORM = np.pi                   # 漂角
COURSE_SPEED_MIN = 0.10             # m/s，低于此速度航迹向不可信，退回艏向

# 速度参考域（与实艇 huamo LOS 速度域一致；实艇滑模不可倒车）
V_REF_MIN = 0.2                     # m/s
V_REF_MAX = 2.0                     # m/s

# 动作限幅（10Hz 下的每步增量）
DV_MAX = 0.10                       # m/s/步 → 参考加速度上限 1.0 m/s²
DCOURSE_MAX = 0.05                  # rad/步 ≈ 2.9°/步 → 参考转率上限 ≈ 28.6°/s

# 源标识（NavCmd.source）
SRC_LOS = 0
SRC_RL = 1
SRC_OVERRIDE = 2


# ---------------------------------------------------------------------------
# 数据结构（纯 Python，无 ROS 依赖）
# ---------------------------------------------------------------------------
@dataclass
class BodyState:
    """船体状态（camera_init 系 + 体轴速度分量）。"""
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0        # 艏向角 (rad, ENU)
    u: float = 0.0          # 体轴纵荡速度 (m/s, 艏向为正)
    v: float = 0.0          # 体轴横荡速度 (m/s, 左舷为正)
    r: float = 0.0          # 偏航角速度 (rad/s, 逆时针为正)

    @property
    def speed(self) -> float:
        return float(np.hypot(self.u, self.v))

    @property
    def course(self) -> float:
        """航迹向（速度方向）。低速时航迹向不可信，退回艏向。"""
        if np.hypot(self.u, self.v) < COURSE_SPEED_MIN:
            return self.psi
        vx = self.u * np.cos(self.psi) - self.v * np.sin(self.psi)
        vy = self.u * np.sin(self.psi) + self.v * np.cos(self.psi)
        return float(np.arctan2(vy, vx))

    @property
    def beta(self) -> float:
        """漂角；u 过小无定义，返回 0。"""
        if abs(self.u) < 0.05:
            return 0.0
        return float(np.arctan2(self.v, self.u))


@dataclass
class PathFeatureOut:
    """路径局部特征（几何量，未归一化）。cte 左正右负。"""
    cte: float = 0.0                # m，横偏（路径切线左侧为正）
    tangent_course: float = 0.0     # rad，最近点处路径切线方向 (ENU)
    la1_bx: float = 0.0             # 前视点1（弧长 +6m）体轴 x
    la1_by: float = 0.0
    la2_bx: float = 0.0             # 前视点2（弧长 +15m）体轴 x
    la2_by: float = 0.0
    s_remaining: float = 0.0        # m，最近点到路径终点的剩余弧长
    closest_idx: int = 0
    s_proj: float = 0.0             # m，投影弧长


@dataclass
class ActionLimits:
    dv_max: float = DV_MAX
    dcourse_max: float = DCOURSE_MAX
    v_min: float = V_REF_MIN
    v_max: float = V_REF_MAX


@dataclass
class ActionOutput:
    delta_v: float = 0.0            # m/s（限幅后实际生效值）
    delta_course: float = 0.0       # rad
    v_ref: float = 0.0              # m/s，积分后的速度参考
    course_ref: float = 0.0         # rad，积分后的航迹向参考 (ENU)


@dataclass
class NavCommand:
    """guidance 层输出指令（对应 rl_local_planner/NavCmd.msg）。"""
    desired_speed: float = 0.0      # m/s，[0, v_max]；0.0 = 停船
    desired_course: float = 0.0     # rad，camera_init 系 ENU 航迹向
    source: int = SRC_RL
    supervisor_active: bool = False


def normalize_v_ref(v: float, v_min: float = V_REF_MIN, v_max: float = V_REF_MAX) -> float:
    """速度参考 → [-1, 1]。"""
    return float(2.0 * (v - v_min) / (v_max - v_min) - 1.0)


def denormalize_v_ref(x: float, v_min: float = V_REF_MIN, v_max: float = V_REF_MAX) -> float:
    """[-1, 1] → 速度参考。"""
    return float(v_min + (x + 1.0) * (v_max - v_min) / 2.0)
