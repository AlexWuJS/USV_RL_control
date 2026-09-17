# -*- coding: utf-8 -*-
"""LiDAR 36 扇区化：体轴点云 → 36 维最近障碍距离。

约定（RL_INTERFACE_DESIGN.md §5 / §13.6）：
- 输入点云已在统一体轴系（艏向 +x），ROS 节点负责 PointCloud2 → (M,3) numpy
  并施加 hull_to_sensor_yaw 标定旋转；Gymnasium 环境 raycast 输出同格式。
- 扇区 0 中心对准艏向，逆时针编号，每扇区 10°；扇区 i 覆盖 [i·10°-5°, i·10°+5°)。
- 每扇区取最近点距离；无回波/超量程 → max_range。
- 高度带过滤（相对传感器原点）：默认 [-0.3, +1.0] m，Phase 7 实艇标定。
"""
import numpy as np

from .contracts import N_SECTORS, LIDAR_MAX_RANGE


class LidarSectorizer:
    def __init__(self,
                 n_sectors: int = N_SECTORS,
                 max_range: float = LIDAR_MAX_RANGE,
                 h_min: float = -0.3,
                 h_max: float = 1.0):
        self.n_sectors = int(n_sectors)
        self.max_range = float(max_range)
        self.h_min = float(h_min)
        self.h_max = float(h_max)
        self._width = 2.0 * np.pi / self.n_sectors
        self._half = 0.5 * self._width

    # ------------------------------------------------------------------
    def process(self, points_xyz) -> np.ndarray:
        """(M,3) 体轴点云 → (n_sectors,) 米制最近距离。"""
        sectors = np.full(self.n_sectors, self.max_range, dtype=np.float64)
        pts = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
        if pts.size == 0:
            return sectors

        pts = pts[np.isfinite(pts).all(axis=1)]
        # 高度带过滤
        band = (pts[:, 2] >= self.h_min) & (pts[:, 2] <= self.h_max)
        pts = pts[band]
        if pts.shape[0] == 0:
            return sectors

        rng = np.hypot(pts[:, 0], pts[:, 1])
        in_range = rng <= self.max_range
        rng = rng[in_range]
        if rng.size == 0:
            return sectors

        ang = np.arctan2(pts[in_range, 1], pts[in_range, 0])
        # 扇区 0 中心 = 艏向；ang=0 → (0+half)/width = 0.5 → 0
        idx = np.floor(((ang + self._half) % (2.0 * np.pi)) / self._width).astype(np.int64)
        idx = np.clip(idx, 0, self.n_sectors - 1)
        np.minimum.at(sectors, idx, rng)
        return sectors

    # ------------------------------------------------------------------
    def normalize(self, sectors_m: np.ndarray) -> np.ndarray:
        """米制距离 → [0,1]（0=紧贴，1=量程外/无障碍）。"""
        return np.clip(np.asarray(sectors_m, dtype=np.float64) / self.max_range, 0.0, 1.0)

    def min_clearance(self, sectors_m: np.ndarray) -> float:
        """全扇区最小间隙（米）。"""
        return float(np.min(sectors_m)) if len(sectors_m) else self.max_range
