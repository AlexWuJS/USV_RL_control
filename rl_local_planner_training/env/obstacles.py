# -*- coding: utf-8 -*-
"""障碍物：静态圆 / 动态圆 / 窄通道门，及其表面点云（喂给 rl_common.LidarSectorizer）。

扇区化管线与实艇一致：世界系表面点 → 体轴变换（env 内做）→
LidarSectorizer.process()。禁止在 env 里另写一套扇区逻辑。
"""
import numpy as np
from dataclasses import dataclass


@dataclass
class Obstacle:
    x: float
    y: float
    radius: float
    vx: float = 0.0
    vy: float = 0.0

    def step(self, dt, bounds=(-25.0, 25.0)):
        if self.vx == 0.0 and self.vy == 0.0:
            return
        self.x += self.vx * dt
        self.y += self.vy * dt
        if self.y < bounds[0] or self.y > bounds[1]:
            self.vy = -self.vy
            self.y = float(np.clip(self.y, bounds[0], bounds[1]))
        if self.x < 2.0 or self.x > 78.0:
            self.vx = -self.vx
            self.x = float(np.clip(self.x, 2.0, 78.0))


def _dist_to_path(path, x, y):
    return float(np.min(np.hypot(path[:, 0] - x, path[:, 1] - y)))


def spawn_static(rng, path, n, radius_range=(1.0, 3.0), margin=3.0,
                 start_clear=8.0, max_try=200):
    """静态障碍：距路径 margin+r 以上、起点 start_clear 内不放。"""
    obstacles = []
    for _ in range(int(n)):
        for _try in range(max_try):
            r = float(rng.uniform(*radius_range))
            x = float(rng.uniform(5, 75))
            y = float(rng.uniform(-25, 25))
            if _dist_to_path(path, x, y) < margin + r:
                continue
            if np.hypot(x - path[0, 0], y - path[0, 1]) < start_clear + r:
                continue
            obstacles.append(Obstacle(x=x, y=y, radius=r))
            break
    return obstacles


def spawn_moving(rng, path, n, radius_range=(1.0, 2.0), speed_range=(0.2, 0.5)):
    """动态障碍：匀速直线 + 边界反射。"""
    obstacles = []
    for _ in range(int(n)):
        r = float(rng.uniform(*radius_range))
        x = float(rng.uniform(15, 70))
        y = float(rng.uniform(-20, 20))
        ang = float(rng.uniform(0, 2 * np.pi))
        spd = float(rng.uniform(*speed_range))
        obstacles.append(Obstacle(x=x, y=y, radius=r,
                                  vx=spd * np.cos(ang), vy=spd * np.sin(ang)))
    return obstacles


def spawn_gates(rng, path, n, gap=4.0, radius=1.5):
    """窄通道：在路径中段沿法线对称放两个障碍，净宽 = gap。"""
    obstacles = []
    n0 = len(path)
    for _ in range(int(n)):
        idx = int(rng.uniform(n0 // 3, 2 * n0 // 3))
        px, py = path[idx]
        # 法线方向（用邻近点差分求切线）
        j = min(idx + 1, n0 - 1)
        k = max(idx - 1, 0)
        tx, ty = path[j, 0] - path[k, 0], path[j, 1] - path[k, 1]
        norm = np.hypot(tx, ty) + 1e-9
        nx, ny = -ty / norm, tx / norm
        off = gap / 2.0 + radius
        obstacles.append(Obstacle(x=px + nx * off, y=py + ny * off, radius=radius))
        obstacles.append(Obstacle(x=px - nx * off, y=py - ny * off, radius=radius))
    return obstacles


def surface_points(obstacles, n_per_circle=90):
    """所有障碍的表面点（世界系, (M,2)）——扇区化的几何来源。"""
    if not obstacles:
        return np.zeros((0, 2))
    ang = np.linspace(0.0, 2.0 * np.pi, n_per_circle, endpoint=False)
    pts = []
    for ob in obstacles:
        pts.append(np.column_stack([ob.x + ob.radius * np.cos(ang),
                                    ob.y + ob.radius * np.sin(ang)]))
    return np.vstack(pts)
