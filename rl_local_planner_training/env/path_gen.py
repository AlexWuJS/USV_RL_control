# -*- coding: utf-8 -*-
"""随机全局路径生成：随机航点 → Catmull-Rom 样条 → 0.5m 等弧长重采样。

0.5m 间距与实艇 a_star_planner 的栅格分辨率一致（a_star_demo.launch
map_resolution=0.5），保证训练/实艇的 PathFeatures 行为对齐。
"""
import numpy as np


def _catmull_rom(points, samples_per_seg=40):
    P = np.asarray(points, dtype=np.float64)
    if len(P) < 2:
        return P
    ext = np.vstack([2 * P[0] - P[1], P, 2 * P[-1] - P[-2]])
    out = []
    for i in range(len(ext) - 3):
        p0, p1, p2, p3 = ext[i], ext[i + 1], ext[i + 2], ext[i + 3]
        t = np.linspace(0.0, 1.0, samples_per_seg, endpoint=False)[:, None]
        out.append(0.5 * ((2 * p1) + (-p0 + p2) * t
                          + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t ** 2
                          + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    out.append(P[-1][None, :])
    return np.vstack(out)


def _resample(dense, spacing=0.5):
    seg = np.hypot(*(dense[1:] - dense[:-1]).T)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    n = max(int(round(s[-1] / spacing)) + 1, 2)
    si = np.linspace(0.0, s[-1], n)
    return np.column_stack([np.interp(si, s, dense[:, 0]),
                            np.interp(si, s, dense[:, 1])])


def gen_path(rng, difficulty: str = "medium", spacing: float = 0.5):
    """随机生成一条 x∈[0,80] 的全局路径，返回 (N,2) 世界坐标点列。"""
    wiggle = {"easy": (12.0, 6.0), "medium": (18.0, 12.0),
              "hard": (14.0, 18.0)}.get(difficulty, (18.0, 12.0))
    dx_step, y_amp = wiggle

    waypoints = [(0.0, float(rng.uniform(-10, 10)))]
    x = 0.0
    while x < 75.0:
        x += float(rng.uniform(0.6 * dx_step, 1.4 * dx_step))
        waypoints.append((x, float(rng.uniform(-y_amp, y_amp))))
    waypoints.append((80.0, float(rng.uniform(-10, 10))))

    dense = _catmull_rom(waypoints)
    return _resample(dense, spacing)
