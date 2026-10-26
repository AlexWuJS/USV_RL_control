# -*- coding: utf-8 -*-
"""障碍物与场景几何：静态/动态圆、窄通道门、岸线走廊，及其表面点云（喂给 rl_common.LidarSectorizer）。

扇区化管线与实艇一致：世界系表面点 → 体轴变换（env 内做）→
LidarSectorizer.process()。禁止在 env 里另写一套扇区逻辑。

场景摆放三种（env 按配置选用）：
    spawn_static        开阔场"避让式"：障碍面距路径 ≥ margin（旧行为，Stage2）
    spawn_path_local    路径局部系摆放：压路径（lat 0~1.5m，阻塞）或走廊内近路
    gen_shore_corridor  岸线走廊：两岸表面点云 + 走廊内跟踪路径（湖上真实约束）

障碍尺寸类（现实化）：小浮标 ≈0.3~0.8m、常规 1~3m、码头/船 ≈3~6m。
配置 key `obstacle_radius_ranges: [[0.3,0.8],[1.0,3.0],[3.0,6.0]]` 让每个障碍随机取类。
"""
import numpy as np
from dataclasses import dataclass
from typing import List, Sequence, Tuple

from .path_gen import _resample


@dataclass
class Obstacle:
    x: float
    y: float
    radius: float
    vx: float = 0.0
    vy: float = 0.0
    bx: Tuple[float, float] = (2.0, 78.0)     # 动态障碍反弹边界（x）
    by: Tuple[float, float] = (-25.0, 25.0)   # 动态障碍反弹边界（y）

    def step(self, dt):
        if self.vx == 0.0 and self.vy == 0.0:
            return
        self.x += self.vx * dt
        self.y += self.vy * dt
        if self.y < self.by[0] or self.y > self.by[1]:
            self.vy = -self.vy
            self.y = float(np.clip(self.y, *self.by))
        if self.x < self.bx[0] or self.x > self.bx[1]:
            self.vx = -self.vx
            self.x = float(np.clip(self.x, *self.bx))


@dataclass
class ShoreCorridor:
    """岸线走廊场景：两岸为"岸"（表面点云喂扇区化），跟踪路径在走廊内。"""
    path: np.ndarray        # (N,2) 跟踪路径（0.5m 间距，与实艇 A* 栅格一致）
    center: np.ndarray      # (N,2) 走廊中心线
    widths: np.ndarray      # (N,)  每点局部净宽（岸到岸，m）
    shore_pts: np.ndarray   # (M,2) 两岸表面点云（世界系）
    bbox: Tuple[float, float, float, float]   # (xmin, xmax, ymin, ymax) 走廊外接框


# ---------------------------------------------------------------------------
def _dist_to_path(path, x, y):
    return float(np.min(np.hypot(path[:, 0] - x, path[:, 1] - y)))


def _path_normals(path: np.ndarray) -> np.ndarray:
    """路径每点单位法线（左正；用中心差分切线）。"""
    t = np.gradient(np.asarray(path, dtype=np.float64), axis=0)
    tl = np.hypot(t[:, 0], t[:, 1])
    tl[tl < 1e-9] = 1e-9
    return np.column_stack([-t[:, 1] / tl, t[:, 0] / tl])


def _sample_radius(rng, radius_range, radius_ranges=None) -> float:
    """半径采样：给定类别表时先随机取类再类内均匀。"""
    if radius_ranges:
        cls = radius_ranges[int(rng.integers(0, len(radius_ranges)))]
        return float(rng.uniform(cls[0], cls[1]))
    return float(rng.uniform(radius_range[0], radius_range[1]))


def _arc_index_window(path, start_clear: float, end_clear: float):
    """按弧长给出可放置的索引窗口 [k0, k1)（距首/尾 start_clear/end_clear 以上）。"""
    seg = np.hypot(*(path[1:] - path[:-1]).T)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    k0 = int(np.searchsorted(s, start_clear))
    k1 = int(np.searchsorted(s, max(s[-1] - end_clear, start_clear + 1.0)))
    if k1 <= k0 + 1:
        k0, k1 = 1, max(2, len(path) - 2)
    return k0, k1


# ---------------------------------------------------------------------------
def spawn_static(rng, path, n, radius_range=(1.0, 3.0), radius_ranges=None,
                 margin=3.0, start_clear=8.0, max_try=200) -> List[Obstacle]:
    """开阔场"避让式"静态障碍：距路径 margin+r 以上、起点 start_clear 内不放。

    （Stage2 旧行为：障碍不挡路。保持默认参数即与历史完全一致。）
    """
    obstacles = []
    for _ in range(int(n)):
        for _try in range(max_try):
            r = _sample_radius(rng, radius_range, radius_ranges)
            x = float(rng.uniform(5, 75))
            y = float(rng.uniform(-25, 25))
            if _dist_to_path(path, x, y) < margin + r:
                continue
            if np.hypot(x - path[0, 0], y - path[0, 1]) < start_clear + r:
                continue
            obstacles.append(Obstacle(x=x, y=y, radius=r))
            break
    return obstacles


def spawn_path_local(rng, path, n, radius_range=(1.0, 3.0), radius_ranges=None,
                     lat_range=(0.0, 1.5), start_clear=8.0, end_clear=10.0,
                     min_center_gap=3.0, widths=None, existing=None,
                     max_try=300) -> List[Obstacle]:
    """路径局部系摆放：沿路径随机取点 + 法向偏移（符号随机）。

    lat_range 单位 m（偏移绝对值范围）：
        (0.0, 1.5)  → 障碍压/贴路径（阻塞，避障核心场景）
        (3.0, 8.0)  → 走廊内近路摆放（配合 widths 使用）
    widths 给定时偏移上限再受走廊净宽约束（离岸留 1m）。
    min_center_gap：与已有障碍中心的最小额外间距，避免并排障碍合成一堵墙。
    """
    path = np.asarray(path, dtype=np.float64)
    n = int(n)
    if n <= 0:
        return []
    nrm = _path_normals(path)
    k0, k1 = _arc_index_window(path, float(start_clear), float(end_clear))
    placed = list(existing) if existing else []
    out: List[Obstacle] = []
    for _ in range(n):
        for _try in range(max_try):
            k = int(rng.integers(k0, k1))
            r = _sample_radius(rng, radius_range, radius_ranges)
            hi = float(lat_range[1])
            if widths is not None:
                hi = min(hi, float(widths[k]) / 2.0 - 1.0)
            lo = min(float(lat_range[0]), hi)
            lat = float(rng.uniform(lo, max(lo, hi)))
            if rng.random() < 0.5:
                lat = -lat
            x = float(path[k, 0] + lat * nrm[k, 0])
            y = float(path[k, 1] + lat * nrm[k, 1])
            if any(np.hypot(x - o.x, y - o.y) < o.radius + r + min_center_gap
                   for o in placed):
                continue
            ob = Obstacle(x=x, y=y, radius=r)
            out.append(ob)
            placed.append(ob)
            break
    return out


def spawn_moving(rng, path, n, radius_range=(1.0, 2.0), speed_range=(0.2, 0.5),
                 path_local=False, widths=None, start_clear=10.0,
                 bx=(2.0, 78.0), by=(-25.0, 25.0)) -> List[Obstacle]:
    """动态障碍：匀速直线 + 边界反射。

    path_local=True 时初始位置贴路径（走廊场景），否则随机撒在开阔场内。
    """
    obstacles = []
    k0, k1, nrm = None, None, None
    if path_local:
        path = np.asarray(path, dtype=np.float64)
        nrm = _path_normals(path)
        k0, k1 = _arc_index_window(path, float(start_clear), 0.0)
    for _ in range(int(n)):
        r = float(rng.uniform(*radius_range))
        ang = float(rng.uniform(0, 2 * np.pi))
        spd = float(rng.uniform(*speed_range))
        if path_local:
            k = int(rng.integers(k0, k1))
            hi = 3.0 if widths is None else min(3.0, float(widths[k]) / 4.0)
            lat = float(rng.uniform(-hi, hi))
            x = float(path[k, 0] + lat * nrm[k, 0])
            y = float(path[k, 1] + lat * nrm[k, 1])
        else:
            x = float(rng.uniform(15, 70))
            y = float(rng.uniform(-20, 20))
        obstacles.append(Obstacle(x=x, y=y, radius=r,
                                  vx=spd * np.cos(ang), vy=spd * np.sin(ang),
                                  bx=tuple(bx), by=tuple(by)))
    return obstacles


def spawn_gates(rng, path, n, gap=4.0, gap_range=None, radius=1.5) -> List[Obstacle]:
    """窄通道：在路径中段沿法线对称放两个障碍，净宽 gap（可用 gap_range 逐门随机）。"""
    obstacles = []
    n0 = len(path)
    nrm = _path_normals(np.asarray(path, dtype=np.float64))
    for _ in range(int(n)):
        idx = int(rng.integers(n0 // 3, max(2 * n0 // 3, n0 // 3 + 1)))
        g = float(rng.uniform(gap_range[0], gap_range[1])) if gap_range else float(gap)
        px, py = path[idx]
        off = g / 2.0 + radius
        obstacles.append(Obstacle(x=float(px + nrm[idx, 0] * off),
                                  y=float(py + nrm[idx, 1] * off), radius=radius))
        obstacles.append(Obstacle(x=float(px - nrm[idx, 0] * off),
                                  y=float(py - nrm[idx, 1] * off), radius=radius))
    return obstacles


def surface_points(obstacles, n_per_circle=90) -> np.ndarray:
    """所有障碍的表面点（世界系, (M,2)）——扇区化的几何来源。"""
    if not obstacles:
        return np.zeros((0, 2))
    ang = np.linspace(0.0, 2.0 * np.pi, n_per_circle, endpoint=False)
    pts = []
    for ob in obstacles:
        pts.append(np.column_stack([ob.x + ob.radius * np.cos(ang),
                                    ob.y + ob.radius * np.sin(ang)]))
    return np.vstack(pts)


# ---------------------------------------------------------------------------
def gen_shore_corridor(rng, difficulty: str = "medium",
                       width_range: Sequence[float] = (8.0, 25.0),
                       path_bias: float = 0.35, spacing: float = 0.5,
                       shore_pt_stride: int = 2) -> ShoreCorridor:
    """生成岸线走廊场景。

    中心线 = 低频正弦组合（曲率有界：R_min ≳ 6m），而不是 gen_path 的随机样条——
    随机样条在急弯处曲率半径可小于半宽，岸线离位会自交、跟踪路径贴岸。

    构造保证（半宽 A(t) = min(w/2, 0.75·R)）：
    - 岸线离位 d ≤ 1.243·A < R → 不自交
    - 跟踪路径离任一侧岸 ≥ (1 - 0.243 - path_bias)·A ≥ ~0.4·A ≥ 1.6m（默认参数）
    - 实际净宽 = 名义净宽 ± ~1.4m（弯道处略收窄，湖道真实感）

    shore_pt_stride：岸线表面点抽样步长（stride=2 即每 1.0m 一个点）。
    """
    w_lo, w_hi = float(width_range[0]), float(width_range[1])
    a1_rng = {"easy": (5.0, 8.0), "medium": (6.0, 11.0),
              "hard": (8.0, 12.0)}.get(difficulty, (6.0, 11.0))
    f1_rng = {"easy": (0.4, 0.9), "medium": (0.5, 1.1),
              "hard": (0.6, 1.2)}.get(difficulty, (0.5, 1.1))

    # --- 中心线：低频正弦（沿 x 0→80） ---
    xs = np.linspace(0.0, 80.0, 400)
    a1 = float(rng.uniform(*a1_rng))
    f1 = float(rng.uniform(*f1_rng))
    p1 = float(rng.uniform(0.0, 2 * np.pi))
    a2 = float(rng.uniform(1.5, 2.0))
    f2 = float(rng.uniform(1.5, 2.2))
    p2 = float(rng.uniform(0.0, 2 * np.pi))
    ys = (a1 * np.sin(2 * np.pi * f1 * xs / 80.0 + p1)
          + a2 * np.sin(2 * np.pi * f2 * xs / 80.0 + p2))
    center = _resample(np.column_stack([xs, ys]), spacing)
    n = len(center)
    s01 = np.linspace(0.0, 1.0, n)

    # --- 曲率半径（平滑后） ---
    tg = np.gradient(center, axis=0)
    ang = np.unwrap(np.arctan2(tg[:, 1], tg[:, 0]))
    kappa = np.abs(np.gradient(ang)) / spacing
    ken = 9
    if n > 2 * ken:
        kappa = np.convolve(np.pad(kappa, ken // 2, mode="edge"),
                            np.ones(ken) / ken, mode="valid")
    R = 1.0 / np.maximum(kappa, 1e-6)

    # --- 净宽剖面：控制点插值 + 滑动平均 ---
    n_ctrl = int(rng.integers(4, 7))
    w = np.interp(s01, np.linspace(0.0, 1.0, n_ctrl),
                  rng.uniform(w_lo, w_hi, size=n_ctrl))
    k = 31
    if n > 2 * k:
        w = np.convolve(w, np.ones(k) / k, mode="same")
        w[:k] = w[k]
        w[-k:] = w[-k - 1]
    w = np.clip(w, w_lo, w_hi)

    nrm = _path_normals(center)
    A = np.minimum(w / 2.0, 0.75 * R)          # 可用半宽（曲率保护）

    # --- 两岸：高频低幅摆动（幅度 ≤ min(0.18A, 0.5)m，不破坏构造保证） ---
    def _wiggle(ph, freq):
        amp = np.minimum(0.18 * A, 0.5)
        return amp * (np.sin(2 * np.pi * freq * s01 + ph)
                      + 0.35 * np.sin(2 * np.pi * 1.7 * freq * s01 + 1.7 * ph))

    fw_l = float(rng.uniform(2.0, 4.0))
    fw_r = float(rng.uniform(2.0, 4.0))
    d_l = A + _wiggle(float(rng.uniform(0, 2 * np.pi)), fw_l)
    d_r = A + _wiggle(float(rng.uniform(0, 2 * np.pi)), fw_r)
    shore_l = center + d_l[:, None] * nrm
    shore_r = center - d_r[:, None] * nrm
    widths = np.hypot(*(shore_l - shore_r).T)

    # --- 跟踪路径：中心线 + 平滑偏置（|偏置| ≤ path_bias·A） ---
    bias = (float(path_bias) * A) * np.sin(
        2 * np.pi * float(rng.uniform(0.5, 1.5)) * s01
        + float(rng.uniform(0.0, 2 * np.pi)))
    path = center + bias[:, None] * nrm

    stride = max(int(shore_pt_stride), 1)
    shore_pts = np.vstack([shore_l[::stride], shore_r[::stride]])
    bbox = (float(center[:, 0].min()), float(center[:, 0].max()),
            float(shore_pts[:, 1].min()), float(shore_pts[:, 1].max()))
    return ShoreCorridor(path=path, center=center, widths=widths,
                         shore_pts=shore_pts, bbox=bbox)
