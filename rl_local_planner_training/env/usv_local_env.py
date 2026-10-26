# -*- coding: utf-8 -*-
"""USV 局部规划 Gymnasium 环境（训练机专用，仅推理/训练，严禁在 NX 运行）。

观测/动作/安全契约全部复用 rl_common（与实艇 ROS 节点同一实现）：
    PathFeatures + LidarSectorizer + ObservationBuilder + ActionProcessor
    + SafetySupervisor（仅 use_speed_envelope=true 时参与训练，默认关闭——
      supervisor 是部署端兜底，训练期让策略在完整动作域内学习）。

奖励与终止条件：RL_INTERFACE_DESIGN.md §8。
"""
import math
from collections import deque

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from rl_common import (
    OBS_DIM, V_REF_MIN, V_REF_MAX, DV_MAX, DCOURSE_MAX,
    BodyState, PathFeatures, LidarSectorizer,
    ObservationBuilder, ObsConfig, ActionLimits, ActionProcessor,
    SafetySupervisor, SafetyConfig, angle_diff,
)
from dynamics import make_dynamics
from .path_gen import gen_path
from .obstacles import (Obstacle, spawn_static, spawn_moving, spawn_gates,
                        spawn_path_local, gen_shore_corridor, surface_points)


class USVLocalEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, config: dict):
        super().__init__()
        self.cfg = dict(config)
        ec = self.cfg

        self.dt = float(ec.get("dt", 0.1))
        self.max_steps = int(ec.get("max_episode_steps", 600))
        self.hull_radius = float(ec.get("hull_radius", 0.6))
        self.stuck_window = int(ec.get("stuck_window_steps", 100))
        self.stuck_min_progress = float(ec.get("stuck_min_progress", 0.5))
        self.cte_abort = float(ec.get("cte_abort", 8.0))
        self.goal_radius = float(ec.get("goal_radius", 1.5))

        # --- 契约组件（与实艇同一实现） ---
        self.dynamics = make_dynamics(ec.get("dynamics", "first_order"),
                                      dict(ec.get("dynamics_params", {})))
        self.obs_builder = ObservationBuilder(ObsConfig(
            la1_dist=float(ec.get("la1_dist", 6.0)),
            la2_dist=float(ec.get("la2_dist", 15.0)),
            v_min=V_REF_MIN, v_max=V_REF_MAX))
        self.sectorizer = LidarSectorizer(
            n_sectors=int(ec.get("n_sectors", 36)),
            max_range=float(ec.get("lidar_max_range", 20.0)))
        self.action_processor = ActionProcessor(ActionLimits(
            dv_max=DV_MAX, dcourse_max=DCOURSE_MAX,
            v_min=V_REF_MIN, v_max=V_REF_MAX))
        self.supervisor = SafetySupervisor(SafetyConfig(
            v_min=V_REF_MIN, v_max=V_REF_MAX))
        self.use_envelope = bool(ec.get("use_speed_envelope", False))

        # --- 奖励权重 ---
        rw = dict(ec.get("reward", {}))
        self.rw = rw
        self.w_progress = float(rw.get("w_progress", 2.0))
        self.w_track = float(rw.get("w_track", 0.10))
        self.w_track_quad = float(rw.get("w_track_quad", 0.0))
        self.w_track_db = float(rw.get("w_track_deadband", 0.0))
        self.w_track_db_min = float(rw.get("w_track_deadband_min", 0.0))
        self.w_heading = float(rw.get("w_heading", 0.05))
        self.w_smooth = float(rw.get("w_smooth", 0.30))
        self.w_safe_speed = float(rw.get("w_safe_speed", 0.5))
        self.a_lat_max = float(rw.get("a_lat_max", 0.5))
        ob = dict(rw.get("obstacle", {}))
        self.ob_far = float(ob.get("far", 8.0))
        self.ob_crawl = float(ob.get("crawl", 3.0))
        self.ob_critical = float(ob.get("critical", 1.5))
        self.ob_gradient = float(ob.get("gradient", 0.2))
        self.ob_near = float(ob.get("near", 0.5))
        self.ob_hit = float(ob.get("hit", 2.0))
        tm = dict(rw.get("terminal", {}))
        self.t_goal = float(tm.get("goal", 20.0))
        self.t_collision = float(tm.get("collision", -50.0))
        self.t_stuck = float(tm.get("stuck", -20.0))
        self.t_timeout = float(tm.get("timeout", -20.0))
        self.t_cte = float(tm.get("cte_abort", -10.0))

        # --- 空间 ---
        self.observation_space = spaces.Box(low=-5.0, high=5.0,
                                            shape=(OBS_DIM,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,),
                                       dtype=np.float32)

        # --- episode 状态 ---
        self.path_features = None
        self.obstacles = []
        self._prev_action = np.zeros(2)
        self._delay_buf = deque(maxlen=1 + int(np.max(np.atleast_1d(
            ec.get("control_delay_steps", 0)))))
        self._d_min = self.sectorizer.max_range
        self._steps = 0
        self._last_s = 0.0
        self._window_start_s = 0.0
        self._traveled = 0.0
        self._scene = "open_field"
        self._shore_pts = None
        self._road_bbox = None

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        ec = self.cfg

        # --- 场景：开阔场 / 岸线走廊（mixed 按概率二选一） ---
        scene = str(ec.get("scene", "open_field"))
        if scene == "mixed":
            scene = ("corridor" if rng.random() < float(ec.get("corridor_mix_p", 0.5))
                     else "open_field")
        if scene not in ("open_field", "corridor"):
            raise ValueError("未知 scene: %r（可选 open_field/corridor/mixed）" % scene)
        self._scene = scene

        if scene == "corridor":
            corr = gen_shore_corridor(
                rng, ec.get("difficulty", "medium"),
                tuple(ec.get("corridor_width_range", [8.0, 25.0])),
                float(ec.get("corridor_path_bias", 0.35)))
            path_pts, widths = corr.path, corr.widths
            self._shore_pts, self._road_bbox = corr.shore_pts, corr.bbox
        else:
            path_pts, widths = gen_path(rng, ec.get("difficulty", "medium")), None
            self._shore_pts, self._road_bbox = None, None

        self.path_features = PathFeatures(
            path_pts,
            la1_dist=self.obs_builder.cfg.la1_dist,
            la2_dist=self.obs_builder.cfg.la2_dist)

        # --- 障碍 ---
        # obstacle_on_path_frac：压路径障碍占比（0=旧"避让式"；1=全阻塞，避障核心场景）
        r_range = ec.get("obstacle_radius_range", [1.0, 3.0])
        r_ranges = ec.get("obstacle_radius_ranges", None)
        n_static = self._sample_count(ec.get("n_obstacles", 0))
        n_moving = self._sample_count(ec.get("n_moving_obstacles", 0))
        n_gates = self._sample_count(ec.get("narrow_gates", 0))
        frac_on = float(ec.get("obstacle_on_path_frac", 0.0))
        n_on = int(round(frac_on * n_static))
        n_off = n_static - n_on
        self.obstacles = []
        pts = self.path_features.pts
        if n_on:
            self.obstacles += spawn_path_local(
                rng, pts, n_on, radius_range=r_range, radius_ranges=r_ranges,
                lat_range=(0.0, 1.5), widths=widths, existing=self.obstacles)
        if n_off:
            if scene == "corridor":
                self.obstacles += spawn_path_local(
                    rng, pts, n_off, radius_range=r_range, radius_ranges=r_ranges,
                    lat_range=(3.0, 8.0), widths=widths, existing=self.obstacles)
            else:
                self.obstacles += spawn_static(
                    rng, pts, n_off, radius_range=r_range, radius_ranges=r_ranges)
        if n_moving:
            if scene == "corridor":
                xmin, xmax, ymin, ymax = self._road_bbox
                self.obstacles += spawn_moving(
                    rng, pts, n_moving, path_local=True, widths=widths,
                    bx=(xmin, xmax), by=(ymin, ymax))
            else:
                self.obstacles += spawn_moving(rng, pts, n_moving)
        if n_gates:
            self.obstacles += spawn_gates(rng, pts, n_gates,
                                          gap_range=ec.get("gate_gap_range", None))

        # 域随机化（每 episode 一次）
        self.dynamics.randomize(rng, ec.get("dr_ranges", None) if
                                ec.get("domain_randomization", False) else None)
        dr = ec.get("dr_ranges", {}) or {}
        cur = float(ec.get("current_max", 0.0))
        self._current = (rng.uniform(-cur, cur), rng.uniform(-cur, cur)) if cur > 0 else (0.0, 0.0)
        delay = ec.get("control_delay_steps", 0)
        delay = int(rng.integers(delay[0], delay[1] + 1)) if isinstance(delay, list) else int(delay)
        self._delay_buf = deque(maxlen=1 + delay)
        self._obs_noise = float(ec.get("obs_noise_std", 0.0))
        self._sector_noise = float(ec.get("sector_noise_std", 0.0))
        self._sector_dropout = float(ec.get("sector_dropout", 0.0))

        # 船体：路径起点，静止，艏向 = 起点切线
        psi0 = self.path_features.tangent_at_start()
        self.dynamics.reset(self.path_features.pts[0, 0], self.path_features.pts[0, 1], psi0)
        self.action_processor.reset(V_REF_MIN, psi0)   # 软起步

        self._prev_action = np.zeros(2)
        self._d_min = self.sectorizer.max_range
        self._steps = 0
        self._last_s = 0.0
        self._window_start_s = 0.0
        self._traveled = 0.0

        obs, _, _, _, _ = self._observe()
        return obs, {"s_total": self.path_features.s_total, "scene": self._scene}

    def step(self, action):
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        self._delay_buf.append(action)
        act = (self._delay_buf.popleft() if len(self._delay_buf) >=
               self._delay_buf.maxlen else np.zeros(2))

        out = self.action_processor.process(act)
        v_eff = out.v_ref
        if self.use_envelope:
            v_eff = min(v_eff, self.supervisor.speed_envelope(self._d_min))

        prev_pos = (self.dynamics.state.x, self.dynamics.state.y)
        st = self.dynamics.step(v_eff, out.course_ref, self.dt)
        # 恒定漂流（模拟风/流）
        if self._current != (0.0, 0.0):
            st.x += self._current[0] * self.dt
            st.y += self._current[1] * self.dt

        for ob in self.obstacles:
            ob.step(self.dt)

        obs, feat, sectors_m, d_min, info = self._observe(noisy=True)
        self._d_min = d_min

        # ---- 奖励 ----
        ds = feat.s_proj - self._last_s
        e_heading = angle_diff(feat.tangent_course, st.course)
        reward = self.w_progress * ds
        cte_abs = abs(float(feat.cte))
        reward -= (self.w_track * cte_abs
                   + self.w_track_quad * feat.cte ** 2
                   + self.w_track_db * max(0.0, cte_abs - self.w_track_db_min))
        reward -= self.w_heading * abs(float(e_heading))
        reward -= self.w_smooth * float(np.sum((act - self._prev_action) ** 2))
        reward -= self._obstacle_reward(d_min)
        reward -= self._safe_speed_reward(st, feat)
        self._prev_action = act.copy()

        # ---- 前进量统计 ----
        self._traveled += float(np.hypot(st.x - prev_pos[0], st.y - prev_pos[1]))
        self._steps += 1
        self._last_s = feat.s_proj

        # ---- 终止 ----
        terminated = False
        truncated = False
        if d_min <= self.hull_radius:
            reward += self.t_collision
            info["end_reason"] = "collision"
            terminated = True
        elif feat.s_remaining <= self.goal_radius:
            reward += self.t_goal
            info["end_reason"] = "goal"
            terminated = True
        elif abs(feat.cte) > self.cte_abort:
            reward += self.t_cte
            info["end_reason"] = "cte_abort"
            terminated = True
        elif self._steps % self.stuck_window == 0:
            if feat.s_proj - self._window_start_s < self.stuck_min_progress:
                reward += self.t_stuck
                info["end_reason"] = "stuck"
                terminated = True
            self._window_start_s = feat.s_proj
        elif self._steps >= self.max_steps:
            reward += self.t_timeout
            info["end_reason"] = "timeout"
            truncated = True

        if "end_reason" not in info:
            info["end_reason"] = "running"
        info.update(episode_steps=self._steps, traveled=self._traveled,
                    s_total=self.path_features.s_total, scene=self._scene)
        return obs, float(reward), terminated, truncated, info

    # ------------------------------------------------------------------
    def _observe(self, noisy=False):
        st = self.dynamics.state
        feat = self.path_features.update_full([st.x, st.y], st.psi)
        pts_world = surface_points(self.obstacles)
        if self._shore_pts is not None and len(self._shore_pts):
            pts_world = (np.vstack([pts_world, self._shore_pts])
                         if len(pts_world) else self._shore_pts)
        if len(pts_world):
            dx = pts_world[:, 0] - st.x
            dy = pts_world[:, 1] - st.y
            c, s = np.cos(st.psi), np.sin(st.psi)
            pts_body = np.column_stack([c * dx + s * dy, -s * dx + c * dy,
                                        np.zeros(len(dx))])
        else:
            pts_body = np.zeros((0, 3))
        sectors_m = self.sectorizer.process(pts_body)
        d_min = self.sectorizer.min_clearance(sectors_m)   # 干净值：终止/包络用

        sectors_obs = sectors_m
        if noisy and self._sector_noise > 0:
            sectors_obs = sectors_m + self.np_random.normal(
                0, self._sector_noise, size=sectors_m.shape)
            sectors_obs = np.clip(sectors_obs, 0.0, self.sectorizer.max_range)
        if noisy and self._sector_dropout > 0:
            drop = self.np_random.random(sectors_m.shape) < self._sector_dropout
            sectors_obs = np.where(drop, self.sectorizer.max_range, sectors_obs)

        obs = self.obs_builder.build(st, feat, sectors_obs,
                                     v_ref=self.action_processor.v_ref,
                                     prev_action=self._prev_action)
        if noisy and self._obs_noise > 0:
            obs = obs + self.np_random.normal(0, self._obs_noise, size=obs.shape).astype(np.float32)

        info = dict(u=st.u, v=st.v, r=st.r, cte=feat.cte,
                    e_heading=float(angle_diff(feat.tangent_course, st.course)),
                    d_min=d_min, s_remaining=feat.s_remaining,
                    speed=st.speed, v_ref=self.action_processor.v_ref)
        return obs, feat, sectors_m, d_min, info

    # ------------------------------------------------------------------
    def _obstacle_reward(self, d_min):
        if d_min >= self.ob_far:
            return 0.0
        if d_min > self.ob_crawl:
            f = (self.ob_far - d_min) / (self.ob_far - self.ob_crawl)
            return -self.ob_gradient * f
        if d_min > self.ob_critical:
            f = (self.ob_crawl - d_min) / (self.ob_crawl - self.ob_critical)
            return -self.ob_near - self.ob_near * f
        return -self.ob_hit

    def _safe_speed_reward(self, st, feat):
        # 曲率：最近点 vs 前视 la1 的切线变化率
        kappa = abs(float(angle_diff(
            self.path_features.tangent_at(feat.s_proj + self.obs_builder.cfg.la1_dist),
            feat.tangent_course))) / max(self.obs_builder.cfg.la1_dist, 1e-6)
        v_curve = math.sqrt(self.a_lat_max / max(kappa, 1e-3))
        v_safe = min(V_REF_MAX, 0.4 + 0.2 * max(self._d_min, 0.0), v_curve)
        excess = max(0.0, st.u - v_safe)
        return self.w_safe_speed * excess ** 2

    def _sample_count(self, spec):
        if isinstance(spec, (list, tuple)):
            return int(self.np_random.integers(spec[0], spec[1] + 1))
        return int(spec)
