#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""批量评估 + 指标输出。【在训练机执行】

用法：
    python3 evaluation/evaluate.py --config configs/stage2_static_obstacles.yaml \
        --model models/stage2_actor.pt --episodes 50 --plot

指标（对齐任务说明 §十六 与 RL_INTERFACE_DESIGN.md §12）：
    成功率 / 碰撞率 / 最小间隙 / CTE RMSE / 航向 RMSE /
    航向变化 RMS / 加速度 RMS / 平均·最大速度 / 行程时间 / 路径效率
"""
import argparse
import json
import os
import sys

import numpy as np
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

import torch  # noqa: E402

from env import USVLocalEnv  # noqa: E402
from rl_common import DCOURSE_MAX, DV_MAX  # noqa: E402
from train.train_sac import make_eval_cfg  # noqa: E402


def evaluate(actor, env, n_episodes: int):
    per_ep = []
    all_cte, all_ehead = [], []
    all_dcourse, all_accel, all_speed = [], [], []

    for ep in range(n_episodes):
        obs, info = env.reset(seed=2000 + ep)
        done = False
        rec = dict(steps=0, traveled=0.0, min_clearance=env.sectorizer.max_range,
                   end_reason="running", s_total=info["s_total"])
        prev_u, prev_t = None, None
        traj = []
        while not done:
            with torch.no_grad():
                a = actor(torch.from_numpy(obs.astype(np.float32)).unsqueeze(0))
            action = a.squeeze(0).cpu().numpy()
            obs, rw, term, trunc, info = env.step(action)
            done = term or trunc
            rec["steps"] += 1
            rec["traveled"] = info["traveled"]
            rec["min_clearance"] = min(rec["min_clearance"], info["d_min"])
            rec["end_reason"] = info["end_reason"]
            all_cte.append(info["cte"])
            all_ehead.append(info["e_heading"])
            all_speed.append(info["speed"])
            all_dcourse.append(abs(float(action[1])) * DCOURSE_MAX)
            if prev_u is not None:
                all_accel.append(abs(info["u"] - prev_u) / env.dt)
            prev_u = info["u"]
            traj.append((env.dynamics.state.x, env.dynamics.state.y))
        rec["time"] = rec["steps"] * env.dt
        rec["path_efficiency"] = rec["traveled"] / max(rec["s_total"], 1e-6)
        rec["traj"] = np.array(traj)
        per_ep.append(rec)
        print("ep %3d: %-10s steps=%3d min_clear=%.2f eff=%.2f" % (
            ep, rec["end_reason"], rec["steps"], rec["min_clearance"],
            rec["path_efficiency"]))

    n_success = sum(r["end_reason"] == "goal" for r in per_ep)
    n_coll = sum(r["end_reason"] == "collision" for r in per_ep)
    metrics = dict(
        episodes=n_episodes,
        success_rate=n_success / n_episodes,
        collision_rate=n_coll / n_episodes,
        min_clearance_mean=float(np.mean([r["min_clearance"] for r in per_ep])),
        min_clearance_min=float(np.min([r["min_clearance"] for r in per_ep])),
        cte_rmse=float(np.sqrt(np.mean(np.square(all_cte)))),
        heading_rmse_deg=float(np.degrees(np.sqrt(np.mean(np.square(all_ehead))))),
        course_change_rms_deg=float(np.degrees(np.sqrt(np.mean(np.square(all_dcourse))))),
        accel_rms=float(np.sqrt(np.mean(np.square(all_accel)))) if all_accel else 0.0,
        speed_mean=float(np.mean(all_speed)),
        speed_max=float(np.max(all_speed)),
        time_mean=float(np.mean([r["time"] for r in per_ep])),
        path_efficiency_mean=float(np.mean([r["path_efficiency"] for r in per_ep])),
    )
    return metrics, per_ep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--model", required=True, help="TorchScript actor (.pt)")
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--output", default="eval_results.json")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    env = USVLocalEnv(make_eval_cfg(cfg))
    actor = torch.jit.load(args.model, map_location="cpu").eval()

    metrics, per_ep = evaluate(actor, env, args.episodes)
    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    with open(args.output, "w") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    print("结果已保存: %s" % args.output)

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 6))
        for r in per_ep:
            color = {"goal": "tab:green", "collision": "tab:red"}.get(
                r["end_reason"], "tab:gray")
            if len(r["traj"]):
                ax.plot(r["traj"][:, 0], r["traj"][:, 1], c=color, alpha=0.5, lw=0.8)
        ax.set_aspect("equal")
        ax.set_title("trajectories (green=goal red=collision gray=other)")
        fig.savefig("eval_trajectories.png", dpi=150)
        print("轨迹图: eval_trajectories.png")


if __name__ == "__main__":
    main()
