# -*- coding: utf-8 -*-
"""策略可视化：2D 轨迹图（路径/障碍/按速度着色的航迹）+ 代表 episode 细节。

用法：python visualize_policy.py [actor.pt] [输出前缀]
"""
import os
import sys

import numpy as np
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import torch  # noqa: E402

from env import USVLocalEnv  # noqa: E402
from train.train_sac import make_eval_cfg  # noqa: E402

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

MODEL = sys.argv[1] if len(sys.argv) > 1 else "models/stage2_actor.pt"
PREFIX = sys.argv[2] if len(sys.argv) > 2 else "viz_stage2"


def run_episode(env, actor, seed):
    obs, info = env.reset(seed=seed)
    traj, cte, spd, dmin, uref, vref = [], [], [], [], [], []
    done, steps = False, 0
    while not done:
        with torch.no_grad():
            a = actor(torch.from_numpy(obs.astype(np.float32)).unsqueeze(0))
        obs, rw, term, trunc, i2 = env.step(a.squeeze(0).cpu().numpy())
        done = term or trunc
        steps += 1
        traj.append((env.dynamics.state.x, env.dynamics.state.y))
        cte.append(float(i2["cte"]))
        spd.append(float(i2["speed"]))
        dmin.append(float(i2["d_min"]))
        vref.append(float(i2["v_ref"]))
    return dict(traj=np.array(traj), cte=np.array(cte), spd=np.array(spd),
                dmin=np.array(dmin), vref=np.array(vref), steps=steps,
                end=i2.get("end_reason", "?"), s_total=info["s_total"],
                obstacles=[(ob.x, ob.y, ob.radius) for ob in env.obstacles],
                path=env.path_features.pts)


def draw_episode(ax, ep, title):
    ax.plot(ep["path"][:, 0], ep["path"][:, 1], "--", c="0.55", lw=1.2,
            zorder=1, label="规划路径")
    for ox, oy, r in ep["obstacles"]:
        ax.add_patch(Circle((ox, oy), r, fc="#d9534f", ec="#a94442",
                            alpha=0.75, zorder=2))
    sc = ax.scatter(ep["traj"][:, 0], ep["traj"][:, 1], c=ep["spd"],
                    cmap="viridis", s=4, zorder=3, vmin=0, vmax=2.0)
    ax.plot(*ep["traj"][0], marker="^", c="lime", ms=10, mec="k", zorder=4)
    ax.plot(*ep["traj"][-1], marker="*", c="red", ms=12, mec="k", zorder=4)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.grid(alpha=0.25, ls=":")
    ax.set_xlabel("x (m)", fontsize=8)
    ax.set_ylabel("y (m)", fontsize=8)
    ax.tick_params(labelsize=8)
    return sc


def main():
    with open("configs/stage2_static_obstacles.yaml") as f:
        cfg = yaml.safe_load(f)
    env = USVLocalEnv(make_eval_cfg(cfg))
    actor = torch.jit.load(MODEL, map_location="cpu").eval()

    # ---- 图 1：6 个 episode 总览（网格） ----
    seeds = [2001, 2003, 2005, 2007, 2009, 2011]
    eps = [run_episode(env, actor, s) for s in seeds]
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    for ax, ep, s in zip(axes.flat, eps, seeds):
        t = "ep%d %s | 障碍%d个 | %.1fs | min间隙%.1fm" % (
            s, {"goal": "✓到达", "collision": "✗碰撞", "timeout": "超时",
                "cte_abort": "偏航终止", "stuck": "停滞"}.get(ep["end"], ep["end"]),
            len(ep["obstacles"]), ep["steps"] * env.dt, ep["dmin"].min())
        sc = draw_episode(ax, ep, t)
    legend = [Line2D([], [], ls="--", c="0.55", label="规划路径"),
              Line2D([], [], marker="o", ls="", c="#d9534f", label="静态障碍"),
              Line2D([], [], marker="^", ls="", c="lime", mec="k", label="起点"),
              Line2D([], [], marker="*", ls="", c="red", mec="k", label="终点")]
    fig.legend(handles=legend, loc="lower center", ncol=4, fontsize=10)
    cb = fig.colorbar(sc, ax=axes, shrink=0.75, pad=0.01)
    cb.set_label("航速 (m/s)")
    fig.suptitle("Stage 2 最终模型 — 静态障碍场景轨迹（颜色=航速）", fontsize=14)
    f1 = "%s_trajectories.png" % PREFIX
    fig.savefig(f1, dpi=130, bbox_inches="tight")
    print("已保存:", f1)

    # ---- 图 2：代表 episode 细节 ----
    best = max((e for e in eps if e["end"] == "goal"),
               key=lambda e: len(e["obstacles"]))
    fig2, axs = plt.subplots(2, 2, figsize=(14, 9))
    t_ax = np.arange(best["steps"]) * env.dt

    sc = draw_episode(axs[0, 0], best,
                      "代表 episode（%d 个障碍，成功）" % len(best["obstacles"]))

    axs[0, 1].plot(t_ax, best["cte"], c="tab:blue", lw=1.2)
    axs[0, 1].axhline(0, c="0.6", lw=0.8)
    axs[0, 1].axhline(0.5, c="tab:red", ls="--", lw=0.8, label="±0.5m 带")
    axs[0, 1].axhline(-0.5, c="tab:red", ls="--", lw=0.8)
    axs[0, 1].fill_between(t_ax, -0.5, 0.5, color="tab:red", alpha=0.08)
    axs[0, 1].set_title("横向偏差 CTE（RMSE=%.3fm）"
                        % float(np.sqrt(np.mean(best["cte"] ** 2))))
    axs[0, 1].set_xlabel("时间 (s)")
    axs[0, 1].set_ylabel("CTE (m)")
    axs[0, 1].grid(alpha=0.3, ls=":")
    axs[0, 1].legend(fontsize=8)

    axs[1, 0].plot(t_ax, best["spd"], c="tab:green", lw=1.3, label="实际航速")
    axs[1, 0].plot(t_ax, best["vref"], c="tab:orange", lw=1.0, ls="--",
                   label="速度指令 v_ref")
    axs[1, 0].axhline(2.0, c="0.5", ls=":", lw=0.8)
    axs[1, 0].set_title("速度曲线")
    axs[1, 0].set_xlabel("时间 (s)")
    axs[1, 0].set_ylabel("速度 (m/s)")
    axs[1, 0].grid(alpha=0.3, ls=":")
    axs[1, 0].legend(fontsize=8)

    axs[1, 1].plot(t_ax, best["dmin"], c="tab:purple", lw=1.3)
    axs[1, 1].axhline(1.5, c="tab:red", ls="--", lw=0.8, label="1.5m 安全线")
    axs[1, 1].axhline(0.6, c="darkred", ls=":", lw=0.9, label="0.6m 船体半径")
    axs[1, 1].set_title("与最近障碍距离（最小 %.2fm）" % best["dmin"].min())
    axs[1, 1].set_xlabel("时间 (s)")
    axs[1, 1].set_ylabel("距离 (m)")
    axs[1, 1].grid(alpha=0.3, ls=":")
    axs[1, 1].legend(fontsize=8)

    fig2.suptitle("代表 episode 细节 — Stage 2 最终模型", fontsize=14)
    fig2.tight_layout(rect=[0, 0, 1, 0.96])
    f2 = "%s_detail.png" % PREFIX
    fig2.savefig(f2, dpi=130)
    print("已保存:", f2)

    for s, ep in zip(seeds, eps):
        print("seed %d: %-10s steps=%3d min_d=%.2f cte_rmse=%.3f" %
              (s, ep["end"], ep["steps"], ep["dmin"].min(),
               float(np.sqrt(np.mean(ep["cte"] ** 2)))))


if __name__ == "__main__":
    main()
