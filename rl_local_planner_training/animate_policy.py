# -*- coding: utf-8 -*-
"""策略 2D 动画：小艇沿航迹航行 GIF，输出到 viz_anim/ 文件夹。

用法：python animate_policy.py [actor.pt]
"""
import os
import sys

import numpy as np
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import torch  # noqa: E402

from env import USVLocalEnv  # noqa: E402
from train.train_sac import make_eval_cfg  # noqa: E402

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

MODEL = sys.argv[1] if len(sys.argv) > 1 else "models/stage2_actor.pt"
OUTDIR = os.path.join(_HERE, "viz_anim")
os.makedirs(OUTDIR, exist_ok=True)

SEEDS = [2001, 2003, 2005]          # 每个 seed 一个 GIF
STRIDE = 2                          # 每 2 个仿真步一帧（0.2s/帧）
FPS = 10


def run_episode(env, actor, seed):
    obs, info = env.reset(seed=seed)
    traj, cte, spd, dmin = [], [], [], []
    done, steps = False, 0
    while not done:
        with torch.no_grad():
            a = actor(torch.from_numpy(obs.astype(np.float32)).unsqueeze(0))
        obs, rw, term, trunc, i2 = env.step(a.squeeze(0).cpu().numpy())
        done = term or trunc
        steps += 1
        traj.append((env.dynamics.state.x, env.dynamics.state.y,
                     env.dynamics.state.psi))
        cte.append(float(i2["cte"]))
        spd.append(float(i2["speed"]))
        dmin.append(float(i2["d_min"]))
    return dict(traj=np.array(traj), cte=np.array(cte), spd=np.array(spd),
                dmin=np.array(dmin), steps=steps,
                end=i2.get("end_reason", "?"),
                obstacles=[(ob.x, ob.y, ob.radius) for ob in env.obstacles],
                path=env.path_features.pts,
                goal=env.path_features.pts[-1])


def make_gif(env, actor, seed):
    ep = run_episode(env, actor, seed)
    T = ep["traj"][::STRIDE]

    fig, ax = plt.subplots(figsize=(9, 6.5))
    ax.plot(ep["path"][:, 0], ep["path"][:, 1], "--", c="0.6", lw=1.2,
            label="规划路径", zorder=1)
    for ox, oy, r in ep["obstacles"]:
        ax.add_patch(Circle((ox, oy), r, fc="#d9534f", ec="#a94442",
                            alpha=0.75, zorder=2))
    ax.plot(*ep["path"][0], marker="^", c="lime", ms=11, mec="k", zorder=4,
            ls="")
    ax.plot(*ep["goal"], marker="*", c="red", ms=14, mec="k", zorder=4, ls="")
    trail, = ax.plot([], [], "-", c="tab:blue", lw=2.0, alpha=0.85, zorder=3)
    boat = Polygon([[0, 0], [-0.9, 0.45], [-0.9, -0.45]], closed=True,
                   fc="gold", ec="k", zorder=5)
    ax.add_patch(boat)
    info = ax.text(0.02, 0.97, "", transform=ax.transAxes, fontsize=10,
                   va="top", bbox=dict(fc="white", ec="0.7", alpha=0.85))
    end_txt = ax.text(0.5, 0.5, "", transform=ax.transAxes, fontsize=20,
                      ha="center", va="center", weight="bold", alpha=0.9)

    pad = 6.0
    ax.set_xlim(ep["traj"][:, 0].min() - pad, ep["traj"][:, 0].max() + pad)
    ax.set_ylim(ep["path"][:, 1].min() - pad, ep["path"][:, 1].max() + pad)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, ls=":")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Stage 2 模型航行动画  seed=%d  障碍 %d 个"
                 % (seed, len(ep["obstacles"])), fontsize=12)

    def update(i):
        k = min(i * STRIDE, ep["steps"] - 1)
        trail.set_data(ep["traj"][:k + 1, 0], ep["traj"][:k + 1, 1])
        x, y, psi = T[min(i, len(T) - 1)]
        c, s = np.cos(psi), np.sin(psi)
        pts = np.array([[1.1, 0], [-0.7, 0.55], [-0.5, 0], [-0.7, -0.55]])
        boat.set_xy(pts @ np.array([[c, s], [-s, c]]) + [x, y])
        info.set_text("t=%.1fs  v=%.2fm/s\nCTE=%+.2fm  障碍距离=%.1fm"
                      % (k * env.dt, ep["spd"][k], ep["cte"][k], ep["dmin"][k]))
        if k >= ep["steps"] - 1:
            end_txt.set_text("到达" if ep["end"] == "goal" else
                             ("碰撞" if ep["end"] == "collision" else ep["end"]))
        return trail, boat, info, end_txt

    anim = FuncAnimation(fig, update, frames=len(T) + 1, interval=1000 // FPS,
                         blit=True)
    out = os.path.join(OUTDIR, "ep%d.gif" % seed)
    anim.save(out, writer=PillowWriter(fps=FPS))
    plt.close(fig)
    print("已保存:", out, "(%.1f MB)" % (os.path.getsize(out) / 1e6),
          "end=%s min_d=%.2f" % (ep["end"], ep["dmin"].min()))


def main():
    with open("configs/stage2_static_obstacles.yaml") as f:
        cfg = yaml.safe_load(f)
    env = USVLocalEnv(make_eval_cfg(cfg))
    actor = torch.jit.load(MODEL, map_location="cpu").eval()
    for seed in SEEDS:
        make_gif(env, actor, seed)


if __name__ == "__main__":
    main()
