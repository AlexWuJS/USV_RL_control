#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SAC 训练入口。【在训练机执行】（严禁在小艇 Jetson NX 上运行本脚本）

用法：
    python3 train/train_sac.py --config configs/stage1_path_follow.yaml
    python3 train/train_sac.py --config configs/stage2_static_obstacles.yaml \
        --init-model models/stage1_best/best_model.zip

输出：
    models/stage{N}_best/best_model.zip     EvalCallback 最优模型
    models/stage{N}_final.zip               训练结束模型
    models/stage{N}_actor.pt                TorchScript Actor（拷贝到 NX 部署）
    tb/                                     TensorBoard 日志
"""
import argparse
import copy
import os
import sys

import numpy as np
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))   # 项目根（rl_common / env / dynamics）

from stable_baselines3 import SAC  # noqa: E402
from stable_baselines3.common.monitor import Monitor  # noqa: E402
from stable_baselines3.common.vec_env import SubprocVecEnv  # noqa: E402
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback  # noqa: E402

from env import USVLocalEnv  # noqa: E402
from rl_common import OBS_DIM  # noqa: E402


def make_env_fn(cfg: dict, rank: int):
    def _init():
        env = USVLocalEnv(cfg)
        env.reset(seed=1000 + rank)     # 每个 env 固定且不同的初始种子
        return Monitor(env)
    return _init


def make_eval_cfg(cfg: dict) -> dict:
    """评估环境：关随机化/噪声/延迟，障碍数取配置中值。"""
    ec = copy.deepcopy(cfg["env"])
    ec["domain_randomization"] = False
    ec["dr_ranges"] = {}
    ec["obs_noise_std"] = 0.0
    ec["sector_noise_std"] = 0.0
    ec["sector_dropout"] = 0.0
    ec["current_max"] = 0.0
    ec["control_delay_steps"] = 0
    for key in ("n_obstacles", "n_moving_obstacles", "narrow_gates"):
        v = ec.get(key, 0)
        ec[key] = int(np.mean(v)) if isinstance(v, (list, tuple)) else int(v)
    return ec


def export_torchscript(model: SAC, out_path: str):
    """导出确定性 Actor（均值动作）为 TorchScript，供 NX 仅推理加载。"""
    import torch

    class ActorWrapper(torch.nn.Module):
        def __init__(self, actor):
            super().__init__()
            self.actor = actor

        def forward(self, obs):
            return self.actor._predict(obs, deterministic=True)

    wrapper = ActorWrapper(model.actor).eval()
    dummy = torch.zeros(1, OBS_DIM, dtype=torch.float32)
    traced = torch.jit.trace(wrapper, dummy)
    traced.save(out_path)
    print("导出 TorchScript Actor: %s" % out_path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--total-timesteps", type=int, default=None)
    ap.add_argument("--init-model", default=None, help="上一阶段 best_model.zip")
    ap.add_argument("--out-dir", default="models")
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    stage = int(cfg["stage"])
    total = args.total_timesteps or int(cfg["total_timesteps"])
    n_envs = int(cfg.get("n_envs", 8))
    sc = cfg["sac"]
    out = os.path.join(_HERE, "..", args.out_dir)
    os.makedirs(out, exist_ok=True)

    vec = SubprocVecEnv([make_env_fn(cfg["env"], i) for i in range(n_envs)])
    eval_env = Monitor(USVLocalEnv(make_eval_cfg(cfg)))

    model = SAC(
        "MlpPolicy", vec,
        policy_kwargs=dict(net_arch=list(sc.get("net_arch", [256, 256]))),
        learning_rate=float(sc["learning_rate"]),
        buffer_size=int(sc["buffer_size"]),
        batch_size=int(sc["batch_size"]),
        gamma=float(sc["gamma"]),
        tau=float(sc["tau"]),
        train_freq=int(sc["train_freq"]),
        gradient_steps=int(sc["gradient_steps"]),
        ent_coef=sc.get("ent_coef", "auto"),
        tensorboard_log="./tb",
        seed=int(cfg.get("seed", 0)),
        verbose=1,
    )
    if args.init_model:
        model = SAC.load(args.init_model, env=vec)
        print("从 %s 续训" % args.init_model)

    callbacks = [
        EvalCallback(eval_env,
                     best_model_save_path=os.path.join(out, "stage%d_best" % stage),
                     eval_freq=int(sc["eval_freq"]),
                     n_eval_episodes=int(sc["n_eval_episodes"]),
                     deterministic=True),
        CheckpointCallback(save_freq=int(sc["checkpoint_freq"]),
                           save_path=os.path.join(out, "stage%d_ckpt" % stage)),
    ]

    model.learn(total_timesteps=total, callback=callbacks,
                tb_log_name="sac_stage%d" % stage)
    model.save(os.path.join(out, "stage%d_final" % stage))
    export_torchscript(model, os.path.join(out, "stage%d_actor.pt" % stage))
    vec.close()


if __name__ == "__main__":
    main()
