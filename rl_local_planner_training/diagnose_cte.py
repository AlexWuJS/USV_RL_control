# -*- coding: utf-8 -*-
"""诊断 CTE 分布：按弧长位置 / 曲率分桶统计，并单独看失败 episode。

用法：python diagnose_cte.py <actor.pt> [episodes]
"""
import os
import sys

import numpy as np
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import torch  # noqa: E402

from env import USVLocalEnv  # noqa: E402
from train.train_sac import make_eval_cfg  # noqa: E402


def main():
    actor = torch.jit.load(sys.argv[1], map_location="cpu").eval()
    n_eps = int(sys.argv[2]) if len(sys.argv) > 2 else 50
    with open("configs/stage1_path_follow.yaml") as f:
        cfg = yaml.safe_load(f)
    env = USVLocalEnv(make_eval_cfg(cfg))

    # 分桶：s 归一化位置 × |cte|
    s_bins = np.linspace(0, 1, 6)
    cte_by_s = [[] for _ in range(len(s_bins) - 1)]
    fails = []
    for ep in range(n_eps):
        obs, info = env.reset(seed=2000 + ep)
        done, steps = False, 0
        cte_seq, s_seq = [], []
        while not done:
            with torch.no_grad():
                a = actor(torch.from_numpy(obs.astype(np.float32)).unsqueeze(0))
            obs, rw, term, trunc, i2 = env.step(a.squeeze(0).cpu().numpy())
            done = term or trunc
            steps += 1
            s_frac = 1.0 - i2["s_remaining"] / max(info["s_total"], 1e-6)
            s_frac = min(max(s_frac, 0.0), 1.0)
            cte_seq.append(abs(float(i2["cte"])))
            s_seq.append(s_frac)
            b = min(int(s_frac * 5), 4)
            cte_by_s[b].append(abs(float(i2["cte"])))
        end = i2.get("end_reason", env.cfg and None)
        if end != "goal":
            fails.append((ep, end, steps, float(np.max(cte_seq)),
                          float(np.argmax(cte_seq) / max(len(cte_seq), 1))))
        rmse = float(np.sqrt(np.mean(np.square(cte_seq))))
        print("ep %2d %-10s steps=%3d rmse=%.3f max=%.2f peak@%.0f%%" %
              (ep, end, steps, rmse, max(cte_seq),
               100 * float(np.argmax(cte_seq)) / max(len(cte_seq), 1)))

    print("\n== CTE 按 s 位置分桶（均值 / p95）==")
    for i in range(5):
        arr = np.array(cte_by_s[i])
        if len(arr):
            print("s %3d%%-%3d%%: mean=%.3f p95=%.3f n=%d" %
                  (s_bins[i] * 100, s_bins[i + 1] * 100,
                   arr.mean(), np.percentile(arr, 95), len(arr)))
    print("\n失败 episode: %s" % fails)


if __name__ == "__main__":
    main()
