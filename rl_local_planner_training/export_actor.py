# -*- coding: utf-8 -*-
"""从已训练的 SAC zip 补导出 TorchScript Actor（train_sac 末尾导出失败时的补救）。

用法：python export_actor.py models/stage1_final.zip models/stage1_actor.pt
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from stable_baselines3 import SAC  # noqa: E402

from train.train_sac import export_torchscript  # noqa: E402


def main():
    src, dst = sys.argv[1], sys.argv[2]
    model = SAC.load(src, device="cpu")
    export_torchscript(model, dst)


if __name__ == "__main__":
    main()
