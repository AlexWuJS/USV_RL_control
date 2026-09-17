# -*- coding: utf-8 -*-
"""角度工具：全库统一的角度 wrap 规则。

wrap_to_pi 输出 [-pi, pi)，wrap_to_2pi 输出 [0, 2pi)。
所有角度运算必须经过这里，禁止在调用方手写 fmod。
"""
import numpy as np

_TWO_PI = 2.0 * np.pi


def wrap_to_pi(a):
    """wrap 到 [-pi, pi)。标量或 ndarray。"""
    return (np.asarray(a) + np.pi) % _TWO_PI - np.pi


def wrap_to_2pi(a):
    """wrap 到 [0, 2pi)。"""
    return np.asarray(a) % _TWO_PI


def angle_diff(a, b):
    """有符号角差 a - b，wrap 到 [-pi, pi)。"""
    return wrap_to_pi(np.asarray(a) - np.asarray(b))


def wrap_scalar_to_pi(a: float) -> float:
    """标量版 wrap_to_pi（返回 python float，供 dataclass 使用）。"""
    return float(wrap_to_pi(a))
