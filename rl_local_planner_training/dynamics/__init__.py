# -*- coding: utf-8 -*-
from .base import BaseDynamics
from .first_order import FirstOrderDynamics
from .fossen_3dof import Fossen3DOF


def make_dynamics(name: str, params: dict) -> BaseDynamics:
    if name == "first_order":
        return FirstOrderDynamics(params)
    if name == "fossen":
        return Fossen3DOF(params)
    raise ValueError("未知动力学: %s (可选 first_order / fossen)" % name)


__all__ = ["BaseDynamics", "FirstOrderDynamics", "Fossen3DOF", "make_dynamics"]
