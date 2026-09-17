# -*- coding: utf-8 -*-
"""核心管线数值检查（NX 与训练机都可运行，仅需 numpy）：

路径生成 → 障碍表面点 → 体轴变换 → 扇区化 → 路径特征 →
动力学推演 → 观测组装 → 动作积分 → 安全监督，全链贯通。

用法：python3 pipeline_check.py
（不依赖 gymnasium，因此可在 NX 上验证与训练机相同的数值管线；
 gym.Env 包装层的冒烟测试见 env/smoke_test.py【在训练机执行】）
"""
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, "env"))   # 直接加载 path_gen/obstacles，绕过 env/__init__（其引用 gymnasium）

from dynamics import make_dynamics                       # noqa: E402
import path_gen                                          # noqa: E402
import obstacles                                         # noqa: E402
from rl_common import (                                  # noqa: E402
    BodyState, PathFeatures, LidarSectorizer, ObservationBuilder, ObsConfig,
    ActionProcessor, ActionLimits, SafetySupervisor, SafetyConfig, NavCommand,
    SRC_RL, OBS_DIM, angle_diff,
)


def main():
    rng = np.random.default_rng(7)

    # 1. 路径
    path = path_gen.gen_path(rng, "medium")
    seg = np.hypot(*(path[1:] - path[:-1]).T)
    assert len(path) > 100 and abs(seg.mean() - 0.5) < 0.05, "路径重采样异常"
    print("[ok] 路径生成: %d 点, 弧长 %.1f m, 平均间距 %.3f m"
          % (len(path), seg.sum(), seg.mean()))

    # 2. 障碍 + 表面点
    obstacles_list = obstacles.spawn_static(rng, path, 3)
    assert len(obstacles_list) >= 1, "静态障碍生成失败"
    pts_world = obstacles.surface_points(obstacles_list)
    for ob in obstacles_list:
        assert _dist_to_path(path, ob.x, ob.y) >= 3.0 + ob.radius - 0.5
    print("[ok] 障碍生成: %d 个, 表面点 %d 个" % (len(obstacles_list), len(pts_world)))

    # 3. 动力学推演：从路径起点沿切线加速
    pf_holder = PathFeatures(path, la1_dist=6.0, la2_dist=15.0)
    psi0 = pf_holder.tangent_at_start()
    dyn = make_dynamics("first_order", {"tau_u": 0.79, "tau_r": 0.53,
                                        "k_r": 0.8, "r_max": 0.6})
    dyn.reset(path[0, 0], path[0, 1], psi0)
    ap = ActionProcessor(ActionLimits())
    ap.reset(0.2, psi0)
    for _ in range(50):                       # 5s：加速到 ~1 m/s 沿路前进
        out = ap.process([0.8, 0.0])
        dyn.step(out.v_ref, out.course_ref, 0.1)
    st = dyn.state
    assert st.u > 0.7, "加速异常 u=%.2f" % st.u
    feat = pf_holder.update_full([st.x, st.y], st.psi)
    assert abs(feat.cte) < 1.5, "直线推进偏离路径 cte=%.2f" % feat.cte
    assert feat.s_proj > 2.0, "未沿路径前进 s_proj=%.2f" % feat.s_proj
    print("[ok] 动力学推演: u=%.2f m/s, 前进 %.1f m, cte=%.3f m"
          % (st.u, feat.s_proj, feat.cte))

    # 4. 扇区化：虚拟把船摆到最近障碍正前方 5m，正前扇区应显著小于量程
    ob0 = min(obstacles_list, key=lambda ob: np.hypot(ob.x - st.x, ob.y - st.y))
    d_bo = float(np.hypot(st.x - ob0.x, st.y - ob0.y))
    dirx, diry = (st.x - ob0.x) / d_bo, (st.y - ob0.y) / d_bo
    bx, by = ob0.x + 5.0 * dirx, ob0.y + 5.0 * diry      # 船在障碍下风向 5m
    psi_t = float(np.arctan2(ob0.y - by, ob0.x - bx))    # 艏向正对障碍
    dx = pts_world[:, 0] - bx
    dy = pts_world[:, 1] - by
    c, s = np.cos(psi_t), np.sin(psi_t)
    pts_body = np.column_stack([c * dx + s * dy, -s * dx + c * dy,
                                np.zeros(len(dx))])
    sec = LidarSectorizer()
    sectors_m = sec.process(pts_body)
    front = sectors_m[0]
    assert front < 10.0, "正前扇区未感知障碍 front=%.1f" % front
    assert front > 5.0 - ob0.radius - 1.0, "正前距离异常 front=%.1f" % front
    # 5m 外的扇区应保持满量程（障碍不在那个方向）
    assert (sectors_m[18] == 20.0), "正后方不应有回波"
    print("[ok] 扇区化: 正前 %.1f m (障碍半径 %.1f), 正后 %.1f m"
          % (front, ob0.radius, sectors_m[18]))

    # 5. 观测组装
    builder = ObservationBuilder(ObsConfig())
    obs = builder.build(st, feat, sectors_m, v_ref=ap.v_ref,
                        prev_action=np.zeros(2))
    assert obs.shape == (OBS_DIM,) and obs.dtype == np.float32
    assert np.isfinite(obs).all()
    print("[ok] 观测组装: %d 维 float32 全部有限" % OBS_DIM)

    # 6. 安全监督：贴近障碍时强制降速
    sv = SafetySupervisor(SafetyConfig())
    d_min = sec.min_clearance(sectors_m)
    cmd = NavCommand(desired_speed=1.5, desired_course=out.course_ref, source=SRC_RL)
    safe_cmd = sv.apply(cmd, d_min, feat.tangent_course)
    assert safe_cmd.desired_speed < 1.5, "近障未降速"
    print("[ok] 安全监督: d_min=%.1f m → 速度 %.1f → %.2f m/s%s"
          % (d_min, 1.5, safe_cmd.desired_speed,
             "（已覆盖）" if safe_cmd.supervisor_active else ""))

    print("\n核心管线在 NX 上贯通 ✓")


def _dist_to_path(path, x, y):
    return float(np.min(np.hypot(path[:, 0] - x, path[:, 1] - y)))


if __name__ == "__main__":
    main()
