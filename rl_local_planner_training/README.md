# rl_local_planner_training — SAC 训练工程

> **运行环境纪律（RL_INTERFACE_DESIGN.md §14）**
> 本工程只在**训练机**（用户本机 / 实验室 GPU 工作站 / GPU 服务器）上运行。
> 小艇 Jetson NX 上**严禁**执行 `train/train_sac.py`、任何 `model.learn(...)`
> 或大规模 episode evaluation。NX 只做接口开发、Shadow Mode 和 Actor 推理。

## 目录结构

```
rl_local_planner_training/
├── rl_common/            # ★ 共用契约层（由 sync_rl_common.sh 从 NX ROS 包同步而来）
│   ├── contracts.py      #   观测/动作/指令的全部维度、单位、归一化定义
│   ├── observation_builder.py
│   ├── path_features.py
│   ├── lidar_sectorizer.py
│   ├── action_processor.py
│   ├── safety_supervisor.py
│   └── angle_utils.py
├── env/
│   ├── usv_local_env.py  # Gymnasium 环境（观测/动作/终止全部走 rl_common）
│   ├── path_gen.py       # 随机路径（0.5m 间距，与实艇 A* 栅格一致）
│   └── obstacles.py      # 静态/动态/窄通道障碍 → 表面点云 → 同一套扇区化
├── dynamics/
│   ├── first_order.py    # 默认：一阶惯性（tau_u=0.79 ← huamo.cpp, tau_r=0.53 ← sys_ident）
│   └── fossen_3dof.py    # 对照实验用（阻尼系数来自 pid_controller.cpp:170，质量待标定）
├── train/train_sac.py    # SAC 训练 + EvalCallback + TorchScript 导出
├── evaluation/evaluate.py# 批量评估（成功率/碰撞率/CTE RMSE/平滑度指标）
├── configs/              # stage1 路径跟踪 / stage2 静态障碍 / stage3 复杂场景
├── models/               # checkpoint、best、actor.pt 产物
└── deployment/README.md  # actor.pt 拷回 NX 的部署说明
```

## 训练/部署共用契约（铁律）

以下模块**只有一份实现**（rl_common/），训练环境与实艇 ROS 节点共用，
禁止在任何一端另写一套：

| 契约 | 内容 |
|------|------|
| Observation | 49 维，顺序/单位/坐标系/归一化逐项一致 |
| Action | [-1,1]² → Δv_ref / Δcourse_ref 限幅积分 |
| Safety | 间隙-速度包络、参考界、急停规则 |
| Sector | 36 扇区、20m 量程、扇区 0 对准艏向 |

## 部署前自检清单（每次改 rl_common 后）

```bash
# 【在小艇 NX 执行】契约层单测
cd message_ws/src/guidance/rl_local_planner && python3 tests/test_rl_common.py
# 【在训练机执行】同步后同样跑一遍，两端必须同时全绿
cd rl_local_planner_training && python3 rl_common_tests/test_rl_common.py
```

## 从 NX 建立训练工程

### 方式 A：训练机是 Windows PC（推荐做法）

本工程的代码是纯 Python，Windows 完全兼容。步骤（Jetson 默认 IP
192.168.50.186、用户名 jetson，按实际替换）：

```powershell
# 1.【在 Windows 训练机执行】把工程从 NX 拉到训练目录
#    （rl_common 已在 NX 上同步并校验一致，无需再跑 sync 脚本）
scp -r jetson@192.168.50.186:/home/jetson/haique/little-boat-motion-control/rl_local_planner_training "E:\无人艇\无人艇\小艇项目\RL部署\"

# 2.【在 Windows 训练机执行】一键安装 + 双测试（见 windows_setup.ps1）
cd E:\无人艇\无人艇\小艇项目\RL部署\rl_local_planner_training
powershell -ExecutionPolicy Bypass -File windows_setup.ps1
```

训练命令（【在 Windows 训练机执行】）：

```powershell
python train\train_sac.py --config configs\stage1_path_follow.yaml
python train\train_sac.py --config configs\stage2_static_obstacles.yaml --init-model models\stage1_best\best_model.zip
python evaluation\evaluate.py --config configs\stage2_static_obstacles.yaml --model models\stage2_actor.pt --episodes 50 --plot
```

### 方式 B：训练机是 Linux 服务器 / 实验室工作站

```bash
# 1.【在小艇 NX 执行】同步契约层到本工程并打包
bash sync_rl_common.sh
cd /home/jetson/haique/little-boat-motion-control
tar czf rl_local_planner_training.tar.gz rl_local_planner_training
scp rl_local_planner_training.tar.gz user@<训练机IP>:~/

# 2.【在训练机执行】解压 + 建环境
tar xzf rl_local_planner_training.tar.gz
cd rl_local_planner_training
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# 3.【在训练机执行】契约层单测（与 NX 结果必须一致）
python3 rl_common_tests/test_rl_common.py

# 4.【在训练机执行】环境冒烟测试（1 条随机动作 episode，非训练）+ 避障场景测试
python3 env/smoke_test.py
python3 env/scene_test.py
```

## 训练（全部【在训练机执行】）

```bash
# Stage 1：无障碍纯路径跟踪（验证：能跟踪、起步平滑、CTE RMSE 小）
python3 train/train_sac.py --config configs/stage1_path_follow.yaml
tensorboard --logdir tb   # 观察 ep_rew_mean / ep_len_mean / ent_coef

# Stage 2：静态障碍（从 stage1 best 续训）
python3 train/train_sac.py --config configs/stage2_static_obstacles.yaml \
    --init-model models/stage1_best/best_model.zip

# Stage 3a：阻塞障碍绕行（从 stage2 best 续训；避障核心能力）
python3 train/train_sac.py --config configs/stage3a_blocking.yaml \
    --init-model models/stage2_best/best_model.zip --out-dir models/stage3a

# Stage 3b：复杂场景（岸线走廊 + 窄道 + 动态障碍，从 3a best 续训）
python3 train/train_sac.py --config configs/stage3_complex.yaml \
    --init-model models/stage3a/stage3_best/best_model.zip --out-dir models/stage3b

# 评估（每个阶段训练完必做；指标不达标不进入下一阶段）
python3 evaluation/evaluate.py --config configs/stage2_static_obstacles.yaml \
    --model models/stage2_actor.pt --episodes 50 --plot
```

## 场景与避障课程（Stage 3a / 3b）

Stage 2 的障碍是"避让式"摆放（障碍面距路径 ≥3m，从不挡路）。真正的避障
（路径被挡住 → 偏离绕行 → 回归路径）从 Stage 3a 开始。场景 key（configs/env 段）：

| 配置 key | 含义 | 默认 |
|---|---|---|
| `scene` | `open_field` / `corridor`（岸线走廊）/ `mixed`（每回合二选一） | `open_field` |
| `obstacle_on_path_frac` | 压路径障碍占比（1=全阻塞，0=旧避让式） | 0.0 |
| `obstacle_radius_range` | 统一障碍半径范围 m | [1.0, 3.0] |
| `obstacle_radius_ranges` | 尺寸类列表（浮标/常规/码头），优先于上一行 | 无 |
| `corridor_width_range` | 走廊净宽范围 m（实际净宽 = 名义 ±~1.4m） | [8, 25] |
| `corridor_path_bias` | 跟踪路径偏离走廊中心线的比例（≤1） | 0.35 |
| `corridor_mix_p` | `mixed` 时取走廊场景的概率 | 0.5 |
| `gate_gap_range` | 窄门净宽逐门随机范围 m（不设 = 旧固定 4m） | 无 |

场景测试（几何 + 环境布线；纯 numpy 部分任何机器可跑）：

```bash
python3 env/scene_test.py     # 摆放/走廊/扇区/尺寸类 + 环境布线
```

验收线（evaluate 输出 `by_scene` 分场景统计）：两种场景 collision=0、
success ≥85%、speed_mean ≥1.2（防"龟速/停船等待"最优解）。达标后才可上艇影子/水试。

## 交付到 NX

训练完成、评估达标后：把 `models/stageN_actor.pt` 拷回 NX，
步骤见 `deployment/README.md`。

## 已知待办（训练机端）

- `export_torchscript` 依赖 SB3 `actor._predict` 的可 trace 性；若 trace 失败，
  在训练机改用 `torch.onnx.export` 或手写 mu 网络前向，不影响其余流程。
- Fossen 3-DOF 的质量/附加质量/回转惯量为估计值，实船数据标定后再启用对照实验。
- 课程难度参数（障碍半径/密度/通道宽度）按 Stage 2/3 收敛情况微调。
