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

# 4.【在训练机执行】环境冒烟测试（1 条随机动作 episode，非训练）
python3 env/smoke_test.py
```

## 训练（全部【在训练机执行】）

```bash
# Stage 1：无障碍纯路径跟踪（验证：能跟踪、起步平滑、CTE RMSE 小）
python3 train/train_sac.py --config configs/stage1_path_follow.yaml
tensorboard --logdir tb   # 观察 ep_rew_mean / ep_len_mean / ent_coef

# Stage 2：静态障碍（从 stage1 best 续训）
python3 train/train_sac.py --config configs/stage2_static_obstacles.yaml \
    --init-model models/stage1_best/best_model.zip

# Stage 3：复杂场景 + 动态障碍（从 stage2 best 续训）
python3 train/train_sac.py --config configs/stage3_complex.yaml \
    --init-model models/stage2_best/best_model.zip

# 评估（每个阶段训练完必做；指标不达标不进入下一阶段）
python3 evaluation/evaluate.py --config configs/stage2_static_obstacles.yaml \
    --model models/stage2_actor.pt --episodes 50 --plot
```

## 交付到 NX

训练完成、评估达标后：把 `models/stageN_actor.pt` 拷回 NX，
步骤见 `deployment/README.md`。

## 已知待办（训练机端）

- `export_torchscript` 依赖 SB3 `actor._predict` 的可 trace 性；若 trace 失败，
  在训练机改用 `torch.onnx.export` 或手写 mu 网络前向，不影响其余流程。
- Fossen 3-DOF 的质量/附加质量/回转惯量为估计值，实船数据标定后再启用对照实验。
- 课程难度参数（障碍半径/密度/通道宽度）按 Stage 2/3 收敛情况微调。
