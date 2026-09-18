# RL 局部规划器：训练 → 实艇部署 全流程文档

> 适用工程：`rl_local_planner_training`（训练机）↔ `message_ws/src/guidance/rl_local_planner`（实艇 NX）
> 模型流向（单向）：**训练机 SAC → best actor → actor.pt (TorchScript) → 拷贝到 NX → 仅推理**。NX 上不做任何训练。
> 本文档整合自 `deployment/README.md`、`WINDOWS_STEPS.md` 与 2026-09 训练机实际执行记录。

---

## 0. 系统架构一览

```
┌────────────────────────── 训练机（Windows，本机）──────────────────────────┐
│  env/USVLocalEnv（仿真环境）                                                │
│    └─ rl_common 契约层：PathFeatures / LidarSectorizer /                    │
│       ObservationBuilder / ActionProcessor / SafetySupervisor              │
│  train/train_sac.py（SAC，8 并行环境，CPU 约 360 FPS）                       │
│    └─ 输出：models/stageN_best/best_model.zip（SB3 格式，续训用）            │
│              models/stageN_final.zip                                       │
│              models/stageN_actor.pt（TorchScript，部署用，约 340KB）         │
└──────────────────────────────────┬─────────────────────────────────────────┘
                                   │ scp（单向拷贝）
┌──────────────────────────────────▼─────────────────────────────────────────┐
│  实艇 Jetson NX：guidance/rl_local_planner_node（仅推理加载 actor.pt）        │
│    Shadow 模式：只观测/记录，不下发（与 huamo LOS 并行）                      │
│    闭环模式（Phase 5 后）：RL 输出经艇上 ActionProcessor 硬限幅 +              │
│       SafetySupervisor 兜底后下发 huamo                                      │
│  安全兜底（永远在 RL 外侧）：地面站 is_auto_control=false 急停；               │
│       重启 a_star_demo.launch 回传统 LOS                                     │
└─────────────────────────────────────────────────────────────────────────────┘
```

观测 49 维 / 动作 2 维（Δv、Δcourse）/ 控制周期 0.1s，两端由同一份 `rl_common` 代码保证数值一致（以 `rl_common_tests/test_rl_common.py` 双端全绿为准）。

---

## 1. 训练机侧（Windows）

### 1.1 前提检查

| 检查项 | 命令 | 通过标准 |
|---|---|---|
| Python ≥ 3.9（推荐 3.11） | `py -0p` | 存在 3.9~3.12 版本；3.14 过新无 wheel，勿用 |
| GPU（可选） | `nvidia-smi` | 无卡 CPU 可训（本机实测 CPU 360 FPS 已够用） |
| 磁盘 | — | ≥ 10GB 空闲 |

> **本机经验**：系统默认 Python 3.14.2 无 torch/sb3 wheel；用 `py -3.11 -m venv .venv` 指定 3.11 建 venv。CUDA 版 torch（2.4GB）下载约 2 小时+，而本工程是小 MLP + train_freq=1（50 万步仅 6.25 万次梯度更新），CPU 与 GPU 总耗时接近，**建议直接 CPU 训练**。

### 1.2 环境搭建

```powershell
cd E:\无人艇\无人艇\小艇项目\RL部署\rl_local_planner_training
py -3.11 -m venv .venv
.venv\Scripts\python -m pip install --upgrade pip
# torch + 其余依赖（国内推荐阿里云镜像；清华镜像会 403）
.venv\Scripts\pip install torch==2.5.1 stable-baselines3 -i https://mirrors.aliyun.com/pypi/simple/
.venv\Scripts\pip install -r requirements.txt
```

### 1.3 三道验证（任一失败禁止训练）

```powershell
.venv\Scripts\python rl_common_tests\test_rl_common.py   # 契约层 8 组，必须全绿（且与艇上结果一致）
.venv\Scripts\python env\smoke_test.py                   # 环境 API + 动力学 + 随机 episode
.venv\Scripts\python pipeline_check.py                   # 全链路数值管线（NX 也可跑，双端对照）
```

### 1.4 训练命令（在工程根目录执行，tb/ 才会落在正确位置）

```powershell
# Stage 1：无障碍纯路径跟踪（50 万步，CPU 约 25 分钟）
.venv\Scripts\python train\train_sac.py --config configs\stage1_path_follow.yaml

# Stage 2：静态障碍 + 域随机化（150 万步，CPU 约 70~90 分钟），从 stage1 best 续训
.venv\Scripts\python train\train_sac.py --config configs\stage2_static_obstacles.yaml --init-model models\stage1_best\best_model.zip
```

产出（`models/` 下）：`stageN_best/best_model.zip`（EvalCallback 最优，**续训/回传用它**）、`stageN_final.zip`、`stageN_actor.pt`（TorchScript，评估/部署用）、`tb/`（TensorBoard 曲线：`tensorboard --logdir tb`）。

### 1.5 评估门槛（达标才进入下一阶段）

```powershell
.venv\Scripts\python evaluation\evaluate.py --config configs\stage1_path_follow.yaml --model models\stage1_actor.pt --episodes 50 --plot --output eval_stage1.json
```

| 阶段 | 门槛 |
|---|---|
| Stage 1 | success ≥ **95%**；CTE RMSE ≤ **0.5m**；速度 max ≤ **2.0** 且起步平滑 |
| Stage 2 | success ≥ **85%**；碰撞率 ≤ **5%**；平均最小间隙 ≥ **1.5m** |
| Stage 3（动态障碍） | 第一版实艇验证可跳过 |

### 1.6 训练机侧已知问题与修复记录（2026-09-18 实测）

| # | 问题 | 修复 | 状态 |
|---|---|---|---|
| 1 | `env/usv_local_env.py` `reset()` 中 `obs, _ = self._observe()` 解包错误（`_observe` 返回 5 值），环境一启动即崩 | 改为 5 值解包 | ✅ 已修复 |
| 2 | `env/smoke_test.py` 动力学测试：下发 2.0 m/s 指令却断言 u>2.0（模型 u 渐近收敛到指令值，不可能超过） | 改为下发 v_max_phys=2.3 | ✅ 已修复 |
| 3 | **三个 stage yaml 的 `reward:`/`dynamics_params:` 写在根层级，而 `USVLocalEnv` 只从 `env:` 子字典读取**——配置从未生效，训练一直用代码默认值 | 两个 section 移入 `env:` 块（stage1/2/3 已全部修正） | ✅ 已修复 |
| 4 | Windows 中文路径下 `torch.jit.save` 误报 "Parent directory does not exist" | `train_sac.py::export_torchscript` 已加"ASCII 临时目录中转 + 移动"兼容层；另有独立补救脚本 `export_actor.py` | ✅ 已修复 |
| 5 | 清华 PyPI 镜像 403 | 改用阿里云镜像 | ✅ 已验证 |
| 6 | **域随机化实现为累计相乘**（`dynamics.randomize` 里 `self.tau_u *= f` 且每回合 reset 都调用）——参数随机游走，约 7000 回合后 tau_u 下溢趋 0 → NaN 崩溃（Stage 2 训练 5.8 万步即崩） | `first_order.py`/`fossen_3dof.py`：`__init__` 保存基准参数，`randomize()` 先恢复基准再单次缩放（每回合独立重采样） | ✅ 已修复 |
| 7 | SB3 `SAC.load()` 沿用包内保存的 lr，微调配置的学习率不生效 | `train_sac.py` 用 `custom_objects={"learning_rate": ...}` 覆盖 | ✅ 已修复 |

**调参经验（Stage 1，13 轮实测）**：
- 默认权重（w_track 0.10）→ 策略全速切弯（success 100%，CTE RMSE 1.18m 超标）；
- 线性加压 w_track 0.6→1.0→1.5 存在**龟速悬崖**：1.5 时策略学会趴线（速度钉在 0.2 m/s 下限、CTE 极小但全部超时）——任何随速度增大的每步偏差税都会诱导"减速逃税"；
- 二次惩罚项从零训练同样落入龟速陷阱；死区惩罚（0.35m 内免税）从零训练亦然；
- **从已收敛的飞行策略上做低学习率（1e-4）微调是安全方向**，但连续多次微调会退化（R10→R11）；
- **最有效的一招：在可收敛的温和奖励地形上把训练时长从 50 万步延长到 100 万步**（R12：success 96%→100%，CTE 0.67→0.55，速度不变）；
- 最终 Stage 1 模型 R12：success **100%** / CTE RMSE **0.55**（门槛 0.5，超 10%，后续三次收尾微调均致成功率恶化，决策带偏差进入 Stage 2，待 Shadow 数据再评估）/ 速度 1.88 / 零碰撞。

**Stage 2 调参经验**：1.5M 步收敛后 0 碰撞、间隙 6.86m，但 success 74%——失败全部是"时间"问题（11 条超时 + 2 条大绕行触发 8m cte_abort）；近障降速包络（d_min 4m 时 v_safe≈1.2 m/s）+ 绕行距离会顶爆 60s 时限。**延长 100 万步（共 2.5M）后 success 升至 94%，三项门槛全部通过**（0 碰撞、平均最小间隙 6.86m、最差 2.27m）。教训：off-policy 的 SAC 在任务变难后 1.5M 步并未真正收敛，先延长训练再考虑改奖励。

## 9.1 最终交付模型（2026-09-18）

| 文件 | 指标 | 用途 |
|---|---|---|
| `models/stage2_actor.pt`（sha256 前 16 位 `c09d9356e6badc16`） | success 94% / 碰撞 0% / 间隙 6.86m / 速度 1.89 | **回传艇上部署** |
| `models/stage1_actor.pt`（sha256 前 16 位 `99cc544cd5b1ce99`） | success 100% / CTE 0.55 / 速度 1.88 | 纯路径跟踪备选（无障碍场景） |
| `archive_rounds/` | 各轮训练产物与说明 | 回溯/对比 |

已知偏差：Stage 1 的 CTE RMSE 0.55 高于 0.5 门槛 10%（Stage 2 门槛不含 CTE）；上艇 Shadow 阶段用真实数据复核，如需更紧跟踪精度，下一杠杆是与艇侧协商前视距离契约参数（la1=6m/la2=15m），而非继续调奖励。

---

## 2. 回传模型到艇（训练机执行）

```powershell
# Stage 2 通过门槛后
scp models\stage2_actor.pt jetson@192.168.50.186:/home/jetson/haique/little-boat-motion-control/message_ws/src/guidance/rl_local_planner/models/actor.pt
```

- 只传 **TorchScript 的 actor.pt**（艇上仅推理）；`best_model.zip` 是训练机续训用的 SB3 格式，不上艇。
- 回传后可用 `sha256sum` / `certutil -hashfile` 比对两端哈希。

---

## 3. 艇上构建（NX 执行）

```bash
cd /home/jetson/haique/little-boat-motion-control/message_ws
catkin_make --pkg rl_local_planner      # 或 catkin build rl_local_planner
source devel/setup.bash
```

> 首次构建会生成 `rl_local_planner/NavCmd` 消息。**huamo 的集成改动（guidance_mode + rl_cmd 订阅）属于 Phase 5**，未完成前 NavCmd 没有订阅者，节点处于纯 shadow 诊断状态，**不影响现有系统**。

---

## 4. Shadow 模式验证（NX 执行，模型上艇后第一站）

**前提**：实艇按现有方式正常航行（bringup + a_star_demo，huamo LOS 控制）。

```bash
# 无模型（STUB，仅打通数据链/日志）
roslaunch rl_local_planner rl_shadow.launch

# 有模型（标准用法）
roslaunch rl_local_planner rl_shadow.launch model_path:=$(rospack find rl_local_planner)/models/actor.pt
```

**产出：**
- `/guidance/rl_status`（`std_msgs/Float32MultiArray`）：
  `[u, r, cte, e_heading, d_min, v_ref, course_ref, cmd_speed, cmd_course, supervisor, infer_ms, stub_flag]`
- 运行日志 CSV：`/home/jetson/rl_logs/`（路径见 `config/rl_planner.yaml`）

**Shadow 期间要做的事**：艇按原有 LOS 方式正常跑若干航线；RL 节点每 0.1s"虚拟操舵"一次并记录"假如由我控制会下发什么"。用 CSV 数据做两件事：
1. **质量检查**：无 NaN、infer_ms < 10ms、stub_flag=0；
2. **对比分析**：RL 虚拟指令 vs LOS 实际指令/实际航迹，验证 CTE、速度、避障行为一致性；同时用实测响应数据**回归修正动力学参数**（tau_u/tau_r/k_r/r_max，代码注释预留的口子），参数偏差大就回训练机改 `configs` 重训（单轮 20~90 分钟）。

---

## 5. 上艇前检查单（进入闭环前逐项打勾）

- [ ] `rl_common` 单测 NX 端全绿：`python3 tests/test_rl_common.py`
- [ ] shadow CSV 无 NaN、infer_ms < 10ms、stub_flag=0
- [ ] `hull_to_sensor_yaw` 已用已知障碍方位标定（§13.6）
- [ ] point_nav_node 未运行（`/auto_thruster_pwm` 单发布者纪律）
- [ ] 地面站急停通道演练过一次
- [ ] 首次闭环测试限速：v_max 临时降到 1.0 m/s

---

## 6. Phase 5 闭环接管（暂缓，需 shadow 数据评审通过）

huamo_node 完成 guidance_mode 改造并评审 shadow 数据后：

```bash
roslaunch rl_local_planner rl_guidance.launch model_path:=$(rospack find rl_local_planner)/models/actor.pt
```

- 首次闭环：v_max 1.0 m/s 起步，直线/大曲率航线先行，逐步放开到 2.0 m/s；
- 每次闭环后回收 CSV 与 `/guidance/rl_status` 复盘。

---

## 7. 紧急回退（两道保险，任何时候可用）

| 手段 | 操作 | 效果 |
|---|---|---|
| 急停 | 地面站 `is_auto_control=false` | 立即中性 PWM |
| 退出 RL | 重启 `a_star_demo.launch` | 回传统 LOS 模式，RL 完全退出回路 |

---

## 8. 常见问题（FAQ）

| 现象 | 处理 |
|---|---|
| `powershell 无法加载文件…执行策略` | `powershell -ExecutionPolicy Bypass -File windows_setup.ps1` |
| pip 装 torch 极慢 / 清华 403 | 阿里云镜像 `-i https://mirrors.aliyun.com/pypi/simple/`；官方 cu121 源约 300KB/s，2.4GB 需 2h+，建议 CPU 版 |
| Windows 多进程 `Can't pickle`/spawn 报错 | 确认以 `python train\train_sac.py` 为入口运行（内部已有 `if __name__ == "__main__"` 保护） |
| 训练导出 actor.pt 报 "Parent directory does not exist" | 中文路径已知问题，`train_sac.py` 已带兼容层；旧版本可用 `python export_actor.py models/stageN_final.zip models/stageN_actor.pt` 补救 |
| ep_rew_mean 长期不涨 | 先回到 Stage 1 配置；检查 reward 权重是否被改过；lr 降到 1e-4 重试 |
| 策略蛇形抖动 | `reward.w_smooth` 0.30 → 0.5 重训 |
| 评估全停在起点 | progress 权重不足或碰撞惩罚过重 |
| 评估 success 100% 但 CTE 超标（切弯） | `reward.w_track` 上调（本次 0.10→0.60 有效） |
| 评估 CTE 达标但全部 60s 超时 | 速度被压过头，`reward.w_progress` 上调（本次 2.0→3.5 有效）；先物理核对路径曲率与艇机动能力 |
| 改了 yaml 权重训练无变化 | 检查 `reward:`/`dynamics_params:` 是否在 `env:` 块内（根层级不会被读取，见修复记录 #3） |

---

## 9. 训练时长参考（本机实测：RTX 3050 4GB / CPU 训练 / 8 并行环境）

| 阶段 | 总步数 | 实测耗时 |
|---|---|---|
| Stage 1 | 500k | ~25 min（364 FPS）+ 评估 1~2 min |
| Stage 2 | 1.5M | ~70~90 min + 评估 ~2 min |
| 一轮调参重训（Stage 1） | 500k | ~25 min |
