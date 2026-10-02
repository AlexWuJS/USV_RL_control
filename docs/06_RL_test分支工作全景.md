# 06 · RL_test 分支工作全景（自创建以来）

> 分支：`RL_test`（自 `develop@7d92011` 分出）｜本文整理时间：2026-10-01
> 分支共 **40 个提交**，时间跨度 2026-02-03 ~ 2026-10-01；相对 develop 改动 **170 个文件，+80359 / -211 行**。
> 主线工作集中在 2026-09-17 ~ 2026-10-01 两周：**RL 局部规划器从设计、训练到上艇控艇全链路打通**，同时带出多点规划交互、水试工程化工具、执行链可靠性修复等配套工作。

---

## 一、分支时间线（按提交分组）

| 时间 | 提交 | 内容 |
|------|------|------|
| 2026-02-03 | `3cacc2a` `d958f5a` | 前置重构：经纬度角度转 ENU 系角度；跟点改为固定速度 |
| 2026-09-17 | `60decde` | RL 局部规划器接口设计定稿（Phase1/2 实艇代码走读） |
| 2026-09-17 | `b5020da` | 新建 `rl_local_planner` ROS 包（rl_common 契约层 + NavCmd + shadow 节点，NX 实跑验证） |
| 2026-09-17 | `5689ec9` | SAC 训练工程脚手架（Gymnasium/SB3，仅训练机运行，含 Windows 步骤） |
| 2026-09-18 | `372c42b` | 系统技术文档套件（docs/00~05） |
| 2026-09-18 | `951b7a2` | RL 节点固定 10Hz 步进（与训练 dt=0.1 对齐） |
| 2026-09-18 | `511600e` | 训练机回艇同步：域随机化基准重采样、reset 解包、中文路径导出、配置嵌套结构、reward 整形；**13 轮训练评估记录** |
| 2026-09-18 | `a34c5a8` | **部署模型 actor.pt 入库**（TorchScript 336KB，sha256 `c09d9356…`） |
| 2026-09-23 | `063418d` | A* 支持多目标串规划 + RViz 点击采集/序列校验/无硬件仿真工具 |
| 2026-09-23 | `1fa37f2` | huamo 修复：路径末端野前视点、轨迹无限累积；speed_max 参数化、取消任务 |
| 2026-09-23 | `6ae0830` | 停止运行时覆盖先验地图（保存门禁 + 产物改名），RViz 统一 map 系 |
| 2026-09-23 | `ced9258` | RTK/IMU 验收脚本；**RL Stage2 部署模型上艇** |
| 2026-09-23 | `1b5bb81` `bef0c36` | 实艇联调参数；水试与 RL 测试操作手册入库 |
| 2026-09-27 | `1713c45` | RL 观测修复：u/v/r 改位姿差分（FAST-LIO twist 恒零）；`hull_to_sensor_yaw` 实测标定 **-91.5°** |
| 2026-09-27 | `5948c27` `60a5fe7` `198ee59` | RL 影子采数录制方案（合并单 bag）、bag 回放流程与工具 |
| 2026-09-28 | `fd867c4` | huamo 到点停车修复（显式停车 PWM + 15m 减速段 + 追路停滞 15s 保护） |
| 2026-09-28 | `f60d5d6` | RL 安全层 P4b 终点限速包络（15→5m 线性压到 0.4 m/s；契约测试 9/9） |
| 2026-09-28 | `3d1826b` | 四个水试工具（基线分析/扇区核对/录制防呆/bag 打捞） |
| 2026-09-29 | `66c236d` | huamo 速度闭环补丁（/Odometry 位姿差分求航速，`use_pose_speed_diff` 开关） |
| 2026-09-29 | `c1f4d29` | RL 观测自回波过滤（`cloud_min_range=1.0m`）+ 触发归因分析脚本 |
| 2026-09-29 | `f498e40` | **Phase 5：huamo 接入 `/guidance/rl_cmd`**（guidance_mode 开关 + 断流自动回退 LOS）+ 滑模 NaN 修复 + 台架注入器与回归脚本 |
| 2026-09-29 | `dccd212` | 《下水测试操作单_闭环第一级》（闭环 A/B + RL 首航完整步骤） |
| 2026-09-30 | `2959d8e` | **多点导航交互 v2**（Publish Point=途经点/图外=清除，2D Nav Goal=终点即时规划） |
| 2026-09-30 | `c39249a` `4ca270f` `8c1bd21` 等 | `water_test.launch` 一条命令启动水试全链路（参数切阶段 + include_bringup 开关）；清除手势改"点非水域" |
| 2026-10-01 | `42d3249` | **定点悬停模式**（hover_set 触发）+ PWM 平滑（摆率限幅 + 死区）+ 台架全项 PASS |

---

## 二、功能一：RL 局部规划器（分支核心）

### 2.1 设计定位与关键决策

**先后的两版设计**（都在仓库根，体现演进）：

| 文档 | 结论 |
|------|------|
| `RL_LOCAL_PLANNER_DESIGN.md`（初版） | 设想 RL 直接输出 `[τ_surge, τ_yaw]` 力，经推力分配矩阵转 PWM。**经代码走读被推翻**：实艇执行链是 huamo 滑模控制，`pid_controller.cpp` 只存在于 Gazebo 仿真链 |
| `RL_INTERFACE_DESIGN.md`（定稿，现行有效） | RL 只**替换 LOS 引导层**，输出期望速度 + 期望航迹向（`Δv_ref/Δcourse_ref` 增量积分而来），底层滑模、PWM 混合、thruster_manager、急停通道全部复用 |

定稿后的数据流：

```
/waypoints_path ──┬─ huamo LOS（保留，基线）──→ /guidance/los_cmd ──┐
                  │                                                ├→ huamo 执行段（guidance_mode 选择）
                  └─ rl_local_planner_node ──→ /guidance/rl_cmd ───┘   speed_huamo / yaw_huamo / remote_data_deal
                          SAC Actor + Safety Supervisor                        │
                                                                        /auto_thruster_pwm → thruster_manager → MCU → 推进器
```

三条铁律：
1. **RL 永远不直接发 PWM**——`/auto_thruster_pwm` 任意时刻只允许一个发布者（huamo_node）；
2. `guidance_mode` 默认 `los`，RL 断流/异常时**无需操作即回退**；
3. 小艇 NX 上**严禁训练**，只做接口开发、Shadow、Actor 推理（`RL_INTERFACE_DESIGN.md` §14 环境纪律）。

### 2.2 涉及文件

**新增包 `message_ws/src/guidance/rl_local_planner/`**（部署侧）：

| 文件 | 作用 |
|------|------|
| `msg/NavCmd.msg` | 引导层输出消息：`desired_speed`(m/s，0=停船) + `desired_course`(camera_init 系 ENU 航迹向) + `source`(0=LOS/1=RL/2=SUPERVISOR) + `supervisor_active` |
| `rl_common/contracts.py` | **双端唯一契约**：49 维观测布局、归一化尺度、速度域 [0.2,2.0]、动作限幅、BodyState/NavCommand 数据结构 |
| `rl_common/observation_builder.py` | 49 维观测组装（唯一实现，训练/部署共用） |
| `rl_common/path_features.py` | 最近点搜索、CTE、路径切线（相邻 pose 位置差分——栅格 A* 发布的单位四元数 yaw 恒 0，差分法对两种规划器都正确）、双前视点（弧长 +6m/+15m）、剩余里程 |
| `rl_common/lidar_sectorizer.py` | 点云 → 36×10° 扇区最近距离（高度带过滤 -0.3~+1.0m） |
| `rl_common/action_processor.py` | SAC 原始输出 [-1,1]² → 增量限幅 → v_ref/course_ref 积分 + 重锚定 reset |
| `rl_common/safety_supervisor.py` | 规则式安全层 P1~P4b（独立于 RL） |
| `rl_common/angle_utils.py` | 角度 wrap 工具 |
| `scripts/rl_local_planner_node.py` | 唯一 ROS 节点（433 行）：ROS 数据 ↔ 契约转换 + Actor 推理 + Shadow 日志 |
| `config/rl_planner.yaml` | 全部参数（动作限幅/安全区/观测/坐标系标定/话题名） |
| `launch/rl_shadow.launch` / `launch/rl_guidance.launch` | Shadow（陪跑）/ Active（执令）两种启动 |
| `models/actor.pt`、`models/stage2_best_actor.pt` | 部署模型（TorchScript，均出自 s2_2m5 那一轮：final 导出与 best checkpoint 导出） |
| `tests/test_rl_common.py` | 契约单测 9 项（含 P4b 终点包络） |
| `scripts/inject_rl_cmd.py` | RL 指令注入器（台架验证 + 转向辨识：解决遥控器 RF 直连不进 ROS、bag 录不到的问题） |
| `scripts/bench_phase5.py` + `bench/run_bench.sh` | Phase 5 台架回归（速度阶跃/航向±30°/急停/悬停三测/看门狗断流翻转），全项 PASS |
| `scripts/verify_lidar_sectors.py` | 36 扇区观测离线核对（hull_to_sensor_yaw 标定用） |
| `scripts/analyze_thrust_baseline.py` | 多档直航/回转 bag → PWM-速度表、v_max_phys、k_r/tau_r/r_max 回归 |
| `scripts/analyze_shadow_supervisor.py` | shadow CSV 触发归因分析 |

**修改 `message_ws/src/control/huamo/`**（Phase 5 执行端，约 74+323 行改动）：
`src/huamo_node.cpp`（guidance_mode 参数、rl_cmd 订阅与看门狗、ENU→huamo 航向换算 `+π`、断流 0.5s 回退、RL 航向参考低通、定点悬停、PWM 平滑）、`src/guidance.cpp`（终点减速、追路停滞保护、野前视点修复）、`CMakeLists.txt`/`package.xml`（依赖 rl_local_planner 消息）。

### 2.3 节点核心设计

- **10Hz 固定步进**（`rl_local_planner_node.py:_maybe_step`）：`dv_max/dcourse_max` 的单位是"每步"，训练 dt=0.1s 固定；`/Odometry` 实测 ~100Hz，不能事件直驱——状态每拍更新，step 严格按 0.1s 节流，长阻塞（>2 周期）后重同步不连发（`951b7a2`）。
- **速度/转率由位姿差分估计**：FAST-LIO 的 `/Odometry` twist 恒为 0（2026-09-27 bag 实测 23046 条全零），直接取 twist 会让策略以为艇永久静止。相邻两帧位姿差分 + 0.15s 一阶低通，跳变/丢包帧保护（`1713c45`）。
- **传感器安装角标定 `hull_to_sensor_yaw = -91.5°`**：46 个直线航行样本的「航迹向 − odom_yaw」中位数（`config/rl_planner.yaml` 注释），同时影响 e_heading、体轴前视点与 36 扇区点云朝向。
- **自回波过滤 `cloud_min_range=1.0m`**：艇体结构（桅杆支杆）在传感器系 -160°/1.40m 处的恒定回波会污染 d_min 与障碍包络（0929 实测）；训练端 raycast 无自身体，过滤后实艇观测才贴近训练分布（`c1f4d29`）。
- **路径 TF 变换**：`/waypoints_path` 在 map 系，节点查 TF map→camera_init 手工变换到 camera_init 后重建 PathFeatures；新路径到达即重锚定（v_ref=当前 u 软起步，course_ref=起点切线）。
- **降级链**：NavCmd 未构建 → 只发诊断；ship_info 不可用 → 视为自主；torch 缺失/模型加载失败 → STUB actor（零动作）；观测 NaN/推理异常 → 冻结参考，持续 1s → Safety 急停。
- **诊断输出**：`/guidance/rl_status`（12 项 Float32：u, r, cte, e_heading, d_min, v_ref, course_ref, cmd_speed, cmd_course, supervisor, infer_ms, stub_flag）+ 每拍 CSV（`~/rl_logs/`，含 s_remaining/tangent 列）。

### 2.4 Safety Supervisor（安全层，独立于 RL）

`rl_common/safety_supervisor.py`，在 RL 输出之后、发布之前执行，触发即 `source=SUPERVISOR_OVERRIDE`：

| 优先级 | 规则 | 动作 |
|--------|------|------|
| P1 | 看门狗：输出缺失/超时 200ms、观测 NaN、模型缺失 | 冻结 v_ref；持续 >1s → desired_speed=0 |
| P2 | 绝对钳位 | v_ref∈[0.2,2.0]、每步增量限幅（节点层已做，这里兜底） |
| P3 | 参考界 | course_ref 偏离路径切线 >60° 截断 |
| P4 | 间隙-速度包络 | d_min>8m 不限；8→3m 线性压到 0.4；3→1.5m 压到 0；<1.5m 立即停 |
| P4b | 终点-速度包络（`f60d5d6`） | s_remaining>15m 不限；15→5m 线性压到 0.4 m/s 爬行，只封顶不判停（到点停车归任务簿记层）；与 P4 取更严者 |

设计意图：**即使 RL 完全失灵，靠近障碍物或接近终点也会被压到停**——RL 负责风格，安全层只兜底。

### 2.5 huamo 执行端集成（Phase 5，`f498e40`）

- 私有参数 `guidance_mode`（`los`/`rl`，默认 los），launch 透传；
- 订阅 `/guidance/rl_cmd`，缓存最新一条；**断流 >0.5s 自动回退 LOS**（日志告警，不需要人工干预）；
- 航向换算一行完成：`desired_heading_ = normalizeAngle(rl_course + π)`——该公式用 0929 bag 两万条 LOS 样本实证（huamo 内部航向约定 = course + π）；
- `supervisor_active 且速度 0` → 兑现为急停；huamo 侧再硬限幅 `spd ≤ speed_max`；
- RL 航向参考低通（`rl_lpf_tau=0.5s`）：策略 course_ref 逐拍摆动（首航实测中位偏差 23°~82°）会 1:1 传导为转向满舵乒乓，每个新指令只向目标靠近一步；
- **PWM 平滑（执行层出口，LOS/RL/悬停共用）**：摆率限幅 50 PWM/s + 转向死区 ±3 PWM（`42d3249`）；
- **定点悬停**：`/guidance/hover_set`（std_msgs/Empty）触发，锚定当前位置（camera_init），位置环 kp=0.30、回归速度上限 0.60 m/s、0.30m 到位归零保持；重复触发不重锚；退出 = 点新航线 / clear / 遥控器切手动（防风浪漂走的"虚拟锚"）。

---

## 三、功能二：模型设计与训练（重点）

### 3.1 训练工程（`rl_local_planner_training/`，仓库根，仅训练机运行）

```
rl_local_planner_training/
├── rl_common/            # ★ 与艇上同名的契约层（sync_rl_common.sh 从 NX 同步，禁止两端各写一套）
├── env/usv_local_env.py  # Gymnasium 环境（观测/动作/终止全部走 rl_common）
│   ├── path_gen.py       # 随机路径（0.5m 间距，与实艇 A* 栅格一致）
│   └── obstacles.py      # 静态/动态/窄通道障碍 → 表面点云 → 同一套扇区化
├── dynamics/first_order.py   # 默认动力学（参数来自实艇，见下）
│   └── fossen_3dof.py        # 对照实验用（质量待标定，未启用）
├── train/train_sac.py    # SAC 训练 + EvalCallback + TorchScript 导出
├── evaluation/evaluate.py# 批量评估（成功率/碰撞率/CTE RMSE/平滑度）
├── configs/stage1_path_follow.yaml / stage2_static_obstacles.yaml / stage3_complex.yaml
├── export_actor.py       # 独立导出补救脚本
├── pipeline_check.py     # 全链路数值管线自检（NX 也可跑，双端对照）
├── windows_setup.ps1 / WINDOWS_STEPS.md / DEPLOYMENT_GUIDE.md
└── train_stage1*.log / train_stage2*.log / eval_*.json   # 13 轮训练评估记录（入库留档）
```

### 3.2 仿真环境与动力学模型

一阶惯性模型（`dynamics/first_order.py`），**参数全部有实艇出处**：

| 参数 | 值 | 来源 |
|------|-----|------|
| `tau_u`（纵荡时间常数） | 0.79 s | `huamo.cpp:7` A=0.793023（稳态 u=B/A·thrust%，B·100≈2.36 m/s 与实艇 speed_max=2 吻合） |
| `tau_r`（偏航时间常数） | 0.53 s | `sys_ident.cpp` Nomoto T≈0.5308（Z 形试验） |
| `k_r`/`r_max` | 0.8 s⁻¹ / 0.6 rad/s | 与 huamo 转弯量级一致的保守初值（sim-to-real 最大不确定项，待 shadow 数据回归） |

- 动作经 ActionProcessor 积分成 v_ref/course_ref 后喂给动力学，**与实艇节点的处理链完全一致**（同一份 rl_common）。
- **域随机化**（Stage 2 起，每 episode 一次）：tau ±40%、r_max ±20%、k_r ±30%、控制延迟 0~2 步、扇区噪声 σ≤0.3m、扇区丢帧 ≤5%、恒定漂流 ≤0.15 m/s、观测噪声。
- 终止条件：碰撞（d_min≤船体半径 0.6m）、到达（剩余里程≤1.5m）、cte>8m 弃线、10s 前进<0.5m 停滞、60s 超时。
- 训练期 `use_speed_envelope=false`——supervisor 包络是部署端兜底，训练让策略在完整动作域学习。

### 3.3 奖励函数设计（`RL_INTERFACE_DESIGN.md` §8 + yaml）

每步奖励 = `w_progress·Δs`（沿轨前进量）− `w_track·|cte|` − `w_heading·|e_heading|` − `w_smooth·‖a_t−a_{t−1}‖²` − 近障梯度惩罚 − `w_safe_speed·max(0, u−v_safe)²`（v_safe 由间隙和路径曲率共同决定）+ 终端项（到达 +20 / 碰撞 −50 / 停滞、超时 −20 / 弃线 −10）。

最终定版权重（configs/stage1、stage2）：`w_progress=4.0, w_track=1.0, w_heading=0.15, w_smooth=0.30, w_safe_speed=0.5`。

### 3.4 SAC 配置

SB3 SAC，`MlpPolicy`，net [256,256]，lr 3e-4，buffer 500k，batch 256，γ=0.99，τ=0.005，train_freq=1，ent_coef=auto；`SubprocVecEnv` 8 并行（每 env 固定不同种子）；EvalCallback（确定性评估 20 回合存 best）+ CheckpointCallback。

### 3.5 训练机环境与纪律

- **训练机 = Windows PC**（本机 RTX 3050 4GB，实际用 **CPU 训练**：小 MLP + train_freq=1，CPU/GPU 总耗时接近，实测 364 FPS）；Python 3.11 venv（3.14 无 wheel）；阿里云 pip 镜像（清华 403）。
- 环境搭建一键脚本 `windows_setup.ps1`；三道验证任一失败禁止训练：契约单测（与艇上双端全绿）→ `env/smoke_test.py` → `pipeline_check.py`。
- 训练时长实测：Stage1 500k ≈ 25 分钟；Stage2 1.5M ≈ 70~90 分钟。

### 3.6 训练迭代：13 轮调参实录（评估 json/log 全部入库）

**Stage 1（无障碍路径跟踪）关键经验：**
- 默认 `w_track=0.10` → 策略全速切弯（success 100% 但 CTE RMSE 1.18m 超标）；
- 线性加压 w_track 0.6→1.0→1.5 出现**龟速悬崖**：1.5 时策略学会趴线（速度钉死 0.2 m/s 下限、CTE 极小但全部超时）——任何随速度增大的每步偏差税都会诱导"减速逃税"；二次惩罚、死区惩罚从零训练同样落坑；
- 连续低学习率微调会退化（R10→R11）；
- **最有效的一招：在可收敛的温和奖励地形上把训练从 50 万步延长到 100 万步**（R12：success 96%→100%，CTE 0.67→0.55）；
- 最终 Stage1 模型（R12）：success **100%** / CTE RMSE 0.55（门槛 0.5，超 10%，决策带偏差进 Stage2，待 shadow 数据复核）/ 速度 1.88 / 零碰撞。

**Stage 2（静态障碍 + 域随机化）**：1.5M 步收敛后 0 碰撞、平均最小间隙 6.86m，但 success 只有 74%——失败全是"时间"问题（近障降速包络 + 绕行顶爆 60s 时限）。**延长到 2.5M 步后 success 94%，三项门槛全过**（碰撞 0%、间隙均值 6.86m、最差 2.27m，`eval_stage2_ext.json`）。教训：**off-policy 的 SAC 在任务变难后 1.5M 步并未真正收敛，先延长训练再考虑改奖励**。

**训练工程本身的 7 个已修复问题**（`DEPLOYMENT_GUIDE.md` §1.6，均有提交）：
1. `reset()` 解包错误（环境启动即崩）；
2. smoke_test 断言 u>2.0 不可能成立；
3. **yaml 的 reward/dynamics_params 写在根层级不生效**（必须放 `env:` 块内）；
4. Windows 中文路径 `torch.jit.save` 误报 → ASCII 临时目录中转 + `export_actor.py` 补救；
5. pip 镜像 403；
6. **域随机化实现为累计相乘**（每回合 `self.tau_u *= f`，参数随机游走，~7000 回合后 tau_u 下溢趋 0 → NaN，Stage2 训 5.8 万步即崩）→ `__init__` 保存基准参数、`randomize()` 先恢复再单次缩放；
7. `SAC.load()` 沿用包内 lr → `custom_objects` 覆盖，使微调可用 1e-4。

### 3.7 交付模型

| 文件 | 指标 | 用途 |
|------|------|------|
| `stage2_actor.pt`（sha256 前 16 位 `c09d9356e6badc16`） | success 94% / 碰撞 0% / 间隙 6.86m / 速度 1.89 | **上艇部署** |
| `stage1_actor.pt`（`99cc544c…`） | success 100% / CTE 0.55 / 速度 1.88 | 纯路径跟踪备选 |

艇上现有 `models/actor.pt`（final 导出）与 `models/stage2_best_actor.pt`（best checkpoint 导出）均出自 s2_2m5（94%）轮，md5 已与训练仓库逐一核对。

---

## 四、功能三：RL 部署与调试全流程（重点）

> 完整文档：`rl_local_planner_training/DEPLOYMENT_GUIDE.md`（整合 WINDOWS_STEPS/deployment/2026-09 实际执行记录）；操作层细则在 `docs/水试与RL测试操作手册.md` 与 `docs/下水测试操作单_闭环第一级.md`。

### 4.0 模型流向（单向）

```
训练机（Windows）SAC 训练
  → models/stageN_best/best_model.zip   （SB3 格式，续训用，不上艇）
  → stageN_actor.pt（TorchScript，~336KB，确定性 Actor 均值动作）
  → scp 拷到 NX  message_ws/src/guidance/rl_local_planner/models/actor.pt
  → NX 仅推理（torch.jit.load，CPU <1ms）
```

### 4.1 Step1 训练机环境（Windows）

```powershell
cd <工程目录>
py -3.11 -m venv .venv
.venv\Scripts\pip install torch==2.5.1 stable-baselines3 -i https://mirrors.aliyun.com/pypi/simple/
.venv\Scripts\pip install -r requirements.txt
# 三道验证（任一失败禁止训练）
.venv\Scripts\python rl_common_tests\test_rl_common.py
.venv\Scripts\python env\smoke_test.py
.venv\Scripts\python pipeline_check.py
```

### 4.2 Step2 训练与评估

```powershell
# Stage 1：无障碍纯路径跟踪（500k，~25min）
.venv\Scripts\python train\train_sac.py --config configs\stage1_path_follow.yaml
# Stage 2：静态障碍 + 域随机化，从 stage1 best 续训（最终 2.5M 步达标）
.venv\Scripts\python train\train_sac.py --config configs\stage2_static_obstacles.yaml --init-model models\stage1_best\best_model.zip
# 评估（达标才进下一阶段：Stage1 success≥95% & CTE RMSE≤0.5；Stage2 success≥85% & 碰撞≤5% & 间隙≥1.5m）
.venv\Scripts\python evaluation\evaluate.py --config configs\stage2_static_obstacles.yaml --model models\stage2_actor.pt --episodes 50 --plot
```

产出：`stageN_best/best_model.zip`（续训/回传用）、`stageN_actor.pt`（部署用）、`tb/`（TensorBoard）。

### 4.3 Step3 回传模型到艇

```powershell
scp models\stage2_actor.pt jetson@192.168.50.186:/home/jetson/haique/little-boat-motion-control/message_ws/src/guidance/rl_local_planner/models/actor.pt
# 两端 sha256sum / certutil -hashfile 比对哈希
```

### 4.4 Step4 NX 构建与契约校验

```bash
cd ~/haique/little-boat-motion-control/message_ws
catkin_make --pkg rl_local_planner      # 首次构建生成 NavCmd 消息
source devel/setup.bash
python3 src/guidance/rl_local_planner/tests/test_rl_common.py   # NX 端契约测试 9 项全绿
```

### 4.5 Step5 Shadow 模式验证（模型上艇第一站，不参与控制）

前提：艇按原 LOS 方式正常航行（bringup + a_star_demo/water_test.launch）。

```bash
# 无模型（STUB，打通数据链）→ 有模型（标准用法）
roslaunch rl_local_planner rl_shadow.launch \
  model_path:=/home/jetson/haique/little-boat-motion-control/message_ws/src/guidance/rl_local_planner/models/stage2_best_actor.pt
rostopic echo /guidance/rl_status
ls -lt /home/jetson/rl_logs | head
```

判据：CSV 无 NaN、`infer_ms < 10ms`、`stub_flag=0`、不超看门狗 200ms。
Shadow 期完成的三件标定/验证（都已做完）：
1. `hull_to_sensor_yaw` 标定 → **-91.5°**（0927 水试 bag，46 个直线样本）；
2. 自回波过滤 → `cloud_min_range=1.0m`（0929 取证艇体 0.8~1.4m 恒定回波）；
3. ENU→huamo 航向换算 `+π` 实证（0929 bag 两万条 LOS 样本）。

同时用 shadow CSV 回归修正训练动力学参数 `k_r/r_max`（sim-to-real 最大不确定项）。

### 4.6 Step6 Phase 5 闭环（RL 执令）

前提：huamo 集成完成（`f498e40`）+ shadow 数据评审通过 + 台架回归全项 PASS（`bench/run_bench.sh`：速度阶跃/航向±30°/急停/悬停三测/断流翻转，`bench/result.txt` 全 PASS）。

上艇前检查单（`DEPLOYMENT_GUIDE.md` §5）：契约单测全绿；shadow CSV 无 NaN；安装角已标定；point_nav_node 未运行（单发布者纪律）；地面站急停演练过；首次闭环 v_max 临时降到 1.0。

**RL 首航（操作单项目 3）的启动组合**——RL 用独立终端启动，急退即 Ctrl+C 该终端：

```bash
# 终端 A：控制链（guidance_mode 切 rl；RL 不由它启动 → rl_mode:=none）
roslaunch a_star_planner water_test.launch guidance_mode:=rl rl_mode:=none use_pose_speed_diff:=true
# 终端 B：RL 节点（active 模式，真正下发指令）
roslaunch rl_local_planner rl_guidance.launch \
  model_path:=/home/jetson/haique/little-boat-motion-control/message_ws/src/guidance/rl_local_planner/models/stage2_best_actor.pt
```

航线：单目标 20~30m、直线为主、静水无浮标。通过标准：RL 独立完成航线并减速停车、无碰撞、CTE ≤ 同航线 LOS 的 2 倍、全程无 NaN。之后做同航线 LOS 对照跑（项目 4）供首航评审。

### 4.7 Step7 安全与回退（四道，按优先级）

| 手段 | 操作 | 效果 |
|------|------|------|
| ① RL 节点急退 | RL 终端 Ctrl+C | rl_cmd 断流 → huamo 0.5s 自动回 LOS（台架已验证） |
| ② 地面站急停 | `S,0,3,c,@,0,`（is_auto_control=false） | thruster_manager 立即中性 PWM，与 RL 无关 |
| ③ 实体遥控器 | 直接接管 | 硬件直连最强，任何时候有效 |
| ④ 取消任务 | `rostopic pub -1 /waypoint_collector/clear std_msgs/Empty` | 空 Path → huamo 停车清轨迹 |

重启 `a_star_demo.launch`（默认 guidance_mode:=los）即完全回传统 LOS 模式。

### 4.8 部署调试中踩过的坑（实艇侧）

| 问题 | 根因 | 修复 |
|------|------|------|
| 观测里 u/v/r 恒为 0，策略以为艇静止 | FAST-LIO `/Odometry` twist 恒零（bag 实测 2.3 万条全零） | 位姿差分 + 0.15s 低通（节点与 huamo 同一套算法，`1713c45`） |
| 扇区观测整体错位风险 | 传感器体轴 ≠ 艇体首向（安装旋转） | `hull_to_sensor_yaw=-91.5°` 实测标定 |
| d_min 被污染、安全层误触发 | 艇体结构 0.8~1.4m 恒定自回波进入扇区 | `cloud_min_range=1.0m` 过滤（`c1f4d29`） |
| 限幅步率与训练不一致 | /Odometry ~100Hz 事件直驱 | 10Hz 固定步进节流（`951b7a2`） |
| RL 模式首航转向满舵乒乓 | course_ref 逐拍摆动 1:1 传导到转向 | huamo 侧航向低通 0.5s + PWM 摆率限幅 50/s + 死区 3（`42d3249`） |
| 滑模趋近律 \|s\|=0 NaN | 除零隐患 | `f498e40` 修复 |

---

## 五、功能四：多点规划与 RViz 交互（v2）

### 5.1 A* 多目标串规划（`063418d`）

修改 `message_ws/src/guidance/planner/a_star_planner/`：
- `src/a_star_planner_node.cpp` + `include/a_star_planner.h`：新增 `/goal_sequence`（nav_msgs/Path）订阅，**逐段规划拼接**（第 i 段起点 = 上一段终点，保证整条连续），相邻段连接点去重；相邻航点间距 < `min_seg_len`(1m) 去重；目标不可达策略 `on_goal_invalid`（abort=整条中止 / skip=跳过该点）；规划结果经 latched 的 `/planner_status` 发布（`OK: N goals, M poses` / `ERROR: waypoint N unreachable (abort)`）。
- **单目标入口起点修复**：起点优先 TF `map→robot_frame`（robot_frame=body，即艇真实位置），`/initialpose` 仅兜底——之前用重定位点当起点会导致规划线起点错。
- `fill_orientation` 默认 false：A* 路径点无真实朝向（单位四元数），填朝向会激活 huamo 转弯分支；RL/LOS 的路径切线一律由位置差分计算。

### 5.2 RViz 点击交互 v2（`2959d8e`，替代终端 commit 流程）

新增 `scripts/waypoint_collector.py`（263 行）：

| RViz 操作 | 效果 |
|-----------|------|
| **Publish Point** 点水域 | 添加途经点（编号蓝球 + 橙色连线，latched 标记 `/waypoint_markers`） |
| **Publish Point** 点岸上/障碍/图外 | **清除整条轨迹**（途经点/终点/规划线/行驶轨迹全清 + 艇停）——清除手势从终端命令改为"点非水域"（`c39249a`） |
| **2D Nav Goal** | 设定/替换终点并**立即**以艇当前位置为起点规划发布 |
| 终端 commit/clear | 保留兼容 |

清除判别用 `/prior_map` 栅格：点击落在非自由栅格（岸/障碍/未知）或图外即触发清空，空 Path 发到 `/waypoints_path`（huamo 语义：取消任务 + 停车 + 清轨迹）。

### 5.3 配套工具

- `scripts/verify_goal_sequence.py`：命令行校验目标序列可规划；
- `scripts/fake_odom_follower.py` + `launch/a_star_demo_sim.launch` + `rviz/a_star_sim.rviz`：**无硬件仿真**（虚拟艇沿规划线走，岸上练习交互）；
- `scripts/test_goal_sequence_offline.sh`：无头回归。

---

## 六、功能五：执行链（huamo）可靠性与水试工程化

### 6.1 huamo 执行链修复（`1fa37f2`、`fd867c4`、`66c236d`）

- **到点不停/绕圈修复**（0928 水试复现）：任务结束**显式发停车 PWM**（thruster_manager 会保持最后收到的 PWM，直接 return 会让艇带巡航油门冲过终点）；剩余 30 点（≈15m）起终点减速段压制原地转弯/提速；感知位置追不上路径持续 15s（CHASE_STALL_CYCLES）自动停车——防定位漂移导致的无限绕圈（`src/guidance.cpp`）。
- **野前视点修复**：剩余点数少于前视距离时，原"切向延长线"算法因路径点朝向恒 0（正东）会算出远离路径的野点 → LOS 钉在路径终点；erase 迭代器失效改下标访问（`guidance.cpp`）。
- **轨迹无限累积修复**：`/motion_path` 只在任务执行期累积，新任务/取消清空（此前 10 分钟单条约 1MB、10Hz 重复发布，曾把磁盘写爆——0928 的 2.2GB bag 里 99% 是同一份轨迹写了 376 遍）。
- **速度闭环开关** `use_pose_speed_diff`（默认关）：位姿差分求航速喂滑模反馈（twist 恒零导致 speed_huamo 反馈失效开环），与 RL 节点同一套算法/参数，保证两端"速度"定义一致。
- **取消任务**：空 `/waypoints_path` → 停车 + 清空路径与轨迹；`speed_max` 参数化（launch 透传）。
- 滑模趋近律 |s|=0 除零 NaN 修复（`f498e40`）。

### 6.2 水试统一入口与工程化工具

- **`launch/water_test.launch`**（`c39249a` + `4ca270f` + `d2bb16f`）：一条命令启动水试全链路，参数切阶段——`speed_max`（首航 0.8）/ `guidance_mode`（los|rl）/ `use_pose_speed_diff` / `use_controller`（false=纯规划，桨绝对不会转）/ `rl_mode`（shadow|active|none）/ `rl_model` / `include_bringup`（默认 false：bringup 终端 1 常驻，各阶段只重启控制部分，**定位保持不断**）。修复过 group 条件的 bool 传参问题。
- 录制防呆 `scripts/record_water.sh`（自动查残留进程/磁盘、停止后自动验证每段）；bag 损坏打捞 `scripts/bag_scavenger.py`（不依赖索引扫 chunk 重建）；回放工具 `rviz/bag_replay.rviz` + `scripts/replay_track_builder.py`（从 /Odometry 重建轨迹）。
- 水试工具脚本：`analyze_thrust_baseline.py`（PWM-速度表、动力学参数回归）、`verify_lidar_sectors.py`（扇区/安装角核对）。
- **地图管理纪律**（`6ae0830`）：定位模式 `pcd_save_en: false` + 建图产物带日期命名（`scans_日期.pcd`）+ 保存门禁，根除"先验地图被运行时覆盖"事故；RViz Fixed Frame 统一为 map（单窗口同时看点云对齐与打点）。
- RTK/IMU 验收脚本：`gps_common/scripts/rtk_status.py`、`wit_imu/scripts/imu_calib_check.py`（`ced9258`）。

---

## 七、文档体系（分支内产出）

| 文档 | 内容 |
|------|------|
| `RL_INTERFACE_DESIGN.md` | **RL 接口设计定稿**（取代三份初版设计），含观测/动作/安全/训练/部署全部定义 |
| `RL_LOCAL_PLANNER_DESIGN.md` / `RL_SIMULATION_DESIGN.md` / `RL_TRAINING_PLAN.md` | 初版设计（已被接口设计部分取代，留作决策依据） |
| `rl_local_planner_training/DEPLOYMENT_GUIDE.md` | **训练→部署全流程**（含 2026-09 实测记录、7 个修复、13 轮调参经验、FAQ） |
| `rl_local_planner_training/README.md` / `WINDOWS_STEPS.md` / `windows_setup.ps1` | 训练工程使用与 Windows 环境搭建 |
| `docs/00~05_*.md` | 系统总览/改动影响评估/模块职责/实艇操作/已知问题/RL 说明 六件套 |
| `docs/水试与RL测试操作手册.md` | 出发前门禁（地图门禁四判据）、录制规范、水试各段、RL T1/T2 门槛、地图管理纪律、速查 |
| `docs/下水测试操作单_闭环第一级.md` | 闭环第一级逐步操作单（速度闭环 A/B + RL 首航 + shadow 复测 + 湖心专段 + 应急速查） |
| `docs/修改说明_20260923.md` | 该轮联调修改说明 |

---

## 八、当前状态与遗留事项

**已达成**（截至 42d3249）：
- 原有系统（链 A point_nav / 链 B huamo LOS）全部可用，未受 RL 改造影响（改动集中在新增包）；
- RL：Stage1/Stage2 训练达标（94%/零碰撞）→ actor.pt 上艇 → shadow 链路验证 → Phase 5 闭环集成 → 台架全项 PASS → RL 首航操作单就绪（多点规划、RViz 交互 v2、water_test 一条命令均已台架/无头验证）；
- 定点悬停、PWM 平滑、到点停车、终点限速包络等执行链保障全部就位。

**遗留 / 下一步**（按文档记载）：
1. Stage1 CTE RMSE 0.55 超门槛 10%——待实艇 shadow 数据复核，下一杠杆是与艇侧协商前视距离契约参数（la1=6m/la2=15m）而非继续调奖励；
2. `k_r/r_max` 用实船数据回归修正训练动力学（shadow CSV 已具备条件）；
3. RL 闭环按操作单逐级推进：低速直线 → 静态浮标 → 多目标，每级与 LOS 同航线对比；
4. Fossen 3-DOF 对照动力学质量参数待实船标定；Stage3 动态障碍第一版实艇验证可跳过；
5. huamo 内部 `+91.5°` 硬编码偏置仍是技术债（换雷达/换艇必须重新标定）。
