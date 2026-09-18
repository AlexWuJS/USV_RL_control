# RL 局部规划器接口设计（Phase 2 · 基于真实代码）

> 状态：设计定稿，未动代码。
> 依据：Phase 1 对实艇链路的完整代码走读（huamo / a_star_planner / thruster_manager / point_nav / FAST-LIO / nmeadriver / sys_ident）。
> 本文取代 `RL_LOCAL_PLANNER_DESIGN.md`、`RL_SIMULATION_DESIGN.md`、`RL_TRAINING_PLAN.md` 中与本文明失的部分（见 §0）。

---

## 0. 与已有三份设计文档的关系

三份旧文档均假设 RL 直接输出 `[τ_surge, τ_yaw]` 力，经 `pid_controller.cpp` 的推力分配矩阵转 PWM 部署。经真实代码走读，该假设**不成立**，原因：

1. **实艇执行链是 huamo 滑模控制，不是 pid_controller**。`pid_controller.cpp` 只存在于 Gazebo 仿真链（`waypoint_los_pid` / `los_pid_follow`，发 `/wamv/thrusters/*_thrust_cmd`）。实艇 A* 自主导航由 `huamo_node` 执行：LOS(`guidance.cpp`) → `speed_huamo`/`yaw_huamo` 滑模 → `remote_data_deal` → `/auto_thruster_pwm`。
2. **与总任务说明 §四冲突**：第一版 RL 输出 `Δv_ref / Δcourse_ref`，不输出力，不碰 PWM。
3. 直接发 PWM 的 RL 节点 = 绕过全部现有控制器，改动面和风险远大于"替换 LOS 层"。

本文按总任务说明的增量式设计，全部接口以实艇代码为准。

---

## 1. 系统架构与数据流

### 1.1 实艇现状（Phase 1 结论）

```
MID-360 → FAST-LIO → /Odometry (camera_init, ~10Hz) ─┐
scans.yaml → map_server → /prior_map ────────────────┤
                                                     ▼
/initialpose + /move_base_simple/goal
  → Hybrid A* 或栅格 A* → /waypoints_path ("map" 系)
  → huamo_node: TF→camera_init
      getDesiredSpeedHeading()   ← 唯一被 RL 替换的层
      speed_huamo() / yaw_huamo()  ← 全部保留
      remote_data_deal()           ← 全部保留
  → /auto_thruster_pwm → thruster_manager (is_auto_control 门控, 1200~1800 钳位)
  → /controller → usart → MCU → 双推进器
```

### 1.2 加入 RL 后

```
/waypoints_path ──┬─ huamo LOS ──→ /guidance/los_cmd ──┐
                  │                                    │
                  └─ rl_local_planner_node ──→ /guidance/rl_cmd
                         (SAC + Safety Supervisor)     │
                                                       ▼
                              huamo_node 执行段 (guidance_mode 参数选择)
                                                       │
                                        speed_huamo / yaw_huamo / remote_data_deal
                                                       ▼
                                            /auto_thruster_pwm → ... → 推进器
```

要点：
- `/auto_thruster_pwm` 在任意时刻只允许一个发布者（huamo_node）。RL 节点永远不直接发 PWM。
- 急停复用现有通道：地面站将 `is_auto_control=false` → thruster_manager 输出中性 PWM，与 RL 无关、无需改代码。
- `guidance_mode` 默认 `los`，RL 异常时无需操作即回退（huamo 不依赖 RL 节点存在）。

---

## 2. 新增消息：`rl_local_planner/NavCmd.msg`

放在新建的 rl_local_planner 包内（而不是 huamo 包）：**实艇上不动任何现有包的构建配置**，消息随新包生成；huamo 对该消息的依赖在 Phase 5 集成时再加：

```
# guidance 层输出指令（LOS 或 RL 产生）
# desired_course 一律为 camera_init 系标准 ENU 航迹向 (-pi, pi]
std_msgs/Header header
float64 desired_speed      # m/s, [0, v_max]；0.0 = 停船
float64 desired_course     # camera_init 系 ENU 航迹向 (rad)
uint8  source              # 0=LOS, 1=RL, 2=SUPERVISOR_OVERRIDE
uint8  supervisor_active   # 1=本拍 Safety Supervisor 介入过（仅诊断用）
```

**航向约定（关键，Phase 1 实测结论）**：huamo 内部航向是自成一体的标定约定
（`psi = getYaw + 91.5°`，`desired_heading = atan2(-(Δy), -(Δx)) = 路径方向 + π`）。
消息层统一用标准 ENU，换算只发生在 huamo_node 内部一行：

```cpp
desired_heading_ = normalizeAngle(nav_cmd.desired_course + M_PI);   // RL 模式
// LOS 模式不变；发布 los_cmd 时取 normalizeAngle(desired_heading_ - M_PI)
```

Shadow 阶段用 LOS 自身输出实证该换算（LOS 前视点方向 vs `desired_heading − π` 应恒等）。

---

## 3. huamo_node 最小修改（执行器）

文件：`message_ws/src/control/huamo/src/huamo_node.cpp`，另改 `CMakeLists.txt` / `package.xml`（消息生成）。

| # | 改动 | 内容 |
|---|------|------|
| 1 | LOS 输出转发 | `huamo_run()` 中 `getDesiredSpeedHeading()` 之后，发布 `/guidance/los_cmd`（ENU 换算后），带 `source=LOS` |
| 2 | RL 指令订阅 | 订阅 `/guidance/rl_cmd`，缓存最新一条；带时间戳过期判断（>0.5s 丢弃） |
| 3 | 模式选择 | 私有参数 `~guidance_mode`（`los`/`rl`，默认 `los`）。`rl` 模式且缓存未过期时用 RL 指令，否则自动用 LOS（静默回退） |
| 4 | 参数动态加载 | 可选：`dynamic_reconfigure` 或 `rosparam` 轮询切换，避免重启节点（第一版用重启 launch 即可） |

不改：`current_state_` 提取、+91.5° 偏置、`speed_huamo`、`yaw_huamo`、`remote_data_deal`、主循环结构、发布频率（/Odometry 驱动 ~10Hz）。

---

## 4. rl_local_planner_node 设计（决策器）

新包：`message_ws/src/guidance/rl_local_planner/`（纯 Python，catkin）。

```
rl_local_planner/
├── package.xml / CMakeLists.txt
├── scripts/rl_local_planner_node.py   # 唯一节点
├── config/rl_planner.yaml             # 全部阈值/参数
├── launch/rl_shadow.launch            # shadow 模式（默认）
├── launch/rl_guidance.launch          # 闭环模式（含 huamo guidance_mode:=rl）
└── models/sac_usv_v1.pt               # TorchScript 导出的 actor
```

### 4.1 Topic 接口

| 方向 | Topic | 类型 | 用途 |
|------|-------|------|------|
| 订阅 | `/Odometry` | nav_msgs/Odometry | u, v, r, 位置, yaw（camera_init 系） |
| 订阅 | `/waypoints_path` | nav_msgs/Path | 全局路径，TF map→camera_init 变换（复用 huamo pathCallback 写法） |
| 订阅 | `/cloud_registered_body` | sensor_msgs/PointCloud2 | 体轴系点云扇区化（FAST-LIO `scan_bodyframe_pub_en: true`，laserMapping.cpp:894） |
| 订阅 | `/ship/ship_info` | tcp/ship_info | `is_auto_control` 门控：非自主模式 RL 静默 |
| 订阅 | TF | map→camera_init | 路径变换 |
| 发布 | `/guidance/rl_cmd` | huamo/NavCmd | 决策输出 |
| 发布 | `/guidance/rl_status` | 自定义诊断（std_msgs/Float32MultiArray） | 观测回放、supervisor 介入标志、推理耗时、NaN 计数 |

### 4.2 运行模式

| 模式 | 行为 |
|------|------|
| `shadow`（默认） | 接收与 LOS 相同的输入，计算并发布 `/guidance/rl_cmd`，但 huamo 用 `guidance_mode:=los`，RL 不参与控制；内部记录 LOS vs RL 双指令对比（csv/rosbag） |
| `active` | 同上，但 huamo `guidance_mode:=rl`，RL 指令进入执行段 |
| `bypass` | 节点不启动或 `is_auto_control=false`：系统行为与现状完全一致 |

### 4.3 节点内部流水线（10Hz 固定步进）

由 /Odometry 触发但节流至 0.1s 固定周期（`control_rate_hz` 参数）：/Odometry 实测 ~100Hz，动作限幅 dv_max/dcourse_max 单位是"每步"，且训练环境 dt=0.1s 固定——步率必须钳在 10Hz 才能与训练一致。长阻塞（>2 周期）后重同步，不连发补偿。

```
ROS 数据 → ObservationBuilder(48维) → SAC Actor(TorchScript, CPU, <1ms)
        → ActionDecoder(Δv_ref, Δcourse_ref) → 参考积分器
        → Safety Supervisor(独立规则) → NavCmd 发布
```

---

## 5. Observation 定义（49 维）

全部归一化后进入网络。路径切线**一律由相邻 pose 位置差分计算**——实测栅格 A* 发布单位四元数（yaw 恒 0），Hybrid A* 才有真朝向；差分法对两者都正确。

| 组 | 索引 | 内容 | 来源 / 计算方式 | 归一化 |
|----|------|------|----------------|--------|
| A 自身 | 0 | u（体轴 surge，m/s） | /Odometry twist 按 huamo 同款投影到体轴 | /2.5 |
| A | 1 | r（yaw rate，rad/s） | `odom.twist.twist.angular.z`（shadow 期与 huamo 的 `10·Δyaw` 差分对比确认符号） | /1.0 |
| A | 2 | β（漂角） | `atan2(v, u)` | /π |
| A | 3 | v_ref 当前值 | 参考积分器状态 | [0.2,2]→[-1,1] |
| B 路径 | 4 | cte（横偏，m，左正/右负） | 最近点搜索 + 路径切线系分解（同 set_of_los.cpp:64-70 方法） | /10 |
| B | 5 | e_heading（航迹向误差） | 路径切线 − 当前航迹向 `atan2(V_y, V_x)`，wrap 到 ±π | /π |
| B | 6,7 | 前视点 1（体轴 x,y） | 沿路径弧长 +6m 处，转体轴 | /20 |
| B | 8,9 | 前视点 2（体轴 x,y） | 沿路径弧长 +15m 处（隐含曲率信息，不单独估计曲率——离散 pose 差分曲率噪声过大） | /20 |
| B | 10 | 剩余路径弧长 | 最近点 → 终点 | /100 |
| C 障碍 | 11-46 | 36 扇区最近障碍距离 | `/cloud_registered_body` 高度带过滤（相对传感器 -0.3~+1.0m，Phase 7 实测标定）→ 36×10° 扇区 → 每扇区最小距离 | min(d,20)/20 |
| D 动作 | 47,48 | 上一拍 (Δv_ref, Δcourse_ref) | 参考积分器输出历史 | 同 Action 归一化 |

49 维 = 4（自身）+ 7（路径）+ 36（扇区）+ 2（上拍动作）。任务说明估算 46 维的偏差：+β 漂角、+v_ref 当前值、+剩余弧长、双前视点替代曲率 1 维，均为实现后确认项。

**仿真一致性**：训练环境中扇区距离用几何 raycast（圆障碍 + 栅格地图两种后端），数值定义与上表逐项一致；观测构建代码在 sim 与 ROS 节点间共享同一份 Python 模块（`usv_obs_builder.py`），杜绝两处实现漂移。

---

## 6. Action 定义与参考积分

```text
SAC 原始输出 a ∈ [-1,1]²  →  Δv_ref = a[0]·DV_MAX,  Δcourse_ref = a[1]·DCOURSE_MAX

v_ref(t+1)      = clip(v_ref(t) + Δv_ref, V_MIN, V_MAX)
course_ref(t+1) = wrap(course_ref(t) + Δcourse_ref)
```

| 参数 | 初值 | 说明 |
|------|------|------|
| `dv_max` | 0.10 m/s/步 | 10Hz → 最大 1.0 m/s² 参考加速度 |
| `dcourse_max` | 0.05 rad/步 | 10Hz → 最大参考转率 ≈ 28.6°/s |
| `v_min / v_max` | 0.2 / 2.0 m/s | 与实艇 LOS 速度域一致；实艇滑模不可倒车，故不设负速度 |

**重置/重锚定**（防积分漂移）：
- 收到新 `/waypoints_path` → `course_ref` = 起点处路径切线，`v_ref` = 当前 u（软起步）；
- `|cte| > cte_reinit`（默认 6m）→ `course_ref` 立即重锚定到当前路径切线（等价于把"把船拉回来"的极限行为交给规则，不指望 RL 学会自我恢复到离谱偏差）。

---

## 7. Safety Supervisor（独立于 RL，规则式）

按优先级从高到低，全部在 RL 输出之后、`/guidance/rl_cmd` 发布之前执行；任何一条触发即 `source=SUPERVISOR_OVERRIDE` 并记录。

| P | 规则 | 动作 |
|---|------|------|
| 1 | 看门狗：策略输出缺失/超时 >200ms、观测 NaN、模型加载失败 | 冻结 v_ref，course_ref = 路径切线；持续 >1s → desired_speed=0 |
| 2 | 绝对钳位 | `v_ref ∈ [v_min, v_max]`，`|Δv_ref| ≤ dv_max`，`|Δcourse_ref| ≤ dcourse_max` |
| 3 | 参考界 | `|wrap(course_ref − path_tangent)| ≤ course_dev_max`（60°），超出截断 |
| 4 | 间隙-速度包络 | `d_min` = 36 扇区最小值：`>8m` 不限制；`8→3m` 线性压到 0.4 m/s；`3→1.5m` 压到 0；`<1.5m` 立即 desired_speed=0 |
| 5 | 外部急停 | `is_auto_control=false`（thruster_manager 现有逻辑，不属本节点，列出仅为完整性） |

设计意图：P4 保证"即使 RL 完全失灵，靠近障碍物也会被压到停"，这是 shadow → 实艇的底线；RL 学到的是"提前减速绕行"，supervisor 只兜底不主导。

---

## 8. Reward 设计 v1（10Hz，dt=0.1s）

| 项 | 公式（每步） | 权重初值 | 针对问题 |
|----|-------------|---------|---------|
| Progress | `+W_p · Δs`，Δs=沿轨前进量(m) | 2.0 | 效率；只奖前进量，不奖速度本身 |
| Tracking | `−W_t · |cte|` | 0.10 | 不远离全局路径 |
| Heading | `−W_h · |e_heading|` | 0.05 | 航向平稳 |
| Obstacle | `d_min>8m: 0；3~8m: −0.2·(8−d)/5；1.5~3m: −0.5−0.5·(3−d)/1.5；<1.5m: −2.0` | — | 近障梯度 |
| Smooth | `−W_s · ‖a_t − a_{t−1}‖²`（a 为归一化网络输出） | 0.30 | 动作振荡 |
| SafeSpeed | `−W_v · max(0, v − v_safe)²`，`v_safe = min(2.0, 0.4+0.2·d_min, sqrt(a_lat_max/|κ_path|))`，`a_lat_max=0.5` | 0.5 | 弯道/近障不减速 |
| Terminal | 到达 +20；碰撞 −50；超时/停滞(10s 前进 <0.5m) −20；cte>8m 弃线 −10 | — | 端事件 |

- γ=0.99，episode ≤ 60s 仿真时。
- 权重是 Stage 1/2 的起点，不是承诺值；调参记录进训练日志。
- 明确禁止：为压指标往 reward 里加场景特判（任务说明 21.3）。

---

## 9. 训练环境动力学与真实参数来源

```text
纵荡:  du = (v_ref_effective − u) / tau_u          # 一阶惯性
偏航:  desired_r = clip(k_r · wrap(course_ref − course), ±r_max)
       dr = (desired_r − r) / tau_r                 # 一阶惯性（等效 Nomoto 降阶）
横荡:  dv = −v / tau_v
运动学: ẋ = u·cosψ − v·sinψ ; ẏ = u·sinψ + v·cosψ ; ψ̇ = r
course = ψ + β
```

| 参数 | 初值 | 来源 | 后续校准 |
|------|------|------|---------|
| `tau_u` | 0.79 s | huamo.cpp:7-8 `A=0.793023`（一阶时间常数）；稳态 `u = B/A·thrust%`，`B·100 ≈ 2.36 m/s` 与实艇 speed_max=2 吻合 | system_identification 加减速数据拟合 |
| `tau_r` | 0.53 s | sys_ident.cpp Nomoto `T≈0.5308`（代码内置目标值，实艇试验日志覆盖） | Z 形试验 |
| `k_r` | 0.8 s⁻¹ | 使 60° 航迹误差约 1.3s 内转完；与 huamo 转弯行为量级一致 | shadow 数据 |
| `r_max` | 0.6 rad/s | 实艇小艇合理量级，保守取值 | shadow 数据 |
| `tau_v` | 0.5 s | 经验值 | — |
| 控制延迟 | 0~2 步 (10Hz) | 域随机化覆盖 | — |

**域随机化（每 episode）**：`tau_u/tau_r ±40%`、`r_max ±20%`、`k_r ±30%`、恒定漂流 0~0.3 m/s、扇区噪声 σ=0~1m、扇区丢帧 0~10%、观测噪声、控制延迟 0~2 步。

**路径生成**：随机航点 + 三次样条，0.5m 间距（与 A* 栅格分辨率一致）；难度梯度按任务说明 §十四的 Stage 1→6 执行。

**训练栈**：Python + Gymnasium + stable-baselines3 SAC，SubprocVecEnv 8 并行，SB3 默认超参起步（lr 3e-4, buffer 500k, batch 256, net [256,256]），checkpoint + EvalCallback；训练在服务器/PC，不在 Jetson。

---

## 10. 参数汇总（config/rl_planner.yaml 结构）

```yaml
guidance: { dv_max: 0.10, dcourse_max: 0.05, v_min: 0.2, v_max: 2.0,
            cte_reinit: 6.0, course_dev_max: 1.047 }
safety:   { slow_zone: 8.0, crawl_zone: 3.0, critical_zone: 1.5,
            crawl_speed: 0.4, watchdog_ms: 200 }
obs:      { n_sectors: 36, sector_range: 20.0, h_min: -0.3, h_max: 1.0,
            la1: 6.0, la2: 15.0 }
node:     { mode: shadow, control_rate_hz: 10, model_path: models/sac_usv_v1.pt }
```

---

## 11. 文件清单

| 操作 | 路径 | 内容 |
|------|------|------|
| 新建 | `message_ws/src/guidance/rl_local_planner/`（含 `msg/NavCmd.msg`、`rl_common/`） | §2、§4、§14 |
| 修改 | `huamo_node.cpp` + `CMakeLists.txt` + `package.xml`（Phase 5 集成时才动） | §3，约 40 行 |
| 新建 | `rl_local_planner_training/`（仓库根，仅文件；在训练机运行） | §14 |
| 新建 | `rl_training/`（仓库根，不进 ROS ws） | `env/`(dynamics, lidar_sim, path_gen, obs_builder, reward), `train_sac.py`, `evaluate.py`, `export_torchscript.py`, `scripts/plot_metrics.py` |
| 不改 | thruster_manager / usart / tcp / 传感器 / FAST-LIO / 两个规划器 / los_guidance / follow_controller / adjust_pid | — |

## 12. Shadow 模式与验收指标

- Shadow 记录：同一时刻的 LOS (v, course) 与 RL (v, course)、观测向量、supervisor 标志；离线脚本出对比图与统计。
- 指标（对齐任务说明 §十六）：成功/碰撞率、最小间隙、CTE RMSE、heading RMSE、加速度 RMS、|Δcourse| RMS、yaw rate RMS、行程时间、最大/平均速度、最大加速度。
- 对照组：huamo LOS 原样输出（不改它，保证基线纯净）。

## 13. 待实艇/实机确认项（Phase 5-7 验证）

1. `/cloud_registered_body` 高度带阈值与扇区化效果（水面杂波、浪花）。
2. `r` 取 `twist.angular.z` 与 huamo 差分值的一致性（符号/偏置）。
3. ENU→huamo 航向换算的实证（LOS 前视点方向 vs `desired_heading−π`）。
4. `tau_u/tau_r/k_r` 用 shadow 数据回归修正。
5. 多发布者纪律：point_nav_node 与 huamo_node 不得同时运行（现状已有此风险，RL 不新增，但操作规程必须写明）。
6. **传感器体轴系标定**：`/cloud_registered_body` 与 /Odometry yaw 定义的"体轴"可能相对艇体首向存在固定安装旋转（huamo 的 +91.5° 偏置即此类问题的补丁）。节点预留 `hull_to_sensor_yaw` 参数（默认 0），Phase 5 shadow 期用已知障碍物方位标定；不标定则仿真与实艇的扇区索引整体错位。

---

## 14. 运行环境划分（2026-09-17 起生效）

| 环境 | 用途 | 禁止 |
|------|------|------|
| 小艇 Jetson NX（当前 SSH） | 阅读真实工程、rl_common、rl_local_planner_node、Shadow Mode、Actor 推理、实艇测试 | **任何 SAC 训练 / model.learn / 大规模 evaluation** |
| 训练机（本机/GPU 工作站） | Gymnasium 环境、SAC 训练、Fossen 动力学、大规模 evaluation | — |

- 共用契约：`rl_common/`（ObservationBuilder / PathFeatures / LidarSectorizer / Normalization / ActionProcessor / Safety 规则）在 NX 上开发，经 `sync_rl_common.sh` 同步到训练工程，两端禁止各写一套。
- 模型流向：训练机 SAC → best actor → `actor.pt`（TorchScript）→ 拷贝到 NX `models/` → 节点仅推理。
- 训练工程脚手架见 `rl_local_planner_training/`（NX 上只保留文件）。
