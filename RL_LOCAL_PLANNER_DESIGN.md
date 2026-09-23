# SAC 强化学习局部规划器（避障）设计方案

## 一、插入位置

把 RL 放在全局路径和底层推力分配之间，直接**替代 LOS 引导律 + PID 控制器**：

```
全局路径 (/waypoints_path)
         │
         ▼
┌─────────────────────────────────────────┐
│         SAC 局部规划器（新增）            │
│                                         │
│  观测 → Actor 网络 → [τ_surge, τ_yaw]    │
│                                         │
│  输入:                                   │
│    · 全局路径 (nav_msgs/Path)            │
│    · 里程计 (位置/速度/姿态)              │
│    · 激光雷达点云 (避障用)                │
│                                         │
│  输出:                                   │
│    · τ_surge  (纵荡力, N)               │
│    · τ_yaw    (偏航力矩, N·m)            │
└─────────────────────────────────────────┘
         │
         ▼
推力分配矩阵 T⁻¹ (复用 pid_controller.cpp)
    T = [[1, 1], [-1.02, 1.02]]
    [F_left, F_right] = T⁻¹ · [τ_surge, τ_yaw]
         │
         ▼
PWM 映射 → /auto_thruster_pwm → thruster_manager → 硬件
```

**为什么输出力而不是速度/航向？**
1. 更接近船的物理响应，RL 学得更直接
2. 推力分配矩阵 T⁻¹ 已在 `pid_controller.cpp:178` 中验证过
3. 跳过了 PID 级联带来的延迟和调参

**为什么输出力而不是直接 PWM？**
1. 力和 PWM 之间是线性的（加阻尼补偿），加一层简单映射反而便于 sim-to-real 迁移——仿真里用牛顿力学，实船上校准 PWM→力 的系数即可
2. `speed2SurgeForce()` 里已有阻尼模型 `(51.3+72.4*|v|)*v`，可直接复用

---

## 二、观测空间（输入）

总共 **48 维**，分四组：

### 2.1 路径跟踪 (10 维)

| 索引 | 含义 | 计算方式 |
|------|------|---------|
| 0 | cross_track_error | 同 `set_of_los.cpp:70` 的 `track_errors_p(1)` |
| 1 | heading_error | 路径切线方向 − 当前 heading，归一化到 [−π, π] |
| 2-3 | lookahead_1 (x,y) | 路径上前瞻 3m 的点，转到**船体坐标系** |
| 4-5 | lookahead_2 (x,y) | 前瞻 6m |
| 6-7 | lookahead_3 (x,y) | 前瞻 10m |
| 8 | path_curvature_near | 前瞻 0~5m 区段的平均曲率 |
| 9 | dist_to_goal | 到路径终点的剩余距离，归一化 |

前瞻点转到船体坐标系的好处：RL 学到的策略对船的绝对位置不敏感，只关心"路在船的哪个方向"，泛化能力更强。

### 2.2 自身状态 (5 维)

| 索引 | 含义 |
|------|------|
| 10 | surge 速度 u (m/s) |
| 11 | sway 速度 v (m/s) |
| 12 | yaw_rate r (rad/s) |
| 13 | 当前速度模长 |
| 14 | 漂角 β = atan2(v, u) |

参考 `pid_controller.cpp:139` 的 `getVelInBodyFrame()` 来获取 u 和 v。

### 2.3 障碍物 (32 维)

从 LiDAR 点云或 costmap 中提取。用**极坐标采样**而非原始点云，减少维度同时保留空间信息：

```
在船体坐标系下，以船头方向为 0°：
  · 32 个角度 bin（0° ~ 360°，每 11.25° 一个）
  · 每个 bin 取该方向 [0, 15m] 范围内的最近障碍物距离
  · 归一化到 [0, 1]（0 = 紧贴, 1 = 15m 外/无障碍）
```

这 32 维也可以用 2D laser scan 的 `ranges` 直接下采样得到。项目用的 MID-360 是 360° 激光雷达，天然适合这种表示。

### 2.4 目标速度 (1 维)

| 索引 | 含义 |
|------|------|
| 47 | desired_cruise_speed，来自全局路径或参数 |

---

## 三、动作空间（输出）

**2 维连续值**，对应 SAC 的 Gaussian policy：

| 索引 | 含义 | 范围 |
|------|------|------|
| 0 | τ_surge 纵荡力 | [−200, 800] N |
| 1 | τ_yaw 偏航力矩 | [−500, 500] N·m |

SAC 输出的是高斯分布的 mean 和 std。推理时直接用 mean（确定性策略），训练时从分布中采样。

PWM 映射（接在 RL 输出之后）：

```cpp
// 1. 推力分配: 力 → 左右推力 (复用 pid_controller.cpp:178)
Eigen::Matrix2d T;
T << 1, 1, -1.02, 1.02;
Eigen::Vector2d thrust = T.inverse() * Eigen::Vector2d(tau_surge, tau_yaw);

// 2. 推力 → PWM (线性映射 + 阻尼补偿，参考 pid_controller.cpp:170)
//    阻尼力 ≈ (51.3 + 72.4 * |u|) * u
double left_pwm  = 1500 + (thrust[0] + damping_force/2) * pwm_per_newton;
double right_pwm = 1500 + (thrust[1] + damping_force/2) * pwm_per_newton;

// 3. 钳制到 [1200, 1800]
int left  = clamp(left_pwm,  1200, 1800);
int right = clamp(right_pwm, 1200, 1800);
```

---

## 四、奖励函数

```python
reward = (
    # 路径跟踪（主导项）
    -0.5 * abs(cross_track_error)          # 偏离路径惩罚
    -0.3 * abs(heading_error)              # 朝向偏差惩罚

    # 避障（安全第一）
    -2.0 * max(0, 1.0 - min_obstacle_dist) # 距障碍物 < 1m 时重罚
    -0.2 * max(0, 3.0 - min_obstacle_dist) # 距障碍物 < 3m 时轻微惩罚

    # 速度跟踪
    -0.1 * abs(current_speed - desired_speed)

    # 平滑性
    -0.05 * sum((action_t - action_{t-1}) ** 2)  # 动作平滑
    -0.02 * abs(yaw_rate)                         # 角速度惩罚（防蛇形）

    # 进度
    +1.0 * along_track_progress               # 沿路径前进的奖励

    # 终止
    +20.0  if reached_goal                    # 到达终点
    -50.0  if collision or cte > 10.0         # 碰撞或严重偏离
)
```

**关键设计思路：**
- 避障项权重 2.0 > 路径跟踪 0.5，保证安全优先级
- `min_obstacle_dist < 1m` 是硬约束，`< 3m` 是软约束，形成梯度引导
- 进度奖励推动船往前走，避免在障碍物前停滞

---

## 五、训练管线

### 5.1 仿真环境（Python + Gym）

```
USV Simulator (Gym Env)
├── 运动学模型 (3-DOF)
│   ├── 状态: [x, y, ψ, u, v, r]
│   ├── 控制: [τ_surge, τ_yaw]
│   └── 动力学参数来自 system_identification 包
├── 障碍物
│   ├── 静态: 随机放置的圆形/矩形障碍物
│   ├── 动态: 匀速移动的障碍物（代表其他船只）
│   └── 用 occupancy grid 或直接几何计算最近距离
├── 路径生成
│   ├── 随机起点和终点
│   ├── A* 或 Hybrid A* 生成参考路径
│   └── 路径场景: 直线 / 弯道 / 狭窄通道
└── 扰动
    ├── 高斯噪声加在状态观测上 (模拟传感器噪声)
    ├── 随机恒定扰动加在 τ 上 (模拟风/流)
    └── 控制延迟 0~100ms 随机
```

### 5.2 训练配置

```python
import gymnasium as gym
from stable_baselines3 import SAC
from stable_baselines3.common.noise import NormalActionNoise

# 网络结构
policy_kwargs = dict(
    net_arch=dict(
        pi=[256, 256],      # Actor: 2 层 MLP
        qf=[256, 256]       # Critic: 2 层 MLP (twin Q)
    )
)

model = SAC(
    "MlpPolicy",
    env,
    policy_kwargs=policy_kwargs,
    learning_rate=3e-4,
    buffer_size=1_000_000,    # 大 buffer 对 off-policy 很重要
    batch_size=256,
    tau=0.005,                # 目标网络软更新系数
    gamma=0.99,
    train_freq=1,
    gradient_steps=1,
    ent_coal="auto",          # 自动熵调节（SAC 的核心优势）
    verbose=1,
)

model.learn(total_timesteps=2_000_000)
model.save("sac_usv_obstacle_avoidance")
```

### 5.3 课程学习（Curriculum Learning）

分阶段训练，逐步增加难度：

| 阶段 | 环境 | 目标 |
|------|------|------|
| 1 (0~500k) | 无障碍物，直线路径 | 学会基本路径跟踪 |
| 2 (500k~1M) | 1-3 个静态障碍物，弯道路径 | 学会避障 + 跟踪 |
| 3 (1M~2M) | 5-10 个障碍物，部分动态 | 学会复杂场景 |
| 4 (可选) | 狭窄通道、密集障碍物 | 边界能力 |

### 5.4 域随机化（Domain Randomization）

每 episode 随机采样以下参数，提高 sim-to-real 迁移能力：

```python
# 动力学参数随机化
surge_damping_scale  ~ Uniform(0.7, 1.3)   # 阻尼变化 ±30%
yaw_damping_scale    ~ Uniform(0.7, 1.3)
thrust_efficiency    ~ Uniform(0.8, 1.2)   # 推进器效率

# 传感器噪声
obs_noise_std        ~ Uniform(0.01, 0.1)  # 观测噪声

# 环境扰动
current_force        ~ Uniform(-50, 50)    # 模拟海流
control_delay_steps  ~ Uniform(0, 10)      # 控制延迟 0~100ms (10Hz 控制)
```

---

## 六、部署到 Jetson

### 6.1 模型导出

```python
# 导出 SAC 的 Actor 网络为 ONNX
import torch

class SACActorWrapper(torch.nn.Module):
    """包装 Actor，只暴露推理逻辑"""
    def __init__(self, actor):
        super().__init__()
        self.actor = actor

    def forward(self, obs):
        # SAC actor 输出 Gaussian 的 mean，推理时用确定性策略
        return self.actor.get_action(obs, deterministic=True)

actor = SACActorWrapper(model.actor)
dummy_input = torch.randn(1, 48)  # 48 维观测

torch.onnx.export(
    actor,
    dummy_input,
    "sac_usv_planner.onnx",
    input_names=["observation"],
    output_names=["action"],
    opset_version=13,
)
```

### 6.2 ROS 节点结构

新建包：`message_ws/src/guidance/planner/rl_local_planner/`

```
rl_local_planner/
├── CMakeLists.txt          # 依赖 ONNX Runtime, ros, Eigen
├── package.xml
├── include/rl_local_planner/
│   ├── sac_planner.h       # 主类
│   ├── observation_builder.h  # 观测向量构建
│   └── pwm_mapper.h        # τ → PWM 映射
├── src/
│   ├── sac_planner_node.cpp    # main()
│   ├── sac_planner.cpp         # 100Hz 主循环
│   ├── observation_builder.cpp # 路径采样 + LiDAR 处理
│   └── pwm_mapper.cpp
├── launch/
│   └── sac_local_planner.launch
├── models/
│   └── sac_usv_planner.onnx
└── scripts/
    └── train.py             # 训练脚本
```

### 6.3 推理主循环（100Hz）

```cpp
void SACPlanner::controlLoop(const ros::TimerEvent&) {
    // 1. 检查全局路径是否有效
    if (global_path_.poses.size() < 2) {
        publishStop();
        return;
    }

    // 2. 构建观测向量 (48 维)
    std::vector<float> obs(48);

    // 2a. 找到路径最近点 (复用 los_pid.cpp:100 的逻辑)
    auto closest = findClosestPoint(global_path_, current_pose_);

    // 2b. 计算 cross_track_error (复用 set_of_los.cpp:69-70)
    double cte = computeCrossTrackError(current_pose_, closest);

    // 2c. 采样前瞻点并转换到船体坐标
    auto lookahead = sampleLookaheadPoints(global_path_, closest, {3, 6, 10});

    // 2d. 处理 LiDAR 数据 → 32 维距离向量
    auto lidar_ranges = processLidarToRanges(latest_pointcloud_, 32, 15.0);

    // 2e. 组装观测向量
    fillObservation(obs, cte, lookahead, lidar_ranges, current_vel_, desired_speed_);

    // 3. ONNX Runtime 推理
    auto action = ort_inference(obs);  // → [τ_surge, τ_yaw]

    // 4. 安全检查：如果动作会导致碰撞，覆盖为紧急避障
    if (minObstacleDist(lidar_ranges) < 0.5) {
        action = emergencyStop();  // 紧急制动
    }

    // 5. τ → PWM
    auto [left_pwm, right_pwm] = forceToPwm(action[0], action[1], current_vel_);

    // 6. 发布
    thruster_manager::Thruster msg;
    msg.left_thruster_pwm  = left_pwm;
    msg.right_thruster_pwm = right_pwm;
    pwm_pub_.publish(msg);
}
```

---

## 七、安全策略（多层降级）

```
Layer 1: RL 推理超时 (> 20ms)
  → 自动切到 LOS+PID

Layer 2: 碰撞风险检测 (min_dist < 0.5m)
  → 紧急制动（两推进器同时全速倒车）

Layer 3: 跟踪偏差过大 (cte > 10m)
  → 切到 LOS+PID 先把船拉回路径上

Layer 4: 模型/ORT 加载失败
  → 启动时就 fallback 到 LOS+PID

Layer 5: 任何时候遥控器接管
  → thruster_manager 直接切到 joystick 模式 (已有功能)
```

所有切换都通过 `ship_info.is_auto_control` 和 thruster_manager 已有的优先级逻辑保证。

---

## 八、实现路线图

| 步骤 | 内容 | 预计产出 |
|------|------|---------|
| 1 | 写 USV 仿真环境 (Python, Gym) | 可以训练的基础环境 |
| 2 | 无避障 SAC 训练 + 验证 | SAC 能跟踪直线/弯道路径 |
| 3 | 加障碍物 + 避障奖励，重新训练 | SAC 能绕开静态障碍物 |
| 4 | 导出 ONNX + C++ 推理节点 | ROS 节点跑通 dummy 推理 |
| 5 | ROS bag 回放测试（离线） | 用实船数据验证观测构建正确 |
| 6 | 实船静水测试（无避障） | RL 跟踪 vs LOS+PID 对比 |
| 7 | 实船避障测试 | 放浮标做障碍物，验证避障 |
| 8 | 迭代优化 | 收集实船数据，微调模型 |

---

## 九、需要修改的现有文件

| 操作 | 文件 | 改动 |
|------|------|------|
| **新建** | `guidance/planner/rl_local_planner/` | 整个 SAC 规划器包 |
| **新建** | `guidance/planner/rl_local_planner/scripts/train.py` | 训练脚本 |
| 修改 | `control/adjust_pid/launch/point_nav.launch` | 加一个 `<arg name="use_rl" default="false"/>` 开关 |
| 不修改 | 其他所有文件 | RL 规划器通过同样的 `/auto_thruster_pwm` 发指令 |

现有代码中可以直接**复用**的部分：
- `pid_controller.cpp:178` — `thrustAllocation()` 矩阵 T⁻¹
- `pid_controller.cpp:170` — 阻尼模型 `(51.3+72.4*|v|)*v`
- `set_of_los.cpp:69-70` — cross-track error 计算公式
- `los_pid.cpp:100-107` — 路径最近点查找逻辑
- `thruster_manager_node.cpp:54-56` — PWM 接收接口已就绪
