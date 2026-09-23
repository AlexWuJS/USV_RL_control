# SAC 训练方案

## 一、从哪里开始

不需要从头写训练框架。用 **stable-baselines3** 的 SAC 实现，只需要写一个 **Gym 仿真环境**。

```
训练栈:
  stable-baselines3 (SAC 实现)
      │
      ▼
  gymnasium.Env (自定义 USV 环境)
      │
      ├── 运动模型 (用项目已有的系统辨识参数)
      ├── 障碍物生成器
      ├── 路径生成器
      └── 奖励计算
```

---

## 二、仿真运动模型

不用从零推导，直接用项目里已经有的参数。

### 2.1 偏航（转向）动力学 — Nomoto 模型

`system_identification` 包已经在实船上辨识出了二阶 Nomoto 模型参数 K, T, δr, n3。辨识方法是标准船舶操纵性试验（定常回转 + Z 形试验）。

```
Nomoto 二阶非线性模型:
  T · dr/dt + r + n3 · r³ = K · (δ + δr)

其中:
  r     = yaw rate (rad/s)
  δ     = 等效舵角 (用差速 PWM 换算)
  K     = 回转性指数 (增益)
  T     = 跟从性指数 (时间常数)
  δr    = 舵角偏移 (船体不对称修正)
  n3    = 非线性阻尼系数
```

在仿真里实现：

```python
def yaw_dynamics(r, delta, dt):
    """一步欧拉积分，计算新的 yaw rate"""
    dr_dt = (K * (delta + delta_r) - r - n3 * r**3) / T
    return r + dr_dt * dt
```

等效舵角 δ 和差速 PWM 的关系：

```python
# 差速转等效舵角：左右 PWM 差值 / 归一化系数
pwm_diff = (left_pwm - right_pwm)  # 正值 = 左转
delta = pwm_diff / PWM_DELTA_SCALE  # 标定系数，约 100-200
```

### 2.2 纵荡（前进）动力学 — 阻尼模型

直接从 `pid_controller.cpp:170` 提取：

```python
# pid_controller.cpp 第170行的阻尼模型
# damping = (51.3 + 72.4 * |u|) * u
# τ_surge = m * du/dt + damping
# → du/dt = (τ_surge - damping) / m

def surge_dynamics(u, tau_surge, dt):
    damping = (51.3 + 72.4 * abs(u)) * u
    du_dt = (tau_surge - damping) / MASS  # MASS 需要估计，约 50-100 kg
    return u + du_dt * dt
```

### 2.3 横荡（侧移）动力学

无人艇没有侧推器，横荡是被动的：

```python
def sway_dynamics(v, dt):
    # 一阶阻尼衰减
    dv_dt = -SWAY_DAMPING * v  # SWAY_DAMPING 约 2-5
    return v + dv_dt * dt
```

### 2.4 运动学更新

```python
def step(self, action):
    tau_surge, tau_yaw = action  # RL 输出

    # 1. 动力学
    u = surge_dynamics(self.u, tau_surge, self.dt)
    v = sway_dynamics(self.v, self.dt)
    r = yaw_dynamics(self.r, tau_yaw_to_delta(tau_yaw), self.dt)

    # 2. 运动学 (世界坐标系)
    self.psi += r * self.dt
    self.x  += (u * cos(self.psi) - v * sin(self.psi)) * self.dt
    self.y  += (u * sin(self.psi) + v * cos(self.psi)) * self.dt

    # 3. 更新状态
    self.u, self.v, self.r = u, v, r
```

### 2.5 参数来源总结

| 参数 | 取值 | 来源 |
|------|------|------|
| K, T, δr, n3 | 实船辨识 | `sys_ident.cpp` 输出，具体值看实船试验日志 |
| 纵荡阻尼 | 51.3 + 72.4·\|u\| | `pid_controller.cpp:170` |
| 推力分配矩阵 | [[1,1],[-1.02,1.02]] | `pid_controller.cpp:178` |
| MASS | ~80 kg (估计) | 可以跑一次加速试验拟合 |
| dt | 0.1s (10Hz 控制) | `los_pid.cpp:15` 的定时器周期 |

---

## 三、Gym 环境实现

```python
import gymnasium as gym
import numpy as np
from gymnasium import spaces

class USVObstacleEnv(gym.Env):
    def __init__(self, config=None):
        super().__init__()

        # === 观测空间: 48 维 ===
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(48,), dtype=np.float32
        )

        # === 动作空间: [τ_surge, τ_yaw] ===
        self.action_space = spaces.Box(
            low=np.array([-200.0, -500.0]),
            high=np.array([800.0, 500.0]),
            dtype=np.float32
        )

        # === 动力学参数 (训练时会域随机化) ===
        self.K  = 0.56      # Nomoto K
        self.T  = 0.53      # Nomoto T
        self.n3 = 0.0       # Nomoto n3 (初始可用 0)
        self.delta_r = 0.0  # 不对称修正

        # === 仿真参数 ===
        self.dt = 0.1        # 控制周期 10Hz
        self.max_steps = 500 # 最长 50 秒

    def reset(self, seed=None):
        # 1. 随机生成路径 (起点 + 终点 → A* 规划)
        self.path = self._generate_path()

        # 2. 随机放置障碍物
        self.obstacles = self._spawn_obstacles(n=random.randint(1, 8))

        # 3. 初始化船状态
        self.x, self.y = self.path[0]
        self.psi = self._path_initial_heading()
        self.u, self.v, self.r = 0.0, 0.0, 0.0

        # 4. 返回观测
        return self._get_obs(), {}

    def step(self, action):
        tau_surge, tau_yaw = action

        # 域随机化噪声
        tau_surge += np.random.normal(0, 5.0)
        tau_yaw  += np.random.normal(0, 3.0)

        # 运动学更新
        self._update_dynamics(tau_surge, tau_yaw)

        # 计算奖励
        reward = self._compute_reward()

        # 判断终止
        done = self._check_done()
        truncated = self.steps >= self.max_steps

        self.steps += 1
        return self._get_obs(), reward, done, truncated, {}
```

### 3.1 路径生成

```python
def _generate_path(self):
    """生成一条从左侧到右侧的随机弯道路径"""
    start_x = 0.0
    start_y = np.random.uniform(-20, 20)
    goal_x  = 80.0
    goal_y  = np.random.uniform(-20, 20)

    # 加一些中间航点制造弯曲
    waypoints = [
        (start_x, start_y),
        (start_x + 20, start_y + np.random.uniform(-10, 10)),
        (start_x + 40, start_y + np.random.uniform(-15, 15)),
        (start_x + 60, goal_y + np.random.uniform(-5, 5)),
        (goal_x, goal_y),
    ]

    # 用三次样条插值成密集路径点 (0.5m 间距)
    return cubic_spline_interpolate(waypoints, spacing=0.5)
```

### 3.2 障碍物生成

```python
def _spawn_obstacles(self, n):
    obstacles = []
    for _ in range(n):
        # 随机位置，但要避开路径起点附近
        ox = np.random.uniform(10, 70)
        oy = np.random.uniform(-25, 25)
        radius = np.random.uniform(1.0, 3.0)  # 1-3m 半径
        obstacles.append((ox, oy, radius))
    return obstacles
```

### 3.3 最近障碍物距离计算

```python
def _min_obstacle_distance(self):
    """计算船到最近障碍物边缘的距离"""
    min_dist = float('inf')
    for ox, oy, radius in self.obstacles:
        d = np.sqrt((self.x - ox)**2 + (self.y - oy)**2) - radius
        min_dist = min(min_dist, d)
    return max(0.0, min_dist)
```

---

## 四、训练流程

### 4.1 环境包装

```python
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback

def make_env(rank, config):
    def _init():
        env = USVObstacleEnv(config)
        return env
    return _init

if __name__ == "__main__":
    # 多进程并行 (利用多核加速)
    n_envs = 8
    env = SubprocVecEnv([make_env(i, config) for i in range(n_envs)])

    # 单独一个环境做评估 (不随机化)
    eval_env = SubprocVecEnv([make_env(99, eval_config)])

    # SAC 配置
    model = SAC(
        "MlpPolicy",
        env,
        policy_kwargs=dict(
            net_arch=dict(pi=[256, 256], qf=[256, 256])
        ),
        learning_rate=3e-4,
        buffer_size=500_000,
        batch_size=256,
        tau=0.005,
        gamma=0.99,
        ent_coef="auto",
        train_freq=1,
        gradient_steps=1,
        tensorboard_log="./sac_usv_tensorboard/",
        verbose=1,
    )

    # 回调: 每 20000 步保存 checkpoint
    checkpoint_callback = CheckpointCallback(
        save_freq=20000, save_path="./checkpoints/"
    )
    # 回调: 每 10000 步在 eval 环境评估
    eval_callback = EvalCallback(
        eval_env, best_model_save_path="./best_model/",
        eval_freq=10000, n_eval_episodes=20,
    )

    model.learn(
        total_timesteps=2_000_000,
        callback=[checkpoint_callback, eval_callback],
        tb_log_name="sac_usv_v1",
    )

    model.save("sac_usv_final")
```

### 4.2 训练硬件

在服务器上跑（不用 Jetson）：

| 配置 | 推荐 |
|------|------|
| CPU | 8 核以上（用于多进程环境） |
| GPU | 一张 GTX 1060 以上即可 |
| 内存 | 16GB+ |
| 训练时间 | 2M 步约 8-12 小时 |

### 4.3 关键调参经验

| 参数 | 默认 | 建议 | 原因 |
|------|------|------|------|
| `buffer_size` | 1M | 500k | 环境简单，不需要太大 |
| `batch_size` | 256 | 256 | 保持不变 |
| `learning_rate` | 3e-4 | 先 3e-4，不行降到 1e-4 | SAC 对学习率敏感 |
| `tau` | 0.005 | 0.01 | 稍快的目标网络更新 |
| `ent_coef` | "auto" | "auto" | 自动调节，不用手动设 |
| `n_envs` | — | 8 | 8 个并行环境足够 |
| `train_freq` | 1 | 1 | 每步都训练 |
| `gradient_steps` | 1 | 1 | 可以调到 2-4 如果 buffer 不够大 |

### 4.4 监控指标

训练时在 tensorboard 关注：

```
rollout/ep_rew_mean     — episode 平均奖励 (应该持续上升)
rollout/ep_len_mean     — episode 平均步数 (增加说明船不撞了)
train/actor_loss        — Actor 损失 (震荡但总体下降)
train/critic_loss       — Critic 损失 (持续下降)
train/ent_coef          — 熵系数 (应该收敛到 0.01 ~ 0.1)
```

---

## 五、课程训练（关键）

不要一上来就全难度训练，分 3 个阶段：

### 阶段 1: 纯路径跟踪 (0 ~ 500k 步)

```python
config_stage1 = {
    "n_obstacles": 0,             # 没有障碍物
    "path_curvature": "easy",     # 只有直线和缓弯
    "noise_scale": 0.0,           # 无噪声
    "domain_randomization": False,
}
```

**目标**：学出来的策略能沿直线和缓弯跟踪路径，cross-track error < 1m。

**预期**：SAC 应该很快收敛，500k 步内 reward 到正数。

### 阶段 2: 静态避障 (500k ~ 1.5M 步)

```python
config_stage2 = {
    "n_obstacles": (1, 5),       # 1-5 个静态障碍物
    "path_curvature": "medium",  # 中等弯道
    "noise_scale": 0.02,         # 轻微观测噪声
    "domain_randomization": True, # 每个 episode 随机化阻尼/推力参数
}
```

**目标**：能绕开障碍物同时保持路径跟踪，碰撞率 < 5%。

**关键**：这个阶段避障权重起主导作用。如果一直撞，提高碰撞惩罚到 -100。

### 阶段 3: 复杂场景 (1.5M ~ 2M 步)

```python
config_stage3 = {
    "n_obstacles": (3, 10),      # 更多障碍物
    "path_curvature": "hard",    # 急弯 + 窄通道
    "noise_scale": 0.05,         # 真实级别噪声
    "domain_randomization": True,
    "moving_obstacles": True,    # 动态障碍物 (2-3 个)
}
```

**目标**：能处理复杂场景，最终 success rate > 80%。

### 加载前阶段权重继续训练

```python
# 阶段 2 从阶段 1 的模型开始
model = SAC.load("./best_model/best_model.zip", env=env_stage2)
model.learn(total_timesteps=1_500_000)

# 阶段 3 从阶段 2 的模型开始
model = SAC.load("./best_model/best_model.zip", env=env_stage3)
model.learn(total_timesteps=2_000_000)
```

---

## 六、常见问题处理

### Q: 训练完全不收敛

1. 检查奖励尺度 — SAC 需要 reward 在 [-10, 10] 左右，不能太大
2. 降低 `learning_rate` 到 1e-4
3. 检查观测是否做了归一化（路径点/距离应归一化到 [-1, 1] 或 [0, 1]）
4. 先用阶段 1（无障碍）验证基本能力

### Q: 学出来的策略只会走直线，看到障碍物就停

碰撞惩罚太大，或者避障激励不够。可以给 `min_obstacle_dist` 一个小的正向奖励（距离 > 5m 时 +0.1），鼓励保持安全距离。

### Q: 策略抖动严重（蛇形前进）

增加动作平滑惩罚，从 0.05 调到 0.1，或加 yaw_rate 惩罚。

### Q: 仿真和实船差距太大

域随机化不够。增大参数随机化范围：
- 阻尼 ±50%
- 推力效率 ±30%
- 增加随机的恒定侧向力（模拟海流）

---

## 七、模型导出

```python
import torch
import onnx

# SAC 的 actor 网络包含 Gaussian 分布
# 推理时取 mean 作为确定性动作
class SACActorInference(torch.nn.Module):
    def __init__(self, sac_model):
        super().__init__()
        self.actor = sac_model.actor

    def forward(self, obs):
        # SAC actor: obs → latent → mean_std → tanh(action) * action_scale + bias
        mean = self.actor.mu(self.actor.latent_pi(obs))
        return mean  # 确定性，不采样

model = SAC.load("best_model/best_model.zip")
inference_actor = SACActorInference(model)

dummy = torch.randn(1, 48)
torch.onnx.export(
    inference_actor,
    dummy,
    "sac_usv_planner.onnx",
    input_names=["observation"],
    output_names=["action"],
    opset_version=11,
    dynamic_axes={"observation": {0: "batch"}, "action": {0: "batch"}},
)
```

---

## 八、文件结构

```
message_ws/src/guidance/planner/rl_local_planner/
│
├── training/                        # 训练相关 (只在服务器上跑)
│   ├── usv_env.py                   # Gym 环境
│   ├── train_stage1_path_follow.py  # 阶段 1 训练脚本
│   ├── train_stage2_static_obs.py   # 阶段 2 训练脚本
│   ├── train_stage3_complex.py      # 阶段 3 训练脚本
│   ├── eval_trained.py              # 训练后评估/可视化
│   ├── export_onnx.py               # 导出 ONNX
│   └── requirements.txt             # gymnasium, sb3, torch, onnx
│
├── models/                          # 导出的 ONNX 模型
│   └── sac_usv_planner.onnx
│
├── src/                             # C++ 推理节点 (在 Jetson 上编)
│   ├── sac_planner_node.cpp
│   ├── sac_planner.cpp
│   ├── observation_builder.cpp
│   └── pwm_mapper.cpp
│
├── include/rl_local_planner/
│   ├── sac_planner.h
│   ├── observation_builder.h
│   └── pwm_mapper.h
│
├── launch/
│   └── sac_local_planner.launch
│
├── CMakeLists.txt
└── package.xml
```
