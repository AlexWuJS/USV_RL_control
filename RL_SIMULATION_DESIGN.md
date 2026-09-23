# 仿真环境设计

## 结论：训练用自研轻量仿真，验证用 Gazebo

| | 自研 Python 仿真 | Gazebo |
|---|---|---|
| 用途 | **训练** (几百万步) | **验证** (几百条 episode) |
| 速度 | ~0.1ms/步，8 进程并行 ≈ 80k 步/秒 | ~50ms/步，单进程 ≈ 20 步/秒 |
| 2M 步耗时 | ~30 秒 | ~28 小时 |
| 物理精度 | 中（3-DOF 动力学模型） | 高（刚体 + 浮力 + 流体） |
| 传感器模拟 | 几何射线投射（快） | 物理级 LiDAR 插件（慢） |
| 并行化 | 简单（SubprocVecEnv） | 困难 |

**Gazebo 太慢了，不能用来训练 RL。**但可以用来做部署前的最后一轮验证。

---

## 自研仿真环境架构

```
usv_sim/
├── __init__.py
├── env.py              # Gym 环境主类
├── dynamics.py         # 运动学 + 动力学
├── lidar_sim.py        # 几何射线投射，模拟 2D 激光
├── obstacles.py        # 障碍物生成
├── path_gen.py         # 随机路径 + 航点生成
├── renderer.py         # matplotlib 可视化 (调试用)
├── domain_random.py    # 域随机化采样
└── test_sim.py         # 手动控制测试仿真
```

---

## 一、动力学模型 (`dynamics.py`)

### 状态向量

```
state = [x, y, psi, u, v, r]

x, y   : 世界坐标 (m)
psi    : 艏向角 (rad)
u      : 纵荡速度 (m/s, 船头方向)
v      : 横荡速度 (m/s, 船右侧方向)
r      : 偏航角速度 (rad/s)
```

### 动力学方程

直接复用项目里已有的参数和模型结构：

```python
import numpy as np

class USVDynamics:
    def __init__(self):
        # === 偏航动力学 (Nomoto 模型) ===
        # 来源: sys_ident.cpp, 参数由实船试验辨识
        self.K  = 0.56      # 回转性指数
        self.T  = 0.53      # 跟从性指数 (s)
        self.n3 = 0.0       # 非线性阻尼 (初始可忽略)
        self.delta_r = 0.0  # 不对称修正 (rad)

        # === 纵荡动力学 ===
        # 来源: pid_controller.cpp:170
        # damping = (51.3 + 72.4 * |u|) * u
        self.d1 = 51.3      # 线性阻尼系数
        self.d2 = 72.4      # 二次阻尼系数
        self.mass = 80.0    # 有效质量 (kg), 需实测标定

        # === 横荡动力学 (被动) ===
        self.sway_damping = 3.0  # 横荡阻尼

        # === 推力 ===
        self.d = 1.02  # 推进器到中线的横向距离 (m)
        # T = [[1, 1], [-d, d]],  T⁻¹ = [[0.5, -0.5/d], [0.5, 0.5/d]]

    def step(self, state, action, dt=0.1):
        """
        state: [x, y, psi, u, v, r]
        action: [tau_surge, tau_yaw]  (RL 输出)
        返回: new_state
        """
        x, y, psi, u, v, r = state
        tau_surge, tau_yaw = action

        # --- 纵荡 ---
        damping = self.d1 * u + self.d2 * abs(u) * u
        du_dt = (tau_surge - damping) / self.mass

        # --- 横荡 (被动衰减) ---
        dv_dt = -self.sway_damping * v

        # --- 偏航 (Nomoto) ---
        # tau_yaw → 等效舵角
        delta = tau_yaw_to_rudder_angle(tau_yaw, u)  # 下详
        dr_dt = (self.K * (delta + self.delta_r) - r - self.n3 * r**3) / self.T

        # --- 欧拉积分 ---
        u_new = u + du_dt * dt
        v_new = v + dv_dt * dt
        r_new = r + dr_dt * dt

        # --- 运动学 (世界系) ---
        psi_new = psi + r_new * dt
        x_new = x + (u_new * np.cos(psi_new) - v_new * np.sin(psi_new)) * dt
        y_new = y + (u_new * np.sin(psi_new) + v_new * np.cos(psi_new)) * dt

        return np.array([x_new, y_new, psi_new, u_new, v_new, r_new])
```

### tau_yaw → 等效舵角的转换

差速推进器的偏航力矩和等效舵角之间的关系：

```python
def tau_yaw_to_rudder_angle(tau_yaw, u):
    """
    将偏航力矩转为 Nomoto 模型的等效舵角。

    物理直觉:
      - tau_yaw = 左右推力差 × 力臂 d
      - 舵产生的侧向力 ≈ 0.5 * ρ * A * C_L * u² * δ
      - 所以 δ ≈ tau_yaw / (d * k_rudder * u²)

    简化:
      - 用一个标定系数把 tau_yaw 映射到 δ
    """
    if abs(u) < 0.1:
        # 低速时舵效很差，但差速推进器仍然有效
        # 用经验系数直接映射
        return tau_yaw * 0.001  # rad / (N·m)
    else:
        # 高速时舵效正比于 u²
        return tau_yaw / (2000.0 * u * u)  # 2000 是标定系数
```

---

## 二、LiDAR 模拟 (`lidar_sim.py`)

用**几何射线投射**模拟 360° 2D 激光雷达：

```python
def simulate_lidar(x, y, psi, obstacles, n_beams=32, max_range=15.0):
    """
    从船的位置发射 n_beams 条射线，返回每条射线上的最近障碍物距离。

    obstacles: [(ox, oy, radius), ...]

    返回: np.array (32,) 归一化到 [0, 1]
    """
    ranges = np.full(n_beams, max_range)

    for i in range(n_beams):
        angle = psi + (2 * np.pi * i / n_beams)  # 世界坐标系下的射线角度
        dx = np.cos(angle)
        dy = np.sin(angle)

        for ox, oy, radius in obstacles:
            # 射线-圆求交 (标准几何计算)
            dist = ray_circle_intersection(x, y, dx, dy, ox, oy, radius)
            if dist < ranges[i]:
                ranges[i] = dist

    # 归一化: 0 = 紧贴, 1 = max_range 以外
    return np.clip(ranges / max_range, 0.0, 1.0)


def ray_circle_intersection(rx, ry, dx, dy, cx, cy, r):
    """射线 (rx,ry) + t*(dx,dy) 与圆 (cx,cy,r) 的交点参数 t"""
    fx = rx - cx
    fy = ry - cy
    a = dx*dx + dy*dy
    b = 2 * (fx*dx + fy*dy)
    c = fx*fx + fy*fy - r*r
    disc = b*b - 4*a*c
    if disc < 0:
        return float('inf')
    t1 = (-b - np.sqrt(disc)) / (2*a)
    t2 = (-b + np.sqrt(disc)) / (2*a)
    if t1 > 1e-6:
        return t1
    elif t2 > 1e-6:
        return t2
    return float('inf')
```

对于 32 条射线、10 个圆形障碍物：约 320 次射线-圆求交，耗时 < 0.01ms。

---

## 三、障碍物生成 (`obstacles.py`)

```python
def spawn_obstacles(path, n_range=(1, 8)):
    """沿路径随机生成障碍物，避开路径本身"""
    n = np.random.randint(*n_range)
    obstacles = []
    for _ in range(n):
        while True:
            ox = np.random.uniform(5, 75)
            oy = np.random.uniform(-25, 25)
            radius = np.random.uniform(1.0, 3.0)

            # 检查是否挡住了路径（路径附近 3m 内不放置）
            too_close = False
            for px, py in path:
                if np.hypot(ox - px, oy - py) < 3.0 + radius:
                    too_close = True
                    break
            if not too_close:
                break

        obstacles.append((ox, oy, radius))
    return obstacles
```

---

## 四、环境包装与并行化

```python
from stable_baselines3.common.vec_env import SubprocVecEnv

def make_env(seed, difficulty="easy"):
    def _init():
        env = USVObstacleEnv(seed=seed, difficulty=difficulty)
        return env
    return _init

# 8 个独立进程，每个有不同的随机种子
env = SubprocVecEnv([make_env(i) for i in range(8)])
```

每个进程独立运行一个仿真世界，共享同一个 SAC 模型，并行收集经验。

---

## 五、调试可视化 (`renderer.py`)

训练时不渲染（浪费算力）。调试时用 matplotlib 看单 episode：

```python
def render(self, ax):
    ax.clear()
    # 路径
    px, py = zip(*self.path)
    ax.plot(px, py, 'g--', linewidth=1, label='path')

    # 障碍物
    for ox, oy, r in self.obstacles:
        circle = plt.Circle((ox, oy), r, color='red', alpha=0.5)
        ax.add_patch(circle)

    # 船 (三角形)
    L = 1.2  # 船长
    bow_x = self.x + L * np.cos(self.psi)
    bow_y = self.y + L * np.sin(self.psi)
    stern_x = self.x - 0.3 * L * np.cos(self.psi)
    stern_y = self.y - 0.3 * L * np.sin(self.psi)
    ax.plot([stern_x, bow_x], [stern_y, bow_y], 'b-', linewidth=3)

    # LiDAR 射线
    for i, d in enumerate(self.lidar_ranges):
        angle = self.psi + 2*np.pi*i/32
        ax.plot([self.x, self.x + d*15*np.cos(angle)],
                [self.y, self.y + d*15*np.sin(angle)],
                'c-', linewidth=0.3, alpha=0.5)

    ax.set_xlim(self.x - 25, self.x + 25)
    ax.set_ylim(self.y - 25, self.y + 25)
    ax.set_aspect('equal')
    ax.grid(True)
```

---

## 六、Gazebo 验证（部署前确认）

项目已有基础设施可复用：

```
USV_description/urdf/ship.urdf.xacro   — URDF 模型 (需加惯性+collision)
USV_nav/config/                         — costmap 参数、TEB 参数
USV_nav/launch/move_base.launch         — move_base 启动文件
```

验证流程：

1. **用 Gazebo 跑几十条 episode**（不是训练，是验证）
2. 加载训练好的 ONNX 模型 → 通过 `rl_local_planner` 节点推理
3. 对比 Gazebo 中的性能和 Python 仿真中的性能
4. 如果差距大 → 增强域随机化 → 重新训练

**注意**：Gazebo 验证是可选的。如果简单 Python 仿真 + 域随机化已经足够好，可以跳过 Gazebo，直接在实船上低速测试。

---

## 七、核心设计原则

### 1. 不追求高保真，追求覆盖率

仿真不必精确匹配实船。域随机化（每个 episode 随机扰动参数）比精确建模更有效：

```python
# 每个 episode 随机采样这些参数
class DomainRandomizer:
    def sample(self):
        return {
            "K":     np.random.uniform(0.3, 0.8),     # Nomoto K ±40%
            "T":     np.random.uniform(0.3, 0.8),     # Nomoto T ±40%
            "d1":    np.random.uniform(30, 80),        # 线性阻尼 ±50%
            "d2":    np.random.uniform(40, 110),       # 二次阻尼 ±50%
            "mass":  np.random.uniform(50, 120),       # 有效质量 ±40%
            "delay": np.random.randint(0, 10),         # 控制延迟 0~10 步
            "current_x": np.random.uniform(-20, 20),   # 恒定海流 x 分量
            "current_y": np.random.uniform(-20, 20),   # 恒定海流 y 分量
            "obs_noise": np.random.uniform(0.01, 0.1), # 观测噪声标准差
        }
```

RL 的策略会因为这种随机化而学会"鲁棒控制"——不管实际参数是什么，都能稳定跟踪路径。

### 2. 动作空间选力而不是 PWM

之前讲的 `[τ_surge, τ_yaw]` 是物理量。仿真里用牛顿力学更新，实船上把力映射到 PWM（推力标定很容易，定速跑几趟就标出来了）。

如果是直接输出 PWM：仿真里就要精确建模 PWM→推力→速度 的完整链条，任何一个环节不准，策略就不能迁移。

### 3. 不求一次完美

训练过程是：

```
训练 2M 步 → 实船低速测试 → 收集问题场景
→ 在仿真里复现 → 针对训练 → 再测 → 迭代
```

前两次迭代模型就能用了，后面是打磨。
