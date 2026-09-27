# 水试 + RL 测试 操作手册

> 分支：RL_test ｜ 更新：2026-09-23
> 本次更新：新增"地图门禁"与"地图管理纪律"（先验图被覆盖事故的根因修复）、map 系打点提醒、精简录制方案；删除已一次性完成的标定步骤。

每个新终端先执行：

```bash
source /home/jetson/haique/little-boat-motion-control/message_ws/devel/setup.bash
```

**出发前先看磁盘**（曾因磁盘写满导致地图文件损坏 + bag 只录到 15 秒）：

```bash
df -h /        # 至少留 20G
```

**已完成项（备查，不必重复做）**

| 项目 | 结论 |
|------|------|
| IMU 陀螺校准 | 台面零偏 0.000 rad/s、装回后与里程计 90° 一致性检查通过 |
| RTK 差分 + 8m 基线 | 已通过（quality=4、测距误差达标） |
| 桨向 / 急停 / PWM 链路 | 已通过 |
| 先验地图 | 已恢复为与 2D 图配套的那份（2025-04-16，坐标零偏差） |
| 地图覆盖根因 | `pcd_save_en` 已关闭 + 建图产物带日期（见 §4，不要改回去） |

例行抽查（可选）：`rtk_status.py` 看 quality=4；`imu_calib_check.py --fast` 看零偏是否仍为 0。

---

## 1. 出发前门禁（三条，缺一不下水）

### 1.1 定位对齐（地图门禁）★ 最重要

先验地图（重定位基准）必须与 2D 规划图**同源**，否则打点、规划、跟线全部带偏。

```bash
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio_localization sentry_build_map.launch rviz:=true
# 另一个终端：
rosrun tf2_ros tf_echo map camera_init
ls -l --time-style=full-iso ~/haique/little-boat-motion-control/message_ws/src/sensor/NEXTE_Sentry/sentry_slam/FAST_LIO_LOCALIZATION/PCD/scans.pcd
```

| # | 判据 | 说明 |
|---|------|------|
| 1 | RViz 里实时点云与 `/prior_map` **重合**（位置+角度） | 不重合不下水 |
| 2 | `tf_echo map camera_init` 是合理小量 | 记录数值 |
| 3 | 重定位日志的 fitness 值 | 记录（越接近 1 越好，实测 0.93 通过） |
| 4 | Ctrl+C 退出后 `scans.pcd` **时间戳不变** | 证明"退出时自动覆盖先验图"已掐断 |

**开机位置**：先验图覆盖区域内、特征多的岸边，尽量靠近当初建图的起点。

> 为什么这么严：系统里有三个"地图"——① 开机读的重定位基准 `PCD/scans.pcd`（3D）；② RViz/A\* 用的 `PCD/scans.yaml`→`map.pgm`（2D）；③ 内存里的里程计地图（不落盘）。①和②必须来自同一次建图，否则点云与静态图永远对不上。

### 1.2 打点坐标系（已简化为单窗口）★ 易错

**已把 `sentry_build_map.rviz` 的 Fixed Frame 从 `camera_init` 改为 `map`** —— 现在 bringup 弹出的那一个 RViz 就同时满足"看点云对齐"和"打点规划"，**不再需要开第二个窗口**。

- 启动后前几十秒（重定位未收敛）点云会因缺少 `map→camera_init` 变换而不显示，**点云出现 = 重定位成功**（可当收敛信号用）；
- 打点必须在 Fixed Frame = **map** 的窗口里（A* 期望 map 系坐标；在 camera_init 系窗口打点会让目标点整体偏一个变换）；
- 备用：想单独开一个窗口时用 `rosrun rviz rviz -d $(rospack find a_star_planner)/rviz/a_star.rviz`。

### 1.3 执行链路例检

```bash
rostopic echo -n1 /ship/ship_info | grep -E "is_auto|is_keyboard|is_armed"
# 台架发 PWM（桨周围清空！测完先改回 1500 再 Ctrl+C）
rostopic pub -r 10 /auto_thruster_pwm thruster_manager/Thruster \
  "{left_thruster_pwm: 1620, right_thruster_pwm: 1500, left_sideThruster_pwm: 1500, right_sideThruster_pwm: 1500}"
```

---

## 2. 水试

### 2.0 录制（开测前先起）

```bash
rosbag record --lz4 --split --size=2048 -O ~/bags/water_$(date +%m%d_%H%M) \
  /Odometry /waypoints_path /LOS_path /auto_thruster_pwm /planner_status \
  /fix /ship/ship_info /tf /guidance/rl_cmd /guidance/rl_status
```

**为什么不录 `/motion_path`**：它每条消息携带"本轮任务开始至今的全部轨迹"（10 分钟任务单条约 1MB，10Hz 发布），上次把磁盘写爆就是它——2.2GB 的包里 99% 是同一份轨迹重复写了 376 遍。轨迹可由 `/Odometry` 离线重建。

### 2.1 水上静态（桨不会动）

```bash
roslaunch bringup bringup.launch
roslaunch a_star_planner a_star_demo.launch use_controller:=false    # 只规划不控制
rosrun a_star_planner waypoint_collector.py
# RViz：用 bringup 弹出的那个（Fixed Frame 已是 map），点 2~3 个近点后提交
# RViz 点 2~3 个近点后提交
rostopic pub -1 /waypoint_collector/commit std_msgs/Empty
rostopic echo /planner_status        # 期望 OK: N goals, M poses
```

判据：绿线依次穿过所有点、不穿岸；abort 用例（点岸上点）报 `ERROR: waypoint N unreachable (abort)` 且无新路径。

### 2.2 单目标回归（验证 huamo 修改无副作用）

```bash
roslaunch a_star_planner a_star_demo.launch speed_max:=0.8    # 首航降速
```

RViz：**2D Pose Estimate**（起点）→ **2D Nav Goal**（终点）。
判据：走线/过弯正常、**终点前约 5m 停船**、实际速度 ≤0.8 m/s。

### 2.3 多点规划完整步骤

**准备（三终端 + RViz）**

```bash
roslaunch bringup bringup.launch                                     # 终端1：传感器+定位（含历史地图重定位，自动弹 RViz）
roslaunch a_star_planner a_star_demo.launch speed_max:=0.8           # 终端2：规划+控制
rosrun a_star_planner waypoint_collector.py                          # 终端3：点击采集器
# RViz：用 bringup 弹出那个（Fixed Frame 已是 map），直接在图上点目标
```

**第 1 步：点目标** —— RViz 工具栏选 **Publish Point** 依次点击。每点一个：日志打印 `已收集第 N 个航点`，RViz 出现带**编号**的蓝球 + 橙色连线（编号即执行顺序）。点错就 `clear` 重来。

**第 2 步：提交**

```bash
rostopic pub -1 /waypoint_collector/commit std_msgs/Empty
rostopic echo /planner_status        # 期望 OK: N goals, M poses
```

提交瞬间发生三件事：旧轨迹（橙/青线）清空 → A* 逐段规划拼接 → 绿线出现。**此时艇开始按新路径行驶，遥控器手持。**

**第 3 步：观察判据**

| 看点 | 正常表现 |
|------|----------|
| 绿线 `/waypoints_path` | 依次穿过所有编号点、不穿岸不穿障碍 |
| 橙线 `/motion_path` | 艇走过的轨迹，从起点重新累积（仅显示，勿录制） |
| 青线 `/LOS_path` | 前视点，贴着绿线、切弯略超前 |
| 红箭头 `/Odometry` | 艇位姿，沿路径移动 |
| 艇行为 | 依次过每个点、中途不停、终点前约 5m 停车 |

**第 4 步：逐级加难**

1. 先 2 个近距离目标（20~30m）→ 通过后加到 4~5 个；
2. abort 演练：混一个岸上/图外点 → 期望 `ERROR: waypoint N unreachable (abort)`、无新路径、**艇原地不动**；
3. 交叉校验（可选）：`rosrun a_star_planner verify_goal_sequence.py X1 Y1 X2 Y2`；
4. 多轮复跑：直接再 `commit`（不清空点列表，同时清空旧轨迹）。

### 2.4 地面站全链路

```bash
rosrun tf2_tools view_frames.py      # 先确认 utm 的 TF 父系
roslaunch publish_waypoints test_recv_gpoints.launch \
  waypoint_mode:=via_astar goal_frame:=<上一步查到的父系>
```

### 2.5 结束、取消与重置

| 目的 | 命令 | 效果 |
|------|------|------|
| **一键全清 / 取消任务** | `rostopic pub -1 /waypoint_collector/clear std_msgs/Empty` | 清空点列表与标记、清空绿线、清空橙/青线、**并立即停车** |
| 重跑同一任务 | 再 `commit` 一次 | 重新规划，同时清空上一轮轨迹 |
| 只取消任务（不经采集器） | `rostopic pub -1 /waypoints_path nav_msgs/Path "{}"` | 停车 + 清绿线 + 清轨迹 |
| 应急停车 | 遥控切手动 `S,0,3,c,@,0,` | 最可靠，随时生效 |
| huamo 状态归零 | 确认艇停稳后重启 `a_star_demo.launch` | 清空静态变量 |

> **轨迹约定**：任务执行期间累积，任务结束/取消后冻结（保留回看），下一轮任务或 `clear` 时清空。
> 别在动桨时杀 huamo：thruster_manager 会保持最后收到的 PWM（≠自动归零）。

### 2.6 RViz 显示排查

**症状**：clear 或换任务后旧的规划路径仍画在 RViz 上。
**原因**：Path 显示项的 `Buffer Length` >1 时会同时显示最近 N 条路径消息。
**处理**：把该显示项 `Buffer Length` 改为 1 并保存配置（`sentry_build_map.rviz` 已改，并把 `/waypoint_markers` 加进显示项）。
**另外**：若 `clear` 后标记无变化，确认只有一个采集器进程：

```bash
ps -eo pid,lstart,cmd | grep "[w]aypoint_collector"
```

---

## 3. RL 测试

**现状**：模型已在艇上（`models/actor.pt`、`models/stage2_best_actor.pt`），契约测试 8/8 通过；**Phase 5（huamo 订阅 rl_cmd）未实现 → RL 只能 shadow，不能控艇**。

### 3.1 T1 双端一致性 + 冒烟

```bash
# NX 侧契约测试（8 项）
python3 -m pytest /home/jetson/haique/little-boat-motion-control/message_ws/src/guidance/rl_local_planner/tests/test_rl_common.py -v
# 训练机侧（Windows）
.venv\Scripts\python -m pytest rl_common_tests/test_rl_common.py -v
# 两端共用一份 rl_common：改动后在 NX 执行
bash /home/jetson/haique/little-boat-motion-control/rl_local_planner_training/sync_rl_common.sh

# 加载真实模型（替换 STUB）
roslaunch rl_local_planner rl_shadow.launch \
  model_path:=/home/jetson/haique/little-boat-motion-control/message_ws/src/guidance/rl_local_planner/models/stage2_best_actor.pt
rostopic echo /guidance/rl_status
ls -lt /home/jetson/rl_logs | head
```

判据：CSV 无 NaN、`infer_ms < 10ms`、`stub_flag=0`、不超 watchdog 200ms。

### 3.2 T2 Shadow 采数（与 LOS 并行，RL 不下发）

```bash
# 终端1：艇按 LOS 正常跑
roslaunch a_star_planner a_star_demo.launch speed_max:=0.8

# 终端2：shadow 并行观测记录（加载真实模型，替换 STUB）
roslaunch rl_local_planner rl_shadow.launch \
  model_path:=/home/jetson/haique/little-boat-motion-control/message_ws/src/guidance/rl_local_planner/models/stage2_best_actor.pt

# 终端3：录制 RL 数据包（含点云 —— 训练机侧离线重建观测必需）
rosbag record --lz4 --split --size=2048 -O ~/bags/rlshadow_$(date +%m%d_%H%M) \
  /Odometry /waypoints_path /cloud_registered_body /tf /tf_static \
  /ship/ship_info /auto_thruster_pwm /LOS_path \
  /guidance/rl_cmd /guidance/rl_status
```

**RL 数据包话题说明**（与 §2.0 的水试包差异在点云和 rl_*）：

| 话题 | 用途 |
|------|------|
| `/Odometry` `/waypoints_path` `/tf` `/tf_static` | 重建观测（艇位姿、路径特征、map→camera_init） |
| `/cloud_registered_body` | 重建 36 扇区障碍观测 —— **仅 RL 采数需要录**，也是包体积的大头（≈1 MB/s，10 分钟约 0.6 GB，lz4 压缩后更小） |
| `/guidance/rl_cmd` `/guidance/rl_status` | RL 影子输出与推理时延，用于与 LOS 对比、检查 NaN/超时 |
| `/auto_thruster_pwm` `/LOS_path` `/ship/ship_info` | 对照量：LOS 实际下发、前视点、模式状态 |

**shadow 节点自己的 CSV 日志**（T3 评估的主要数据，比 bag 更重要）：

```bash
ls -lt /home/jetson/rl_logs | head          # 每次 shadow 运行一个 CSV
```

采集要求：多圈、覆盖直线/转弯/不同航速，每圈记录航线+速度+结果。
**采数目标**：① 验证观测链路；② **用实船数据回归修正 `k_r` / `r_max`**——训练用的这两个参数是保守估计值（一阶动力学模型，其余参数 tau_u/tau_r 来自实船标定），是 sim-to-real 最大不确定项。

### 3.3 门槛与后续（未达成就别闭环）

| 门槛 | 现状 |
|------|------|
| Stage 2 成功率 ≥90% | 现在 74%（碰撞率 0%） |
| `k_r`/`r_max` 用实船数据修正 | 待 T2 数据 |
| Phase 5：huamo 接 rl_cmd + ActionProcessor 硬限幅 + SafetySupervisor 兜底 | **未实现** |

三者齐了才谈闭环（静水单目标低速 → 静态浮标 → 多目标，每级与 LOS 同航线对比）。
模型选择：纯航线跟踪用 stage1 系；带浮标避障用 stage2。

---

## 4. 地图管理纪律（防止再次踩坑）

历史上"先验地图被每次运行覆盖"导致整场水试定位失败。根因修复三处，**不要改回去**：

| 改动 | 位置 | 效果 |
|------|------|------|
| A | `FAST_LIO_LOCALIZATION/config/mid360.yaml` → `pcd_save_en: false` | 定位模式下不再自动保存 |
| B | `FAST_LIO_LOCALIZATION/launch/sentry_build_map.launch` → `save_pcd` 参数（默认 false，放在 yaml 加载之后覆盖） | 只在建图时显式开启 |
| C | `laserMapping.cpp` 保存文件名加日期 | 建图产物为 `scans_日期.pcd`，不占用先验图路径 |

**建图正确姿势**：

```bash
roslaunch fast_lio_localization sentry_build_map.launch enable_localization:=false save_pcd:=true
# 产物：PCD/scans_YYYYMMDD_HHMM.pcd；人工确认后才决定是否替换先验图（PCD + 2D 图成对更新）
```

**归档规范**：PCD + yaml + pgm 三件套放同一目录（`maps/<区域>/<日期>/`）。

---

## 5. 速查

**关键话题**

| 话题 | 内容 |
|------|------|
| `/waypoints_path` | A* 规划路径（map 系）；**发空 Path = 取消任务并停车** |
| `/goal_sequence` | 多目标输入（采集器提交到这里，非 latched） |
| `/planner_status` | 规划结果（latched）：OK / ERROR |
| `/waypoint_markers` | 采集的航点编号标记（latched） |
| `/motion_path`、`/LOS_path` | 实际轨迹、前视点（仅 RViz 显示，勿录制） |
| `/auto_thruster_pwm` → `/controller` → `/usart/toMcu` | 控制输出链路 |
| `/guidance/rl_cmd`、`/guidance/rl_status` | RL 影子输出与状态 |

**模式切换（S 帧，帧尾逗号不能漏）**

```bash
rostopic pub -1 /tcp/raw_data std_msgs/String "{data: 'S,0,3,c,#,0,'}"   # auto
rostopic pub -1 /tcp/raw_data std_msgs/String "{data: 'S,0,3,c,@,0,'}"   # 手动
```

**岸上演练（不需要艇/传感器）**

```bash
roslaunch a_star_planner a_star_demo_sim.launch boat_speed:=2.0   # 虚拟艇 + RViz 多点演示
bash ~/haique/little-boat-motion-control/message_ws/src/guidance/planner/a_star_planner/scripts/test_goal_sequence_offline.sh
```
