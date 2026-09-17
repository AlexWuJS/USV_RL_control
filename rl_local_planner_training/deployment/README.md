# 模型部署到小艇 Jetson NX

> 模型流向（单向）：训练机 SAC → best actor → `actor.pt` (TorchScript)
> → 拷贝到 NX → `rl_local_planner_node` 仅推理。NX 上不做任何训练。

## 1. 拷贝模型【在训练机执行】

```bash
scp models/stageN_actor.pt jetson@<NX的IP>:/home/jetson/haique/little-boat-motion-control/message_ws/src/guidance/rl_local_planner/models/actor.pt
```

## 2. 构建 ROS 包【在小艇 NX 执行】

```bash
cd /home/jetson/haique/little-boat-motion-control/message_ws
catkin_make --pkg rl_local_planner      # 或 catkin build rl_local_planner
source devel/setup.bash
```

注意：首次构建会生成 `rl_local_planner/NavCmd` 消息；huamo 的集成改动
（guidance_mode + rl_cmd 订阅）属于 Phase 5，未完成前 NavCmd 没有订阅者，
节点处于纯 shadow 诊断状态，不影响现有系统。

## 3. Shadow 模式运行【在小艇 NX 执行】

前提：实艇按现有方式正常航行（bringup + a_star_demo，huamo LOS 控制）。

```bash
# 无模型（STUB，仅打通数据链/日志）
roslaunch rl_local_planner rl_shadow.launch

# 有模型
roslaunch rl_local_planner rl_shadow.launch model_path:=$(rospack find rl_local_planner)/models/actor.pt
```

输出：
- `/guidance/rl_status`（std_msgs/Float32MultiArray）：[u, r, cte, e_heading, d_min, v_ref, course_ref, cmd_speed, cmd_course, supervisor, infer_ms, stub_flag]
- 运行日志 CSV：`/home/jetson/rl_logs/`（默认，见 config/rl_planner.yaml）

## 4. Phase 5 集成后的闭环（暂缓）

huamo_node 完成 guidance_mode 改造并评审 shadow 数据后：

```bash
roslaunch rl_local_planner rl_guidance.launch model_path:=.../actor.pt
```

紧急回退：地面站 `is_auto_control=false`（立即中性 PWM）；
重启动 a_star_demo.launch 即回到 LOS 模式。

## 5. 上艇前检查单

- [ ] `rl_common` 单测 NX 端全绿：`python3 tests/test_rl_common.py`
- [ ] shadow CSV 无 NaN、infer_ms < 10ms、stub_flag=0
- [ ] `hull_to_sensor_yaw` 已用已知障碍方位标定（§13.6）
- [ ] point_nav_node 未运行（`/auto_thruster_pwm` 单发布者纪律）
- [ ] 地面站急停通道演练过一次
- [ ] 首次闭环测试限速：v_max 临时降到 1.0 m/s
