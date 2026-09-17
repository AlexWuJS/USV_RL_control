# Windows 训练机操作步骤（完整版）

> 环境分工：本文件在**小艇 Jetson NX**（192.168.50.186）上编写，
> 全部命令在 **Windows 训练机** 上执行。NX 端只做接口/部署/实艇，不做训练。

---

## 0. 前提检查（PowerShell）

```powershell
# Python 3.9+（推荐 3.10/3.11）
python --version

# 有无 NVIDIA 显卡（决定 torch 装哪个版本）
nvidia-smi
```

- 有输出 → 有 GPU，第 2 步装 CUDA 版 torch（训练快几倍）
- 报错 → 纯 CPU 也能训，Stage 1 慢一些

---

## 1. 从小艇拉取训练工程

```powershell
cd E:\无人艇\无人艇\小艇项目\RL部署

# 拷贝（rl_common 已在 NX 上同步校验一致，无需额外处理）
scp -r jetson@192.168.50.186:/home/jetson/haique/little-boat-motion-control/rl_local_planner_training .
```

完成后确认 `E:\无人艇\无人艇\小艇项目\RL部署\rl_local_planner_training\rl_common\` 存在。

---

## 2. 建立环境

```powershell
cd E:\无人艇\无人艇\小艇项目\RL部署\rl_local_planner_training

# （仅 GPU 机器）先装 CUDA 版 torch，再跑安装脚本
pip install torch --index-url https://download.pytorch.org/whl/cu121

# 一键安装 + 双测试（venv → 依赖 → 契约层单测 → 环境冒烟）
powershell -ExecutionPolicy Bypass -File windows_setup.ps1
```

**通过标准**（脚本会自动检查，失败即中止）：
- `rl_common_tests\test_rl_common.py` → `全部通过 ✓`（8 组，与 NX 端结果一致）
- `env\smoke_test.py` → `冒烟测试通过 ✓`（API / 动力学 / 随机 episode 三项）

---

## 3. Stage 1 训练（无障碍纯路径跟踪）

```powershell
# 独立终端运行（数小时级长任务），另开一个终端看曲线：
python train\train_sac.py --config configs\stage1_path_follow.yaml

# 观察训练曲线
tensorboard --logdir tb
# 浏览器打开 http://localhost:6006
```

TensorBoard 关注：`rollout/ep_rew_mean` 上升、`rollout/ep_len_mean` 上升（不撞不弃线）、
`train/ent_coef` 收敛到 0.01~0.1 量级。

**Stage 1 达标门槛**（`evaluation\eval_results.json`，初版值可按实际调整）：

| 指标 | 门槛 |
|------|------|
| success_rate | ≥ 0.95 |
| cte_rmse | ≤ 0.5 m |
| speed_max | ≤ 2.0 m/s 且起步 5s 内未贴满速（平滑起步） |
| accel_rms | 无持续高频抖动（数值结合 eval 轨迹图目视确认） |

```powershell
python evaluation\evaluate.py --config configs\stage1_path_follow.yaml --model models\stage1_actor.pt --episodes 50 --plot
```

**不达标不进下一阶段**（任务说明 §二十 阶段纪律）。

---

## 4. Stage 2 / Stage 3（续训）

```powershell
# Stage 2：静态障碍 + 域随机化
python train\train_sac.py --config configs\stage2_static_obstacles.yaml --init-model models\stage1_best\best_model.zip
python evaluation\evaluate.py --config configs\stage2_static_obstacles.yaml --model models\stage2_actor.pt --episodes 50 --plot

# Stage 3：复杂场景 + 动态障碍（可选，第一版实艇验证可跳过）
python train\train_sac.py --config configs\stage3_complex.yaml --init-model models\stage2_best\best_model.zip
python evaluation\evaluate.py --config configs\stage3_complex.yaml --model models\stage3_actor.pt --episodes 50 --plot
```

**Stage 2 达标门槛**：success_rate ≥ 0.85，collision_rate ≤ 0.05，
min_clearance_mean ≥ 1.5 m，且绕障后回到路径（轨迹图目视确认）。

---

## 5. 回传模型到小艇

```powershell
# 以 Stage 2 模型为例
scp models\stage2_actor.pt jetson@192.168.50.186:/home/jetson/haique/little-boat-motion-control/message_ws/src/guidance/rl_local_planner/models/actor.pt
```

回传后到**艇上会话（ZCode / SSH）**继续：Phase 5 huamo 集成 → Shadow Mode。
NX 端只加载 `actor.pt` 推理，不做任何训练。

---

## 常见问题

| 现象 | 处理 |
|------|------|
| `powershell 无法加载文件…执行策略` | 用 `powershell -ExecutionPolicy Bypass -File windows_setup.ps1`（脚本已带） |
| pip 装 torch 极慢 | 用上面的 `--index-url` 官方源；或先配国内镜像 |
| 训练报 `Can't pickle` / spawn 错误 | 确认用 `python train\train_sac.py` 入口运行（内部已有 `if __name__ == "__main__"` 保护，Windows 多进程必需） |
| ep_rew_mean 长期不涨 | 先回到 Stage 1 配置；检查 reward 权重是否被改过；lr 降到 1e-4 重试 |
| 策略蛇形抖动 | `reward.w_smooth` 0.30 → 0.5 重训 |
| 评估全停在起点 | progress 权重不足或碰撞惩罚过重，见 RL_TRAINING 文档 Q&A |

---

## 训练时长参考

| 阶段 | 步数 | GPU（如 RTX 3060+） | 纯 CPU |
|------|------|--------------------|--------|
| Stage 1 | 500k | ~30-60 min | ~2-4 h |
| Stage 2 | 1.5M | ~2-4 h | ~8-12 h |
| Stage 3 | 2M | ~3-6 h | 建议加 GPU |

（数量级参考，受 CPU 核数影响大；8 并行环境主要吃 CPU。）
