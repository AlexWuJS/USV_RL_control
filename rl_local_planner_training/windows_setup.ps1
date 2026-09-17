# windows_setup.ps1 【在 Windows 训练机执行】
# 用途：在 Windows 上建立 SAC 训练环境（venv + 依赖 + 双测试）
# 用法：
#   cd E:\无人艇\无人艇\小艇项目\RL部署\rl_local_planner_training
#   powershell -ExecutionPolicy Bypass -File windows_setup.ps1
# 前提：已安装 Python 3.9+（命令行里 python 可用）；
#       若有 NVIDIA 显卡想用 GPU 训练，先手动装 CUDA 版 torch：
#       pip install torch --index-url https://download.pytorch.org/whl/cu121
$ErrorActionPreference = "Stop"

Write-Host "== 1/4 创建虚拟环境 ==" -ForegroundColor Cyan
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip

Write-Host "== 2/4 安装依赖 ==" -ForegroundColor Cyan
pip install -r requirements.txt

Write-Host "== 3/4 契约层单测（必须与小艇 NX 端结果一致）==" -ForegroundColor Cyan
python rl_common_tests\test_rl_common.py
if ($LASTEXITCODE -ne 0) { throw "契约层单测失败，禁止开始训练" }

Write-Host "== 4/4 环境冒烟测试（非训练，秒级）==" -ForegroundColor Cyan
python env\smoke_test.py
if ($LASTEXITCODE -ne 0) { throw "冒烟测试失败，禁止开始训练" }

Write-Host ""
Write-Host "环境就绪 ✓  开始训练：" -ForegroundColor Green
Write-Host "  python train\train_sac.py --config configs\stage1_path_follow.yaml"
Write-Host "训练为长时任务，建议在独立终端运行并用 tensorboard --logdir tb 观察曲线。"
