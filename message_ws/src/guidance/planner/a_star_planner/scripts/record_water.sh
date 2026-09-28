#!/bin/bash
# record_water.sh — 水试/RL采数录制（防呆 + 自验证版）
#
# 用法:
#   bash record_water.sh          # 基础列表（不含点云）
#   bash record_water.sh cloud    # 二合一列表（RL采数/扇区，含 /cloud_registered_body）
#
# 它替你做对四件事:
#   1. 启动前查残留录制进程 —— 有就拒绝启动（杜绝"同一文件两个进程互踩"）;
#   2. 查磁盘 ≥20G;
#   3. 前台录制，Ctrl+C 一次即停;
#   4. 停止后自动对每个分段 rosbag info 验证 + 检查 .active 残留，当场给出 好/坏 结论。
set -u

BASE_TOPICS="/Odometry /waypoints_path /LOS_path /auto_thruster_pwm /controller /planner_status /fix /ship/ship_info /tf /guidance/rl_cmd /guidance/rl_status"
CLOUD_TOPICS="/tf_static /cloud_registered_body"

# --- 1) 残留进程检查（关键防呆：同名文件绝不能有两个写者） ---
LEFT=$(ps -eo pid,etime,cmd | grep "[r]osbag record" || true)
if [ -n "$LEFT" ]; then
    echo "==============================================================="
    echo "!! 已有录制进程在跑，拒绝启动（两个进程写同一文件 = 包必坏）："
    echo "$LEFT"
    echo "确认它没用就先停掉:  kill <PID>   然后重新运行本脚本"
    echo "==============================================================="
    exit 1
fi

# --- 2) 磁盘 ---
FREE=$(df --output=avail -BG / | tail -1 | tr -dc '0-9')
echo "磁盘剩余: ${FREE}G"
if [ "$FREE" -lt 20 ]; then
    echo "!! 磁盘不足 20G，先清理（旧 bag / 日志）再录"
    exit 1
fi

# --- 3) 录制（前台；停止 = 按一次 Ctrl+C 等提示符回来） ---
PREFIX="$HOME/bags/water_$(date +%m%d_%H%M)"
mkdir -p "$HOME/bags"
TOPICS="$BASE_TOPICS"
if [ "${1:-}" = "cloud" ]; then
    TOPICS="$BASE_TOPICS $CLOUD_TOPICS"
fi
echo "输出前缀: $PREFIX"
echo "话题: $TOPICS"
echo "-----------------------------------------------"
echo "停止方法: 按【一次】Ctrl+C，等提示符回来（几秒，在写索引）"
echo "-----------------------------------------------"
rosbag record --lz4 --split --size=2048 -O "$PREFIX" $TOPICS
RC=$?
echo ""
echo "录制退出（rc=$RC），自动验证分段..."

# --- 4) 自动验证 ---
FAIL=0
SH=0
for f in ${PREFIX}_*.bag "$PREFIX.bag"; do
    [ -e "$f" ] || continue
    SH=$((SH+1))
    if rosbag info "$f" > /tmp/_baginfo.$$ 2>&1; then
        NMSG=$(grep -m1 'messages:' /tmp/_baginfo.$$ | head -1 | awk '{print $2}')
        NMSG=${NMSG:-0}
        echo "[OK] $(basename "$f")  (${NMSG} 条消息)"
    else
        echo "[坏] $(basename "$f")  —— rosbag info 失败，当场重录！"
        echo "     或用打捞器: python3 ~/haique/little-boat-motion-control/message_ws/src/guidance/planner/a_star_planner/scripts/bag_scavenger.py \"$f\" \"${f%.bag}_rec.bag\""
        FAIL=1
    fi
    rm -f /tmp/_baginfo.$$
done
for a in ${PREFIX}*.active; do
    [ -e "$a" ] || continue
    echo "[坏] 仍有未关闭文件: $(basename "$a")（录制没正常退出）"
    FAIL=1
done
if [ "$SH" = "0" ]; then
    echo "!! 没找到任何分段文件 —— 录制可能没写进数据"
    FAIL=1
fi
echo "-----------------------------------------------"
if [ "$FAIL" = "0" ]; then
    echo "全部通过 ✓  可以进行下一项测试（下一项重新运行本脚本 = 新的一段）"
else
    echo "有坏包 ✗  当场重录这一段！"
fi
exit $FAIL
