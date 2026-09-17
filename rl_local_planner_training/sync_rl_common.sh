#!/usr/bin/env bash
# 同步 rl_common 契约层：NX(ROS 包) → 训练工程目录
# 用法【在小艇 NX 执行】:  bash sync_rl_common.sh
# 之后整体打包 rl_local_planner_training/ 拷贝到训练机（见 README.md）
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../message_ws/src/guidance/rl_local_planner/rl_common"
DST="$HERE/rl_common"

if [ ! -d "$SRC" ]; then
    echo "错误: 找不到 $SRC" >&2
    exit 1
fi

rm -rf "$DST"
cp -r "$SRC" "$DST"
find "$DST" -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null || true

if diff -rq -x '__pycache__' "$SRC" "$DST" >/dev/null; then
    echo "同步完成并校验一致: $DST"
else
    echo "警告: 同步后内容不一致！" >&2
    exit 1
fi
