#!/usr/bin/env bash
# RL 路径规划演示（方案 B）：跑一条策略 episode，把关节路径发布出去。
#   ./scripts/rl_demo.sh                  # 从当前构型出发
#   ./scripts/rl_demo.sh --from-home      # 从 Q_HOME 出发
#   ./scripts/rl_demo.sh --episodes 5     # 连跑 5 条
set -e
DEMO_DIR="${RL_DEMO_DIR:-/home/xz/rosim_ws/.scratch/rl_demo_ws/rosim_ws/rl_demo}"
source /opt/ros/humble/setup.bash
source /home/xz/ws_moveit/install/setup.bash
if [ -f /home/xz/rosim_ws/install/setup.bash ]; then
  source /home/xz/rosim_ws/install/setup.bash
fi
export RL_DEMO_DIR="$DEMO_DIR"
export PYTHONPATH="$DEMO_DIR/.pylibs:$DEMO_DIR:${PYTHONPATH}"
export MPLCONFIGDIR="$DEMO_DIR/.mpl"
exec python3 /home/xz/rosim_ws/scripts/rl_demo_node.py "$@"
