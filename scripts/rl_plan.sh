#!/usr/bin/env bash
# RL 策略推理节点：提供 /rl_plan 服务（供 C++ 的 RlRemotePlanner 调用）
set -e
RL_A_DIR="${RL_A_DIR:-/home/xz/rosim_ws/rl_a}"
LIBS="${RL_PYLIBS:-/home/xz/rosim_ws/.pylibs}"
source /opt/ros/humble/setup.bash
source /home/xz/ws_moveit/install/setup.bash
[ -f /home/xz/rosim_ws/install/setup.bash ] && source /home/xz/rosim_ws/install/setup.bash
export RL_PLAN_LOG="${RL_PLAN_LOG:-/home/xz/rosim_ws/.scratch/rl_requests.jsonl}"
export ROSIM_RL_MAX_DELTA="${ROSIM_RL_MAX_DELTA:-1.5}"
export RL_A_DIR
export ROS_LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
export ROS_HOME="${ROS_HOME:-$HOME/.ros}"
export PYTHONPATH="$LIBS:$RL_A_DIR:${PYTHONPATH}"
export MPLCONFIGDIR="$LIBS/../.mpl"
exec python3 /home/xz/rosim_ws/scripts/rl_plan_node.py "$@"
