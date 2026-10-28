#!/usr/bin/env bash
# 在另一台机器上跑命令（笔记本指挥主力机用）
#
#   ./scripts/remote_run.sh desktop "cd ~/rosim_ws && git pull --rebase && colcon build"
#   ./scripts/remote_run.sh desktop "bash scripts/check_env.sh"
#
# 会自动 source ROS + MoveIt、创建 ROS_LOG_DIR，并把输出原样回传。
# 主机别名走 ~/.ssh/config（desktop / laptop），可用 DESKTOP_HOST 覆盖。
set +u
HOST="${1:-desktop}"; shift 2>/dev/null
CMD="$*"
[ -z "$CMD" ] && { echo "用法: remote_run.sh <host> \"命令\""; exit 1; }

# 远端环境前缀：按需改 MOVeit 路径
PRE='set +u; source /opt/ros/humble/setup.bash 2>/dev/null; source $HOME/ws_moveit/install/setup.bash 2>/dev/null; export ROS_LOG_DIR=$HOME/rosim_ws/.scratch/roslog ROS_HOME=$HOME/rosim_ws/.scratch/roshome; mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"; cd $HOME/rosim_ws 2>/dev/null;'

echo "▶ [$HOST] $CMD"
ssh -o ConnectTimeout=8 -o BatchMode=yes "$HOST" "$PRE $CMD"
rc=$?
echo "◀ [$HOST] 退出码 $rc"
exit $rc
